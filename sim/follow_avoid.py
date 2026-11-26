"""Lesson 7: 避障——挡路的不一定是人。

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/follow_avoid.py

两处新东西：
1) 第 6 课只有一根激光，树干只要偏开一点点就被漏掉。
   这一课改成“扫一片”：每帧从车头往 ±50° 打 101 根射线，取最近的那个，
   真车的 2D 激光雷达（转一圈几千个点）干的就是这件事。
2) 激光近不代表要停：人本来就会走到很近。
   得先分清激光看到的到底是谁——相机估的人距和激光读数差不多的，是人；
   激光读数比人近很多的，中间夹了别的东西，进入“绕障”。
绕障时比一比左右两侧哪边更空，往空的一边转；离障碍很近就先原地转。

窗口里只画了 0° 和 ±30° 三根黄线，免得 101 根糊成一片。

拖动：左键双击选中“人”或“车”，再按住 Ctrl + 右键拖动。
看画面：按 ] 先俯瞰全景、再按一次切到车头视角，Esc 回到自由视角。
"""

import math
import os
import time

import mujoco
import numpy as np

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.xml")
W, H = 320, 240            # 摄像头分辨率
FY = (H / 2) / math.tan(math.radians(65.0 / 2))   # 镜头焦距（像素）

TARGET = 0.45              # 想保持的间距（米）
KP_SPEED = 3.0             # 距离每差 1 米，补多少轮速
MAX_SPEED = 6.0            # 前进/后退的轮速上限
DEADBAND_DIST = 0.05       # 间距误差小于它就不动（防抖动）
PIXEL_K = 110.0            # 像素数换算距离的标定系数：约 4200 px ↔ 1.7 m

KP_TURN = 2.0              # 方位每差 1 弧度，补多少轮速
MAX_TURN = 3.0             # 轮速差上限
DEADBAND_TURN = math.radians(3)
SEARCH_TURN = 2.0          # 找不到人时的原地搜索转速

FAN_HALF = 50              # 激光扫描半角（度）
FAN_STEP = 1               # 每 1° 一根射线
FRONT_HALF = 15            # ±15° 以内算“正前方”
SIDE_IN = 30               # 30°~50° 这段算“侧面”

OBS_MARGIN = 0.45          # 激光比“相机估的人距”近这么多 → 前头不是人
OBS_RANGE = 1.3            # 只对这么近的东西做绕障
OBS_MIN_EST = 1.0          # 人比这更近时画面会裁掉半个人，估距不可靠，不做判断
OBS_TIE = 0.15             # 左右一样空（差距小于它）时算平手
SIDE_SAFE = 0.30           # 侧面比这更近：贴着障碍了，往外让
CREEP_SPEED = 1.0          # 绕障时允许的最大前进轮速
STOP_CLEAR = 0.35          # 比它更近就原地转，不再往前蹭

PERSON_Z = 0.61
CAR_Z = 0.09

MIN_PIXELS = 20            # 少于这么多橙色像素就当没看见
CONTROL_DT = 1 / 30.0      # 视觉只跑 30 帧/秒，物理在两次之间补步


def find_person(img):
    """在摄像头画面里找橙色的人。返回 (方位弧度, 像素数)；看不见时方位为 None。"""
    r = img[..., 0].astype(int)
    g = img[..., 1].astype(int)
    mask = (r > 90) & (r - g > 35)
    xs = np.nonzero(mask)[1]
    if xs.size < MIN_PIXELS:
        return None, xs.size
    return -math.atan((xs.mean() - W / 2) / FY), xs.size


def scan(model, data, site_id, exclude_body, half_deg, step_deg):
    """把车头扫成一片扇形，返回 [(角度, 距离), ...]，打空的距离是 -1。

    角度是相对车头的度数。mj_ray 会跳过 exclude_body 里的身体，
    所以射线不会打到自己车身上。
    """
    pos = data.site_xpos[site_id]
    z_axis = data.site_xmat[site_id].reshape(3, 3)[:, 2]   # site 的 z 轴 = 激光方向
    heading = math.atan2(z_axis[1], z_axis[0])
    rays = []
    for deg in range(-half_deg, half_deg + 1, step_deg):
        a = heading + math.radians(deg)
        vec = np.array([math.cos(a), math.sin(a), 0.0])
        dist = mujoco.mj_ray(model, data, pos, vec, None, True, exclude_body, None, None)
        rays.append((deg, dist))
    return rays


def nearest(rays, lo, hi):
    """扇形里 [lo, hi] 这段角度上最近的一根，返回 (角度, 距离)；全打空返回 (0, -1)。"""
    best = None
    for deg, dist in rays:
        if lo <= deg <= hi and dist >= 0.0 and (best is None or dist < best[1]):
            best = (deg, dist)
    return best if best else (0.0, -1.0)


def control(bearing, front_angle, front, left, right, pixels, cutoff):
    """返回 (状态, 前进量, 转向量, 间距, 是否在绕障)。

    新增的第一步：激光看到的，是不是人？
      person_ahead: 激光读数和相机估的人距差不多 → 前面就是目标；
      blocked:      激光读数比人近得多 → 中间夹了障碍物，够近才触发。
    逃避方向看障碍偏在哪边：障碍偏左就往右打，反之亦然。
    转向量 > 0 表示向左转。
    """
    if bearing is None:
        return "找人", 0.0, SEARCH_TURN, front, False

    est = PIXEL_K / math.sqrt(pixels)      # 相机估的人距（和激光用同一把尺子）
    front_hit = 0.0 <= front < cutoff

    person_ahead = front_hit and abs(front - est) <= OBS_MARGIN
    blocked = (front_hit and not person_ahead
               and front < OBS_RANGE and est > OBS_MIN_EST)
    near_left = 0.0 <= left < SIDE_SAFE
    near_right = 0.0 <= right < SIDE_SAFE

    if blocked or near_left or near_right:
        if blocked:
            left_clear = left if 0.0 <= left < cutoff else cutoff
            right_clear = right if 0.0 <= right < cutoff else cutoff
            if abs(front_angle) >= 4.0:
                # 障碍偏左 → 往右绕；障碍偏右 → 往左绕
                away = -1.0 if front_angle > 0 else 1.0
            elif left_clear > right_clear + OBS_TIE:
                away = 1.0
            elif right_clear > left_clear + OBS_TIE:
                away = -1.0
            else:
                # 障碍正对着、两边一样空：朝人那一侧绕，路最短
                away = 1.0 if bearing >= 0 else -1.0
            speed = 0.0
            if front >= STOP_CLEAR:
                # 离障碍越远蹭得越快，靠近了自动收脚
                speed = CREEP_SPEED * (front - STOP_CLEAR) / (OBS_RANGE - STOP_CLEAR)
        else:
            # 已经贴着东西从旁边过：往离它远的那边让，慢慢蹭过去
            if near_left and near_right:
                away = -1.0 if left > right else 1.0
            else:
                away = -1.0 if near_left else 1.0
            near = min(left if near_left else 9.9, right if near_right else 9.9)
            speed = 0.0 if near < 0.15 else CREEP_SPEED * 0.5
        return "绕障", speed, away * MAX_TURN, front, True

    gap = front if front_hit else est
    err = gap - TARGET
    if err > DEADBAND_DIST:
        state = "靠近" if front_hit else "往前探"
    elif err < -DEADBAND_DIST:
        # 只有激光可信时才主动后退；靠像素估的距离太粗，不拿它倒车
        if front_hit:
            state = "后退"
        else:
            state, err = "保持", 0.0
    else:
        state, err = "保持", 0.0

    speed = max(-MAX_SPEED, min(MAX_SPEED, KP_SPEED * err))

    # 人偏得越厉害，前进分量越小：还没对准就先转弯，别一头冲过去
    speed *= max(0.0, math.cos(bearing))

    turn = 0.0
    if abs(bearing) > DEADBAND_TURN:
        turn = max(-MAX_TURN, min(MAX_TURN, KP_TURN * bearing))
    return state, speed, turn, gap, False


def clamp_person(data, per_q, per_v):
    """人只能贴地拖着走：贴地、站直、不留速度。"""
    data.qpos[per_q + 2] = PERSON_Z
    data.qpos[per_q + 3:per_q + 7] = (1.0, 0.0, 0.0, 0.0)
    data.qvel[per_v:per_v + 6] = 0.0


def clamp_car(data, car_q, car_v):
    """车也只能在地面上拖：保留位置和朝向，扶正、贴地。"""
    w, x, y, z = data.qpos[car_q + 3:car_q + 7]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    data.qpos[car_q + 2] = CAR_Z
    data.qpos[car_q + 3] = math.cos(yaw / 2)
    data.qpos[car_q + 4:car_q + 6] = (0.0, 0.0)
    data.qpos[car_q + 6] = math.sin(yaw / 2)
    data.qvel[car_v + 2] = 0.0
    data.qvel[car_v + 3:car_v + 5] = 0.0


def show(value, cutoff):
    """激光读数打印：打空就显示 --。"""
    return "  -- " if not (0.0 <= value < cutoff) else "%5.2f" % value


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    front_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "range_front")
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    cutoff = 6.0

    car_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    car_q = model.jnt_qposadr[car_joint]
    car_v = model.jnt_dofadr[car_joint]
    per_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "person_root")
    per_q = model.jnt_qposadr[per_joint]
    per_v = model.jnt_dofadr[per_joint]

    renderer = mujoco.Renderer(model, H, W)
    # 激光束是窗口里的辅助线，真摄像头拍不到：渲染相机画面时关掉
    cam_opt = mujoco.MjvOption()
    cam_opt.flags[int(mujoco.mjtVisFlag.mjVIS_RANGEFINDER)] = 0
    substeps = max(1, round(CONTROL_DT / model.opt.timestep))

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("拖动“人”或“车”：左键双击选中，再按住 Ctrl + 右键拖动", flush=True)
        print("看画面：按 ] 先俯瞰全景、再按一次切到车头视角，Esc 回到自由视角", flush=True)
        print("把车开到树/石头/灌木前面，车会显示“绕障”并绕过去", flush=True)
        last_report = 0.0
        next_tick = time.monotonic()
        while viewer.is_running():
            clamp_person(data, per_q, per_v)
            clamp_car(data, car_q, car_v)

            rays = scan(model, data, front_sid, chassis, FAN_HALF, FAN_STEP)
            front_angle, front = nearest(rays, -FRONT_HALF, FRONT_HALF)
            left = nearest(rays, SIDE_IN, FAN_HALF)[1]
            right = nearest(rays, -FAN_HALF, -SIDE_IN)[1]

            renderer.update_scene(data, camera="eye", scene_option=cam_opt)
            bearing, pixels = find_person(renderer.render())
            state, speed, turn, gap, _ = control(
                bearing, front_angle, front, left, right, pixels, cutoff
            )

            data.ctrl[0] = speed - turn   # 左前
            data.ctrl[1] = speed + turn   # 右前
            data.ctrl[2] = speed - turn   # 左后
            data.ctrl[3] = speed + turn   # 右后

            for _ in range(substeps):
                mujoco.mj_step(model, data)
            viewer.sync()

            now = time.monotonic()
            if now - last_report >= 1.0:
                where = "看不见" if bearing is None else "%+4.0f°" % math.degrees(bearing)
                print(
                    "  看见 %4d px  方位 %-5s  前 %s  左 %s  右 %s  %-6s  轮速 左%+.1f 右%+.1f"
                    % (pixels, where, show(front, cutoff), show(left, cutoff),
                       show(right, cutoff), state, speed - turn, speed + turn),
                    flush=True,
                )
                last_report = now

            next_tick += CONTROL_DT
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    renderer.close()
    print("已关闭", flush=True)


if __name__ == "__main__":
    main()
