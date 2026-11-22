"""Lesson 6: 比例控制（P 控制）——误差越大大动作，误差小小动作。

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/follow_p_control.py

和上一课 follow_camera.py 只差一个 control()：
  上一课  轮速只有 0 / ±4 两档        → 一冲一停
  这一课  轮速 = 误差 × 系数，连续变化 → 靠近时自动减速，滑进目标间距

激光是一根线，摄像头是一整片视野：人偏出那根线时激光打空。
这时改用“人在画面里占多少像素”估距离（像素越多 = 离得越近），
和真实里用 YOLO 框的高度估距离是同一个思路。

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
MAX_SPEED = 6.0            # 前进/后退的轮速上限（真车上要按电机能力调）
DEADBAND_DIST = 0.05       # 间距误差小于它就不动（防抖动）
PIXEL_K = 110.0            # 像素数换算距离的标定系数：约 4200 px ↔ 1.7 m

KP_TURN = 2.0              # 方位每差 1 弧度，补多少轮速
MAX_TURN = 3.0             # 轮速差上限
DEADBAND_TURN = math.radians(3)
SEARCH_TURN = 2.0          # 找不到人时的原地搜索转速

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


def control(bearing, distance, cutoff, pixels):
    """返回 (状态, 前进量, 转向量, 间距估计)。

    P 控制的全部内容就是两个乘法：
      前进量 = 距离误差 × KP_SPEED
      转向量 = 方位误差 × KP_TURN
    误差大 → 动作大；误差小 → 动作小；误差接近 0 → 停下。
    """
    if bearing is None:
        return "找人", 0.0, SEARCH_TURN, distance

    if 0.0 <= distance < cutoff:
        gap = distance
    else:
        # 激光打空（人不在细线上，或远过量程）：用人在画面里的像素数反推距离。
        # 面积 ∝ 1/距离²，所以 距离 ≈ PIXEL_K / sqrt(像素数)。
        gap = PIXEL_K / math.sqrt(pixels)

    err = gap - TARGET
    if err > DEADBAND_DIST:
        state = "靠近" if 0.0 <= distance < cutoff else "往前探"
    elif err < -DEADBAND_DIST:
        # 只有激光可信时才主动后退；靠像素估的距离太粗，不拿它倒车
        if 0.0 <= distance < cutoff:
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
    return state, speed, turn, gap


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


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "front_range")
    range_adr = model.sensor_adr[sensor]
    cutoff = float(model.sensor_cutoff[sensor])

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
        print("目标间距 %.2f m：离得越远走得越快，靠近时自动减速" % TARGET, flush=True)
        last_report = 0.0
        next_tick = time.monotonic()
        while viewer.is_running():
            clamp_person(data, per_q, per_v)
            clamp_car(data, car_q, car_v)

            distance = float(data.sensordata[range_adr])
            renderer.update_scene(data, camera="eye", scene_option=cam_opt)
            bearing, pixels = find_person(renderer.render())
            state, speed, turn, gap = control(bearing, distance, cutoff, pixels)

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
                label = "前方" if 0.0 <= distance < cutoff else "估距"
                print(
                    "  看见 %4d px  方位 %-5s  %s %5.2f m  %-6s  轮速 左%+.1f 右%+.1f"
                    % (pixels, where, label, gap, state, speed - turn, speed + turn),
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
