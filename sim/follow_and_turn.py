"""Lesson 4: the car turns toward you first, then keeps the gap.

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/follow_and_turn.py

Moving the person: double-click it to select, then hold Ctrl and drag with
the right mouse button.

和真车一致的两路信息：
  方位  —— 人在车的左边还是右边、偏多少度（实车上由摄像头/YOLO 给出）
  距离  —— 正前方测距传感器，只在“大致对准”时才有读数
"""

import math
import os
import time

import mujoco

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.xml")
HOLD_FAR = 0.50          # 比这远就靠近（米）
HOLD_NEAR = 0.35         # 比这近就后退（米）
SPEED = 4.0              # 轮速（rad/s）
PERSON_Z = 0.61          # 胶囊底端刚好落在地面上的高度

AIM_TOL = math.radians(8)      # 偏差小于它才算“对准”
DEADBAND = math.radians(3)     # 偏差小于它就不补转向
KP_TURN = 2.0                  # 每 1 弧度偏差补多少轮速
MAX_TURN = 3.0                 # 轮速差的上限


def yaw_of(qpos, car_q):
    w, x, y, z = qpos[car_q + 3:car_q + 7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def control(data, car_q, per_q, distance, cutoff):
    """返回 (state, speed, turn, bearing)：speed 前进量，turn 转向量。"""
    cx, cy = data.qpos[car_q], data.qpos[car_q + 1]
    yaw = yaw_of(data.qpos, car_q)

    # 人在车坐标系里的位置：ahead 朝前，lateral 朝左
    dx = data.qpos[per_q] - cx
    dy = data.qpos[per_q + 1] - cy
    ahead = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    bearing = math.atan2(lateral, ahead)   # >0：人在左边

    hit = 0.0 <= distance < cutoff
    if abs(bearing) > AIM_TOL:
        state, speed = "转身对准", 0.0
    elif hit and distance > HOLD_FAR:
        state, speed = "靠近", SPEED
    elif hit and distance < HOLD_NEAR:
        state, speed = "后退", -SPEED
    elif hit:
        state, speed = "保持距离", 0.0
    else:
        state, speed = "往前探", SPEED * 0.4

    turn = 0.0
    if abs(bearing) > DEADBAND:
        turn = max(-MAX_TURN, min(MAX_TURN, KP_TURN * bearing))
    return state, speed, turn, bearing


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "front_range")
    range_adr = model.sensor_adr[sensor]
    cutoff = float(model.sensor_cutoff[sensor])

    car_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    car_q = model.jnt_qposadr[car_joint]
    per_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "person_root")
    per_q = model.jnt_qposadr[per_joint]
    per_v = model.jnt_dofadr[per_joint]

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("移动“人”：左键双击选中它，再按住 Ctrl + 右键拖动", flush=True)
        last_report = 0.0
        while viewer.is_running():
            # 人只能贴地拖着走：贴地、站直、不留速度
            data.qpos[per_q + 2] = PERSON_Z
            data.qpos[per_q + 3:per_q + 7] = (1.0, 0.0, 0.0, 0.0)
            data.qvel[per_v:per_v + 6] = 0.0

            distance = float(data.sensordata[range_adr])
            state, speed, turn, bearing = control(
                data, car_q, per_q, distance, cutoff
            )

            # 左轮 = speed - turn，右轮 = speed + turn（turn>0 向左转）
            data.ctrl[0] = speed - turn   # 左前
            data.ctrl[1] = speed + turn   # 右前
            data.ctrl[2] = speed - turn   # 左后
            data.ctrl[3] = speed + turn   # 右后

            mujoco.mj_step(model, data)
            viewer.sync()

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(
                    "  方位 %+4.0f°   前方 %5.2f m   %-6s  轮速 左%+.1f 右%+.1f"
                    % (math.degrees(bearing), distance, state,
                       speed - turn, speed + turn),
                    flush=True,
                )
                last_report = now
            time.sleep(model.opt.timestep)

    print("已关闭", flush=True)


if __name__ == "__main__":
    main()
