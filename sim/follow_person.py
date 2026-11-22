"""Lesson 3: the car keeps a fixed gap to the person you drag around.

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/follow_person.py

Moving the person: double-click it to select, then hold Ctrl and drag with
the right mouse button. The car reacts on the next frame.
"""

import os
import time

import mujoco

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.xml")
HOLD_FAR = 0.5
HOLD_NEAR = 0.35
SPEED = 4.0
PERSON_Z = 0.61  # 胶囊底端刚好落在地面上的高度


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "front_range")
    range_adr = model.sensor_adr[sensor]
    cutoff = float(model.sensor_cutoff[sensor])

    # “人”是虚拟目标而不是物理物体：鼠标只能改变它的水平位置。
    # 它没有重力（car.xml 里 gravcomp），不夹住的话会被拖到天上悬着。
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "person_root")
    qadr = model.jnt_qposadr[joint]
    dof = model.jnt_dofadr[joint]

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("移动“人”：左键双击选中它，再按住 Ctrl + 右键拖动", flush=True)
        last_report = 0.0
        while viewer.is_running():
            # 贴地 + 站直 + 不留速度：拖动只保留 x/y 两个分量
            data.qpos[qadr + 2] = PERSON_Z
            data.qpos[qadr + 3:qadr + 7] = (1.0, 0.0, 0.0, 0.0)
            data.qvel[dof:dof + 6] = 0.0

            distance = float(data.sensordata[range_adr])
            # 打空时 rangefinder 返回 -1；超过 cutoff 也说明没命中
            if distance < 0 or distance >= cutoff:
                speed, state = 0.0, "看不见目标"
            elif distance > HOLD_FAR:
                speed, state = SPEED, "靠近"
            elif distance < HOLD_NEAR:
                speed, state = -SPEED, "后退"
            else:
                speed, state = 0.0, "保持距离"

            data.ctrl[:] = speed
            mujoco.mj_step(model, data)
            viewer.sync()

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(
                    "  前方 %5.2f m   %s   轮速 %+.1f" % (distance, state, speed),
                    flush=True,
                )
                last_report = now
            time.sleep(model.opt.timestep)

    print("已关闭", flush=True)


if __name__ == "__main__":
    main()
