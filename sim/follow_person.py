"""Lesson 3: the car keeps a fixed gap to the person you drag around.

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/follow_person.py

Moving the person: right-double-click it to select, then hold Ctrl and drag
with the right mouse button. The car reacts on the next frame.
"""

import os
import time

import mujoco

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.xml")
HOLD_FAR = 0.5
HOLD_NEAR = 0.35
SPEED = 4.0


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)

    sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "front_range")
    range_adr = model.sensor_adr[sensor]
    cutoff = float(model.sensor_cutoff[sensor])

    # 阻尼：松开鼠标后“人”不会继续滑走
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "person_root")
    dof = model.jnt_dofadr[joint]
    model.dof_damping[dof:dof + 6] = 5.0

    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("移动“人”：右键双击选中它，再按住 Ctrl + 右键拖动", flush=True)
        last_report = 0.0
        while viewer.is_running():
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
