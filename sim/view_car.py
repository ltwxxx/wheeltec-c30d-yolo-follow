"""Lesson 1: drive the car forward 1 m in the MuJoCo viewer.

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/view_car.py

Not `-m mujoco.viewer`: that entry builds the Simulate window on the script
thread, and the macOS UI thread requirement is only honored by launch_passive.
"""

import os
import time

import mujoco

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.xml")
TARGET_X = 1.0
SPEED = 5.0


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    pos_adr = model.jnt_qposadr[root]
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("窗口已打开，关闭窗口或 Ctrl+C 退出", flush=True)
        last_report = 0.0
        while viewer.is_running():
            speed = SPEED if data.qpos[pos_adr] < TARGET_X else 0.0
            data.ctrl[:] = speed
            mujoco.mj_step(model, data)
            viewer.sync()

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(
                    "  x = %.2f m   ctrl = %.1f rad/s" % (data.qpos[pos_adr], speed),
                    flush=True,
                )
                last_report = now
            time.sleep(model.opt.timestep)

    print("已关闭", flush=True)


if __name__ == "__main__":
    main()
