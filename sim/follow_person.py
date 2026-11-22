"""Lesson 2: the car senses the person and stops by itself.

Run on macOS:  ~/mujoco-sim/bin/mjpython sim/follow_person.py

This is the first closed loop: sensor -> decision -> actuator.
Compare with view_car.py, which just pushes the motors blindly.
"""

import os
import time

import mujoco

XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car.xml")
STOP_DISTANCE = 0.5
APPROACH_SPEED = 4.0


def main() -> None:
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "front_range")
    adr = model.sensor_adr[sid]
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("窗口已打开：车会自己走向那个人，停在 0.5 m 外", flush=True)
        last_report = 0.0
        while viewer.is_running():
            distance = float(data.sensordata[adr])
            speed = APPROACH_SPEED if distance > STOP_DISTANCE else 0.0
            data.ctrl[:] = speed
            mujoco.mj_step(model, data)
            viewer.sync()

            now = time.monotonic()
            if now - last_report >= 1.0:
                print(
                    "  前方障碍 %.2f m   轮速 %.1f rad/s" % (distance, speed),
                    flush=True,
                )
                last_report = now
            time.sleep(model.opt.timestep)

    print("已关闭", flush=True)


if __name__ == "__main__":
    main()
