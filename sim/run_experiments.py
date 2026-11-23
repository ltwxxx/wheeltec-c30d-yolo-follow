"""Batch MuJoCo following experiments; no ROS or robot connection."""
import argparse
from contextlib import nullcontext
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
import xml.etree.ElementTree as ET

from experiment_core import Controller, Scenario, generate_scenarios, summarize
from experiment_report import write_report

HERE = Path(__file__).resolve().parent
SOURCES = ('experiment_core.py', 'run_experiments.py', 'experiment_report.py',
           'follow_p_control.py')
SCHEMA = 1


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                    encoding='utf-8')


def experiment_model(xml):
    """Preserve car physics. Move only the scripted target with a mocap body."""
    root = ET.fromstring(xml)
    person = root.find("./worldbody/body[@name='person']")
    if person is None or person.find('freejoint') is None:
        raise ValueError('Model must contain person with a freejoint')
    person.remove(person.find('freejoint'))
    person.attrib.pop('gravcomp', None)
    person.set('mocap', 'true')
    return ET.tostring(root, encoding='unicode')


def collision_active(model, data, robot_bodies, person_body):
    """Count robot/person contacts only; wheel/floor contact is expected."""
    for contact in data.contact:
        if contact.dist > 0:
            continue
        a, b = (int(model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2))
        if (a in robot_bodies and b == person_body or
                b in robot_bodies and a == person_body):
            return True
    return False


def run_episode(mujoco, model, scenario, controller, evaluation, observation,
                viewer=False):
    import numpy as np

    data = mujoco.MjData(model)
    person = model.body('person').id
    mocap = int(model.body_mocapid[person])
    chassis = model.body('chassis').id
    robot_bodies = {chassis}
    for body in range(model.nbody):
        if int(model.body_parentid[body]) in robot_bodies:
            robot_bodies.add(body)
    robot_geoms = [g for g in range(model.ngeom)
                   if int(model.geom_bodyid[g]) in robot_bodies
                   and (model.geom_contype[g] or model.geom_conaffinity[g])]
    root_q = int(model.jnt_qposadr[model.joint('root').id])
    range_site = model.site('range_front').id
    target_geom = next(g for g in range(model.ngeom) if int(model.geom_bodyid[g]) == person)
    target_radius = float(model.geom_size[target_geom, 0])
    sensor = model.sensor('front_range').id
    sensor_adr = int(model.sensor_adr[sensor])
    cutoff = float(model.sensor_cutoff[sensor])
    motor_ids = [model.actuator('motor_' + name).id for name in ('fl', 'fr', 'rl', 'rr')]
    dt = evaluation['control_dt_s']
    substeps = round(dt / model.opt.timestep)
    ticks = round(scenario.duration_s / dt)
    renderer = None
    if observation == 'camera':
        from follow_p_control import find_person, PIXEL_K
        renderer = mujoco.Renderer(model, 240, 320)
        options = mujoco.MjvOption()
        options.flags[int(mujoco.mjtVisFlag.mjVIS_RANGEFINDER)] = 0

    def pose_target(t):
        x, y = scenario.position(t)
        data.mocap_pos[mocap] = (x, y, .61)
        data.mocap_quat[mocap] = (1., 0., 0., 0.)

    pose_target(0.)
    for _ in range(round(.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    data.time = 0.
    mujoco.mj_forward(model, data)
    rows, collisions = [], 0
    previous_contact = False
    interval_collision = False
    completed, termination = True, None
    if viewer:
        import mujoco.viewer
        view_context = mujoco.viewer.launch_passive(model, data)
    else:
        view_context = nullcontext(None)
    try:
        with view_context as window:
            for tick in range(ticks + 1):
                if tick and window is not None and not window.is_running():
                    completed, termination = False, 'viewer_closed'
                    break
                started = time.monotonic()
                t = tick * dt
                pose_target(t)
                mujoco.mj_forward(model, data)
                x, y = scenario.position(t)
                sx, sy = data.site_xpos[range_site, :2]
                w, qx, qy, qz = data.qpos[root_q + 3:root_q + 7]
                yaw = math.atan2(2 * (w*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
                true_bearing = math.atan2(y-data.qpos[root_q+1], x-data.qpos[root_q]) - yaw
                true_bearing = math.atan2(math.sin(true_bearing), math.cos(true_bearing))
                # Explicit 2D ground-truth reference, independent of sensor validity.
                true_gap = math.hypot(x-sx, y-sy) - target_radius
                observed_bearing, observed_gap = true_bearing, true_gap
                pixels = None
                lidar = float(data.sensordata[sensor_adr])
                lidar = lidar if 0 <= lidar < cutoff else None
                if renderer is not None:
                    renderer.update_scene(data, camera='eye', scene_option=options)
                    observed_bearing, pixels = find_person(renderer.render())
                    observed_gap = (lidar if lidar is not None else
                                    PIXEL_K / math.sqrt(pixels) if pixels else None)
                if not scenario.visible(t):
                    observed_bearing, observed_gap = None, None
                valid = observed_bearing is not None and observed_gap is not None
                state, left, right = controller.command(observed_bearing, observed_gap)
                data.ctrl[motor_ids] = (left, right, left, right)
                # Physical bumper-to-target distance is separate from planar tracking gap.
                clearance = min(mujoco.mj_geomDistance(model, data, g, target_geom,
                                                       100., None) for g in robot_geoms)
                velocity = np.zeros(6)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                        chassis, velocity, 1)
                rows.append(dict(time_s=t, car_x_m=float(data.qpos[root_q]),
                                 car_y_m=float(data.qpos[root_q+1]), car_yaw_rad=yaw,
                                 car_forward_mps=float(velocity[3]),
                                 car_yaw_rate_rad_s=float(velocity[2]),
                                 target_x_m=x, target_y_m=y, true_gap_m=true_gap,
                                 true_bearing_rad=true_bearing,
                                 physical_clearance_m=clearance,
                                 observed_gap_m=observed_gap, observed_bearing_rad=observed_bearing,
                                 observation_valid=valid, range_m=lidar, pixels=pixels,
                                 state=state, left_rad_s=left, right_rad_s=right,
                                 collision=interval_collision))
                if tick == ticks:
                    break
                interval_collision = False
                for substep in range(substeps):
                    pose_target(t + substep * model.opt.timestep)
                    mujoco.mj_step(model, data)
                    contact = collision_active(model, data, robot_bodies, person)
                    collisions += int(contact and not previous_contact)
                    previous_contact = contact
                    interval_collision |= contact
                if (not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all()
                        or any(data.warning.number)):
                    completed, termination = False, 'physics_warning_or_nonfinite_state'
                    break
                if float(data.xmat[chassis, 8]) < .5:
                    completed, termination = False, 'robot_tipped'
                    break
                if window is not None:
                    window.sync()
                    time.sleep(max(0., dt - (time.monotonic() - started)))
    finally:
        if renderer is not None:
            renderer.close()
    summary = summarize(rows, scenario, controller, evaluation, collisions, completed, termination)
    summary['minimum_physical_clearance_m'] = min(r['physical_clearance_m'] for r in rows)
    return rows, summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New output directory; existing files are preserved')
    parser.add_argument('--seed', type=int)
    parser.add_argument('--repeats', type=int)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--case', help='Run one exact case ID from the generated/saved suite')
    parser.add_argument('--observation', choices=('truth', 'camera'))
    parser.add_argument('--turn-gain', type=float)
    parser.add_argument('--replay', type=Path, help='Saved manifest.json; preserves scenarios and settings')
    parser.add_argument('--viewer', action='store_true', help='One case only; macOS uses mjpython')
    args = parser.parse_args(argv)
    try:
        import mujoco
        if args.replay:
            if any(v is not None for v in (args.seed, args.repeats, args.duration,
                                           args.observation, args.turn_gain)):
                raise ValueError('Replay uses saved parameters; remove suite/controller overrides')
            manifest = json.loads(args.replay.read_text(encoding='utf-8'))
            if manifest['schema_version'] != SCHEMA:
                raise ValueError('Unsupported manifest schema')
            if manifest['runtime']['mujoco'] != mujoco.__version__:
                raise ValueError('Replay requires the recorded MuJoCo version')
            if set(manifest['source_sha256']) != set(SOURCES):
                raise ValueError('Replay source list is incomplete')
            for name, checksum in manifest['source_sha256'].items():
                if name not in SOURCES or digest(HERE / name) != checksum:
                    raise ValueError(f'Source changed since experiment: {name}')
            model_file = args.replay.resolve().parent / 'model.xml'
            if digest(model_file) != manifest['model_sha256']:
                raise ValueError('Saved model checksum mismatch')
            xml = model_file.read_text(encoding='utf-8')
            controller = Controller(**manifest['controller'])
            evaluation = manifest['evaluation']
            observation = manifest['observation']
            cases = [Scenario(**s) for s in manifest['scenarios']]
        else:
            seed = 42 if args.seed is None else args.seed
            cases = generate_scenarios(seed, 6 if args.repeats is None else args.repeats,
                                       24. if args.duration is None else args.duration)
            controller = Controller(turn_gain=2. if args.turn_gain is None else args.turn_gain)
            evaluation = dict(control_dt_s=.034, hold_s=2., bearing_tolerance_deg=8.)
            observation = args.observation or 'truth'
            xml = (HERE / 'car.xml').read_text(encoding='utf-8')
            manifest = dict(schema_version=SCHEMA, seed=seed, controller=asdict(controller),
                            evaluation=evaluation, observation=observation,
                            scenarios=[asdict(s) for s in cases],
                            source_sha256={name: digest(HERE/name) for name in SOURCES},
                            runtime=dict(mujoco=mujoco.__version__, python=platform.python_version(),
                                         platform=platform.platform()),
                            metric_reference='2D distance from range_front XY to person capsule footprint',
                            fidelity='teaching model; not calibrated against the robot')
        if args.case:
            cases = [s for s in cases if s.case_id == args.case]
            if not cases:
                raise ValueError('Case ID is not in this suite')
        if args.viewer and len(cases) != 1:
            raise ValueError('--viewer requires --case')
        model = mujoco.MjModel.from_xml_string(experiment_model(xml))
        dt = evaluation['control_dt_s']
        if abs(round(dt/model.opt.timestep)*model.opt.timestep - dt) > 1e-9:
            raise ValueError('Control period must be an integer multiple of physics timestep')
        output = args.output or HERE/'results'/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        output.mkdir(parents=True, exist_ok=False)
        (output/'model.xml').write_text(xml, encoding='utf-8')
        manifest = dict(manifest, scenarios=[asdict(s) for s in cases],
                        model_sha256=digest(output/'model.xml'),
                        created_at=datetime.now(timezone.utc).isoformat(),
                        replay_of=str(args.replay.resolve()) if args.replay else None,
                        physics_dt_s=float(model.opt.timestep))
        write_json(output/'manifest.json', manifest)
        summaries = []
        for case in cases:
            rows, summary = run_episode(mujoco, model, case, controller, evaluation,
                                        observation, args.viewer)
            case_dir = output/case.case_id
            case_dir.mkdir()
            write_json(case_dir/'scenario.json', asdict(case))
            write_json(case_dir/'summary.json', summary)
            with (case_dir/'telemetry.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            summaries.append(summary)
            print(f'{case.case_id}: {"PASS" if summary["success"] else "FAIL"} '
                  f'band_error={summary["mean_band_error_m"]:.3f} m '
                  f'collisions={summary["collision_events"]}', flush=True)
        write_json(output/'summary.json', summaries)
        with (output/'summary.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
        write_report(output, manifest, summaries)
        print(f'Report: {(output/"report.html").resolve()}')
        return 0
    except (ValueError, KeyError, OSError, ImportError) as exc:
        parser.exit(2, f'Experiment error: {exc}\n')


if __name__ == '__main__':
    sys.exit(main())
