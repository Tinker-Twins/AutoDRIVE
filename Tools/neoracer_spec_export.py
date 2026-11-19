"""
Writes the NeoRacer's physics, as neoracer-mujoco models it, for AutoDRIVE:
Assets/Prefabs/NeoRacer/neoracer_spec.json, which NeoRacerPhysics (menu
AutoDRIVE > NeoRacer: Physics from the Spec) puts on the NeoRacer prefab.

neoracer-mujoco (Neobotics-Foundation-Inc/neoracer-mujoco) describes the car
once, in neoracer_spec; its MuJoCo model is computed from that. This script
compiles that model, lets the car settle on the floor, and takes from it what
a Unity car needs: Unity's car is one rigid body on four wheel colliders, so
the body's mass, center of mass and inertia are the whole settled car's, about
its center of mass. The steering servo's, the motor's and the firmware speed
loop's numbers are the spec's derived ones (neoracer_spec.derive), and so are
the body's outline (what it collides as), its resting heights and the camera's,
the LiDAR's and the IMU's places and settings.

Frame of the output: +X forward, +Y left, +Z up, from the middle of the
wheelbase on the axle line, the line through the settled wheels' centers (the
spec's chassis origin sits the spec's axle drop above it). SI units.

Run with neoracer-mujoco importable (its repository's environment):
    python Tools/neoracer_spec_export.py [OUT.json] [--source "neoracer-mujoco <revision>"]
--source names the model's revision where its checkout is not a git repository.
"""

import json
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from neoracer_mujoco import assets, sim
from neoracer_spec.derive import axle_drop, derive
from neoracer_spec.neoracer import NEORACER

OUT = Path(__file__).resolve().parent.parent / "Assets/Prefabs/NeoRacer/neoracer_spec.json"


def source() -> str:
    if "--source" in sys.argv:
        return sys.argv[sys.argv.index("--source") + 1]
    import neoracer_spec

    root = Path(neoracer_spec.__file__).resolve().parents[2]
    try:
        rev = subprocess.run(["git", "-C", str(root), "describe", "--always", "--dirty"],
                             capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        rev = "unknown revision"
    return f"neoracer-mujoco {rev} ({root})"


def body_inertia_world(model, data, b):
    """Body b's inertia about its own center of mass, in the world frame."""
    r = data.ximat[b].reshape(3, 3)
    return r @ np.diag(model.body_inertia[b]) @ r.T


def main():
    args = [a for i, a in enumerate(sys.argv[1:], 1) if a != "--source" and sys.argv[i - 1] != "--source"]
    out = Path(args[0]) if args else OUT
    params = derive(NEORACER)
    scene = assets.load()
    model = scene.model
    data = sim.settle(model)
    mujoco.mj_forward(model, data)

    car = model.body(scene.car.body.name).id
    rot = data.xmat[car].reshape(3, 3)  # car frame -> world
    wheels_at = [model.body(f"{corner}_wheel_body").id for corner in ("fl", "fr", "rl", "rr")]
    origin = np.mean([data.xpos[b] for b in wheels_at], axis=0)  # the axle line's middle
    # the spec's chassis frame (its outline, its sensors) in this one: its origin, settled
    chassis = rot.T @ (data.xpos[car] - origin)
    bodies = [b for b in range(model.nbody) if b == car or model.body_rootid[b] == car]
    mass = sum(model.body_mass[b] for b in bodies)
    com_world = sum(model.body_mass[b] * data.xipos[b] for b in bodies) / mass
    inertia = np.zeros((3, 3))
    for b in bodies:
        d = data.xipos[b] - com_world
        inertia += body_inertia_world(model, data, b) + model.body_mass[b] * (d @ d * np.eye(3) - np.outer(d, d))
    inertia_car = rot.T @ inertia @ rot
    com_car = rot.T @ (com_world - origin)

    # the steering input's effective inertia, damping and dry friction: the input joint and the two
    # front wheels it turns (the Ackermann constraints couple them nearly 1:1)
    steer = {}
    for name in ("steer_input", "fl_steer", "fr_steer"):
        j = model.joint(name)
        dof = j.dofadr[0]
        b = j.bodyid[0]
        izz = body_inertia_world(model, data, b)[2, 2] if name != "steer_input" else model.body_inertia[b][2]
        steer[name] = (model.dof_armature[dof] + izz, model.dof_damping[dof], model.dof_frictionloss[dof])
    servo = params.model.common.servo
    springs = params.model.common.springs
    wheels = params.model.common.wheels
    drive = params.model.drive
    loop = params.speed_loop
    spec = NEORACER
    outline = spec.chassis.outline.value
    resting = spec.chassis.resting_heights.value

    def place(p):  # a point of the spec's chassis frame in this one
        return [float(v) for v in chassis + np.asarray(p)]
    result = {
        "source": source(),
        "frame": "+X forward, +Y left, +Z up, from the middle of the wheelbase on the axle line; SI units",
        "timestep": float(model.opt.timestep),
        "body": {
            "mass": float(mass),
            "center_of_mass": [float(v) for v in com_car],
            "inertia": [[float(v) for v in row] for row in inertia_car],
        },
        "outline": {
            "corners": [[float(x + chassis[0]), float(y + chassis[1])] for x, y in outline.corners],
            "bottom": float(outline.bottom + chassis[2]), "top": float(outline.top + chassis[2]),
        },
        "resting_heights": {
            "tub_underside": resting.tub_underside, "lidar_top": resting.lidar_top,
            "axle_drop": axle_drop(spec),
        },
        "wheels": {
            "half_wheelbase": wheels.half_wheelbase, "half_track": wheels.half_track, "radius": wheels.radius,
            "mass": wheels.mass, "drive_damping": wheels.damping, "friction": wheels.friction[0],
        },
        "camera": {
            "position": place(spec.camera.position.value), "fov_degrees": spec.camera.fov_degrees.value,
            "width": spec.camera.stream.value.width, "height": spec.camera.stream.value.height,
            "fps": spec.camera.stream.value.fps,
        },
        "lidar": {
            "position": place(spec.lidar.position.value), "scan_rate": spec.lidar.scan_rate.value,
            "resolution_degrees": spec.lidar.resolution_degrees.value, "field_degrees": spec.lidar.field_degrees.value,
            "range": spec.lidar.range_bright.value,
        },
        "imu": {"position": place(spec.imu.position.value), "rate": spec.imu.rate.value},
        "suspension": {
            "stiffness": springs.stiffness[0], "damping": springs.damping[0],
            "droop": -springs.range[0], "jounce": springs.range[1],
        },
        "motor": {
            "rated_voltage": drive.nominal[0], "stall_torque": drive.nominal[1], "free_speed": drive.nominal[2],
            "gear": drive.gear, "battery_voltage": drive.ctrl_limit,
        },
        "servo": {
            "voltage": servo.nominal[0], "stall_torque": servo.nominal[1], "free_speed": servo.nominal[2],
            "kp": servo.kp, "kd": servo.kd, "voltage_limit": servo.voltage_limit, "lock": servo.lock,
            "latency": servo.delay.latency,
            "inertia": float(sum(v[0] for v in steer.values())),
            "damping": float(sum(v[1] for v in steer.values())),
            "frictionloss": float(sum(v[2] for v in steer.values())),
        },
        "speed_loop": {
            "period": loop.measure.window, "filter": loop.measure.alpha, "kp": loop.kp, "ki": loop.ki,
            "kd": loop.kd, "integral_limit": loop.integral_limit, "error_deadband": loop.error_deadband,
            "stop_speed": loop.stop_speed, "feed_forward": loop.feed_forward, "deadband": loop.deadband,
            "feed_forward_limit": loop.feed_forward_limit, "pulse_limit": loop.pulse_limit,
            "full_throttle_pulse": loop.full_throttle_pulse, "max_speed": loop.max_speed,
        },
    }
    out.write_text(json.dumps(result, indent=1) + "\n")
    print(f"wrote {out}")
    print(json.dumps(result["body"], indent=1))
    print(json.dumps(result["servo"], indent=1))


if __name__ == "__main__":
    main()
