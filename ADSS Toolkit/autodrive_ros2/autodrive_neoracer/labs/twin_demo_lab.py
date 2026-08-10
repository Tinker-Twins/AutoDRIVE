"""
NeoRacer digital twin racing lab.

Pure MIT racecar_core student code: rc.lidar.get_samples() in,
rc.drive.set_speed_angle() out. This exact file runs on the physical
NeoRacer and on the AutoDRIVE digital twin without changing a line.

Conventions (identical on twin and metal):
  - lidar samples are in CENTIMETERS; get_samples() is a full-circle array
    (~1440 bins), index 0 = front, increasing clockwise; the blind rear
    wedge and no-return rays read 0
  - speed is normalized [-1, 1] (1.0 = full forward)
  - angle is normalized [-1, 1] (positive = turn RIGHT)

Racing strategy (follow-the-gap with disparity extension):
  1. DISPARITY EXTENSION: at every range discontinuity, extend the closer
     edge over the gap by the car's half-width at that distance. This erases
     every direction that would clip a corner -- the cure for early apexing.
  2. smooth what remains and prefer directions near straight ahead
  3. full throttle on open straights; shed speed for walls and hard turns
"""

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "library"))

import numpy as np
import racecar_core

rc = racecar_core.create_racecar()

prev_steer = 0.0
frame_count = 0


def start():
    rc.drive.set_max_speed(1.0)
    rc.drive.set_speed_angle(0.0, 0.0)
    print(">> MIT racecar_core RACING the NeoRacer twin -- full send")


def update():
    global prev_steer, frame_count

    samples = np.asarray(rc.lidar.get_samples())
    if samples.size < 100:
        return

    n = samples.size                    # ~1440 bins over the full circle
    mid = n // 2
    per_deg = n / 360.0
    # Library convention: index 0 = front, clockwise. Recenter so the array
    # midpoint is straight ahead and the window math stays symmetric.
    scan = np.roll(samples, mid)
    scan = np.where(np.isfinite(scan) & (scan > 5.0), scan, 1000.0)  # cm; 0 = no return

    # Planning window: +/-80 deg around straight ahead
    half = int(80 * per_deg)
    window = scan[mid - half:mid + half].copy()

    # Disparity extension: the car is ~40 cm wide, not a point. Wherever the
    # range jumps (a corner edge), overwrite the far side of the jump with the
    # near distance for the angular width the car body needs at that range.
    HALF_WIDTH_CM = 22.0
    jumps = np.abs(np.diff(window))
    for i in np.nonzero(jumps > 80.0)[0]:
        near = min(window[i], window[i + 1])
        n_ext = int(np.degrees(np.arctan2(HALF_WIDTH_CM, max(near, 20.0))) * per_deg) + 1
        if window[i] < window[i + 1]:   # edge on the left of the jump: extend right
            window[i + 1:i + 1 + n_ext] = np.minimum(window[i + 1:i + 1 + n_ext], near)
        else:                            # edge on the right: extend left
            window[max(0, i - n_ext + 1):i + 1] = np.minimum(window[max(0, i - n_ext + 1):i + 1], near)

    # Smooth (~4 deg) and bias toward straight ahead
    kernel = np.ones(17) / 17.0
    smooth = np.convolve(window, kernel, mode='same')
    angles = (np.arange(smooth.size) - half) / per_deg          # degrees
    smooth = smooth * (0.55 + 0.45 * np.cos(np.radians(angles)))

    best = int(np.argmax(smooth))
    target_deg = angles[best]                                   # negative = left

    # Steering: proportional + derivative lead to damp weave
    steer = float(np.clip(target_deg / 35.0, -1.0, 1.0))
    steer = float(np.clip(steer + 0.25 * (steer - prev_steer), -1.0, 1.0))
    prev_steer = steer

    # Speed: full send at 2.5 m of road. Quadratic turn penalty: micro
    # corrections on straights are free, real cornering still sheds speed.
    # Clearance measured on the EXTENDED window: respects car width.
    c0 = half - int(12 * per_deg)
    front = float(np.min(window[c0:half + int(12 * per_deg)]))
    clearance_term = front / 250.0
    turn_term = 1.0 - 0.6 * steer * steer
    speed = float(np.clip(min(clearance_term, turn_term), 0.30, 1.0))
    if front < 60.0:                                            # under 60 cm: stop
        speed = 0.0

    frame_count += 1
    if frame_count % 6 == 0 or speed == 0.0:
        print(f"[lab] target={target_deg:+6.1f}deg steer={steer:+.2f} "
              f"speed={speed:.2f} front={front:5.0f}cm "
              f"L90={scan[mid - int(90 * per_deg)]:5.0f} R90={scan[mid + int(90 * per_deg)]:5.0f}")

    rc.drive.set_speed_angle(speed, steer)


rc.set_start_update(start, update)
rc.go()
