#!/usr/bin/env python3
"""Plots the APF docking law's commanded vectors over a grid around the dock.

Standalone (no ROS/Gazebo needed, same spirit as the isolated Python tests
mentioned in the project's dev journal) -- samples ApfDockController.compute()
on every integer-metre point of a square centred on the dock, using a FRESH
controller instance per point so each sample reflects the field at that
position alone, not a trajectory-dependent hysteresis state (_switch_value/
_active carried over from wherever a stateful run happened to pass through
first).

Each arrow's direction is theta_cmd and its length is v_cmd (the actual
speed command, already clamped to vmax and damped near the dock) -- not the
raw unclamped potential vector -- so the plot doubles as a picture of where
the speed damping/approach-braking kicks in.

Interactive by default: sliders for c11/c12/c21/c22 (the four main APF
gains) redraw the field live. Pass --static to instead render one PNG and
exit (e.g. headless/no display), still overridable via --c11 etc.

Usage:
  python3 plot_apf_field.py [--size 15] [--step 1] [--dock-heading-deg 0]
                             [--c11 50] [--c12 200] [--c21 1] [--c22 300]
                             [--vmax 0.8] ...  (any DEFAULT_GAINS key)
  python3 plot_apf_field.py --static --out apf_field.png
"""

import argparse
import math
import os
import sys

import matplotlib
import numpy as np
from matplotlib.patches import Circle

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vrx_apf_dock.apf_lib import ApfDockController, DEFAULT_GAINS  # noqa: E402

SLIDER_GAINS = ('c11', 'c12', 'c21', 'c22')
# (min, max) range per slider, generous around the current tuned defaults.
SLIDER_RANGES = {
    'c11': (0.0, 200.0),
    'c12': (0.0, 500.0),
    'c21': (0.0, 100.0),
    'c22': (0.0, 500.0),
}


def build_arg_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--size', type=float, default=15.0,
                         help='Square side length in metres (default: 15)')
    parser.add_argument('--step', type=float, default=1.0,
                         help='Grid spacing in metres (default: 1)')
    parser.add_argument('--dock-heading-deg', type=float, default=0.0,
                         help='Dock heading theta_dock in degrees (default: 0)')
    parser.add_argument('--static', action='store_true',
                         help='Render one PNG to --out and exit instead of showing sliders')
    parser.add_argument('--out', default='apf_field.png',
                         help='Output PNG path, only used with --static (default: apf_field.png)')
    for key, default in DEFAULT_GAINS.items():
        parser.add_argument(f'--{key.replace("_", "-")}', type=float, default=default,
                             help=f'Initial gains["{key}"] (default: {default})')
    return parser


def compute_field(gains, theta_dock, half, step):
    p_dock = np.array([0.0, 0.0])
    coords = np.arange(-half, half + 1e-9, step)

    xs, ys, us, vs, speeds = [], [], [], [], []
    for x in coords:
        for y in coords:
            if math.hypot(x, y) < 1e-6:
                continue  # skip the dock's own position (singular)
            controller = ApfDockController(gains=dict(gains))
            v_cmd, theta_cmd = controller.compute((x, y), p_dock, theta_dock)
            xs.append(x)
            ys.append(y)
            us.append(v_cmd * math.cos(theta_cmd))
            vs.append(v_cmd * math.sin(theta_cmd))
            speeds.append(abs(v_cmd))
    return np.array(xs), np.array(ys), np.array(us), np.array(vs), np.array(speeds)


def setup_axes(ax, gains, theta_dock, half, dock_heading_deg):
    p_dock = np.array([0.0, 0.0])
    reached_circle = Circle(p_dock, gains['reached_radius'], fill=False,
                             linestyle='--', color='red', label='reached_radius')
    ax.add_patch(reached_circle)
    ax.plot(*p_dock, marker='*', markersize=18, color='red', label='dock')
    dock_dir = np.array([math.cos(theta_dock), math.sin(theta_dock)])
    ax.annotate('', xy=p_dock + dock_dir, xytext=p_dock,
                arrowprops=dict(arrowstyle='->', color='red', lw=2))
    ax.set_xlim(-half - 0.5, half + 0.5)
    ax.set_ylim(-half - 0.5, half + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right')


def title_for(gains, dock_heading_deg):
    return (f'APF setpoint field (dock heading={dock_heading_deg:.0f} deg)\n'
            f"c11={gains['c11']:g} c12={gains['c12']:g} c21={gains['c21']:g} "
            f"c22={gains['c22']:g} vmax={gains['vmax']:g}")


def run_static(args, gains, theta_dock, half):
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    xs, ys, us, vs, speeds = compute_field(gains, theta_dock, half, args.step)
    fig, ax = plt.subplots(figsize=(8, 8))
    quiv = ax.quiver(xs, ys, us, vs, speeds, cmap='viridis',
                      angles='xy', scale_units='xy', scale=1.0 / args.step, width=0.004)
    fig.colorbar(quiv, ax=ax, label='|v_cmd| (m/s)')
    setup_axes(ax, gains, theta_dock, half, args.dock_heading_deg)
    ax.set_title(title_for(gains, args.dock_heading_deg))
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f'Wrote {args.out}')


def run_interactive(args, gains, theta_dock, half):
    from matplotlib.widgets import Slider
    import matplotlib.pyplot as plt

    xs, ys, us, vs, speeds = compute_field(gains, theta_dock, half, args.step)

    fig, ax = plt.subplots(figsize=(8, 9))
    fig.subplots_adjust(bottom=0.3)
    quiv = ax.quiver(xs, ys, us, vs, speeds, cmap='viridis',
                      angles='xy', scale_units='xy', scale=1.0 / args.step, width=0.004,
                      clim=(0.0, gains['vmax']))
    fig.colorbar(quiv, ax=ax, label='|v_cmd| (m/s)')
    setup_axes(ax, gains, theta_dock, half, args.dock_heading_deg)
    ax.set_title(title_for(gains, args.dock_heading_deg))

    sliders = {}
    for i, key in enumerate(SLIDER_GAINS):
        slider_ax = fig.add_axes([0.2, 0.2 - i * 0.05, 0.6, 0.03])
        lo, hi = SLIDER_RANGES[key]
        sliders[key] = Slider(slider_ax, key, lo, hi, valinit=gains[key])

    def on_change(_val):
        live_gains = dict(gains)
        for key, slider in sliders.items():
            live_gains[key] = slider.val
        _xs, _ys, us, vs, speeds = compute_field(live_gains, theta_dock, half, args.step)
        quiv.set_UVC(us, vs, speeds)
        ax.set_title(title_for(live_gains, args.dock_heading_deg))
        fig.canvas.draw_idle()

    for slider in sliders.values():
        slider.on_changed(on_change)

    plt.show()


def main():
    args = build_arg_parser().parse_args()
    gains = {key: getattr(args, key) for key in DEFAULT_GAINS}
    theta_dock = math.radians(args.dock_heading_deg)
    half = args.size / 2.0

    if args.static:
        run_static(args, gains, theta_dock, half)
    else:
        run_interactive(args, gains, theta_dock, half)


if __name__ == '__main__':
    main()
