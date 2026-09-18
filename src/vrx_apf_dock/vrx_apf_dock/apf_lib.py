"""Artificial potential field docking law, ported from controller.py.

Equations match TabAPF_robmooc.png:
  - attractive conical / attractive plane-line -> pulls the robot towards the
    dock point or onto the dock heading axis
  - uniform -> constant push along (or against) the dock heading
The two branches below correspond to the two cases from the assignment:
  front-of-dock (attraction) vs behind-the-dock (repulsion + push back in).

Tuning refined against classBoat.py (a more complete reference implementation
of the same law): near-target speed damping, an explicit speed cap, a
transition distance scaled to the boat/dock size, and re-arming the
attraction phase after the robot has drifted back away from the dock.
"""

import numpy as np

L = 1

# Defaults only -- live values live on each ApfDockController instance as
# .gains (see __init__), tunable at runtime as ROS parameters on
# apf_controller_node with `ros2 param set` or rqt_reconfigure.
DEFAULT_GAINS = {
    'c11': 80.0,
    'c12': 150.0,
    'c21': 2.0,   # 50 originally
    'c22': 100.0,
    'vmax': 1,
    'margin': 20,
    'transition_dist': 20.0 * L,
    'reached_radius': 0.8,  # 5.2 for the initial wam-v, 1 for the mini wam-v
    # Dedicated hull-avoidance term for back_branch only: c21*diff/r**3 alone
    # only becomes significant for r < ~1 (given c21 vs c22's magnitude),
    # which let the robot pass within ~1 m of the dock's own hull -- too
    # close for two real WAM-V-sized hulls. This adds a proper Khatib-style
    # repulsive potential with a finite radius of influence (safety_radius),
    # zero at/beyond that radius and growing smoothly as r shrinks, so the
    # boat visibly curves away well before getting hull-close. Not applied
    # outside back_branch: getting close is exactly the point of the other
    # branch (docking approach).
    'safety_radius': 50.0,
    'safety_gain': 400000.0,
}

_EPS = 1e-6


class ApfDockController:
    """Stateful port of controller.py's controller(), refined against
    classBoat.py.

    Internal state:
      _switch_value: hysteresis threshold on the projection of (robot - dock)
        onto the dock heading. Starts at 0, jumps to gains['transition_dist']
        the first time the "front" (attractive) branch engages, and then
        stays there permanently (classBoat.py never resets it) -- once the
        robot has approached the dock at least once, the wide
        transition_dist band makes the front/attractive branch the default
        regime again after any brief excursion into the back/repulsive one.
      _active: latch controlling whether the front branch can engage at all.
        Cleared once the robot comes within 0.1 m of the dock point (docked
        already, hold/center instead of re-attracting), but re-armed if the
        robot later drifts far enough behind the dock (past
        -_switch_value / 3) -- e.g. pushed out by waves/drift after docking.
      _reached: final latch, set once the robot comes within
        gains['reached_radius'] of the dock point WHILE in the
        front/attractive branch. Gating on
        the branch (not just distance) matters: approaching from the back
        (behind-the-dock side) can pass within REACHED_RADIUS of the dock
        point without ever having crossed to the correct entry side, which
        would otherwise latch "reached" with the robot still facing the
        wrong way outside the dock instead of having actually entered it.
        Once set, compute() always returns (0, 0) -- the caller is expected
        to hold the robot's current heading instead of this literal 0 (see
        ApfControllerNode), since a fixed absolute heading of 0 rad would
        otherwise make it swing to face east.
    """

    def __init__(self, gains=None):
        self.gains = dict(DEFAULT_GAINS if gains is None else gains)
        self._switch_value = 0.0
        self._active = True
        self._reached = False

    def reset(self):
        self._switch_value = 0.0
        self._active = True
        self._reached = False

    @property
    def reached(self):
        return self._reached

    def compute(self, p_robot, p_dock, theta_dock):
        """p_robot, p_dock: (x, y) arrays. theta_dock: dock heading (rad).

        Returns (v_cmd, theta_cmd). Once docked (see `reached`), always
        returns (0, 0) -- motors stop, no more corrections are commanded.
        """
        p_robot = np.asarray(p_robot, dtype=float)
        p_dock = np.asarray(p_dock, dtype=float)
        gains = self.gains

        diff = p_robot - p_dock
        r = max(np.linalg.norm(diff), _EPS)

        if self._reached:
            return 0.0, 0.0

        unit = np.array([np.cos(theta_dock), np.sin(theta_dock)])
        n = np.array([np.cos(theta_dock + np.pi / 2), np.sin(theta_dock + np.pi / 2)])
        p_dock_approach = p_dock + gains['margin'] * unit

        k = 1.0
        back_branch = unit @ diff < self._switch_value and self._active
        if back_branch:
            v_vec = gains['c21'] * diff / r ** 3 + gains['c22'] * unit
            safety_radius = gains['safety_radius']
            if r < safety_radius:
                v_vec = v_vec + gains['safety_gain'] * (
                    1.0 / r - 1.0 / safety_radius) / r ** 2 * (diff / r)
            if self._switch_value == 0.0:
                self._switch_value = gains['transition_dist']
        else:
            self._active = False
            k = -np.sign(unit @ (p_dock - p_robot))
            v_vec = -gains['c11'] * n * (n @ diff) + gains['c12'] * (-unit)
            if unit @ diff < -self._switch_value / 3.0:
              self._active = True

        theta_cmd = np.arctan2(v_vec[1], v_vec[0])
        # /2: strong extra damping close to the approach waypoint, avoids
        # the large overshoot seen with the un-damped clamp.
        v_cmd = min(np.linalg.norm(v_vec), k * np.linalg.norm(p_dock_approach - p_robot) / 2.0)
        vmax = gains['vmax']
        v_cmd = max(min(vmax, v_cmd), -vmax)

        if r < gains['reached_radius']:
            self._reached = True

        return v_cmd, theta_cmd
