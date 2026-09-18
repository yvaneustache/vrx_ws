#!/usr/bin/env python3
# Nouveau composant, sans equivalent dans docking/ — voir CORRESPONDANCE_DOCKING.md
"""ROS-side (RViz2) counterpart to vrx_apf_dock.gz_marker_utils (reused by
import, unchanged -- see CORRESPONDANCE_DOCKING.md): builds
visualization_msgs/Marker ARROW messages instead of Gazebo's gz-transport
Marker_V, so the same vectors are visible in RViz2 too -- works headless,
and is the eventual real-vehicle target per ARCHITECTURE_HARDWARE.md §7.
Published *in addition to* the existing Gazebo GUI markers, not instead of
them (see PROPOSITION_SIMULATEUR_COMPLET.md §3.3) -- callers compute each
vector once and hand it to both gz_marker_utils and this module.

RViz2's native ARROW marker draws a proper 3D shaft+head from a single
Marker, unlike gz-transport's LINE_LIST limitation that forced
gz_marker_utils to compose two primitives per arrow.

Uses the single-pose ARROW form (no `points`): placement (origin_xy, angle)
is carried entirely by marker.pose, scale.x = overall length. A first
attempt used the two-point form (`points` = start/end), first with those
points holding absolute world coordinates and pose left at identity, then
with pose holding the placement and `points` made purely local. Both
rendered fine with Fixed Frame == frame_id ("world") but stayed wrong once
Fixed Frame was set to a moving frame ("wamv", livrable 7's camera-follow
feature) -- confirming RViz2's two-point ARROW form ignores `pose`
entirely and interprets `points` directly in header.frame_id, regardless
of what pose holds. Switching to the single-pose form (no `points` at all)
removes that ambiguity: pose is the only placement RViz2 can use, so the
existing header.frame_id -> Fixed Frame TF lookup (already relied on
elsewhere, e.g. the field markers vs. Fixed Frame "world") is applied
correctly under any Fixed Frame.
"""

from visualization_msgs.msg import Marker

from vrx_apf_dock.geometry_utils import quaternion_from_yaw


def make_arrow_marker(frame_id, ns, marker_id, origin_xy, angle, length,
                       color_rgb, stamp, shaft_diameter=0.3, head_diameter=0.6,
                       head_length=1.0, z_offset=1.0):
    """stamp: node.get_clock().now().to_msg() -- left at the ROS default
    (0, 0) this silently fails to render in RViz2's tf2 message filter
    (message received, never displayed) instead of raising any visible
    error, so it is a required argument rather than a default here.

    head_length is not independently settable in the single-pose ARROW
    form (RViz2 derives head proportions from scale.x/y/z as length/shaft
    diameter/head diameter) -- accepted for call-site compatibility with
    the previous two-point form but otherwise unused."""
    del head_length
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    ox, oy = origin_xy
    marker.pose.position.x = ox
    marker.pose.position.y = oy
    marker.pose.position.z = z_offset
    qx, qy, qz, qw = quaternion_from_yaw(angle)
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw

    # ARROW single-pose form: scale.x/y/z = length, shaft diameter, head diameter.
    marker.scale.x = length
    marker.scale.y = shaft_diameter
    marker.scale.z = head_diameter

    r, g, b = color_rgb
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, 1.0
    return marker
