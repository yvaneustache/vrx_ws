"""Shared helpers for drawing arrows directly in the Gazebo GUI via gz-transport.

Uses gz-transport directly (not ROS) to call the /marker_array Gazebo
service, since these are a Gazebo-native visual overlay with no ROS
equivalent/bridge.
Each arrow is a CYLINDER (shaft) + CONE (head): LINE_LIST/LINE_STRIP markers
render as hairline (~1px) segments in gz-sim's Ogre2 backend, effectively
invisible on a world spanning hundreds of meters, whereas solid primitives
respect `scale` and stay clearly visible regardless of camera distance.

/marker_array is a Gazebo *service* (request gz.msgs.Marker_V, response
gz.msgs.Boolean -- NOT gz.msgs.Empty, despite /marker's own response being
Empty; mismatching this silently fails the request), not a plain topic --
publishing to it goes nowhere since MarkerManager never subscribes as such.

MarkerManager is a GUI-only plugin (gz-gui-8/plugins/libMarkerManager.so),
never active in headless:=True -- callers of publish_markers() will just
silently get no visual effect there, not an error.
"""

import math

from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.boolean_pb2 import Boolean

MARKER_SERVICE_TIMEOUT_MS = 50

SHAFT_RADIUS = 0.15
HEAD_LENGTH = 2.0
HEAD_RADIUS = 0.5
Z_OFFSET = 1.0  # draw above the water line so it's not hidden by the hull


def direction_quaternion(angle):
    """Quaternion rotating the primitive's local +Z axis onto the world
    direction (cos(angle), sin(angle), 0). Z and that direction are always
    90 degrees apart, so this is a fixed pi/2 rotation about the axis
    (-sin(angle), cos(angle), 0) = Z x direction."""
    half = math.pi / 4.0  # half of the pi/2 rotation angle
    s, c = math.sin(half), math.cos(half)
    return (-math.sin(angle) * s, math.cos(angle) * s, 0.0, c)


def set_color(marker, color_rgb):
    r, g, b = color_rgb
    for channel in (marker.material.ambient, marker.material.diffuse):
        channel.r, channel.g, channel.b, channel.a = r, g, b, 1.0


def make_segment_marker(ns, marker_id, marker_type, origin_xy, angle,
                         center_dist, length, radius, color_rgb, z_offset=Z_OFFSET):
    marker = Marker()
    marker.ns = ns
    marker.id = marker_id
    marker.action = Marker.ADD_MODIFY
    marker.type = marker_type
    set_color(marker, color_rgb)

    ox, oy = origin_xy
    cx = ox + center_dist * math.cos(angle)
    cy = oy + center_dist * math.sin(angle)
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = cx, cy, z_offset
    qx, qy, qz, qw = direction_quaternion(angle)
    marker.pose.orientation.x = qx
    marker.pose.orientation.y = qy
    marker.pose.orientation.z = qz
    marker.pose.orientation.w = qw

    marker.scale.x, marker.scale.y, marker.scale.z = radius * 2.0, radius * 2.0, length
    return marker


def make_arrow_markers(ns, base_id, origin_xy, angle, shaft_length, color_rgb,
                        shaft_radius=SHAFT_RADIUS, head_length=HEAD_LENGTH,
                        head_radius=HEAD_RADIUS, z_offset=Z_OFFSET):
    """base_id and base_id+1 are used for the shaft/head -- callers batching
    several arrows must space base_ids at least 2 apart."""
    shaft = make_segment_marker(
        ns, base_id, Marker.CYLINDER, origin_xy, angle,
        shaft_length / 2.0, shaft_length, shaft_radius, color_rgb, z_offset)
    head = make_segment_marker(
        ns, base_id + 1, Marker.CONE, origin_xy, angle,
        shaft_length + head_length / 2.0, head_length, head_radius, color_rgb, z_offset)
    return [shaft, head]


def publish_markers(gz_node, markers, timeout_ms=MARKER_SERVICE_TIMEOUT_MS):
    """Batches all markers into one /marker_array request (instead of one
    blocking /marker call per primitive) to avoid compounding per-call
    latency. Returns the service call's success bool."""
    batch = Marker_V()
    for marker in markers:
        batch.marker.append(marker)
    return gz_node.request('/marker_array', batch, Marker_V, Boolean, timeout_ms)
