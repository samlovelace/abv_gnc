#!/usr/bin/env python3
"""Debug tool: republishes each ObjectObservation's PointCloud2 onto its own
topic so they show up individually in rviz2 (a single ObjectObservations
message isn't something rviz can subscribe to directly - the clouds are
nested inside it).

Usage:
    ros2 run autobot_obj_segmentation debug_republish_clouds.py
    # or directly:
    python3 debug_republish_clouds.py

Topics published: /percep/object_observations/debug/object_<index>_<label>
Add each as a PointCloud2 display in rviz2, Fixed Frame = "map".

Nothing in the sim or detection/segmentation nodes publishes any /tf at
all, so the cloud's own frame_id (e.g. "overhead_camera") is unknown to
tf2 and rviz2 refuses to render it even if you set Fixed Frame to that
same name. This node papers over that for debugging by broadcasting one
static identity transform map -> <frame_id> the first time it sees each
distinct frame_id, just so the frame exists - it is NOT the camera's real
extrinsics, so don't trust absolute position/orientation against other
"map"-frame data, only use it to sanity-check the cloud's own shape/size.
"""
import re

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from autobot_msgs.msg import ObjectObservations

INPUT_TOPIC = "/percep/object_observations"
OUTPUT_PREFIX = "/percep/object_observations/debug"
MAP_FRAME = "map"


def sanitize(label: str) -> str:
    # topic names can't contain spaces/punctuation - keep it readable but safe
    return re.sub(r"[^a-zA-Z0-9_]", "_", label) or "object"


class DebugRepublisher(Node):
    def __init__(self):
        super().__init__("object_observations_debug_republisher")
        self._cloud_publishers = {}  # index -> (topic_name, Publisher)
        self._known_frames = set()
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(ObjectObservations, INPUT_TOPIC, self._on_observations, 10)
        self.get_logger().info(f"Subscribed to {INPUT_TOPIC}, republishing clouds under {OUTPUT_PREFIX}/*")

    def _ensure_frame_known(self, frame_id: str):
        if not frame_id or frame_id in self._known_frames:
            return
        self._known_frames.add(frame_id)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = frame_id
        t.transform.rotation.w = 1.0  # identity - see module docstring
        self._tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Broadcast identity static transform {MAP_FRAME} -> {frame_id}")

    def _on_observations(self, msg: ObjectObservations):
        for i, obs in enumerate(msg.observations):
            topic = f"{OUTPUT_PREFIX}/object_{i}_{sanitize(obs.label)}"

            entry = self._cloud_publishers.get(i)
            if entry is None or entry[0] != topic:
                # label (and thus topic name) for this index can change
                # frame-to-frame as detections come and go - swap the
                # publisher if so, rather than accumulating stale ones.
                publisher = self.create_publisher(PointCloud2, topic, 10)
                self._cloud_publishers[i] = (topic, publisher)
                self.get_logger().info(f"Publishing object {i} ('{obs.label}') on {topic}")

            self._ensure_frame_known(obs.points.header.frame_id)
            self._cloud_publishers[i][1].publish(obs.points)


def main():
    rclpy.init()
    node = DebugRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
