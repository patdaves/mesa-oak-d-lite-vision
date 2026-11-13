"""ROS 2 node: confirm camera detections with LiDAR, publish alerts and speech requests."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from mesa_vision_msgs.msg import DetectedObjectArray, ObstacleAlert, SpeechRequest

from mesa_vision.core import AskManager, decide, lidar_range_at, request_text


class ObstacleConfirmNode(Node):
    def __init__(self):
        super().__init__("obstacle_confirm_node")
        for name, default in (("tolerance_m", 0.5), ("corridor_half_angle_deg", 20.0),
                              ("blocking_distance_m", 1.5), ("lidar_window_deg", 5.0),
                              ("ask_timeout_s", 20.0), ("max_asks", 2), ("allow_depth_only", False),
                              ("language", "en")):
            self.declare_parameter(name, default)
        self.scan = None
        self.asker = AskManager(self.p("ask_timeout_s"), int(self.p("max_asks")))
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(DetectedObjectArray, "/mesa/vision/detections", self.on_dets, 10)
        self.pub_alert = self.create_publisher(ObstacleAlert, "/mesa/vision/obstacle_alerts", 10)
        self.pub_speech = self.create_publisher(SpeechRequest, "/mesa/speech/request", 10)
        # Confirmed obstacles as a point cloud (camera frame) for a Nav2 obstacle-layer observation source.
        self.pub_cloud = self.create_publisher(PointCloud2, "/mesa/vision/obstacle_points", 10)

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, msg):
        self.scan = msg

    def on_dets(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        confirmed_pts = []
        for o in msg.objects:
            lidar = None
            if self.scan is not None:
                lidar = lidar_range_at(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                                       o.bearing_rad, math.radians(self.p("lidar_window_deg")),
                                       self.scan.range_min, self.scan.range_max)
            d = decide(o.label, o.distance_m, o.bearing_rad, lidar,
                       tolerance_m=self.p("tolerance_m"),
                       corridor_half_angle=math.radians(self.p("corridor_half_angle_deg")),
                       blocking_distance_m=self.p("blocking_distance_m"),
                       allow_depth_only=self.p("allow_depth_only"))
            a = ObstacleAlert()
            a.header = msg.header
            a.label, a.distance_m, a.bearing_rad = o.label, o.distance_m, o.bearing_rad
            a.position = o.position
            a.confirmed, a.confirmation_source = d.confirmed, d.source
            a.is_person, a.movable, a.recommended_action = d.is_person, d.movable, d.action
            self.pub_alert.publish(a)
            if d.confirmed and not d.is_person:
                confirmed_pts.append((o.position.x, o.position.y, o.position.z))
            if d.action == "ask_to_move" and self.asker.should_ask(o.label, now):
                s = SpeechRequest()
                s.header = msg.header
                s.text = request_text(o.label, self.p("language"))
                s.priority, s.reason, s.language = 1, f"{o.label} blocks path", self.p("language")
                self.pub_speech.publish(s)
            elif not d.blocking:
                self.asker.clear(o.label)
        if confirmed_pts:
            self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(msg.header, confirmed_pts))


def main():
    rclpy.init()
    node = ObstacleConfirmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
