"""ROS 2 node: associate person detections over time (track IDs and walking speed)."""
import rclpy
from rclpy.node import Node
from mesa_vision_msgs.msg import DetectedObjectArray

from mesa_vision.core import PERSON_LABELS, Tracker


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__("person_tracker_node")
        for name, default in (("gate_m", 0.8), ("max_misses", 10), ("min_hits", 2)):
            self.declare_parameter(name, default)
        g = self.get_parameter
        self.tracker = Tracker(g("gate_m").value, int(g("max_misses").value), int(g("min_hits").value))
        self.create_subscription(DetectedObjectArray, "/mesa/vision/detections", self.on_dets, 10)
        self.pub = self.create_publisher(DetectedObjectArray, "/mesa/vision/people", 10)

    def on_dets(self, msg):
        people = [o for o in msg.objects if o.label in PERSON_LABELS]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tracks = self.tracker.update([(o.position.x, o.position.y, o.position.z) for o in people], t)
        out = DetectedObjectArray()
        out.header = msg.header
        for o, tr in zip(people, tracks):
            if tr is None:
                continue  # not yet confirmed
            o.track_id, o.speed_mps = tr.id, float(tr.speed_mps)
            out.objects.append(o)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
