"""ROS 2 node: Luxonis OAK-D Lite -> RGB image, depth image and spatial detections.

Written against the DepthAI v2 Python API (depthai>=2.24,<3). NOT yet tested on hardware
(camera not yet connected at Deliverable II); see docs/test_plan.md, tests T1-T4.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from mesa_vision_msgs.msg import DetectedObject, DetectedObjectArray

from mesa_vision.core import bearing_from_xz, distance_from_xyz

# PASCAL VOC labels used by MobileNet-SSD (index = class id).
VOC_LABELS = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
              "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant",
              "sheep", "sofa", "train", "tvmonitor"]


def build_pipeline(blob_path: str, confidence: float, fps: int, publish_rgb: bool, publish_depth: bool):
    import depthai as dai

    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    stereo = p.create(dai.node.StereoDepth)
    nn = p.create(dai.node.MobileNetSpatialDetectionNetwork)

    cam.setPreviewSize(300, 300)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(fps)
    for m, sock in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        m.setBoardSocket(sock)
        m.setFps(fps)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(640, 400)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    nn.setBlobPath(blob_path)
    nn.setConfidenceThreshold(confidence)
    nn.input.setBlocking(False)
    nn.setBoundingBoxScaleFactor(0.5)
    nn.setDepthLowerThreshold(200)    # mm
    nn.setDepthUpperThreshold(6000)   # mm
    cam.preview.link(nn.input)
    stereo.depth.link(nn.inputDepth)

    x_det = p.create(dai.node.XLinkOut)
    x_det.setStreamName("detections")
    nn.out.link(x_det.input)
    if publish_rgb:
        x_rgb = p.create(dai.node.XLinkOut)
        x_rgb.setStreamName("rgb")
        cam.video.link(x_rgb.input)
    if publish_depth:
        x_d = p.create(dai.node.XLinkOut)
        x_d.setStreamName("depth")
        stereo.depth.link(x_d.input)
    return p


class OakCameraNode(Node):
    def __init__(self):
        super().__init__("oak_camera_node")
        self.declare_parameter("blob_path", "")
        self.declare_parameter("confidence", 0.5)
        self.declare_parameter("fps", 15)
        self.declare_parameter("frame_id", "oak_rgb_camera_optical_frame")
        self.declare_parameter("publish_rgb", False)    # 1080p frames are ~47 MB/s: enable for debugging only
        self.declare_parameter("publish_depth", False)
        self.declare_parameter("labels", VOC_LABELS)
        g = self.get_parameter
        if not g("blob_path").value:
            raise RuntimeError("Parameter blob_path is required (MobileNet-SSD .blob compiled for RVC2).")
        self.labels = list(g("labels").value)
        self.frame_id = g("frame_id").value
        self.pub_det = self.create_publisher(DetectedObjectArray, "/mesa/vision/detections", 10)
        self.pub_rgb = self.create_publisher(Image, "/mesa/vision/rgb", qos_profile_sensor_data) if g("publish_rgb").value else None
        self.pub_depth = self.create_publisher(Image, "/mesa/vision/depth", qos_profile_sensor_data) if g("publish_depth").value else None
        import depthai as dai
        from cv_bridge import CvBridge
        self.bridge = CvBridge()
        pipeline = build_pipeline(g("blob_path").value, g("confidence").value, g("fps").value,
                                  self.pub_rgb is not None, self.pub_depth is not None)
        self.device = dai.Device(pipeline)
        self.q_det = self.device.getOutputQueue("detections", maxSize=4, blocking=False)
        self.q_rgb = self.device.getOutputQueue("rgb", 2, False) if self.pub_rgb else None
        self.q_depth = self.device.getOutputQueue("depth", 2, False) if self.pub_depth else None
        self.timer = self.create_timer(1.0 / (2.0 * g("fps").value), self.poll)
        self.get_logger().info("OAK-D Lite pipeline started")

    def poll(self):
        stamp = self.get_clock().now().to_msg()
        msg_in = self.q_det.tryGet()
        if msg_in is not None:
            out = DetectedObjectArray()
            out.header.stamp, out.header.frame_id = stamp, self.frame_id
            for d in msg_in.detections:
                o = DetectedObject()
                o.header = out.header
                o.label = self.labels[d.label] if 0 <= d.label < len(self.labels) else str(d.label)
                o.confidence = float(d.confidence)
                x, y, z = (d.spatialCoordinates.x / 1000.0, d.spatialCoordinates.y / 1000.0,
                           d.spatialCoordinates.z / 1000.0)
                if z <= 0.0 or math.isnan(z):
                    continue  # no valid depth for this box
                o.position = Point(x=x, y=y, z=z)
                o.distance_m = distance_from_xyz(x, y, z)
                o.bearing_rad = bearing_from_xz(x, z)
                o.xmin, o.ymin, o.xmax, o.ymax = d.xmin, d.ymin, d.xmax, d.ymax
                out.objects.append(o)
            self.pub_det.publish(out)
        if self.q_rgb is not None:
            f = self.q_rgb.tryGet()
            if f is not None:
                m = self.bridge.cv2_to_imgmsg(f.getCvFrame(), "bgr8")
                m.header.stamp, m.header.frame_id = stamp, self.frame_id
                self.pub_rgb.publish(m)
        if self.q_depth is not None:
            f = self.q_depth.tryGet()
            if f is not None:
                m = self.bridge.cv2_to_imgmsg(f.getFrame(), "16UC1")  # millimetres
                m.header.stamp, m.header.frame_id = stamp, self.frame_id
                self.pub_depth.publish(m)

    def destroy_node(self):
        try:
            self.device.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = OakCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
