# MESA vision interfaces (v0.1)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| /mesa/vision/detections | mesa_vision_msgs/DetectedObjectArray | oak_camera_node -> all | label, confidence, x/y/z (m, camera optical frame), distance, bearing (+left) |
| /mesa/vision/rgb | sensor_msgs/Image (bgr8) | oak_camera_node -> | 1080p video frames |
| /mesa/vision/depth | sensor_msgs/Image (16UC1, mm) | oak_camera_node -> | aligned to RGB |
| /scan | sensor_msgs/LaserScan | LiDAR driver -> obstacle_confirm_node | 360 deg |
| /mesa/vision/people | mesa_vision_msgs/DetectedObjectArray | person_tracker_node -> navigation, speech | persons only, with track_id and speed_mps |
| /mesa/vision/obstacle_points | sensor_msgs/PointCloud2 | obstacle_confirm_node -> Nav2 | confirmed non-person obstacles, camera frame; use as an obstacle-layer observation source |
| /mesa/vision/obstacle_alerts | mesa_vision_msgs/ObstacleAlert | obstacle_confirm_node -> navigation | confirmed flag, recommended_action |
| /mesa/speech/request | mesa_vision_msgs/SpeechRequest | obstacle_confirm_node -> speech | text, priority, reason |

Reserved for later (not implemented, ethics approval pending): identity and face-enrolment topics.
