# mesa_vision_ws: MESA vision subsystem (Deliverable II initial package)

ROS 2 workspace for the Luxonis OAK-D Lite camera on the MESA social wellness robot (RIT Dubai capstone).
It detects people and obstacles, measures their distance, confirms obstacles against LiDAR, and asks the
speech subsystem to request help (for example, "could you please move the chair?").

**Status (Deliverable II):** interface definitions, decision logic, person tracker and 17 unit tests are complete. The camera and ROS 2
nodes are written but **not yet run on hardware**; the benchmark script and test plan (docs/test_plan.md) are ready for that.
Face recognition and identity storage are intentionally not included (ethics approval pending).

## Layout
- `src/mesa_vision_msgs`: DetectedObject(Array), ObstacleAlert, SpeechRequest
- `src/mesa_vision`: `core.py` (pure logic: decision rules, request manager, person tracker), `oak_camera_node`, `person_tracker_node`, `obstacle_confirm_node`, launch (with a static camera transform) and config
- `scripts/benchmark_oak.py`: camera bring-up and FPS/latency/depth-fill benchmark (no ROS needed)
- `docs/interfaces.md`, `docs/test_plan.md`

## Quick start
```bash
pip install -r requirements.txt
# 1. unit tests (no ROS, no camera)
cd src/mesa_vision && python -m pytest -q test && cd ../..
# 2. camera bring-up (camera on a USB 3 port; compile a MobileNet-SSD blob with blobconverter first)
python3 scripts/benchmark_oak.py --blob models/mobilenet-ssd.blob --seconds 60 --save-frames
# 3. ROS 2 (Humble or newer)
colcon build --symlink-install && source install/setup.bash
ros2 launch mesa_vision vision.launch.py blob_path:=$PWD/models/mobilenet-ssd.blob
```
Powering: the OAK-D Lite draws up to 1.5 A; use a USB 3 port or the Luxonis Y-adapter.

## Known limitations
DepthAI v2 API is pinned (v3 is the default in newer ROS 2 drivers; migration is a tracked risk). The "path blocked" test is a
fixed corridor (+/-20 deg, 1.5 m) until integrated with the Nav2 costmap. LiDAR/camera extrinsics are not yet calibrated.
