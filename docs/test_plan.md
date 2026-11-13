# Initial test plan (Deliverable II)

| ID | Test | Method | Pass criterion |
|---|---|---|---|
| T1 | Camera enumeration | `python3 -c "import depthai as d; print(d.Device.getAllAvailableDevices())"` | OAK-D Lite listed over USB 3 |
| T2 | RGB/depth streams | `benchmark_oak.py --save-frames` | both images saved; depth valid-pixel fraction recorded |
| T3 | Detection performance | 60 s runs with 1 person and 1 chair at 1, 2, 3 m | FPS >= 10; median latency recorded; detections of person and chair |
| T4 | Distance accuracy | tape measure at 0.5-4 m, 5 repeats | error per distance recorded (target <= 5 % at 2 m) |
| T5 | Confirmation | chair in LiDAR corridor, camera + /scan | ObstacleAlert.confirmed = true; false for a poster on the wall |
| T6 | Speech request | publish chair alert | one SpeechRequest per 20 s, max 2 |
| T7 | Power | USB power meter on camera; host power logger | idle/active W recorded |
| T8 | Build | `colcon build` on the target host | both packages build |
