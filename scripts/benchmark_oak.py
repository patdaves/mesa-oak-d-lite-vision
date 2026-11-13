#!/usr/bin/env python3
"""Camera bring-up and benchmark (no ROS needed): RGB + depth + MobileNet-SSD spatial detections.

Usage: python3 scripts/benchmark_oak.py --blob models/mobilenet-ssd_openvino_2021.4_6shave.blob --seconds 60
Writes results/benchmark_<timestamp>.csv (one row per detection message) and prints a summary
(FPS, latency, depth fill ratio). Needs: pip install "depthai>=2.24,<3" numpy opencv-python
"""
import argparse, csv, os, statistics, time
import numpy as np
import depthai as dai

ap = argparse.ArgumentParser()
ap.add_argument("--blob", required=True)
ap.add_argument("--seconds", type=float, default=30)
ap.add_argument("--fps", type=int, default=15)
ap.add_argument("--save-frames", action="store_true")
a = ap.parse_args()

p = dai.Pipeline()
cam, l, r = p.create(dai.node.ColorCamera), p.create(dai.node.MonoCamera), p.create(dai.node.MonoCamera)
st, nn = p.create(dai.node.StereoDepth), p.create(dai.node.MobileNetSpatialDetectionNetwork)
cam.setPreviewSize(300, 300); cam.setInterleaved(False); cam.setFps(a.fps)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
for m, s in ((l, dai.CameraBoardSocket.CAM_B), (r, dai.CameraBoardSocket.CAM_C)):
    m.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P); m.setBoardSocket(s); m.setFps(a.fps)
st.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
st.setDepthAlign(dai.CameraBoardSocket.CAM_A); st.setOutputSize(640, 400)
l.out.link(st.left); r.out.link(st.right)
nn.setBlobPath(a.blob); nn.setConfidenceThreshold(0.5); nn.input.setBlocking(False)
nn.setBoundingBoxScaleFactor(0.5); nn.setDepthLowerThreshold(200); nn.setDepthUpperThreshold(6000)
cam.preview.link(nn.input); st.depth.link(nn.inputDepth)
for name, src in (("det", nn.out), ("depth", st.depth), ("rgb", cam.video)):
    x = p.create(dai.node.XLinkOut); x.setStreamName(name); src.link(x.input)

os.makedirs("results", exist_ok=True)
fn = time.strftime("results/benchmark_%Y%m%d_%H%M%S.csv")
lat, n_msg, fill = [], 0, []
with dai.Device(p) as dev, open(fn, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t_s", "label_id", "confidence", "x_mm", "y_mm", "z_mm", "latency_ms"])
    q = {k: dev.getOutputQueue(k, 4, False) for k in ("det", "depth", "rgb")}
    t0 = time.time(); saved = False
    while time.time() - t0 < a.seconds:
        d = q["det"].tryGet()
        if d is not None:
            n_msg += 1
            L = (dai.Clock.now() - d.getTimestamp()).total_seconds() * 1000.0
            lat.append(L)
            for x in d.detections:
                s = x.spatialCoordinates
                w.writerow([round(time.time() - t0, 3), x.label, round(x.confidence, 3), s.x, s.y, s.z, round(L, 1)])
        dp = q["depth"].tryGet()
        if dp is not None:
            fr = dp.getFrame(); fill.append(float(np.count_nonzero(fr)) / fr.size)
            if a.save_frames and not saved:
                import cv2
                cv2.imwrite("results/depth_sample.png", fr)
                rg = q["rgb"].tryGet()
                if rg is not None: cv2.imwrite("results/rgb_sample.png", rg.getCvFrame()); saved = True
dur = a.seconds
print(f"detection msgs: {n_msg} ({n_msg/dur:.1f}/s)")
if lat: print(f"latency ms: median {statistics.median(lat):.1f}, p95 {sorted(lat)[int(0.95*len(lat))-1]:.1f}")
if fill: print(f"depth valid-pixel fraction: mean {statistics.mean(fill):.2f}")
print("csv:", fn)
