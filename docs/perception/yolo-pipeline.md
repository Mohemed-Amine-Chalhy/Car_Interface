# YOLO11n perception pipeline

The vision pipeline connects camera capture, YOLO11n inference, annotated
detections, and approximate distance estimates to the operator interface.

## Data flow

```text
camera index 0
  -> OpenCV capture (DirectShow first on Windows)
  -> bounded 10-frame queue
  -> Ultralytics YOLO11n inference
  -> boxes + class IDs + confidence values
  -> approximate monocular distance estimate
  -> annotated 640 x 360 Tkinter view
```

## Camera capture

The camera layer opens index 0 with `cv2.CAP_DSHOW` on Windows and falls back to
OpenCV's default backend. A worker thread reads frames into a queue capped at ten
items. When the queue fills, the oldest frame is discarded so the operator sees
recent data instead of a growing delay.

## Inference and overlay

The view loads `yolo11n.pt` through Ultralytics and runs inference on each frame.
For every detection it extracts:

- `xyxy` bounding-box coordinates;
- the class ID and label;
- the confidence score; and
- bounding-box width in pixels.

The interface draws a green bounding box with the class, confidence, and
approximate distance. A red `WARNING: Object Close!` message appears when the
distance estimate falls below 150 cm.

## Distance estimate

The pipeline estimates range from an assumed object width:

```text
distance_cm = assumed_object_width_cm x focal_length_px / box_width_px
```

| Class | Assumed width |
| --- | ---: |
| Person | 50 cm |
| Car | 180 cm |
| Bus | 250 cm |
| Bottle | 6.5 cm |
| Any other class | 100 cm |

The focal-length constant is 720 pixels. The resulting value is an operator aid;
a specific camera and lens should be calibrated before using it as a
quantitative distance measurement.

## Software components

- OpenCV handles camera capture, color conversion, and overlays.
- Ultralytics runs YOLO inference.
- NumPy supports result handling.
- Pillow transfers annotated frames into Tkinter.
- Tkinter presents the camera view alongside vehicle information.

The bounded queue and worker split keep camera capture responsive when inference
takes longer than the camera frame interval. The
[perception validation plan](validation-plan.md) covers camera calibration,
latency measurement, model evaluation, and synchronized vehicle testing.
