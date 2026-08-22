# Historical YOLO11n perception pipeline

This page reconstructs the computer-vision implementation that existed in the
original project tree. It is an engineering record, not a statement that the
same dependencies or model weights ship with maintained v0.1.

## Data flow

```text
camera index 0
  -> OpenCV capture (DirectShow first on Windows)
  -> bounded 10-frame queue (oldest frame dropped when full)
  -> Ultralytics YOLO11n inference
  -> boxes + class IDs + confidence values
  -> approximate monocular distance heuristic
  -> annotated 640 × 360 Tkinter view
```

## Camera capture

The historical `CameraHandler`:

- opened camera index 0 with `cv2.CAP_DSHOW`, then retried with OpenCV's default
  backend;
- read frames on a daemon thread;
- used a queue with a maximum of 10 frames;
- dropped the oldest queued frame when the queue filled; and
- exposed frames to the UI with a bounded read timeout.

This is a sensible freshness-oriented prototype design: overload discarded old
frames instead of allowing unbounded latency. It did not record the selected
camera mode, actual capture FPS, queue occupancy, or end-to-end latency.

## YOLO inference and overlay

The view loaded `yolo11n.pt` using Ultralytics and ran inference on each frame.
For every result it extracted:

- `xyxy` bounding-box coordinates;
- class ID and the model's class label;
- confidence score; and
- bounding-box width in pixels.

It drew a green box with class/confidence text and an approximate distance. A
red `WARNING: Object Close!` overlay appeared when the heuristic distance was
below 150 cm.

The historical code establishes YOLO11n integration, not the origin or training
of the weights. The removed binary's checksum, model card, dataset, license,
training configuration, and evaluation metrics were not preserved in the
maintained tree.

## Distance heuristic

The prototype estimated range from a class-width assumption:

```text
distance_cm = assumed_object_width_cm × focal_length_px / box_width_px
```

Historical constants:

| Class | Assumed width |
| --- | ---: |
| Person | 50 cm |
| Car | 180 cm |
| Bus | 250 cm |
| Bottle | 6.5 cm |
| Any other class | 100 cm fallback |

The focal-length constant was 720 pixels. It was marked as requiring
adjustment, and no camera calibration or error report accompanied it. Bounding
box width also changes with orientation and partial occlusion, so these values
should be described as a visual heuristic rather than metrology.

## Historical software stack

The removed pipeline imported:

- OpenCV for capture, color conversion, and overlays;
- Ultralytics for YOLO inference;
- NumPy for result handling;
- Pillow for Tkinter image display; and
- Tkinter for the desktop view.

None of those vision-specific packages is required by maintained v0.1. A future
vision dependency group must pin versions independently and include the model
artifact in its provenance and SBOM review.

## Professional reconstruction targets

| Concern | Historical prototype | Maintained target |
| --- | --- | --- |
| Model loading | Relative `yolo11n.pt` path | Model provider with explicit path, checksum, metadata |
| Work scheduling | Inference in UI update path | Dedicated bounded inference worker |
| Result schema | Ultralytics result object | Immutable typed detection records |
| Errors | Printed and returned original frame | Structured event with observable degraded state |
| Performance | Not measured | Capture/inference/display metrics and budgets |
| Distance | Single focal constant and class widths | Calibrated camera model or clearly labeled heuristic with error bounds |
| Testing | Live camera required | Recorded fixtures plus adapter/worker/UI tests |
| Packaging | Loose local model file | Optional, reproducible dependency/model bundle |

The original work remains valuable because it demonstrates an end-to-end
camera-to-detection UI. The target architecture makes that same contribution
repeatable, measurable, and independently testable.
