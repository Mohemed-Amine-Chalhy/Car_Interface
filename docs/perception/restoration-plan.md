# Perception restoration and validation plan

The goal is to restore the original camera/YOLO capability as a polished,
optional subsystem without coupling model inference to vehicle-control timing.
Each phase has a concrete artifact and an objective completion check.

## Phase 1 — recover the physical and model record

Collect:

- camera make, model, lens, mount height/pitch, supported resolutions, and
  Windows device identity;
- original YOLO weight file, SHA-256, source, license, class list, and model
  version;
- training dataset/configuration and evaluation output if the model was trained
  or fine-tuned by the team;
- representative recorded drives with permission to publish; and
- host CPU/GPU, memory, OS, and accelerator/runtime versions.

**Complete when:** a reviewed model card and camera inventory contain no unknown
artifact provenance.

## Phase 2 — introduce an optional typed boundary

Define interfaces for:

```text
FrameSource -> Frame(timestamp, dimensions, pixels)
Detector    -> tuple[Detection, ...]
VisionWorker -> latest VisionSnapshot + health/metrics
```

`Detection` should carry class ID/name, confidence, normalized box coordinates,
source-frame timestamp, model identifier, and optional estimated range. The
core application must remain importable and testable without OpenCV,
Ultralytics, or model weights installed.

**Complete when:** unit tests use fake frame/detector adapters and the base
simulation/build remains unchanged without the optional dependency group.

## Phase 3 — build a freshness-oriented inference worker

- capture into a bounded queue of one or two frames;
- discard superseded frames under load;
- perform inference outside the Tkinter thread;
- publish immutable latest snapshots;
- expose capture FPS, inference latency, end-to-end age, dropped frames, and
  worker health; and
- stop and join the worker within a measured shutdown bound.

**Complete when:** overload tests prove bounded memory and result age, and
disconnect/failure tests prove the UI remains responsive.

## Phase 4 — configure and package the model

Add a dedicated optional configuration surface:

```toml
[vision]
enabled = true
camera_index = 0
capture_width = 640
capture_height = 480
model_path = "models/<verified-weight-file>"
model_sha256 = "<verified-sha256>"
confidence_threshold = 0.50
device = "auto"
max_result_age_seconds = 0.50
```

The exact schema should be versioned when implemented; this block is a design
target and is not accepted by the current parser. Model files should be fetched
or attached to a release with a checksum rather than committed as an opaque
large binary.

**Complete when:** a clean machine can reproduce the environment, verify the
model, run a recorded-video smoke test, and build an artifact with an accurate
SBOM.

## Phase 5 — camera and distance calibration

1. Calibrate camera intrinsics with a documented target and save calibration
   metadata tied to the camera serial/mode.
2. Place representative objects at measured distances and orientations.
3. Compare the historical box-width heuristic with a calibrated method.
4. Report median, 95th-percentile, and maximum absolute/range-relative error.
5. Keep approximate ranges visibly labeled if the error is unsuitable for
   physical control decisions.

**Complete when:** the UI wording and documentation match the measured error
envelope.

## Phase 6 — model evaluation

Create a versioned, publishable evaluation set representing the car's actual
environment. Report per-class counts, precision, recall, confusion, mAP at the
chosen IoU thresholds, and failure examples. Separate training, validation, and
test data and record the exact evaluation command.

**Complete when:** every headline model claim links to an immutable metric file
and model hash.

## Phase 7 — vehicle integration

- display detection overlays alongside Lidar state;
- keep vision results advisory until a separately reviewed fusion/control design
  exists;
- record synchronized camera, Lidar, command, and timestamp traces;
- run stationary, wheels-clear, low-speed, and representative-scene tests; and
- record a demonstration tied to exact host, firmware, profile, and model
  versions.

**Complete when:** the physical demonstration is reproducible from the release
record and all degradation paths are observable.

## Benchmark report template

| Metric | Environment | Result |
| --- | --- | --- |
| Camera mode / measured capture FPS | TBD host + camera | TBD |
| Median / p95 inference latency | TBD host + device | TBD |
| Median / p95 end-to-end frame age | TBD host + device | TBD |
| Dropped-frame rate | Representative 10-minute run | TBD |
| Model precision / recall / mAP | Versioned test set | TBD |
| Range-estimate median / p95 error | Measured distance set | TBD |
| RPLidar known-distance error | Fixed targets | TBD |
| Synchronized system stability | 30-minute vehicle run | TBD |

Leaving results as `TBD` until measured avoids fabricated performance numbers
while presenting a complete, professional validation strategy.
