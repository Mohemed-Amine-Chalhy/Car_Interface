# Perception engineering

The vehicle uses two complementary perception streams: RPLidar geometry for
obstacle proximity and a camera with YOLO11n for semantic object detection.

![Perception pipeline](../assets/perception-pipeline.svg)

## Capability map

| Capability | Implementation |
| --- | --- |
| RPLidar acquisition | Threaded scan source with immutable latest-scan snapshots |
| Range normalization | Millimetres converted to centimetres for analysis and display |
| Vehicle-relative geometry | Sensor angles rotated into the vehicle frame |
| Projected-path detection | Points filtered against the configured vehicle corridor |
| Assisted-stop distance | 50 cm default threshold connected to the control service |
| Camera capture | OpenCV camera input with a bounded freshness-oriented queue |
| YOLO inference | YOLO11n inference through Ultralytics |
| Detection overlay | Class, confidence, bounding box, and approximate distance |

## My contribution

I integrated the perception path with the operator application. My work covered
RPLidar processing, the vehicle-relative display, projected-path obstacle
detection, camera capture, YOLO inference, and annotated detections.

## Lidar geometry

Each RPLidar measurement is converted from millimetres to centimetres and
rotated so 90 degrees represents vehicle forward. The application then checks
whether the point falls inside the configured vehicle corridor:

```text
vehicle_angle = sensor_angle - 90 degrees
lateral_distance = abs(distance_cm x sin(vehicle_angle))
in_path = lateral_distance <= vehicle_width_cm / 2
```

With the 42 cm corridor, points within 21 cm of the projected centerline are
treated as in-path. The operator view uses a 180-degree front scan, while the
control service tracks the closest in-path point and compares it with the
assisted-stop threshold.

## YOLO integration

The camera pipeline sends frames through YOLO11n and displays the resulting
class labels, confidence values, bounding boxes, and approximate distance
estimates. A bounded frame queue favors recent images when inference takes
longer than the camera frame interval.

See the [YOLO11n pipeline](yolo-pipeline.md) for the implementation details and
the [perception validation plan](validation-plan.md) for calibration,
performance measurements, and model evaluation.
