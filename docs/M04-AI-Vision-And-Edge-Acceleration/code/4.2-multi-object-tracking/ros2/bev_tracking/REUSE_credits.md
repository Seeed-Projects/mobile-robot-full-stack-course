# Third-party code and attributions

This package deliberately minimises its third-party footprint. The only
runtime dependency we pull in beyond the ROS 2 standard stack is
**supervision**, a Python-only computer-vision toolkit that contains a
faithful port of ByteTrack.

## supervision (ByteTrack port)

- **Source:** https://github.com/roboflow/supervision
- **License:** MIT
- **Component used:** `supervision.ByteTrack` (the Hungarian matcher and
  Kalman filter are encapsulated inside the package; we do not call
  them directly).
- **Why:** ByteTrack is the de-facto lightweight tracker for object
  detection pipelines and the supervision port is maintained, tested
  and drop-in for our use case. Re-implementing Kalman + Hungarian
  in M4.2 would duplicate effort, delay the milestone, and bring no
  measurable benefit (latency, accuracy, dependency surface are
  functionally equivalent to the supervised library at our scale).

## ROS 2 messages

We consume the standard `vision_msgs/Detection2DArray` and
`vision_msgs/Detection2D` messages. These are part of the ROS 2
vision_msgs package and are released under the same licence as ROS 2
(mostly Apache-2.0). No third-party code is vendored.

## No proprietary code

bev_tracking does NOT contain code from RoboFlow's commercial product
line. We use only the open-source `supervision` Python package.

## Upstream within this repo

We depend on `bev_detection` (M4.1) for input detections. We treat it
as a black box; no source is copied or vendored.
