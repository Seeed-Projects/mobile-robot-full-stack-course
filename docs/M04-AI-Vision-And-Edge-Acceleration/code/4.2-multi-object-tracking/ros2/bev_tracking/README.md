/** \mainpage bev_tracking
 *
 * \section intro_sec Introduction
 *
 * bev_tracking is the M4.2 milestone of the ROS2 BEV pipeline. It
 * consumes `vision_msgs/Detection2DArray` produced by bev_detection
 * (M4.1) and produces a `vision_msgs/Detection2DArray` whose per-track
 * Detection2D entries carry a stable `id` field. Tracking is done by
 * supervision.ByteTrack (a faithful port of ByteTrack).
 *
 * \section arch_sec Architecture
 *
 * \code
 *   /perception/cameras/front/image  (sensor_msgs/Image)
 *             |
 *     bev_detection/yolo_trt_node    (M4.1)
 *             |
 *             v
 *   /perception/detections           (vision_msgs/Detection2DArray)
 *             |
 *     bev_tracking/tracking_node     (M4.2 - this package)
 *             |
 *             v
 *   /perception/tracks               (vision_msgs/Detection2DArray)
 *             |
 *             v
 *        downstream consumers
 *        (BEV fusion, depth alignment, FoundationPose, etc.)
 * \endcode
 *
 * The package owns exactly three artefacts:
 *   1. adapter.py        - pure-Python ROS <-> supervision conversion
 *   2. tracking_node.py  - rclpy node wrapping ByteTrack
 *   3. mock_detection_publisher.py - dev-time synthetic detector
 *
 * No calibration data, no GPU code, no model artifacts live here.
 *
 * \section build_sec Building
 *
 * \code{.sh}
 *   source /opt/ros/humble/setup.bash
 *   cd ros2_ws
 *   colcon build --packages-select bev_tracking
 *   colcon test --packages-select bev_tracking --ctest-args -V
 * \endcode
 *
 * \section run_sec Running
 *
 * Standalone (with mock detector):
 * \code{.sh}
 *   ros2 launch bev_tracking tracking.launch.py
 * \endcode
 *
 * End-to-end (with bev_detection):
 * \code{.sh}
 *   ros2 launch bev_tracking tracking_with_detection.launch.py
 * \endcode
 *
 * \section demo_sec Real-time camera demo
 *
 * The student-facing one-click demo brings up the full pipeline
 * (camera -> M4.1 YOLO -> M4.2 ByteTrack -> overlays -> image viewer)
 * with a single command:
 *
 * \code{.sh}
 *   ./scripts/m4/run_m4_2_demo.sh                # auto-detect camera + GUI
 *   ./scripts/m4/run_m4_2_demo.sh --no-gui      # headless (no rqt_image_view)
 *   CAMERA_SOURCE=test ./scripts/m4/run_m4_2_demo.sh   # synthetic source
 * \endcode
 *
 * The visualizer is `tracking_visualizer.py` and is **completely
 * separate from `tracking_node.py`** — ByteTrack itself stays
 * camera- and image-independent. See `docs/M4.2_DEMO.md` for details
 * on the overlay format and shutdown contract.
 */
