# One synchronized 6-camera frame set.
#
# Ordering contract: camera_ids[i] names the camera of images[i] /
# camera_infos[i]. The ENGINE input order (nuScenes-derived) is
# CAM_FRONT_LEFT, CAM_FRONT, CAM_FRONT_RIGHT, CAM_BACK_LEFT, CAM_BACK,
# CAM_BACK_RIGHT. Bevdet consumers reorder by camera_ids, never by index.
#
# NOTE: bounded sequences (<=6) instead of fixed-size arrays: fixed arrays of
# complex messages with nested variable-length fields (Image.data) broke CDR
# round-tripping (destroyed payload bytes in both rclpy/rclcpp paths).
# The "exactly 6" contract is enforced in the nodes (see camera_sync / bevdet).
std_msgs/Header header
string[<=6] camera_ids
sensor_msgs/Image[<=6] images
sensor_msgs/CameraInfo[<=6] camera_infos