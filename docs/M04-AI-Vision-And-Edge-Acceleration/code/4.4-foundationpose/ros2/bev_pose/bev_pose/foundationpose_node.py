"""bev_pose.foundationpose_node — ROS2 node that runs NVlabs/FoundationPose.

Subscribes (synchronised via ApproximateTime):
    /perception/cameras/front/image          (sensor_msgs/Image — RGB bgr8 or rgb8)
    /perception/cameras/front/depth          (sensor_msgs/Image — 32FC1 in meters)
    /perception/cameras/front/camera_info    (sensor_msgs/CameraInfo)
    /perception/object_mask                  (sensor_msgs/Image — mono8 0/255)

Publishes:
    /perception/object_pose                  (geometry_msgs/PoseStamped)
    /perception/object_poses_3d              (vision_msgs/Detection3DArray)  [optional]
    /tf                                     (camera_frame -> object_frame)
    /perception/mesh_meta                    (std_msgs/String — JSON dump)
    /perception/foundationpose/stats         (std_msgs/String — JSON dump, once per second)
    /perception/foundationpose/pose_image    (sensor_msgs/Image — debug overlay)  [optional]

Lifecycle:
    1. Wait for /perception/object_mask to have a non-empty mask.
    2. Call engine.register(rgb, depth, K, mask) ONCE.
    3. For every subsequent synchronised frame, call engine.track(rgb, depth, K).
    4. Publish the resulting PoseStamped + TF + (optional) Detection3DArray.

Mask semantics:
    * /perception/object_mask is mono8, 0 = background, 255 = object.
      This is NOT a Detection2DArray. The plan forbids using
      /perception/detections here because Detection2DArray is bbox
      semantics, not segmentation. The dedicated object_mask_node
      provides P0 single-object mask generation (ROI prompt + depth
      threshold) until register() finishes; it can exit afterwards.

Performance:
    * register latency:  logged to /perception/foundationpose/stats (1Hz)
    * tracking FPS:     rolling mean over 30 frames
    * e2e latency:      ImageHeader.stamp → PoseStamped.stamp deltas
"""
from __future__ import annotations

import json
import logging
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros.transform_broadcaster import TransformBroadcaster
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .foundationpose_engine import FoundationPoseEngine
from .types import EngineConfig


log = logging.getLogger(__name__)


def _qos() -> QoSProfile:
    """Best-effort for camera streams; reliability is irrelevant for RGB."""
    return QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class FoundationPoseNode(Node):
    def __init__(self):
        super().__init__('foundationpose_node')

        # ---- declare parameters ---------------------------------
        self.declare_parameter('rgb_topic', '/perception/cameras/front/image')
        self.declare_parameter('depth_topic', '/perception/cameras/front/depth')
        self.declare_parameter('camera_info_topic', '/perception/cameras/front/camera_info')
        self.declare_parameter('mask_topic', '/perception/object_mask')
        self.declare_parameter('model_dir', '')                  # FoundationPose weights dir
        self.declare_parameter('mesh_npz_path', '')              # precomputed .npz
        self.declare_parameter('mesh_obj', '')                   # for NVlabs source
        self.declare_parameter('frame_id', 'object')             # tf child_frame
        self.declare_parameter('camera_frame_id', 'camera_front')  # tf parent
        self.declare_parameter('score_topic', '/perception/foundationpose/stats')
        self.declare_parameter('pose_topic', '/perception/object_pose')
        self.declare_parameter('det3d_topic', '/perception/object_poses_3d')
        self.declare_parameter('publish_det3d', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', '/perception/foundationpose/pose_image')
        self.declare_parameter('refiner_iterations', 5)
        self.declare_parameter('score_threshold', 0.3)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('e2e_latency_warn_ms', 200.0)
        self.declare_parameter('auto_register_on_first_mask', True)
        self.declare_parameter('require_mask_for_register', True)

        # ---- read parameters -----------------------------------
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        score_topic = self.get_parameter('score_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        det3d_topic = self.get_parameter('det3d_topic').get_parameter_value().string_value

        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._camera_frame_id = \
            self.get_parameter('camera_frame_id').get_parameter_value().string_value

        publish_det3d = self.get_parameter('publish_det3d').get_parameter_value().bool_value
        publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        publish_debug = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        debug_image_topic = \
            self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self._e2e_warn_ms = \
            self.get_parameter('e2e_latency_warn_ms').get_parameter_value().double_value
        self._auto_register = \
            self.get_parameter('auto_register_on_first_mask').get_parameter_value().bool_value
        self._require_mask = \
            self.get_parameter('require_mask_for_register').get_parameter_value().bool_value

        # ---- initialise engine ---------------------------------
        engine_cfg = EngineConfig(
            model_dir=self.get_parameter('model_dir').get_parameter_value().string_value,
            mesh_npz_path=self.get_parameter('mesh_npz_path').get_parameter_value().string_value,
            refiner_iterations=\
                self.get_parameter('refiner_iterations').get_parameter_value().integer_value,
            scorer_threshold=\
                self.get_parameter('score_threshold').get_parameter_value().double_value,
            device=self.get_parameter('device').get_parameter_value().string_value,
        )
        if not engine_cfg.model_dir or not engine_cfg.mesh_npz_path:
            self.get_logger().error(
                'model_dir and mesh_npz_path must be set; cannot start')
            raise RuntimeError('model_dir / mesh_npz_path not configured')

        self.engine = FoundationPoseEngine(engine_cfg)
        try:
            mesh = self.engine.load_object_model(engine_cfg.mesh_npz_path)
        except Exception as e:
            self.get_logger().error(f'load_object_model failed: {e!r}')
            raise
        # Use the frame_id from the .npz if user did not override it.
        if mesh.frame_id and self._frame_id == 'object':
            self._frame_id = mesh.frame_id

        # ---- pubs + subs ---------------------------------------
        from geometry_msgs.msg import PoseStamped
        self._bridge = CvBridge()
        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        if publish_det3d:
            self._det3d_pub = self.create_publisher(Detection3DArray, det3d_topic, 10)
        self._score_pub = self.create_publisher(String, score_topic, 10)
        if publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
        if publish_debug:
            self._debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)

        # Mesh-meta is static; publish once at start so visualiser + web
        # can grab bbox corners without re-importing trimesh.
        mesh_meta_pub = self.create_publisher(
            String, '/perception/mesh_meta', rclpy.qos.QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        meta_payload = json.dumps({
            'frame_id': self._frame_id,
            'diameter_m': float(mesh.diameter),
            'bbox_corners': mesh.bbox_corners.tolist(),
            'centroid': mesh.centroid.tolist(),
            'bounds_min': mesh.bounds_min.tolist(),
            'bounds_max': mesh.bounds_max.tolist(),
            'source_obj': mesh.source_obj,
            'vertex_count': int(mesh.vertices.shape[0]),
            'face_count': int(mesh.faces.shape[0]),
        })
        meta_msg = String(); meta_msg.data = meta_payload
        mesh_meta_pub.publish(meta_msg)
        self.get_logger().info(
            f'mesh_meta: diameter={mesh.diameter:.4f}m, '
            f'vertices={mesh.vertices.shape[0]}, frame_id={self._frame_id}')

        qos = _qos()
        self._rgb_sub = Subscriber(self, Image, rgb_topic, qos_profile=qos)
        self._depth_sub = Subscriber(self, Image, depth_topic, qos_profile=qos)
        self._info_sub = Subscriber(self, CameraInfo, info_topic, qos_profile=qos)
        self._mask_sub = Subscriber(self, Image, mask_topic, qos_profile=qos)

        self._sync = ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub, self._info_sub, self._mask_sub],
            queue_size=10, slop=0.10,
        )
        self._sync.registerCallback(self._on_synchronized)

        # 1Hz stats timer
        self._last_stats_emit = 0.0
        self._stats_timer = self.create_timer(1.0, self._emit_stats)

        # Cached camera intrinsics (re-built only when camera_info arrives)
        self._K_cache: Optional[np.ndarray] = None
        self._mask_area_log_state = 'waiting'

        self.get_logger().info(
            f'foundationpose_node ready:\n'
            f'  rgb_topic     = {rgb_topic}\n'
            f'  depth_topic   = {depth_topic}\n'
            f'  info_topic    = {info_topic}\n'
            f'  mask_topic    = {mask_topic}\n'
            f'  pose_topic    = {pose_topic}\n'
            f'  det3d_topic   = {det3d_topic} (publish={publish_det3d})\n'
            f'  model_dir     = {engine_cfg.model_dir}\n'
            f'  mesh_npz      = {engine_cfg.mesh_npz_path}\n'
            f'  frame_id      = {self._frame_id} (parent: {self._camera_frame_id})\n'
        )

    # -----------------------------------------------------------
    # cv_bridge helpers
    # -----------------------------------------------------------
    def _img_to_rgb(self, msg: Image) -> np.ndarray:
        """Convert ROS Image → HxWx3 uint8 RGB. Tries hard to coerce."""
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            try:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            except Exception as e:
                raise RuntimeError(f"cannot decode rgb image: {e!r}")
        return img

    def _depth_to_meters(self, msg: Image) -> np.ndarray:
        """Convert ROS Image depth → HxW float32 in meters."""
        if msg.encoding in ('32FC1',):
            d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            return d.astype(np.float32)
        if msg.encoding in ('16UC1',):
            d = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            return d.astype(np.float32) * 0.001  # mm → m
        # Fall back to passthrough float
        d = self._bridge.imgmsg_to_cv2(msg)
        return d.astype(np.float32)

    def _mask_to_binary(self, msg: Image) -> np.ndarray:
        """Convert ROS Image mask → HxW uint8 of {0,1} (or {0,255}).

        FoundationPose expects an ob_mask that is True/255 where the
        object is. We coerce anything to {0,255} to be safe.
        """
        m = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        # Convert any non-zero to 255 (some publishers emit {0,1} or {0,128}).
        m = ((m > 0).astype(np.uint8)) * 255
        return m

    # -----------------------------------------------------------
    # callback
    # -----------------------------------------------------------
    def _on_synchronized(
        self,
        rgb_msg: Image,
        depth_msg: Image,
        info_msg: CameraInfo,
        mask_msg: Image,
    ) -> None:
        try:
            self._handle_frame(rgb_msg, depth_msg, info_msg, mask_msg)
        except Exception as e:
            self.get_logger().warn(f'frame dispatch failed: {e!r}')

    def _handle_frame(
        self,
        rgb_msg: Image,
        depth_msg: Image,
        info_msg: CameraInfo,
        mask_msg: Image,
    ) -> None:
        rgb = self._img_to_rgb(rgb_msg)
        depth = self._depth_to_meters(depth_msg)
        mask = self._mask_to_binary(mask_msg)

        # K from CameraInfo
        K = self._extract_K(info_msg)

        # Decide if we should re-register
        mask_area = int((mask > 0).sum())
        self._mask_area_log_state = self._log_mask_state(mask_area, self._mask_area_log_state)

        if not self.engine.is_registered and self._auto_register and mask_area > 0:
            self.get_logger().info(
                f'first non-empty mask seen (area={mask_area}); calling register()')
            try:
                result = self.engine.register(rgb, depth, K, mask)
            except Exception as e:
                self.get_logger().error(f'register() raised: {e!r}')
                return
            self.get_logger().info(
                f'register() ok in {result.latency_ms:.1f}ms; pose will be published')
        elif not self.engine.is_registered:
            return  # waiting for first valid mask
        else:
            try:
                result = self.engine.track(rgb, depth, K)
            except Exception as e:
                self.get_logger().warn(f'track() raised: {e!r}; will re-register on next mask')
                self.engine.reset()
                return

        # Publish TF + PoseStamped
        try:
            self._publish_tf_and_pose(rgb_msg, result)
        except Exception as e:
            self.get_logger().warn(f'publish failed: {e!r}')

        # Optional Detection3DArray
        try:
            if hasattr(self, '_det3d_pub'):
                self._publish_det3d(rgb_msg, result)
        except Exception:
            pass

    # -----------------------------------------------------------
    def _extract_K(self, info_msg: CameraInfo) -> np.ndarray:
        # CameraInfo.K is 3x3 row-major. We trust it; cache for early frames.
        K = np.asarray(info_msg.k, dtype=np.float32).reshape(3, 3)
        self._K_cache = K
        return K

    def _log_mask_state(self, area: int, prev: str) -> str:
        if area == 0 and prev != 'empty':
            self.get_logger().info('mask empty — waiting for /perception/object_mask')
            return 'empty'
        if 0 < area < 200 and prev != 'tiny':
            self.get_logger().info(f'mask tiny ({area} px); ignoring until >200 px')
            return 'tiny'
        if area >= 200 and prev != 'ok':
            self.get_logger().info(f'mask OK (area={area} px)')
            return 'ok'
        return prev

    # -----------------------------------------------------------
    def _publish_tf_and_pose(self, rgb_msg: Image, result) -> None:
        from geometry_msgs.msg import PoseStamped
        from tf_transformations import quaternion_from_matrix

        pose = result.pose  # (4,4)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rgb_msg.header.stamp
        pose_msg.header.frame_id = self._camera_frame_id
        pose_msg.pose.position.x = float(pose[0, 3])
        pose_msg.pose.position.y = float(pose[1, 3])
        pose_msg.pose.position.z = float(pose[2, 3])
        qx, qy, qz, qw = quaternion_from_matrix(pose)
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self._pose_pub.publish(pose_msg)

        if hasattr(self, '_tf_broadcaster'):
            tf = TransformStamped()
            tf.header.stamp = rgb_msg.header.stamp
            tf.header.frame_id = self._camera_frame_id
            tf.child_frame_id = self._frame_id
            tf.transform.translation.x = float(pose[0, 3])
            tf.transform.translation.y = float(pose[1, 3])
            tf.transform.translation.z = float(pose[2, 3])
            tf.transform.rotation.x = float(qx)
            tf.transform.rotation.y = float(qy)
            tf.transform.rotation.z = float(qz)
            tf.transform.rotation.w = float(qw)
            self._tf_broadcaster.sendTransform(tf)

        # E2E latency warning (informational): stamp_from_msg → rclpy Time
        from rclpy.time import Time
        e2e_ms = (self.get_clock().now() - Time.from_msg(rgb_msg.header.stamp)
                  ).nanoseconds / 1e6
        if e2e_ms > self._e2e_warn_ms:
            self.get_logger().warn(
                f'e2e latency {e2e_ms:.0f}ms > warn threshold ({self._e2e_warn_ms:.0f}ms)')

    def _publish_det3d(self, rgb_msg: Image, result) -> None:
        dets = Detection3DArray()
        dets.header.stamp = rgb_msg.header.stamp
        dets.header.frame_id = self._camera_frame_id
        d = Detection3D()
        from geometry_msgs.msg import Pose, Point, Quaternion
        d.results.append(ObjectHypothesisWithPose())
        d.results[0].pose.pose = Pose(
            position=Point(
                x=float(result.pose[0, 3]),
                y=float(result.pose[1, 3]),
                z=float(result.pose[2, 3]),
            ),
        )
        from tf_transformations import quaternion_from_matrix
        qx, qy, qz, qw = quaternion_from_matrix(result.pose)
        d.results[0].pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        # Bounding box hint from mesh
        if self.engine.mesh is not None:
            bbmin = self.engine.mesh.bounds_min
            bbmax = self.engine.mesh.bounds_max
            size = bbmax - bbmin
            d.bbox.size.x = float(size[0])
            d.bbox.size.y = float(size[1])
            d.bbox.size.z = float(size[2])
            d.bbox.center.position.x = float(self.engine.mesh.centroid[0])
            d.bbox.center.position.y = float(self.engine.mesh.centroid[1])
            d.bbox.center.position.z = float(self.engine.mesh.centroid[2])
            d.bbox.center.orientation.w = 1.0
        dets.detections.append(d)
        self._det3d_pub.publish(dets)

    # -----------------------------------------------------------
    def _emit_stats(self) -> None:
        summary = self.engine.summary()
        # GPU probe (best-effort; non-fatal if nvidia-smi missing)
        gpu_mem, gpu_util = self._gpu_probe()
        summary['gpu_mem_mb'] = gpu_mem
        summary['gpu_util_pct'] = gpu_util
        summary['engine_registered'] = self.engine.is_registered
        msg = String(); msg.data = json.dumps(summary)
        self._score_pub.publish(msg)


    @staticmethod
    def _gpu_probe() -> Tuple[float, float]:
        try:
            import subprocess
            out = subprocess.run(
                ['nvidia-smi', '--query-gpu=memory.used,utilization.gpu',
                 '--format=csv,noheader,nounits'],
                capture_output=True, text=True, timeout=2.0,
            ).stdout.strip()
            if not out:
                return 0.0, 0.0
            parts = out.split(',')
            return float(parts[0]), float(parts[1])
        except Exception:
            return 0.0, 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = FoundationPoseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
