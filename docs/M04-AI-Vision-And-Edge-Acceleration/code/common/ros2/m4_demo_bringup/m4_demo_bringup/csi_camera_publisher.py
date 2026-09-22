#!/usr/bin/env python3
"""Live camera capture → ROS Image publisher, using GStreamer Python binding.

Why not cv2.VideoCapture?  opencv 4.14 cuda build has GStreamer linked but
its build-info reports NO and CAP_GSTREAMER fails.  Using gi.repository.Gst
directly avoids that path entirely.

Pipeline modes (--source):
  auto  Try CSI v4l2src first (with nvjpegdec), fall back to test source
  csi   Force v4l2src /dev/videoX (CSI / V4L2)
  usb   Force v4l2src on USB UVC
  test  GStreamer videotestsrc (synthetic, always works)

CLI:
  csi_camera_publisher.py --source auto --device /dev/video0
                          --width 1920 --height 1080 --fps 30
                          [--timeout 0]
                          [--topic /perception/cameras/front/image]

Performance notes (both are load-bearing, do not regress):
  * The v4l2 path uses `nvvidconv` for the YUYV -> BGR conversion because
    the CPU `videoconvert` route measured only 10.2 fps at 1920x1080 while
    nvvidconv reaches 30 fps. See build_pipeline_string().
  * `msg.data` must be assigned an `array.array`, never `bytes`, because
    rclpy converts a bytes assignment element-by-element (~149 ns/byte =
    927 ms/frame at 1080p). See numpy_to_image_msg().
"""
import argparse
import array
import os
import signal
import sys
import time

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rcl_interfaces.msg import SetParametersResult

try:
    from m4_demo_bringup.control_settings import load_settings
except ImportError:  # Allows the standalone camera smoke test to keep working.
    def load_settings():
        return {"undistort_enabled": False}, "m4_demo_bringup unavailable"


DEFAULT_CALIBRATION_FILE = \
    "/home/seeed/ros2_ws/src/j501_avm_calib/config/camera_info/front.yaml"

# GStreamer Python binding (gi)
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib  # noqa


def numpy_to_image_msg(arr, frame_id, stamp, data_buf=None):
    """Wrap an HxW[xC] uint8 array into a sensor_msgs/Image.

    PERFORMANCE CONTRACT — read before touching the `msg.data` line.

    `sensor_msgs/Image.data` is a `uint8[]`, which rclpy backs with an
    `array.array`. Assigning a **`bytes`** object makes rclpy convert
    element-by-element through Python: measured ~149 ns/byte, i.e.
    **927 ms for one 1920x1080 bgr8 frame** (799-927 ms across runs).
    That single line capped this publisher at 1.2 fps and, because every
    downstream node inherited the same rate, the whole M4 pipeline ran at
    ~1.16 Hz while the pure GStreamer path could already do 30 fps.

    Assigning an `array.array('B', ...)` instead uses the buffer protocol
    and costs ~0.6 ms for the same frame (verified: 1.2 fps -> 29.7 fps).
    A `memoryview` is NOT a fix — it measured 1249 ms, even worse.

    `data_buf` is an optional reusable `array.array('B')` of exactly
    `arr.nbytes` bytes, which lets the caller avoid a 6 MB allocation per
    frame (0.63 ms measured). Never assign a `bytes` object here.
    """
    msg = __import__('sensor_msgs.msg', fromlist=['Image']).Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    if arr.ndim == 2:
        msg.encoding = 'mono8'
        msg.step = arr.shape[1]
    else:
        c = arr.shape[2]
        if c == 3:
            msg.encoding = 'bgr8'
            msg.step = arr.shape[1] * 3
        elif c == 4:
            msg.encoding = 'bgra8'
            msg.step = arr.shape[1] * 4
        else:
            raise ValueError(f'unsupported channels {c}')
    nbytes = int(arr.nbytes)
    if data_buf is not None and len(data_buf) == nbytes:
        # Bulk buffer-protocol copy straight into the reused array.
        np.frombuffer(data_buf, dtype=np.uint8)[:] = arr.reshape(-1)
        msg.data = data_buf
    else:
        msg.data = array.array('B', arr.tobytes())
    msg.is_bigendian = 0
    return msg


def build_pipeline_string(source, device, width, height, fps, hw_csc=True):
    """Return a GStreamer pipeline string for the requested source.

    The pipeline always ends in appsink name=appsink so the python
    caller can pull frames with GstApp.AppSink.try_pull_sample().

    COLOR CONVERSION — measured on the Orin with the real CSI sensor at
    1920x1080@30:

        ... ! videoconvert ! BGR                     10.2 fps   <-- CPU CSC
        ... ! nvvidconv ! BGRx ! videoconvert ! BGR  30.0 fps   <-- HW CSC

    The sensor emits YUYV (4:2:2). Doing YUYV -> BGR in `videoconvert`
    is a pure-CPU color conversion and it alone capped the whole M4
    pipeline at ~10 fps even after the Image.data fix. `nvvidconv` is the
    Tegra hardware converter (operates on NVMM) and reaches 30 fps. The
    trailing BGRx -> BGR `videoconvert` is only an alpha-channel drop, so
    it stays fast — and it preserves the bgr8 encoding that
    yolo_trt_node's cv_bridge::toCvShare(msg, "bgr8") requires.
    (nvvidconv cannot output 3-channel BGR directly; RGBx/BGRx only.)

    `hw_csc=False` selects the portable CPU path; try_open_sources()
    falls back to it automatically when nvvidconv is unavailable.
    """
    if source == 'test':
        return (
            'videotestsrc is-live=true pattern=ball '
            f'! video/x-raw,width={width},height={height},framerate={fps}/1 '
            '! videoconvert ! video/x-raw,format=BGR '
            '! appsink name=appsink drop=true max-buffers=2 sync=false'
        )
    # v4l2src path (CSI or USB)
    if hw_csc:
        csc = (
            '! nvvidconv ! video/x-raw,format=BGRx '
            '! videoconvert ! video/x-raw,format=BGR '
        )
    else:
        csc = '! videoconvert ! video/x-raw,format=BGR '
    return (
        f'v4l2src device={device} io-mode=2 '
        f'! video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 '
        f'{csc}'
        '! appsink name=appsink drop=true max-buffers=2 sync=false'
    )


def try_open(source, device, width, height, fps, probe_seconds=2.0, hw_csc=True):
    """Build pipeline, try to receive at least one frame in probe_seconds.
    Returns (appsink, label) on success, (None, error_msg) on failure.
    """
    Gst.init(None)
    desc = build_pipeline_string(source, device, width, height, fps, hw_csc)
    try:
        pipeline = Gst.parse_launch(desc)
    except Exception as e:
        return None, f'parse_launch failed: {e}'
    appsink = pipeline.get_by_name('appsink')
    if appsink is None:
        return None, 'no appsink in pipeline'

    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)

    deadline = time.monotonic() + probe_seconds
    while time.monotonic() < deadline:
        msg = bus.timed_pop_filtered(
            50 * Gst.MSECOND,
            Gst.MessageType.ERROR | Gst.MessageType.EOS
        )
        if msg:
            if msg.type == Gst.MessageType.ERROR:
                err, dbg = msg.parse_error()
                pipeline.set_state(Gst.State.NULL)
                return None, f'gst error: {err} {dbg}'
            if msg.type == Gst.MessageType.EOS:
                pipeline.set_state(Gst.State.NULL)
                return None, 'EOS before frame'
        sample = appsink.try_pull_sample(Gst.SECOND // 10)
        if sample is not None:
            # success — give the pipeline back to caller
            return (appsink, pipeline,
                    f'{source}({device}) {width}x{height}@{fps}'
                    f'{" hw-csc" if (hw_csc and source != "test") else " cpu-csc"}'), None

    pipeline.set_state(Gst.State.NULL)
    return None, 'no frames in {probe_seconds}s — sensor disconnected?'


def try_open_sources(sources, device, width, height, fps):
    """Try each source in order; for v4l2 sources prefer the hardware CSC
    pipeline and fall back to the CPU one. Returns the first that yields
    frames as (appsink, pipeline, label), else (None, None, tried_messages).
    """
    tried = []
    for src in sources:
        # Test source already uses videoconvert; nvvidconv would need an
        # NVMM upload and buys nothing there.
        variants = (True,) if src == 'test' else (True, False)
        for hw_csc in variants:
            result = try_open(src, device, width, height, fps, hw_csc=hw_csc)
            # result: ((appsink, pipeline, label), None) on success
            #         (None, error_msg) on failure
            if result[0] is not None:
                appsink, pipeline, label = result[0]
                return appsink, pipeline, label
            tried.append(f'{src}{"" if src == "test" else ("/hw-csc" if hw_csc else "/cpu-csc")}: {result[1]}')
    return None, None, tried


class CameraPublisher(Node):
    def __init__(self, source, device, width, height, fps, topic, timeout_s,
                 capture_width=None, capture_height=None):
        super().__init__('csi_camera_publisher')
        # depth=1 + RELIABLE. A single 1920x1080 bgr8 frame is 6.2 MB, so the
        # previous depth=10 could buffer ~60 MB per subscriber. RELIABLE (not
        # SensorDataQoS) is kept deliberately so `ros2 topic hz` and other
        # RELIABLE tooling still see this topic; yolo_trt_node subscribes with
        # SensorDataQoS (BEST_EFFORT), which a RELIABLE publisher satisfies.
        self.pub = self.create_publisher(
            __import__('sensor_msgs.msg', fromlist=['Image']).Image, topic,
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ))
        self.topic = topic
        self.timeout_s = timeout_s
        self._output_width = int(width)
        self._output_height = int(height)
        self._capture_width = int(capture_width or width)
        self._capture_height = int(capture_height or height)
        if self._capture_width < self._output_width or self._capture_height < self._output_height:
            raise ValueError('capture resolution must be at least the published resolution')
        self._crop_x = (self._capture_width - self._output_width) // 2
        self._crop_y = (self._capture_height - self._output_height) // 2
        self.start = time.monotonic()
        self.frame_count = 0
        self.last_log = self.start
        # Reusable data buffer so we do not allocate 6 MB per frame.
        # See numpy_to_image_msg() for why the assignment form matters.
        self._data_buf = None
        self._undistort_map1 = None
        self._undistort_map2 = None
        self._undistort_error = ""
        self._undistort_available = False

        persisted, persisted_warning = load_settings()
        self._undistort_enabled = self.declare_parameter(
            'undistort_enabled', bool(persisted.get('undistort_enabled', False))).value
        self._calibration_file = self.declare_parameter(
            'undistort_calibration_file', DEFAULT_CALIBRATION_FILE).value
        self._undistort_balance = self.declare_parameter(
            # balance=0 retains only pixels that map to the source image.
            # The prior 0.2 setting intentionally kept extra FOV, but exposed
            # very visible black arcs on this fisheye lens.
            'undistort_balance', 0.0).value
        self._undistort_fov_scale = self.declare_parameter(
            # A deliberately narrower perspective view keeps edge stretching
            # useful for people/objects rather than visually disorienting.
            'undistort_fov_scale', 0.55).value
        # Exposed for the web control plane; callbacks refuse writes to them.
        self.declare_parameter('undistort_available', False)
        self.declare_parameter('undistort_error', '')

        # Pick source
        if source == 'auto':
            order = ['csi', 'usb', 'test']
        else:
            order = [source]
        appsink, pipeline, label_or_tried = try_open_sources(
            order, device, self._capture_width, self._capture_height, fps
        )
        if appsink is None:
            self.get_logger().fatal(
                f'No camera source worked. Tried:\n  ' +
                '\n  '.join(label_or_tried)
            )
            sys.exit(2)
        self.appsink, self.pipeline = appsink, pipeline
        self.get_logger().info(f'Camera live: {label_or_tried}')

        self._prepare_undistort_maps(self._capture_width, self._capture_height)
        if persisted_warning:
            self.get_logger().warn(persisted_warning)
        if self._undistort_enabled and not self._undistort_available:
            self.get_logger().error(
                f'Undistortion disabled: {self._undistort_error}')
            self._undistort_enabled = False
            self.set_parameters([
                Parameter('undistort_enabled', value=False)])
        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_set_parameters)

        # spin via GLib timer — no main loop, drive from ROS timer
        self.timer = self.create_timer(1.0 / max(fps, 1), self.tick)

    def _set_undistort_status(self, available, error=''):
        self._undistort_available = bool(available)
        self._undistort_error = str(error or '')
        self.set_parameters([
            Parameter('undistort_available', value=self._undistort_available),
            Parameter('undistort_error', value=self._undistort_error),
        ])

    def _prepare_undistort_maps(self, width, height):
        """Build native-size maps once; crop only after rectification.

        The front calibration was created at 1920x1536.  Capturing that exact
        native frame prevents the V4L2 driver's opaque 1080p crop/scale mode
        from changing the camera matrix before OpenCV sees the pixels.  Both
        raw and rectified paths are then centre-cropped to the unchanged
        1920x1080 ROS output contract.
        """
        try:
            with open(self._calibration_file, 'r', encoding='utf-8') as f:
                calib = yaml.safe_load(f)
            if calib.get('distortion_model') != 'equidistant':
                raise ValueError('front calibration is not an equidistant/fisheye model')
            calib_w, calib_h = int(calib['image_width']), int(calib['image_height'])
            k = np.asarray(calib['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
            d = np.asarray(calib['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
            if d.shape[0] != 4:
                raise ValueError('fisheye calibration must contain four coefficients')

            if (width, height) == (calib_w, calib_h):
                pass
            elif abs(width / calib_w - height / calib_h) < 1e-6:
                scale_x, scale_y = width / calib_w, height / calib_h
                k[0, :] *= scale_x
                k[1, :] *= scale_y
            else:
                raise ValueError(
                    f'cannot safely adapt {calib_w}x{calib_h} calibration to {width}x{height}')

            size = (int(width), int(height))
            new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                k, d, size, np.eye(3), balance=float(self._undistort_balance),
                new_size=size, fov_scale=float(self._undistort_fov_scale))
            self._undistort_map1, self._undistort_map2 = cv2.fisheye.initUndistortRectifyMap(
                k, d, np.eye(3), new_k, size, cv2.CV_16SC2)
            cv2.setNumThreads(4)
            self._set_undistort_status(True)
            self.get_logger().info(
                f'undistort maps ready: {self._calibration_file} {width}x{height} '
                f'balance={self._undistort_balance} fov_scale={self._undistort_fov_scale}')
        except Exception as exc:
            self._undistort_map1 = self._undistort_map2 = None
            self._set_undistort_status(False, str(exc))

    def _on_set_parameters(self, params):
        result = SetParametersResult(successful=True, reason='')
        for param in params:
            if param.name in ('undistort_available', 'undistort_error',
                              'undistort_calibration_file', 'undistort_balance',
                              'undistort_fov_scale'):
                result.successful = False
                result.reason = f'{param.name} is read-only while capture is running'
                return result
            if param.name == 'undistort_enabled':
                if not isinstance(param.value, bool):
                    result.successful = False
                    result.reason = 'undistort_enabled must be boolean'
                    return result
                if param.value and not self._undistort_available:
                    result.successful = False
                    result.reason = self._undistort_error or 'front calibration unavailable'
                    return result
                self._undistort_enabled = param.value
        return result

    def tick(self):
        # Timeout
        if self.timeout_s > 0 and \
           (time.monotonic() - self.start) > self.timeout_s:
            self.get_logger().info(
                f'timeout {self.timeout_s}s reached'
            )
            self.cleanup()
            # Use SIGTERM-style exit so the finally in main() runs cleanup again
            os.kill(os.getpid(), signal.SIGTERM)

        sample = self.appsink.try_pull_sample(Gst.MSECOND // 5)
        if sample is None:
            return
        buf = sample.get_buffer()
        caps = sample.get_caps()
        s = caps.get_structure(0)
        w = s.get_value('width')
        h = s.get_value('height')
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return
        try:
            arr = np.frombuffer(info.data, dtype=np.uint8).reshape(h, w, 3).copy()
        finally:
            buf.unmap(info)

        if self._undistort_enabled:
            arr = cv2.remap(arr, self._undistort_map1, self._undistort_map2,
                            interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT)

        # Keep the public shared topic at 1920x1080 without ever applying a
        # 1536p calibration to an unknown driver-side 1080p crop.
        if self._crop_x or self._crop_y:
            arr = arr[self._crop_y:self._crop_y + self._output_height,
                      self._crop_x:self._crop_x + self._output_width]

        stamp = self.get_clock().now().to_msg()
        if self._data_buf is None or len(self._data_buf) != arr.nbytes:
            self._data_buf = array.array('B', bytes(arr.nbytes))
        msg = numpy_to_image_msg(arr, 'camera_front', stamp, self._data_buf)
        self.pub.publish(msg)
        self.frame_count += 1

        now = time.monotonic()
        if now - self.last_log >= 2.0:
            elapsed = now - self.start
            self.get_logger().info(
                f'live: {self.frame_count} frames in {elapsed:.1f}s '
                f'({self.frame_count / elapsed:.1f} fps) on {self.topic}'
            )
            self.last_log = now

    def cleanup(self):
        """Release camera / GStreamer resources. Safe to call multiple times."""
        if getattr(self, '_cleanup_done', False):
            return
        self._cleanup_done = True
        if self.timer is not None:
            try:
                self.timer.cancel()
            except Exception:
                pass
        if self.pipeline is not None:
            try:
                # Send EOS to drain any in-flight buffers, then NULL the pipeline
                self.pipeline.send_event(Gst.Event.new_eos())
                bus = self.pipeline.get_bus()
                # wait briefly for EOS or up to 1s
                end = time.monotonic() + 1.0
                while time.monotonic() < end:
                    msg = bus.timed_pop_filtered(
                        50 * Gst.MSECOND,
                        Gst.MessageType.EOS | Gst.MessageType.ERROR
                    )
                    if msg is not None and msg.type == Gst.MessageType.EOS:
                        break
                self.pipeline.set_state(Gst.State.NULL)
            except Exception as e:
                self.get_logger().warn(f'pipeline cleanup error: {e}')
            self.pipeline = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', choices=['auto', 'csi', 'usb', 'test'],
                        default='auto')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1920)
    parser.add_argument('--height', type=int, default=1080)
    parser.add_argument('--capture-width', type=int, default=0,
                        help='native acquisition width before optional output crop')
    parser.add_argument('--capture-height', type=int, default=0,
                        help='native acquisition height before optional output crop')
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--topic', default='/perception/cameras/front/image')
    parser.add_argument('--timeout', type=float, default=0,
                        help='seconds to publish before exit (0=forever)')
    args = parser.parse_args()

    rclpy.init()
    node = CameraPublisher(
        args.source, args.device, args.width, args.height, args.fps,
        args.topic, args.timeout, args.capture_width or args.width,
        args.capture_height or args.height,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
