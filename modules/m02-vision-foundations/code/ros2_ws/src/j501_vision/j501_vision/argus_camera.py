"""
J501 Argus 相机采集类（GMSL2 RAW 相机路径）

功能：
  1. 通过 GStreamer + nvarguscamerasrc 获取 ISP 处理后的彩色图像
  2. 支持 1080p@30fps / 720p@60fps 等组合（实际可用组合取决于传感器模式）
  3. 提供 OpenCV 兼容的 numpy 数组输出
  4. 支持多相机（sensor-id 0/1）
  5. 完善的错误处理和资源管理（上下文管理器）

说明：
  本类通过 nvarguscamerasrc（NVIDIA Argus ISP）采集，
  适用于 AR0233 / IMX390 / AR0820 等 RAW 输出的 GMSL2 相机。

依赖：
  - GStreamer 1.0 + nvarguscamerasrc 插件
  - OpenCV (cv2)
  - gi (PyGObject)

使用示例：
  # 方式一：上下文管理器（推荐）
  with ArgusCamera(sensor_id=0, width=1920, height=1080, fps=30) as cam:
      cam.start()
      frame = cam.read()

  # 方式二：传统方式
  cam = ArgusCamera(sensor_id=0)
  cam.start()
  frame = cam.read()
  cam.stop()
"""

import cv2
import numpy as np
import gi
import logging
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

logger = logging.getLogger(__name__)


class ArgusCameraError(Exception):
    """Argus 相机异常基类"""
    pass


class ArgusCamera:
    """Jetson Argus 相机采集类（GMSL2 RAW 相机路径）"""

    def __init__(self, sensor_id=0, width=1920, height=1080, fps=30,
                 wbmode='auto', exposure='auto', flip_method=0):
        """
        初始化 Argus 相机

        Args:
            sensor_id: 相机 ID（0 或 1）
            width: 图像宽度
            height: 图像高度
            fps: 帧率
            wbmode: 白平衡模式（auto/daylight/incandescent/...）
            exposure: 曝光模式（'auto' 或 'min max' 纳秒字符串）
            flip_method: 翻转方式（0=不翻转, 2=180°, 4=水平, 6=垂直）

        Raises:
            ArgusCameraError: 管线构建或初始化失败
        """
        Gst.init(None)
        self.sensor_id = sensor_id
        self.width = width
        self.height = height
        self.fps = fps
        self.flip_method = flip_method
        self.pipeline = None
        self.appsink = None
        self._is_running = False

        try:
            self._build_pipeline(wbmode, exposure)
        except Exception as e:
            raise ArgusCameraError(f"管线构建失败: {e}")

    def _build_pipeline(self, wbmode, exposure):
        """构建 GStreamer 管线"""
        # 曝光参数处理
        if exposure == 'auto':
            exposure_str = ''
        else:
            exposure_str = f'exposuretimerange="{exposure}"'

        pipeline_str = (
            f'nvarguscamerasrc sensor-id={self.sensor_id} '
            f'wbmode={wbmode} {exposure_str} ! '
            f'video/x-raw(memory:NVMM),'
            f'width={self.width},height={self.height},'
            f'framerate={self.fps}/1 ! '
            f'nvvidconv flip-method={self.flip_method} ! '
            f'video/x-raw,format=BGRx ! '
            f'videoconvert ! '
            f'video/x-raw,format=BGR ! '
            f'appsink name=sink drop=true sync=false'
        )
        logger.debug(f"Pipeline: {pipeline_str}")

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except GLib.Error as e:
            raise ArgusCameraError(f"GStreamer 管线解析失败: {e}")

        # 获取 appsink 并配置
        self.appsink = self.pipeline.get_by_name('sink')
        if self.appsink is None:
            raise ArgusCameraError("无法获取 appsink 元素")
        self.appsink.set_property('emit-signals', True)
        self.appsink.set_property('max-buffers', 1)
        self.appsink.set_property('drop', True)

    def start(self):
        """
        启动采集

        Raises:
            ArgusCameraError: 启动失败
        """
        if self._is_running:
            logger.warning("相机已在运行")
            return

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise ArgusCameraError("管线启动失败，请检查 nvargus-daemon 服务")

        # 等待管线进入 PLAYING 状态
        ret = self.pipeline.get_state(Gst.SECOND * 5)
        if ret[1] != Gst.State.PLAYING:
            raise ArgusCameraError(f"管线未进入 PLAYING 状态: {ret[1]}")

        self._is_running = True
        logger.info(f"Argus 相机已启动: {self.width}x{self.height}@{self.fps}fps "
                     f"(sensor={self.sensor_id})")

    def stop(self):
        """停止采集"""
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self._is_running = False
        logger.info("Argus 相机已停止")

    def read(self):
        """
        读取一帧图像

        Returns:
            numpy.ndarray: BGR 格式图像，或 None（失败时）
        """
        if not self._is_running:
            return None

        sample = self.appsink.emit('pull-sample')
        if sample is None:
            return None

        buf = sample.get_buffer()
        success, info = buf.map(Gst.MapFlags.READ)
        if not success:
            return None

        try:
            frame = np.frombuffer(info.data, dtype=np.uint8)
            expected_size = self.height * self.width * 3
            if frame.size != expected_size:
                logger.warning(f"帧大小不匹配: {frame.size} != {expected_size}")
                return None
            frame = frame.reshape((self.height, self.width, 3))
            return frame
        except Exception as e:
            logger.error(f"帧解析失败: {e}")
            return None
        finally:
            buf.unmap(info)

    def is_opened(self):
        """检查相机是否可用"""
        return self.pipeline is not None and self._is_running

    def __enter__(self):
        """上下文管理器入口"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.stop()

    def __del__(self):
        """析构函数：确保资源释放"""
        self.stop()


if __name__ == '__main__':
    # 示例：采集并显示 1080p@30fps
    logging.basicConfig(level=logging.INFO)

    try:
        with ArgusCamera(sensor_id=0, width=1920, height=1080, fps=30) as cam:
            print(f"Argus 相机已启动: {cam.width}x{cam.height}@{cam.fps}fps")

            while True:
                frame = cam.read()
                if frame is not None:
                    cv2.imshow('Argus Camera', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
    except ArgusCameraError as e:
        print(f"相机错误: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
