# -*- coding: utf-8 -*-
"""cam_info_io：CameraInfo YAML 读写与消息换算测试。"""
import numpy as np

from j501_avm_calib import cam_info_io


def _data():
    K = np.array([[510.0, 0, 950.0], [0, 508.0, 760.0], [0, 0, 1.0]])
    D = np.array([0.1, -0.02, 0.003, 0.001])
    return cam_info_io.camera_info_to_yaml_dict(
        "front", 1920, 1536, K, D, "equidistant")


def test_yaml_roundtrip(tmp_path):
    K = np.array([[510.0, 0, 950.0], [0, 508.0, 760.0], [0, 0, 1.0]])
    D = np.array([0.1, -0.02, 0.003, 0.001])
    p = tmp_path / "front.yaml"
    cam_info_io.write_calibration_yaml(p, "front", 1920, 1536, K, D,
                                       "equidistant")
    out = cam_info_io.read_calibration_yaml(p)
    assert out["name"] == "front"
    assert (out["width"], out["height"]) == (1920, 1536)
    assert out["model"] == "equidistant"
    assert np.allclose(out["K"], K)
    assert np.allclose(out["D"], D)


def test_empty_template_not_calibrated(tmp_path):
    e = cam_info_io.empty_calibration_yaml("left", 640, 480)
    assert not cam_info_io.is_calibrated(e)
    cam_info_io.write_calibration_yaml(
        tmp_path / "l.yaml", "left", 640, 480,
        np.array([[300.0, 0, 320], [0, 300.0, 240], [0, 0, 1.0]]),
        np.zeros(4))
    d = cam_info_io.read_calibration_yaml(tmp_path / "l.yaml")
    assert cam_info_io.is_calibrated(
        cam_info_io.camera_info_to_yaml_dict(
            "left", d["width"], d["height"], d["K"], d["D"], d["model"]))


def test_msg_roundtrip():
    from builtin_interfaces.msg import Time
    data = _data()
    stamp = Time(sec=1, nanosec=2)
    msg = cam_info_io.to_camera_info_msg(stamp, "cam", data)
    back = cam_info_io.from_camera_info_msg(msg)
    assert back["image_width"] == 1920
    assert round(back["camera_matrix"]["data"][0], 3) == 510.0
    assert back["distortion_model"] == "equidistant"
    assert len(back["distortion_coefficients"]["data"]) == 4