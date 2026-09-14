#!/usr/bin/env python3
"""nuScenes (data_infos) -> rosbag2 for the Phase 2 ROS pipeline.

Topics:
  /camera/{front,front_left,front_right,back,back_left,back_right}/image_raw/compressed (jpeg)
  /camera/{...}/camera_info
  /tf  (static base_link->camera_* sensor2ego; per-frame map->base_link ego2global)

Usage (system python with ROS2 sourced):
  python3 tools/dataset_converter/nuscenes_to_rosbag.py \
      --nuscenes datasets/nuscenes --out datasets/bags/nuscenes_mini_60f --frames 60
"""
import argparse
import json
import os

import rclpy
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from sensor_msgs.msg import CompressedImage, CameraInfo
from tf2_msgs.msg import TFMessage

CAMERAS = ["front_left", "front", "front_right", "back_left", "back", "back_right"]
CHANNEL = {
    "front_left": "CAM_FRONT_LEFT", "front": "CAM_FRONT",
    "front_right": "CAM_FRONT_RIGHT", "back_left": "CAM_BACK_LEFT",
    "back": "CAM_BACK", "back_right": "CAM_BACK_RIGHT",
}


def q_from_ns(ns_q):
    """nuScenes [w,x,y,z] -> geometry_msgs/Quaternion."""
    q = Quaternion()
    q.w, q.x, q.y, q.z = ns_q[0], ns_q[1], ns_q[2], ns_q[3]
    return q


def mk_header(frame, stamp_ns):
    from std_msgs.msg import Header
    h = Header()
    h.frame_id = frame
    h.stamp.sec = stamp_ns // 1_000_000_000
    h.stamp.nanosec = stamp_ns % 1_000_000_000
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nuscenes", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--fps", type=float, default=0.0, help="playback rate override (0 = preserve 2Hz)")
    args = ap.parse_args()

    ns = args.nuscenes
    dinfo = os.path.join(ns, "data_infos")
    tokens = json.load(open(os.path.join(dinfo, "sample_tokens.json")))
    calib = {c["token"]: c for c in json.load(open(os.path.join(ns, "v1.0-mini", "calibrated_sensor.json")))}
    egopose = {e["token"]: e for e in json.load(open(os.path.join(ns, "v1.0-mini", "ego_pose.json")))}

    import yaml
    rec0 = yaml.safe_load(open(os.path.join(dinfo, "samples_info", "sample0000.yaml")))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)

    rclpy.init()
    writer = SequentialWriter()
    storage = StorageOptions(uri=args.out, storage_id="sqlite3")
    conv = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    writer.open(storage, conv)

    def add_topic(name, msg_type, qos=""):
        tm = TopicMetadata(name=name, type=msg_type,
                           serialization_format="cdr",
                           offered_qos_profiles=qos)
        writer.create_topic(tm)

    # Ground truth: profile serialization captured from `ros2 bag record`
    # (best-effort, volatile). Format mirrored exactly:
    INF = "sec: 9223372036\n    nsec: 854775807"
    sensor_qos = (
        "- history: 3\n  depth: 0\n  reliability: 2\n  durability: 2\n"
        f"  deadline:\n    {INF}\n  lifespan:\n    {INF}\n"
        f"  liveliness: 1\n  liveliness_lease_duration:\n    {INF}\n"
        "  avoid_ros_namespace_conventions: false\n")
    tf_qos = (
        "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n"
        f"  deadline:\n    {INF}\n  lifespan:\n    {INF}\n"
        f"  liveliness: 1\n  liveliness_lease_duration:\n    {INF}\n"
        "  avoid_ros_namespace_conventions: false\n")
    for cam in CAMERAS:
        add_topic(f"/camera/{cam}/image_raw/compressed", "sensor_msgs/msg/CompressedImage", sensor_qos)
        add_topic(f"/camera/{cam}/camera_info", "sensor_msgs/msg/CameraInfo", sensor_qos)
    add_topic("/tf", "tf2_msgs/msg/TFMessage", tf_qos)
    add_topic("/tf_static", "tf2_msgs/msg/TFMessage", tf_qos)

    ns_to_ns = 1_000  # nuScenes timestamps are µs -> ns

    # ---- static TF: base_link -> camera_* (sensor2ego) -----------------------
    # CRITICAL: stamps must be NON-ZERO. tf2 treats a transform with stamp 0 as
    # "latest for all time", which would make every per-frame /tf update look
    # like TF_OLD_DATA and be ignored. Use 0.5s, just before the first frame
    # (t0 = 1.0s), so per-frame entries at >=1.0s are always accepted.
    static_stamp_ns = 500_000_000
    static_tfs = TFMessage()
    for cam in CAMERAS:
        ch = CHANNEL[cam]
        # reuse the data_infos calibration (same source the BEVDet runtime consumed)
        intr = rec0["cams"][ch]["cam_intrinsic"]
        rot = rec0["cams"][ch]["sensor2ego_rotation"]
        trans = rec0["cams"][ch]["sensor2ego_translation"]
        st = TransformStamped()
        st.header.stamp.sec = static_stamp_ns // 1_000_000_000
        st.header.stamp.nanosec = static_stamp_ns % 1_000_000_000
        st.header.frame_id = "base_link"
        st.child_frame_id = f"camera_{cam}"
        st.transform.translation = Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2]))
        st.transform.rotation = q_from_ns(rot)
        static_tfs.transforms.append(st)
    writer.write("/tf_static", serialize_message(static_tfs), static_stamp_ns)

    # ---- camera data + per-frame tf ---------------------------------------
    sd_all = json.load(open(os.path.join(ns, "v1.0-mini", "sample_data.json")))
    cam_intr_cache = {}

    n = min(args.frames, len(tokens))
    t0_ns = 1_000_000_000       # bag-local time base (1 s)
    sample_step_ns = 500_000_000  # 2 Hz sample spacing (nuScenes keyframes)
    for rank in range(n):
        token = tokens[rank]
        # normalized playback stamps: absolute 2018-epoch stamps would make
        # `ros2 bag play --loop` schedule a decades-long bag duration
        # (see docs/DATASET.md)
        stamp_ns = t0_ns + rank * sample_step_ns
        tf_msg = TFMessage()
        frame_samples = [d for d in sd_all if d["sample_token"] == token]

        # ego pose chain: map -> odom -> base_link; camera static transforms
        # are ALSO re-published on /tf per frame (volatile QoS) so tf listeners
        # never depend on transient-local QoS of /tf_static.
        # IMPORTANT: /tf is written BEFORE this sample's camera messages so
        # tf lookups at this stamp never race the image delivery.
        egosd = next((d for d in frame_samples
                      if "__CAM_FRONT__" in d["filename"] and d["is_key_frame"]), None)
        ep = egopose[egosd["ego_pose_token"]]
        for st in static_tfs.transforms:
            st2 = TransformStamped()
            st2.header = mk_header("base_link", stamp_ns)
            st2.child_frame_id = st.child_frame_id
            st2.transform = st.transform
            tf_msg.transforms.append(st2)
        for parent, child, rot, trans in (
            ("map", "odom", ep["rotation"], ep["translation"]),
            ("odom", "base_link", [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0])):
            st = TransformStamped()
            st.header = mk_header(parent, stamp_ns)
            st.child_frame_id = child
            st.transform.rotation = q_from_ns(rot)
            st.transform.translation = Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2]))
            tf_msg.transforms.append(st)
        writer.write("/tf", serialize_message(tf_msg), stamp_ns)

        for cam in CAMERAS:
            ch = CHANNEL[cam]
            # delimiter-aware match: "CAM_BACK" is a substring of
            # "CAM_BACK_LEFT"/"CAM_BACK_RIGHT" — must compare on
            # the "__<CHANNEL>__" token, not a bare substring
            needle = f"__{ch}__"
            cam_sd = next((d for d in frame_samples
                           if needle in d["filename"] and d["fileformat"] == "jpg"
                           and d["is_key_frame"]), None)
            if cam_sd is None:
                raise RuntimeError(f"missing {ch} for sample rank {rank}")
            img_path = os.path.join(ns, cam_sd["filename"])

            # CompressedImage
            with open(img_path, "rb") as f:
                jpeg = f.read()
            ci = CompressedImage()
            ci.header = mk_header(f"camera_{cam}", stamp_ns)
            ci.format = "jpeg"
            ci.data = jpeg
            writer.write(f"/camera/{cam}/image_raw/compressed", serialize_message(ci), stamp_ns)

            # CameraInfo (K; P = K|0 for identity rectification)
            cs = calib[cam_sd["calibrated_sensor_token"]]
            k = cs["camera_intrinsic"]
            fx, fy, cx, cy = k[0][0], k[1][1], k[0][2], k[1][2]
            info = CameraInfo()
            info.header = mk_header(f"camera_{cam}", stamp_ns)
            info.height = cam_sd.get("height", 900)
            info.width = cam_sd.get("width", 1600)
            info.distortion_model = ""
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            writer.write(f"/camera/{cam}/camera_info", serialize_message(info), stamp_ns)

        if rank % 20 == 0:
            print(f"  wrote frame {rank}/{n}")

    writer.close()
    print(f"bag written: {args.out} ({n} frames)")
    print(f"play: ros2 bag play {args.out} --loop --rate 1.0")


if __name__ == "__main__":
    main()