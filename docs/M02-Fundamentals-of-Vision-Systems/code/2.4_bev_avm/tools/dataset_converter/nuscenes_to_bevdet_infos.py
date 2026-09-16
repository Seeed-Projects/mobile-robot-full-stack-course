#!/usr/bin/env python3
"""Convert nuScenes v1.0-mini (raw json) to bevdet_vendor data_infos format.

Outputs (into <out_dir>/data_infos):
  time_sequence.yaml           globally-ordered sample index sequence (scene order)
  samples_info/sample%04d.yaml per-sample calibration/pose yaml
Format matches DataLoader/camParams in bevdet_vendor (see src/data.cpp).
"""
import json
import os
import re
import sys
from collections import defaultdict

CAM_NAMES = ["CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT",
             "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"]


def channel_of(sd):
    """Newer nuScenes metadata dropped the 'channel' field; derive from filename."""
    if "channel" in sd and sd["channel"]:
        return sd["channel"]
    m = re.search(r"__([A-Z_]+)__", sd["filename"])
    return m.group(1) if m else None


def qvec(ns_q):
    """nuScenes quaternion [w,x,y,z] -> yaml list [w,x,y,z] (Eigen ctor order)."""
    return [float(ns_q[0]), float(ns_q[1]), float(ns_q[2]), float(ns_q[3])]


def tvec(v):
    return [float(v[0]), float(v[1]), float(v[2])]


def main():
    ns_root = sys.argv[1]
    out_dir = sys.argv[2]
    os.makedirs(f"{out_dir}/data_infos/samples_info", exist_ok=True)

    js = lambda name: json.load(open(f"{ns_root}/v1.0-mini/{name}.json"))

    sample = js("sample")                 # {'token','timestamp','scene_token','next','prev'}
    sample_data = js("sample_data")       # {'token','sample_token','ego_pose_token','calibrated_sensor_token','filename','fileformat','is_key_frame','channel'}
    ego_pose = js("ego_pose")
    calib = js("calibrated_sensor")
    scene = js("scene")
    log = js("log")
    sensor = js("sensor")

    sd_by_token = {d["token"]: d for d in sample_data}
    ep_by_token = {d["token"]: d for d in ego_pose}
    cs_by_token = {d["token"]: d for d in calib}
    scene_by_token = {d["token"]: d for d in scene}
    log_by_token = {d["token"]: d for d in log}

    # order samples: scene (by first sample timestamp) -> timestamp asc
    scenes = sorted(scene, key=lambda s: s["first_sample_token"])
    ordered = []
    for s in scenes:
        scenes_in = [x for x in sample if x["scene_token"] == s["token"]]
        scenes_in.sort(key=lambda x: x["timestamp"])
        for x in scenes_in:
            if x not in ordered:
                ordered.append(x)
    print(f"scenes={len(scenes)} samples={len(ordered)}")

    lidar2ego = None
    time_sequence = []
    for idx, samp in enumerate(ordered):
        # camera sample_data for this sample
        cams = {}
        for cname in CAM_NAMES:
            chan = cname
            hits = [sd for sd in sd_by_token.values()
                    if sd["sample_token"] == samp["token"] and channel_of(sd) == chan
                    and sd["fileformat"] == "jpg" and sd["is_key_frame"]]
            if not hits:
                raise RuntimeError(f"missing {chan} keyframe for sample {samp['token'][:8]}")
            sd = hits[0]
            cs = cs_by_token[sd["calibrated_sensor_token"]]
            k = cs["camera_intrinsic"]
            # new-format: 3x3 matrix [[fx,0,cx],[0,fy,cy],[0,0,1]] (vendor format)
            if len(k) == 1:  # defensive: flat 4/9-vector legacy layout
                k = [[k[0][0], 0.0, k[0][2]], [0.0, k[0][1], k[0][3]], [0.0, 0.0, 1.0]]
            cams[cname] = {
                # data.cpp prepends "."; final string is "./../../datasets/..."
                # which resolves from CWD=tools/build to repo-root/datasets.
                "data_path": f"/../../datasets/nuscenes/{sd['filename']}",
                "cam_intrinsic": k,
                "sensor2ego_rotation": qvec(cs["rotation"]),
                "sensor2ego_translation": tvec(cs["translation"]),
            }
        # ego pose via any camera keyframe of this sample (CAM_FRONT)
        fr_sd = next(sd for sd in sd_by_token.values()
                     if sd["sample_token"] == samp["token"] and channel_of(sd) == "CAM_FRONT"
                     and sd["is_key_frame"])
        ep = ep_by_token[fr_sd["ego_pose_token"]]
        # lidar2ego once (LIDAR_TOP on CAM_FRONT sample)
        if lidar2ego is None:
            lid = sorted(
                [sd for sd in sd_by_token.values()
                 if sd["sample_token"] == samp["token"] and channel_of(sd) == "LIDAR_TOP"],
                key=lambda d: d["timestamp"])[-1]
            lcs = cs_by_token[lid["calibrated_sensor_token"]]
            lidar2ego = {"lidar2ego_rotation": qvec(lcs["rotation"]),
                         "lidar2ego_translation": tvec(lcs["translation"])}

        rec = {
            "ego2global_rotation": qvec(ep["rotation"]),
            "ego2global_translation": tvec(ep["translation"]),
            **lidar2ego,
            "timestamp": int(samp["timestamp"]),
            "scene_token": samp["scene_token"],
            "cams": cams,
        }
        with open(f"{out_dir}/data_infos/samples_info/sample{idx:04d}.yaml", "w") as f:
            json.dump(rec, f, indent=2)
        time_sequence.append(idx)
        if idx % 50 == 0:
            print(f"  wrote sample{idx:04d}.yaml")

    with open(f"{out_dir}/data_infos/time_sequence.yaml", "w") as f:
        f.write("time_sequence: " + json.dumps(time_sequence) + "\n")
    # ordering sidecar so evaluators reproduce the exact sample ranking
    with open(f"{out_dir}/data_infos/sample_tokens.json", "w") as f:
        json.dump([s["token"] for s in ordered], f, indent=1)
    print(f"done: {len(time_sequence)} samples -> {out_dir}/data_infos")


if __name__ == "__main__":
    main()