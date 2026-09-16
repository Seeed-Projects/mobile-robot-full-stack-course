#!/usr/bin/env python3
"""Phase 1 offline validation: TRT detections vs nuScenes-mini GT.

Inputs:
  <nuscenes_root>/v1.0-mini/*.json        GT (global frame)
  <pred_dir>/bevdet_egoboxes_%d.txt       TRT detections (ego frame, x y z l w h r score class [vx vy])
  <lidar2ego yaml>                         offset for ego->lidar (from data_infos sample0000.yaml)
  <time_sequence yaml>                     sample_idx -> rank mapping (data_infos/time_sequence.yaml)

Outputs:
  accuracy_comparison.json
  bev_frames/*.png                        GT vs pred overlay
"""
import json, os, sys, math, re
import numpy as np
import yaml

NS_ROOT, PRED_DIR, OUT_JSON = sys.argv[1], sys.argv[2], sys.argv[3]
SCORE_THR = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
OUT_PNG_DIR = os.path.join(os.path.dirname(OUT_JSON), "bev_frames")

# --- class map (model 10-class) -------------------------------------------
CATEGORY2CLASS = {
    "vehicle.car": 0, "vehicle.truck": 1, "vehicle.construction": 2,
    "vehicle.bus": 3, "vehicle.trailer": 4, "movable_object.barrier": 5,
    "vehicle.motorcycle": 6, "vehicle.bicycle": 7,
    "human.pedestrian.adult": 8, "human.pedestrian.child": 8,
    "human.pedestrian.construction_worker": 8, "human.pedestrian.police_officer": 8,
    "human.pedestrian.stroller": 8, "human.pedestrian.personal_mobility": 8,
    "human.pedestrian.wheelchair": 8,
    "movable_object.trafficcone": 9,
}
CLASS_NAMES = ["car","truck","construction_vehicle","bus","trailer","barrier",
               "motorcycle","bicycle","pedestrian","traffic_cone"]
SMALL_CLS = {6, 7, 8, 9}

def load_json(n): return json.load(open(f"{NS_ROOT}/v1.0-mini/{n}.json"))

def quat_mul(q, r):
    w1, x1, y1, z1 = q; w2, x2, y2, z2 = r
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2,
                     w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2])

def quat_rot(q, v):
    w, x, y, z = q
    return quat_mul(quat_mul(q, [0, v[0], v[1], v[2]]),
                    [w, -x, -y, -z])[1:]

def quat_inv(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z]) / (w*w+x*x+y*y+z*z)

def yaw_of(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))

def wrap(deg):
    while deg > 180: deg -= 360
    while deg < -180: deg += 360
    return deg

def _plot_frame(rank, preds, gt_lid, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect("equal"); ax.grid(True, lw=0.3)
    ax.set_title(f"frame {rank}: green=GT red=pred")
    for g in gt_lid:
        ax.add_patch(Rectangle((g["xyz"][0]-g["size"][0]/2, g["xyz"][1]-g["size"][1]/2),
                               g["size"][0], g["size"][1], angle=g["yaw"],
                               rotation_point="center", fill=False, edgecolor="green", lw=0.8))
    for p in preds:
        ax.add_patch(Rectangle((p["xyz"][0]-p["size"][0]/2, p["xyz"][1]-p["size"][1]/2),
                               p["size"][0], p["size"][1], angle=p["yaw"],
                               rotation_point="center", fill=False, edgecolor="red", lw=1.0))
    ax.set_xlim(-50, 60); ax.set_ylim(-50, 50)
    fig.savefig(path); plt.close(fig)


# --- GT collection (module level, sequential script) -----------------------
ann = load_json("sample_annotation")
cat = {c["token"]: c["name"] for c in load_json("category")}
inst = {i["token"]: i["category_token"] for i in load_json("instance")}

# GT by sample token, global frame, mapped to 10 classes
# (new mini metadata: annotation -> instance -> category)
gt_by_sample = {}
for a in ann:
    cls = CATEGORY2CLASS.get(cat.get(inst.get(a["instance_token"])))
    if cls is None:
        continue
    gt_by_sample.setdefault(a["sample_token"], []).append({
        "xyz": np.array(a["translation"], dtype=np.float64),
        "size": np.array(a["size"], dtype=np.float64),
        "q": np.array(a["rotation"], dtype=np.float64),
        "cls": cls,
        "v": np.array(a.get("velocity", [0.0, 0.0]), dtype=np.float64),
    })
print(f"GT boxes: {sum(len(v) for v in gt_by_sample.values())} "
      f"across {len(gt_by_sample)} samples")

# sample meta
samples = {s["token"]: s for s in load_json("sample")}
egopose = {e["token"]: {"q": np.array(e["rotation"]), "t": np.array(e["translation"])}
           for e in load_json("ego_pose")}
sd_by_samp = {}
for d in load_json("sample_data"):
    m = re.search(r"__([A-Z_]+)__", d["filename"])
    chan = d.get("channel") or (m.group(1) if m else None)
    if chan == "CAM_FRONT" and d["is_key_frame"]:
        sd_by_samp[d["sample_token"]] = d["ego_pose_token"]

# lidar2ego from data_infos sample0000 (created by converter)
dinfo = f"{NS_ROOT}/data_infos"
rec0 = yaml.safe_load(open(f"{dinfo}/samples_info/sample0000.yaml"))
l2e_q = np.array(rec0["lidar2ego_rotation"], dtype=np.float64)
l2e_t = np.array(rec0["lidar2ego_translation"], dtype=np.float64)
time_seq = yaml.safe_load(open(f"{dinfo}/time_sequence.yaml"))["time_sequence"]
# rank -> sample token: read the converter's ordering sidecar (exact match)
rank2token = json.load(open(f"{dinfo}/sample_tokens.json"))

def ego_to_lidar(xyz, yaw_deg):
    """ego frame (x fwd, y left) -> lidar frame (x fwd, y left, translated + slight rot)."""
    v = xyz - l2e_t
    v = quat_rot(quat_inv(l2e_q), v)
    return v, wrap(yaw_deg - yaw_of(l2e_q))

def global_to_lidar(xyz, q, sample_token):
    e = egopose[sd_by_samp[sample_token]]
    # global -> ego
    gt_global = quat_rot(quat_inv(e["q"]), xyz - e["t"])
    gt_q = quat_mul(quat_inv(e["q"]), q)
    # ego -> lidar
    gt_lid = quat_rot(quat_inv(l2e_q), gt_global - l2e_t)
    gt_lq = quat_mul(quat_inv(l2e_q), gt_q)
    return gt_lid, yaw_of(gt_lq)

def hungarian(cost):
    n, m = cost.shape
    if n == 0 or m == 0: return [], list(range(m))
    col2row = np.full(m, -1)
    row2col = np.full(n, -1)
    for i in range(n):
        cand = np.argsort(cost[i])
        for j in cand:
            if cost[i, j] > 1e9: break
            if col2row[j] == -1:
                col2row[j] = i; row2col[i] = j; break
            j0, jj = col2row[j], j
            if cost[j0, jj] > cost[i, j]:  # simple swap improvement
                row2col[j0] = -1; col2row[j] = i; row2col[i] = j
                break
    pairs = [(i, j) for i, j in enumerate(row2col) if j >= 0]
    unassigned = [j for j in range(m) if col2row[j] == -1]
    return pairs, unassigned

# --- evaluate --------------------------------------------------------------
stats = {"frames": {}, "global": {}}
cent_errs, yaw_errs, size_errs, cls_tp, cls_gt, cls_pred = [], [], [], {}, {}, {}
yaw_by_cls = {}
match_counts = []
per_frame = []

frames_to_plot = []

for rank, token in enumerate(rank2token):
    fname = f"{PRED_DIR}/bevdet_egoboxes_{rank}.txt"
    if not os.path.exists(fname):
        continue
    gt = gt_by_sample.get(token)
    if gt is None:
        print(f"[warn] no GT for rank {rank} {token}")
        continue

    preds = []
    for line in open(fname):
        p = line.split()
        if len(p) == 11:      # with_vel: x y z l w h r vx vy score label
            x, y, z, l, w, h, r, vx, vy, s, c = (float(p[0]), float(p[1]), float(p[2]),
                float(p[3]), float(p[4]), float(p[5]), float(p[6]),
                float(p[7]), float(p[8]), float(p[9]), int(p[10]))
        elif len(p) == 9:     # no vel: x y z l w h r score label
            x, y, z, l, w, h, r, s, c = (float(p[0]), float(p[1]), float(p[2]),
                float(p[3]), float(p[4]), float(p[5]), float(p[6]),
                float(p[7]), int(p[8]))
        else:
            continue
        if s < SCORE_THR: continue
        # TestNuscenes writes EG0-frame boxes; convert ego -> lidar (like demo's
        # Egobox2Lidarbox) so it is comparable with GT in lidar frame.
        xyz_l, yaw_l = ego_to_lidar(np.array([x, y, z]), math.degrees(r))
        preds.append({"xyz": xyz_l, "size": np.array([l, w, h]),
                      "yaw": yaw_l, "score": s, "cls": c})

    # GT in lidar frame
    gt_lid = []
    for g in gt:
        xyz, yaw = global_to_lidar(g["xyz"], g["q"], token)
        gt_lid.append({"xyz": xyz, "size": g["size"], "yaw": yaw, "cls": g["cls"], "v": g["v"]})

    # match: same class, BEV center distance <= 2m (1m small)
    cost = np.full((len(preds), len(gt_lid)), 1e10)
    for i, p in enumerate(preds):
        for j, g in enumerate(gt_lid):
            if p["cls"] != g["cls"]: continue
            d = float(np.hypot(*(p["xyz"][:2] - g["xyz"][:2])))
            thr = 1.0 if p["cls"] in SMALL_CLS else 2.0
            if d <= thr: cost[i, j] = d
    pairs, unmatched = hungarian(cost)
    tp = len(pairs)
    for i, j in pairs:
        p, g = preds[i], gt_lid[j]
        ye = abs(wrap(p["yaw"] - g["yaw"]))
        cent_errs.append(cost[i, j])
        yaw_errs.append(ye)
        yaw_by_cls.setdefault(p["cls"], []).append(ye)
        size_errs.append(np.abs(p["size"] - g["size"]))
        cls_tp[p["cls"]] = cls_tp.get(p["cls"], 0) + 1
    for g in gt_lid:
        cls_gt[g["cls"]] = cls_gt.get(g["cls"], 0) + 1
    for p in preds:
        cls_pred[p["cls"]] = cls_pred.get(p["cls"], 0) + 1
    match_counts.append((len(gt_lid), len(preds), tp))
    per_frame.append({
        "rank": rank, "gt": len(gt_lid), "pred": len(preds), "matched": tp,
        "match_frac_of_gt": tp / len(gt_lid) if gt_lid else 0.0,
    })

    if rank in frames_to_plot:
        os.makedirs(OUT_PNG_DIR, exist_ok=True)
        _plot_frame(rank, preds, gt_lid, f"{OUT_PNG_DIR}/frame_{rank:03d}.png")

g_gt, g_pred, g_tp = map(sum, zip(*match_counts)) if match_counts else (0,0,0)
stats["global"] = {
    "frames_evaluated": len(per_frame),
    "gt_boxes": g_gt, "pred_boxes": g_pred, "matched_boxes": g_tp,
    "match_frac_of_gt": g_tp / g_gt if g_gt else 0,
    "mean_center_err_m": float(np.mean(cent_errs)) if cent_errs else None,
    "p50_center_err_m": float(np.percentile(cent_errs, 50)) if cent_errs else None,
    "p95_center_err_m": float(np.percentile(cent_errs, 95)) if cent_errs else None,
    "max_center_err_m": float(np.max(cent_errs)) if cent_errs else None,
    "mean_yaw_err_deg": float(np.mean(yaw_errs)) if yaw_errs else None,
    "p95_yaw_err_deg": float(np.percentile(yaw_errs, 95)) if yaw_errs else None,
    "mean_size_err_m": [float(x) for x in np.mean(size_errs, axis=0)] if size_errs else None,
    "per_class": {CLASS_NAMES[c]: {"gt": cls_gt.get(c,0), "pred": cls_pred.get(c,0),
                                   "matched": cls_tp.get(c,0)}
                  for c in sorted(set(cls_gt) | set(cls_pred))},
    "per_class_yaw_err_deg": {CLASS_NAMES[c]: {
        "mean": float(np.mean(v)), "p95": float(np.percentile(v, 95)), "n": len(v)}
        for c, v in sorted(yaw_by_cls.items())},
}
stats["frames"] = per_frame
json.dump(stats, open(OUT_JSON, "w"), indent=1)
print(json.dumps(stats["global"], indent=1))