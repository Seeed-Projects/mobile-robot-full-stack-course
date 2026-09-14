#!/usr/bin/env python3
"""Phase 0 evidence generator: parse lidarbox txt -> BEV debug png + inference_result.json"""
import json, sys, math

BOXES = sys.argv[1]
OUT_PNG = sys.argv[2]
OUT_JSON = sys.argv[3]

# nuScenes class palette (label id -> (name, color))
CLS = {
    0: ("car", "#1f77b4"),
    1: ("truck", "#ff7f0e"),
    2: ("construction_vehicle", "#2ca02c"),
    3: ("bus", "#d62728"),
    4: ("trailer", "#9467bd"),
    5: ("barrier", "#8c564b"),
    6: ("motorcycle", "#e377c2"),
    7: ("bicycle", "#7f7f7f"),
    8: ("pedestrian", "#17becf"),
    9: ("traffic_cone", "#bcbd22"),
}

boxes = []
for line in open(BOXES):
    p = line.split()
    if len(p) < 9:
        continue
    x, y, z, l, w, h, r, s, c = (float(p[0]), float(p[1]), float(p[2]),
                                 float(p[3]), float(p[4]), float(p[5]),
                                 float(p[6]), float(p[7]), int(p[8]))
    boxes.append(dict(x=x, y=y, z=z, l=l, w=w, h=h, yaw=r, score=s, class_id=c))

# --- matplotlib figure: ego footprint + objects (filter score>=0.25 for readability)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

fig, ax = plt.subplots(figsize=(11, 11))
ax.set_aspect("equal")
ax.set_title("Phase0 BEVDet FP16 — BEV lidar-frame view (score>=0.25)\n"
             "6-camera sample scene n015-2018-08-02-17-16-37", fontsize=11)
ax.set_xlabel("x (m, forward)")
ax.set_ylabel("y (m, left)")
ax.grid(True, lw=0.4)

shown = 0
for b in boxes:
    if b["score"] < 0.25:
        continue
    shown += 1
    _, color = CLS.get(b["class_id"], ("?", "#999999"))
    # lidar frame: x FWD, y LEFT ; swap to plot-xy
    rect = Rectangle((b["x"] - b["l"] / 2, -b["y"] - b["w"] / 2),
                     b["l"], b["w"], angle=math.degrees(b["yaw"]),
                     rotation_point="center", fill=False, edgecolor=color,
                     linewidth=1.2, label=None)
    ax.add_patch(rect)
    ax.annotate(f"{b['score']:.2f}", (b["x"], -b["y"]), fontsize=6, color=color)

# ego footprint origin (lidar frame): lidar2ego = (0.944, 0, 1.84) -> ego at (-0.944, 0)
ax.add_patch(Rectangle((-0.944 - 2.4, -1.2), 4.8, 2.4, fill=False,
                       edgecolor="red", linewidth=2))
ax.annotate("ego", (-0.944, 0.2), color="red", fontsize=10)

ax.set_xlim(-60, 60)
ax.set_ylim(-60, 60)  # swapped y
leg = [plt.Line2D([0], [0], color=c, lw=2, label=n) for i, (n, c) in CLS.items()]
ax.legend(handles=leg, loc="upper right", fontsize=8)
fig.tight_layout()
fig.savefig(OUT_PNG, dpi=110)
print(f"wrote {OUT_PNG} ({shown} boxes shown)")

result = {
    "sample": "sample0 (n015-2018-08-02-17-16-37+0800, ts=1533201470448696)",
    "model": "bevdet_one_lt_d",
    "precision": "fp16",
    "tensorrt": "10.3.0",
    "device": "AGX Orin 32GB (sm_87)",
    "num_detections": len(boxes),
    "num_detections_score_ge_0.25": shown,
    "score_stats": {
        "min": min(b["score"] for b in boxes),
        "median": sorted(b["score"] for b in boxes)[len(boxes) // 2],
        "max": max(b["score"] for b in boxes),
    },
    "reference_boxes_in_repo": 104,
    "detections": boxes,
}
json.dump(result, open(OUT_JSON, "w"), indent=1)
print(f"wrote {OUT_JSON}")