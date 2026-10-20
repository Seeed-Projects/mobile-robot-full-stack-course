"""SlowFast multi-tier history sampling.

A SlowFast tier set is a list of tier dicts describing how to sample history frames by
*age* (age=0 is the current/most-recent frame, increasing into the past, in native
frames). Sampling modes:

- ``dense``:  every frame in [age_lo, age_hi]                  (current / fast tiers)
- ``burst``:  adjacent frame pairs (a, a+1) every ``pair_stride`` frames from age_lo
- ``span``:   ``num_pairs`` adjacent pairs spread uniformly over [age_lo, reachable_hi],
              anchoring the oldest pair to the episode start when within reach
- ``anchor``: the absolute episode-start frames [0 .. num_frames-1]

``pool_spatial`` / ``pool_mode`` are the per-tier spatial pooling applied downstream.

Training and inference MUST derive their segment layout from :func:`slowfast_segments`
so the two paths are bit-identical: absolute frame ids, oldest segment first, dense
frames take priority over anchor frames, burst/span pairs that hit an anchored frame are
dropped whole, and odd dense segments are padded to even length by duplicating the
newest frame.
"""

from typing import Any, Dict, List, Optional

# Reference tier sets, kept as documented examples of the schema (256-frame reach @ 4 fps).
DEFAULT_SLOWFAST_TIERS: List[Dict[str, Any]] = [
    {"name": "current", "age_lo": 0, "age_hi": 1, "mode": "dense", "pool_spatial": 1},
    {"name": "fast", "age_lo": 2, "age_hi": 17, "mode": "dense", "pool_spatial": 2},
    {"name": "mid", "age_lo": 18, "age_hi": 89, "mode": "burst", "pair_stride": 6, "pool_spatial": 2},
    {"name": "long", "age_lo": 90, "age_hi": 255, "mode": "burst", "pair_stride": 12, "pool_spatial": 4},
]

# Variant with an "anchor" tier that pins the absolute episode start (frames 0, 1) so it
# is ALWAYS in the input regardless of episode length / stride boundaries, and a "span"
# long tier that adaptively covers the far-middle at a fixed pair budget.
DEFAULT_SLOWFAST_TIERS_SPAN: List[Dict[str, Any]] = [
    {"name": "current", "age_lo": 0, "age_hi": 1, "mode": "dense", "pool_spatial": 1},
    {"name": "fast", "age_lo": 2, "age_hi": 17, "mode": "dense", "pool_spatial": 2},
    {"name": "mid", "age_lo": 18, "age_hi": 89, "mode": "burst", "pair_stride": 6, "pool_spatial": 2},
    {"name": "long", "age_lo": 90, "age_hi": 1_000_000, "mode": "span", "num_pairs": 14, "pool_spatial": 4},
    {"name": "anchor", "age_lo": 0, "age_hi": 0, "mode": "anchor", "num_frames": 2, "pool_spatial": 4},
]


def validate_slowfast_tiers(tiers: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Validate + normalize a SlowFast tier list. Raises ValueError on bad config.

    Returns a new list with defaults filled (`pool_mode="avg"`, `name`).
    Enforces: integer non-negative ages, age_lo<=age_hi, non-overlapping ranges
    ordered by age, pool_spatial>=1, burst pair_stride>=2, span num_pairs>=1,
    anchor num_frames>=1, mode in {dense,burst,span,anchor}.

    `span` is an adaptive-stride memory tier: it places `num_pairs` adjacent frame
    pairs uniformly across [age_lo, min(age_hi, episode_start)], anchoring the oldest
    pair to the episode start (frame 0) when within reach. Its stride depends on the
    episode position, so it needs the number of available frames; set age_hi large to
    always reach frame 0.
    """
    if not isinstance(tiers, list) or len(tiers) == 0:
        raise ValueError("slowfast_tiers must be a non-empty list of tier dicts")
    norm: List[Dict[str, Any]] = []
    prev_hi = -1
    for i, t in enumerate(tiers):
        lo, hi = int(t["age_lo"]), int(t["age_hi"])
        mode = str(t["mode"])
        pool = int(t["pool_spatial"])
        if mode not in {"dense", "burst", "span", "anchor"}:
            raise ValueError(f"tier[{i}].mode must be 'dense', 'burst', 'span' or 'anchor', got {mode!r}")
        if pool < 1:
            raise ValueError(f"tier[{i}].pool_spatial must be >= 1, got {pool}")
        entry = {
            "name": str(t.get("name", f"tier{i}")),
            "age_lo": lo,
            "age_hi": hi,
            "mode": mode,
            "pool_spatial": pool,
            "pool_mode": str(t.get("pool_mode", "avg")),
        }
        if mode == "anchor":
            # Absolute episode-start frames [0..num_frames-1]; unconditionally
            # present so frame 0 never drops out (age ranges are ignored).
            nf = int(t.get("num_frames", 2))
            if nf < 1:
                raise ValueError(f"tier[{i}].num_frames must be >= 1 for anchor, got {nf}")
            entry["num_frames"] = nf
            norm.append(entry)
            continue  # anchor is absolute -> skip age-range ordering checks
        if lo < 0 or hi < lo:
            raise ValueError(f"tier[{i}] requires 0 <= age_lo <= age_hi, got [{lo},{hi}]")
        if lo <= prev_hi:
            raise ValueError(f"tier[{i}] age_lo={lo} overlaps previous tier (prev age_hi={prev_hi}); tiers must be ordered and disjoint")
        if mode == "burst":
            ps = int(t.get("pair_stride", 2))
            if ps < 2:
                raise ValueError(f"tier[{i}].pair_stride must be >= 2 for burst, got {ps}")
            entry["pair_stride"] = ps
        elif mode == "span":
            npairs = int(t.get("num_pairs", 1))
            if npairs < 1:
                raise ValueError(f"tier[{i}].num_pairs must be >= 1 for span, got {npairs}")
            entry["num_pairs"] = npairs
        norm.append(entry)
        prev_hi = hi
    return norm


def _span_pair_starts(age_lo: int, age_hi: int, num_pairs: int, n_available: int) -> List[int]:
    """Adaptive pair-start ages for a span tier: `num_pairs` pairs uniformly over
    [age_lo, reachable_hi], where reachable_hi anchors the episode start (frame 0)
    when within age_hi reach. Enforces >=2 spacing so pairs never share a frame.
    """
    reachable_hi = min(age_hi, n_available - 2)  # need pair (a, a+1) -> a+1 <= n_available-1
    if reachable_hi < age_lo:
        return []
    if num_pairs <= 1 or reachable_hi == age_lo:
        return [reachable_hi]  # oldest only (anchor)
    span = reachable_hi - age_lo
    raw = sorted({age_lo + round(i * span / (num_pairs - 1)) for i in range(num_pairs)})
    starts: List[int] = []
    for s in raw:
        if not starts or s - starts[-1] >= 2:
            starts.append(s)
    return starts


def _tier_ages(tier: Dict[str, Any], n_available: Optional[int]) -> List[int]:
    """Ages this tier keeps. `n_available` caps to frames that exist (age < n);
    None means no cap (full fixed pattern).

    dense -> consecutive ages; burst -> adjacent pairs (dropped whole when the
    older member is unavailable so temporal_patch_size=2 pairing stays aligned);
    span -> adaptive adjacent pairs, requires n_available.
    """
    lo, hi = tier["age_lo"], tier["age_hi"]
    mode = tier["mode"]
    if mode == "span":
        if n_available is None:
            raise ValueError(
                "span tier needs the episode position (n_available); it is not "
                "expressible as a fixed age pattern."
            )
        ages: List[int] = []
        for a in _span_pair_starts(lo, hi, tier["num_pairs"], n_available):
            ages.extend((a, a + 1))
        return ages
    avail = (lambda a: True) if n_available is None else (lambda a: a < n_available)
    if mode == "dense":
        return [a for a in range(lo, hi + 1) if avail(a)]
    ages = []
    for a in range(lo, hi + 1, tier["pair_stride"]):
        if avail(a) and avail(a + 1):
            ages.extend((a, a + 1))
    return ages


def slowfast_segments(
    current_abs: int,
    n_available: int,
    tiers: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Shared (train + inference) segment layout for SlowFast history.

    Args:
        current_abs: absolute episode frame index of the current frame (0-based).
        n_available: number of frames available going back from current
            (= current_abs + 1 when the window reaches episode start).
        tiers: validated tier list.

    Returns a chronologically ordered (oldest segment first) list of:
        {"frame_ids": [abs ids, ascending], "pool_spatial", "pool_mode", "tier"}
    Each segment has an even frame count (dense padded by duplicating the newest
    frame; burst/span even by construction). `frame_ids` are ABSOLUTE episode
    indices -> timestamps = idx/fps reflect true elapsed time, and ViT-cache keys
    are stable across steps.

    `anchor` tiers emit the absolute episode-start frames [0..num_frames-1] so
    frame 0 is never dropped regardless of episode length or stride boundaries --
    EXCEPT frames a dense tier (current/fast) already covers this step: dense
    carries the highest-resolution view, and letting anchor reserve frames 0/1
    away from the pool1 'current' tier would demote the live observation to a
    pool4 thumbnail for the first two episode steps. Burst/span tiers still exclude
    anchored frames; a pair that hits one is dropped whole (keeps
    temporal_patch_size=2 pairing aligned). Empty tiers are skipped; at least one
    segment always survives (current and/or anchor).
    """
    # Frames the dense tiers emit this step (cannot overlap each other or
    # burst/span: non-anchor age ranges are validated disjoint).
    dense_ids: set = set()
    for tier in tiers:
        if tier["mode"] == "dense":
            dense_ids.update(current_abs - a for a in _tier_ages(tier, n_available))

    # Frames claimed by anchor tiers (absolute episode-start), reserved from
    # burst/span. Dense takes priority over anchor (see docstring).
    anchor_ids: set = set()
    for tier in tiers:
        if tier["mode"] == "anchor":
            anchor_ids.update(
                i for i in range(tier["num_frames"]) if i <= current_abs and i not in dense_ids
            )

    segments: List[Dict[str, Any]] = []
    for ti, tier in enumerate(tiers):
        mode = tier["mode"]
        if mode == "anchor":
            ids = sorted(i for i in anchor_ids if i < tier["num_frames"])
        elif mode == "dense":
            ids = sorted(current_abs - a for a in _tier_ages(tier, n_available))
        else:  # burst / span: ages arrive as adjacent pairs; drop a pair that hits an anchor
            ages = _tier_ages(tier, n_available)
            ids = []
            for k in range(0, len(ages), 2):
                f0, f1 = current_abs - ages[k], current_abs - ages[k + 1]
                if f0 in anchor_ids or f1 in anchor_ids:
                    continue
                ids.extend((f0, f1))
            ids.sort()
        if not ids:
            continue
        if len(ids) % 2 == 1:
            ids.append(ids[-1])  # pad-to-even: duplicate newest (matches processor)
        segments.append(
            {
                "frame_ids": ids,
                "pool_spatial": tier["pool_spatial"],
                "pool_mode": tier["pool_mode"],
                "tier": ti,
            }
        )
    if not segments:
        raise ValueError(f"slowfast_segments produced no segments (current_abs={current_abs}, n_available={n_available})")
    segments.sort(key=lambda s: s["frame_ids"][0])  # oldest segment first
    return segments


def slowfast_video_segments(
    episode_frames,
    current_abs: int,
    n_available: int,
    tiers: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Build per-tier video_segment dicts by indexing a full-episode frame tensor.

    `episode_frames` is the whole episode (N, C, H, W) (torch or numpy); indexing
    with absolute frame ids gives each tier its frames. `frame_indices` are kept
    ABSOLUTE so downstream timestamps (idx/fps) reflect true elapsed time and the
    ViT cache keys stay stable across steps.

    Returns segments (oldest first) ready for the Qwen3-VL processor (caller still
    resizes each ``video``):
        {"video", "frame_indices", "total_frames", "pool_spatial", "pool_mode"}
    """
    segs = slowfast_segments(current_abs, n_available, tiers)
    total = int(episode_frames.shape[0])
    out: List[Dict[str, Any]] = []
    for s in segs:
        video = episode_frames[s["frame_ids"]]
        out.append(
            {
                "video": video,
                "frame_indices": list(s["frame_ids"]),
                "total_frames": total,
                "pool_spatial": s["pool_spatial"],
                "pool_mode": s["pool_mode"],
            }
        )
    return out
