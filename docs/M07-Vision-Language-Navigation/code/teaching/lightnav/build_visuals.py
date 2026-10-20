#!/usr/bin/env python3
"""Build the LightNav-0 explainer animations offline. No model, GPU or navigation.

Five 16-second GIFs plus one static poster each, drawn deterministically with Pillow:

  01 decision loop   images -> 5 tokens -> RVQ decode -> 10 waypoints -> control
  02 pointing grid   pixel -> 48x27 cell -> one token id, and back
  03 RVQ residual    three codewords -> one trajectory; stop is a code, not a magnitude
  04 SE(2) compose   per-step deltas in the previous frame -> absolute chunk poses
  05 training        embodied reasoning -> SFT + DAgger -> online RL (GRPO)

Numbers come from three frozen places: ``example-config.json`` (grid/vocabulary/RVQ
bundle constants), ``rvq-teaching-example.json`` (one decoded code triple, produced by
``extract_rvq_example.py`` from the released bundle) and ``lightnav_reference.py`` (the
released encode/decode/compose functions). Nothing here is a measurement of the running
system, and the scene artwork is the same image_gen atlas used by 7.2.3.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps, ImageSequence

HERE = Path(__file__).resolve().parent
W, H, FPS, SECONDS = 1200, 675, 10, 16
BG = '#f5f6f2'
INK = '#193c40'
MUTED = '#60767a'
EDGE = '#dce4df'
PANEL = '#ffffff'
C_APOS = '#d27336'
C_OPOS = '#147f7b'
C_ACT = '#427eae'
C_L0 = '#687681'
C_L1 = '#427eae'
C_L2 = '#147f7b'
C_RL = '#8665a9'
FONT_PATHS = [Path('/System/Library/Fonts/Supplemental/Arial.ttf'),
              Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')]
BOLD_PATHS = [Path('/System/Library/Fonts/Supplemental/Arial Bold.ttf'),
              Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')]
FONT_CACHE = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in FONT_CACHE:
        paths = BOLD_PATHS if bold else FONT_PATHS
        candidate = next((p for p in paths if p.is_file()), None)
        if candidate is None:
            raise RuntimeError('Install Arial or DejaVu Sans, or edit FONT_PATHS.')
        FONT_CACHE[key] = ImageFont.truetype(str(candidate), size)
    return FONT_CACHE[key]


def text(d, xy, value, size=20, color=INK, bold=False, anchor=None):
    d.text(xy, str(value), font=font(size, bold), fill=color, anchor=anchor)


def width_of(value, size=20, bold=False):
    return font(size, bold).getlength(str(value))


def panel(d, box, fill=PANEL, outline=EDGE, radius=14, width=1):
    d.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def clamp(t):
    return min(1., max(0., t))


def fade(a, b, k):
    def rgb(c):
        return tuple(int(c[i:i + 2], 16) for i in (1, 3, 5))
    return tuple(round(x + (y - x) * clamp(k)) for x, y in zip(rgb(a), rgb(b)))


def reveal(sec, start, dur=0.6):
    return clamp((sec - start) / dur)


def header(d, eyebrow, title, subtitle):
    text(d, (40, 21), eyebrow, 14, MUTED, True)
    text(d, (40, 45), title, 31, INK, True)
    text(d, (40, 88), subtitle, 19, MUTED)


def picture(im, img, box, radius=10):
    x, y, w, h = box
    fitted = ImageOps.fit(img, (w, h), Image.Resampling.LANCZOS)
    mask = Image.new('L', (w, h))
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, w - 1, h - 1), radius=radius, fill=255)
    im.paste(fitted, (x, y), mask)


def arrow(d, p0, p1, color=INK, width=3, head=9):
    x0, y0 = p0
    x1, y1 = p1
    ang = math.atan2(y1 - y0, x1 - x0)
    bx, by = x1 - head * math.cos(ang), y1 - head * math.sin(ang)
    d.line((x0, y0, bx, by), fill=color, width=width)
    d.polygon([(x1, y1),
               (bx - head * 0.5 * math.sin(ang), by + head * 0.5 * math.cos(ang)),
               (bx + head * 0.5 * math.sin(ang), by - head * 0.5 * math.cos(ang))], fill=color)


def dashed(d, points, color=MUTED, width=2, dash=8, gap=6):
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        if length == 0:
            continue
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        t = 0.0
        while t < length:
            e = min(t + dash, length)
            d.line((x0 + ux * t, y0 + uy * t, x0 + ux * e, y0 + uy * e), fill=color, width=width)
            t = e + gap


def chip(d, box, label, fill, color='white', size=14, bold=True, outline=None, radius=8):
    panel(d, box, fill=fill, outline=outline or fill, radius=radius)
    cx, cy = (box[0] + box[2]) // 2, (box[1] + box[3]) // 2
    text(d, (cx, cy), label, size, color, bold, anchor='mm')


def bar(d, box, frac, fill, back='#e7ece8', radius=5):
    x0, y0, x1, y1 = box
    d.rounded_rectangle(box, radius=radius, fill=back)
    w = (x1 - x0) * clamp(frac)
    if w > 2:
        d.rounded_rectangle((x0, y0, x0 + w, y1), radius=radius, fill=fill)


def beat(t, sec):
    """Caption text for the current time from a (start, text) table."""
    current = t[0][1]
    for start, value in t:
        if sec >= start:
            current = value
    return current


class Frame2D:
    """Top-down metric plot: world +x (forward) points up, +y (left) points screen-left."""

    def __init__(self, box, xlim, ylim):
        x0, y0, x1, y1 = box
        self.box = box
        self.xlim = xlim
        self.ylim = ylim
        self.s = min((x1 - x0) / (xlim[1] - xlim[0]), (y1 - y0) / (ylim[1] - ylim[0]))
        self.cx = (x0 + x1) / 2
        self.cy = (y0 + y1) / 2
        self.xc = (xlim[0] + xlim[1]) / 2
        self.yc = (ylim[0] + ylim[1]) / 2

    @classmethod
    def fit(cls, box, points, pad=0.18, min_range=0.5):
        """Frame that fits ``points`` with equal m/px on both axes and no overflow.

        World +x maps to screen vertical and +y to screen horizontal, so the x extent has
        to fit the box HEIGHT and the y extent the box WIDTH. Sizing both by width (or by
        whatever happens to be smaller) pushes the far end of a forward path off the top.
        """
        xs = [float(p[0]) for p in points] or [0.0]
        ys = [float(p[1]) for p in points] or [0.0]
        x0, y0, x1, y1 = box
        w, h = x1 - x0, y1 - y0
        xr = max(max(xs) - min(xs), min_range) * (1 + 2 * pad)
        yr = max(max(ys) - min(ys), min_range) * (1 + 2 * pad)
        s = min(w / yr, h / xr)
        xr, yr = h / s, w / s
        cx = (max(xs) + min(xs)) / 2
        cy = (max(ys) + min(ys)) / 2
        return cls(box, (cx - xr / 2, cx + xr / 2), (cy - yr / 2, cy + yr / 2))

    def xy(self, x, y):
        return (self.cx - self.s * (y - self.yc), self.cy - self.s * (x - self.xc))

    def grid(self, d, step=0.25, color='#e9efe8'):
        x0, y0, x1, y1 = self.box
        n = int(math.floor(self.xlim[1] / step))
        for i in range(-n, n + 1):
            v = i * step
            if self.xlim[0] <= v <= self.xlim[1]:
                p0, p1 = self.xy(v, self.ylim[0]), self.xy(v, self.ylim[1])
                d.line((*p0, *p1), fill=color, width=1)
            if self.ylim[0] <= v <= self.ylim[1]:
                p0, p1 = self.xy(self.xlim[0], v), self.xy(self.xlim[1], v)
                d.line((*p0, *p1), fill=color, width=1)

    def origin(self, d):
        p = self.xy(0, 0)
        d.line((p[0] - 5, p[1], p[0] + 5, p[1]), fill=INK, width=2)
        d.line((p[0], p[1] - 5, p[0], p[1] + 5), fill=INK, width=2)

    def scalebar(self, d, metres=0.5, label=None, axis='y'):
        """Bar in the lower-right corner of the plot, drawn at the plot's own scale.

        ``axis='y'`` draws it horizontally (a lateral extent), ``axis='x'`` vertically.
        It is a length reference only: no axis ticks or numeric labels are implied.
        """
        x_edge, y_edge = self.xlim[1] - 0.12, self.ylim[0] + 0.10
        if axis == 'y':
            p0, p1 = self.xy(x_edge - metres, y_edge), self.xy(x_edge, y_edge)
        else:
            p0, p1 = self.xy(x_edge, y_edge), self.xy(x_edge, y_edge + metres)
        d.line((*p0, *p1), fill=INK, width=2)
        for p in (p0, p1):
            d.line((p[0] - 4, p[1] - 4, p[0] + 4, p[1] + 4), fill=INK, width=2)
            d.line((p[0] - 4, p[1] + 4, p[0] + 4, p[1] - 4), fill=INK, width=2)
        ly = p1[1] + 4 if axis == 'y' else p0[1] - 22
        text(d, ((p0[0] + p1[0]) / 2, ly), label or f'{metres:.1f} m', 11, MUTED, False, 'ma')


def robot(d, plot, x, y, theta, size=0.075, color=INK):
    """Triangle at pose (x, y, theta) in the chunk-start frame."""
    tip = plot.xy(x + size * math.cos(theta), y + size * math.sin(theta))
    c = plot.xy(x, y)
    left = plot.xy(x - 0.4 * size * math.cos(theta) + 0.55 * size * math.sin(theta),
                   y - 0.4 * size * math.sin(theta) - 0.55 * size * math.cos(theta))
    right = plot.xy(x - 0.4 * size * math.cos(theta) - 0.55 * size * math.sin(theta),
                    y - 0.4 * size * math.sin(theta) + 0.55 * size * math.cos(theta))
    d.polygon([tip, left, right], fill=color)
    d.line((*c, *tip), fill=BG, width=1)


# ── data ──────────────────────────────────────────────────────────────────────
def load_reference():
    path = HERE / 'lightnav_reference.py'
    spec = importlib.util.spec_from_file_location('m07_lightnav_reference', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_data():
    cfg = json.loads((HERE / 'example-config.json').read_text())
    rvq = json.loads((HERE / 'rvq-teaching-example.json').read_text())
    for name, key in (('lightnav_reference.py', 'reference_sha256'),
                      ('rvq-teaching-example.json', 'rvq_example_sha256')):
        got = hashlib.sha256((HERE / name).read_bytes()).hexdigest()
        if got != cfg[key]:
            raise SystemExit(f'{name} hash {got} != example-config.json {key}; regenerate the config.')
    ref = load_reference()
    assert ref.POINT_GRID_W == cfg['point_grid_w'] and ref.POINT_GRID_H == cfg['point_grid_h']
    return cfg, rvq, ref


def load_scenes(atlas_path, cfg):
    got = hashlib.sha256(Path(atlas_path).read_bytes()).hexdigest()
    if got != cfg['scene_atlas_sha256']:
        raise SystemExit(f'scene atlas hash {got} != configured {cfg["scene_atlas_sha256"]}; '
                         f'pass the atlas that 7.2.3 uses.')
    with Image.open(atlas_path) as atlas:
        return [atlas.crop(tuple(box)).convert('RGB') for box in cfg['scene_crops']]


# ── 01 decision loop ─────────────────────────────────────────────────────────
BEATS_01 = [
    (0.0, 'One request: the instruction and the selected history go in; five tokens come out.'),
    (2.6, 'SlowFast already chose which frames to keep and how far to pool them (see 7.2.3).'),
    (5.0, 'Prefill processes the context once, then decode emits the five task tokens in order.'),
    (8.6, 'Three action codes are looked up in the RVQ bundle and summed into motion deltas.'),
    (11.2, 'The deltas compose into ten absolute waypoints in the chunk-start frame.'),
    (13.8, 'The controller tracks them; the next observation closes the loop at t+1.'),
]


def anim_loop(sec, scenes, rvq, cfg, poster=False):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    header(d, '01 / ONE DECISION, FIVE TOKENS', 'From one instruction to ten waypoints',
           'Shared Qwen3-VL-4B backbone, no navigation head: every output is one vocabulary id')
    tokens = cfg['demo_tokens']
    colours = [C_APOS, C_OPOS, C_L0, C_L1, C_L2]
    labels = ['APOS', 'OPOS', 'L0', 'L1', 'L2']
    t = 15.8 if poster else sec

    # ── column 1: instruction + history
    panel(d, (40, 130, 336, 196))
    text(d, (56, 140), 'INSTRUCTION (EXAMPLE)', 11, MUTED, True)
    text(d, (56, 160), 'Go to the refrigerator.', 20, INK, True)
    for j, (scene, stamp) in enumerate(zip(scenes[:3], cfg['demo_timestamps'])):
        x = 40 + j * 102
        picture(im, scene, (x, 210, 94, 53), radius=6)
        text(d, (x, 269), stamp, 11, MUTED)
    text(d, (40, 292), 'History is fed oldest -> newest,', 12, MUTED)
    text(d, (40, 308), 'each block carrying its own timestamp.', 12, MUTED)
    panel(d, (40, 332, 336, 408), fill='#eef3ee', outline='#d7e2d7')
    text(d, (56, 342), 'SLOWFAST ALREADY APPLIED', 11, MUTED, True)
    text(d, (56, 362), '240 stored frames', 14, INK, True)
    text(d, (56, 382), '82 selected  |  pool s = 1 / 2 / 4', 13, MUTED)
    arrow(d, (188, 414), (188, 430), MUTED, 2, 7)
    text(d, (40, 442), 'The task lives in the instruction text;', 12, MUTED)
    text(d, (40, 458), 'there is no task-identification token.', 12, MUTED)

    # ── column 2: backbone + tokens
    k_sf = reveal(t, 2.6)
    panel(d, (356, 130, 742, 262), outline=fade(EDGE, '#9fbcb4', k_sf), width=2 if k_sf > .5 else 1)
    text(d, (372, 140), 'SHARED BACKBONE', 11, MUTED, True)
    text(d, (372, 158), 'Qwen3-VL-4B-Instruct', 22, INK, True)
    text(d, (372, 190), 'prefill', 12, MUTED, True)
    bar(d, (428, 192, 726, 202), reveal(t, 4.6, 1.4), C_ACT)
    text(d, (372, 210), 'decode', 12, MUTED, True)
    bar(d, (428, 212, 726, 222), reveal(t, 6.0, 2.4), C_OPOS)
    text(d, (372, 234), 'one forward pass, then five tokens left to right', 12, MUTED)

    text(d, (356, 276), 'P(y_t | I, O<t)', 17, INK, True)
    text(d, (356 + width_of('P(y_t | I, O<t)', 17, True) + 8, 276), '= p1 x p2 x p3 x p4 x p5',
         17, MUTED)
    for j in range(5):
        k = reveal(t, 6.2 + j * 0.45, 0.35)
        x = 356 + j * 78
        panel(d, (x, 306, x + 72, 360), fill=fade('#eef1ee', colours[j], k),
              outline=fade(EDGE, colours[j], max(k, 0.25)), width=1)
        text(d, (x + 36, 316), labels[j], 10, 'white' if k > .55 else MUTED, True, 'mm')
        text(d, (x + 36, 336), tokens[j], 12, 'white' if k > .55 else fade(MUTED, INK, k), True, 'mm')
    text(d, (356, 368), 'Template text and the end marker are separate from these five.', 12, MUTED)

    # ── column 3: decode chain
    k_dec = reveal(t, 8.6)
    panel(d, (760, 130, 1160, 266), outline=fade(EDGE, '#9fbcb4', k_dec), width=2 if k_dec > .5 else 1)
    text(d, (776, 140), 'RVQ DECODE (OFFLINE LOOKUP)', 11, MUTED, True)
    steps = ['3 codes', 'sum', '/ weights', '(10, 3)', 'compose']
    for j, s in enumerate(steps):
        x = 776 + j * 76
        k = reveal(t, 8.8 + j * 0.3, 0.3)
        chip(d, (x, 164, x + 68, 196), s, fill=fade('#eef1ee', '#d6e6e0', k), color=fade(EDGE, INK, k),
             size=11, bold=True)
        if j < len(steps) - 1:
            arrow(d, (x + 69, 180), (x + 75, 180), fade(EDGE, MUTED, k), 2, 5)
    text(d, (776, 210), 'Each code is a row in one of three 256-entry', 13, MUTED)
    text(d, (776, 230), 'codebooks. Level 0 alone is executable;', 13, MUTED)
    text(d, (776, 250), 'the residuals refine it (see 7.2.5).', 13, MUTED)

    # ── column 3: the ten waypoints on a top-down plane
    wps = rvq['waypoints_full']
    n = 10 if poster else int(round(10 * reveal(t, 11.2, 2.4)))
    text(d, (786, 272), 'TEN WAYPOINTS, CHUNK-START FRAME', 11, MUTED, True)
    plot = Frame2D.fit((786, 288, 1146, 528), [(0, 0)] + [(p[0], p[1]) for p in wps], pad=0.10)
    plot.grid(d)
    plot.origin(d)
    plot.scalebar(d, 0.5, axis='x')
    robot(d, plot, 0, 0, 0, size=0.055, color=INK)
    if n:
        pts = [plot.xy(0, 0)] + [plot.xy(p[0], p[1]) for p in wps[:n]]
        if len(pts) > 1:
            d.line(pts, fill=C_ACT, width=3, joint='curve')
        for j, p in enumerate(wps[:n]):
            c = plot.xy(p[0], p[1])
            d.ellipse((c[0] - 4, c[1] - 4, c[0] + 4, c[1] + 4), fill='white', outline=C_ACT, width=2)
            if j in (0, 4, 9):
                text(d, (c[0] + 8, c[1] - 8), f'{j + 1}', 10, MUTED, True)
        text(d, (786, 536), f'1 -> 10: forward {wps[-1][0]:.2f} m, left {wps[-1][1]:.2f} m, '
                            f'turn {wps[-1][2]:.2f} rad', 11, MUTED)
    text(d, (786, 552), '+x forward, +y left; one dot is one pose, not one second', 10, MUTED)

    # ── feedback strip
    k_loop = reveal(t, 13.8, 1.0)
    panel(d, (40, 500, 736, 528), fill='#eef3ee', outline='#dbe6db')
    text(d, (56, 508), 'controller tracks the path  ->  next observation at t+1', 13,
         fade(BG, MUTED, k_loop))
    dashed(d, [(690, 514), (640, 514)], fade(BG, MUTED, k_loop), 2, 8, 5)

    text(d, (40, 580), beat(BEATS_01, t), 20, INK, True)
    text(d, (40, 616), 'Five tokens per decision. The path is local to the chunk start, not a world pose.',
         13, MUTED)
    text(d, (40, 646), 'Illustrated pipeline: instruction, frame counts and codes are teaching examples, '
                       'not a recorded request.', 12, MUTED)
    return im


# ── 02 pointing grid ─────────────────────────────────────────────────────────
BEATS_02 = [
    (0.0, 'The model points with one vocabulary id per channel, not with coordinates in text.'),
    (2.4, 'Forward: a pixel becomes a grid cell, and the cell becomes one id.'),
    (5.4, 'Backward: the id alone restores the cell centre, so nothing else has to be stored.'),
    (8.4, 'APOS says where to move; OPOS says where the goal is. Same grid, two ids.'),
    (11.4, 'Four extra ids carry rotate/stop; OPOS has none, only 0 for "not visible".'),
    (14.2, 'A pixel token still has no depth: it is a direction on the image, not a 3-D point.'),
]


def anim_pointing(sec, scenes, rvq, cfg, ref, poster=False):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    gw, gh = cfg['point_grid_w'], cfg['point_grid_h']
    fw, fh = cfg['demo_frame_wh']
    cw, ch = fw / gw, fh / gh
    pixel = cfg['demo_pixel']
    opos_id = cfg['demo_opos_id']
    apos_pixel = cfg['demo_apos_pixel']
    apos_id = cfg['demo_apos_id']
    row, col = int(pixel[1] // ch), int(pixel[0] // cw)
    t = 15.8 if poster else sec
    header(d, '02 / POINTING AS ONE TOKEN', 'Pixel -> cell -> vocabulary id, and back',
           f'{gw} x {gh} grid = {gw * gh} cells; one cell is {cw:.0f} x {ch:.0f} px '
           f'on a {fw} x {fh} frame')

    ox, oy = 40, 132
    picture(im, scenes[3], (ox, oy, fw, fh), radius=10)

    k_grid = reveal(t, 1.4, 0.8)
    if k_grid > 0:
        gcol = fade(BG, '#7d9a94', k_grid * 0.45)
        for i in range(1, gw):
            x = ox + i * cw
            d.line((x, oy, x, oy + fh), fill=gcol, width=1)
        for j in range(1, gh):
            y = oy + j * ch
            d.line((ox, y, ox + fw, y), fill=gcol, width=1)
        d.rectangle((ox, oy, ox + fw, oy + fh), outline=fade(BG, '#5f7d78', k_grid), width=2)

    k_cell = reveal(t, 2.4, 0.5)
    if k_cell > 0:
        pad = (1 - k_cell) * 14
        d.rectangle((ox + col * cw - pad, oy + row * ch - pad,
                     ox + (col + 1) * cw + pad, oy + (row + 1) * ch + pad),
                    outline=C_OPOS, width=3)

    k_span = reveal(t, 3.0, 0.6)
    if k_span > 0:
        x0, y0 = ox + col * cw, oy + row * ch
        text(d, (x0 - 2, y0 - 18), f'col {col}: x {col * cw:.0f}..{(col + 1) * cw:.0f}',
             10, fade(BG, C_OPOS, k_span), True)
        text(d, (x0 + cw + 4, y0 + 4), f'row {row}', 10, fade(BG, C_OPOS, k_span), True)

    k_cross = reveal(t, 5.4, 0.6)
    if k_cross > 0:
        cx, cy = ox + pixel[0], oy + pixel[1]
        L = 15 * k_cross
        d.line((cx - L, cy, cx + L, cy), fill=C_OPOS, width=2)
        d.line((cx, cy - L, cx, cy + L), fill=C_OPOS, width=2)

    k_ch = reveal(t, 8.4, 0.6)
    if k_ch > 0:
        for px, colour, name in ((apos_pixel, C_APOS, 'APOS'), (pixel, C_OPOS, 'OPOS')):
            cx, cy = ox + px[0], oy + px[1]
            rr = 8 * k_ch
            d.ellipse((cx - rr, cy - rr, cx + rr, cy + rr), fill=colour, outline='white', width=2)
            text(d, (cx + 12, cy - 22), name, 11, colour, True)

    text(d, (40, 412), f'1 cell = {cw:.0f} x {ch:.0f} px   |   ids 1..{gw * gh}, 0 reserved',
         12, MUTED)

    # bottom-left: two channels
    panel(d, (40, 434, 520, 564))
    text(d, (56, 444), 'TWO CHANNELS, TWO IDS PER FRAME', 11, MUTED, True)
    for j, (name, tid, colour, what) in enumerate((
            ('APOS', apos_id, C_APOS, 'next move: floor'),
            ('OPOS', opos_id, C_OPOS, 'goal: refrigerator'))):
        y = 466 + j * 40
        chip(d, (56, y, 158, y + 24), f'<{name.lower()}_{tid}>', fill=colour, size=12)
        text(d, (168, y + 5), what, 11, MUTED)
    text(d, (56, 530), 'Both are the model\'s own judgement, not a detector.', 11, MUTED)
    text(d, (56, 546), 'The request never contains these two ids.', 11, MUTED)

    # top-right: forward and backward arithmetic
    panel(d, (548, 132, 1160, 296))
    text(d, (564, 140), 'FORWARD: PIXEL -> ID', 11, MUTED, True)
    rows = [(2.4, f'pixel (u, v) = ({pixel[0]:.0f}, {pixel[1]:.0f})'),
            (3.2, f'col = floor({pixel[0]:.0f} / {cw:.0f}) = {col}      '
                  f'row = floor({pixel[1]:.0f} / {ch:.0f}) = {row}'),
            (4.0, f'id = 1 + row * {gw} + col = 1 + {row} * {gw} + {col} = {opos_id}')]
    for j, (start, line) in enumerate(rows):
        k = reveal(t, start, 0.4)
        text(d, (564, 166 + j * 30), line, 17, fade(BG, INK, k), True)
    k_id = reveal(t, 4.6, 0.5)
    chip(d, (930, 252, 1144, 282), f'<opos_{opos_id}>', fill=fade('#eef1ee', C_OPOS, k_id),
         color='white' if k_id > .6 else MUTED, size=14)

    panel(d, (548, 302, 1160, 432))
    text(d, (564, 314), 'BACKWARD: ID -> CELL CENTRE', 11, MUTED, True)
    back = [(5.4, f'k = id - 1 = {opos_id - 1}'),
            (6.0, f'col = {opos_id - 1} mod {gw} = {col}      row = floor({opos_id - 1} / {gw}) = {row}'),
            (6.6, f'u = (col + 0.5) * {cw:.0f} = {pixel[0]:.0f}      '
                  f'v = (row + 0.5) * {ch:.0f} = {pixel[1]:.0f}')]
    for j, (start, line) in enumerate(back):
        k = reveal(t, start, 0.4)
        text(d, (564, 344 + j * 26), line, 16, fade(BG, INK, k), True)
    text(d, (564, 414), 'No pixel lookup table: the centre is arithmetic on the id.', 11, MUTED)

    panel(d, (548, 438, 1160, 562))
    text(d, (564, 444), 'SPECIAL IDS (APOS ONLY) AND THE OPOS ZERO', 11, MUTED, True)
    specials = [('0', 'no grid point'),
                (str(gw * gh + 1), 'rotate left in place'),
                (str(gw * gh + 2), 'rotate right in place'),
                (str(gw * gh + 3), 'stop here')]
    for j, (tid, what) in enumerate(specials):
        k = reveal(t, 11.4 + j * 0.25, 0.3)
        x = 564 + j * 148
        text(d, (x, 468), tid, 19, fade(BG, INK, k), True)
        text(d, (x, 494), what, 11, fade(BG, MUTED, k))
    k_zero = reveal(t, 12.6, 0.4)
    text(d, (564, 518), 'OPOS has no rotate/stop ids: 0 means "not visible this frame".',
         13, fade(BG, C_OPOS, k_zero), True)
    text(d, (564, 540), f'Vocabulary sizes: APOS {gw * gh + 4}, OPOS {gw * gh + 1}.', 11, MUTED)

    text(d, (40, 580), beat(BEATS_02, t), 20, INK, True)
    text(d, (40, 616), 'One 480 x 270 frame with the grid drawn on top. The cell is a token, '
                       'not a detection.', 13, MUTED)
    text(d, (40, 646), 'Grid, ids and the 505 example come from the released encoder; the scene is a '
                       'teaching illustration.', 12, MUTED)
    return im


# ── 03 RVQ residual ──────────────────────────────────────────────────────────
def beats_03(rvq):
    """Captions built from the frozen example, so the words cannot drift from the numbers."""
    g1 = rvq['refinement'][0]['mean_position_gap_m'] * 100
    g2 = rvq['refinement'][1]['mean_position_gap_m'] * 100
    return [
        (0.0, 'A whole ten-point path is too many numbers to write out, so it becomes three ids.'),
        (2.4, 'Level 0 picks the coarse pattern; each later level encodes what the previous one '
              'missed.'),
        (5.0, 'Sum the three codewords, divide by the weights, reshape: ten motion deltas return.'),
        (7.8, f'Each level removes path error. Here L0 alone is {g1:.1f} cm off and L0+L1 is '
              f'{g2:.1f} cm off.'),
        (11.0, 'Stop is a specific code triple, not a near-zero path: the codeword is only near '
               'zero.'),
        (13.6, f'3 x 256 entries give {rvq["combo_count"]:,} combinations: a vocabulary, not a '
               'feasibility guarantee.'),
    ]


def anim_rvq(sec, scenes, rvq, cfg, poster=False):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    t = 15.8 if poster else sec
    header(d, '03 / THREE CODES, ONE TRAJECTORY', 'Residual vector quantization',
           f"Released bundle: 3 levels x 256 entries, horizon {rvq['horizon']}, "
           f"feature {rvq['feature_dim']}, {rvq['representation']}")

    # left top: 2-D arithmetic
    panel(d, (40, 130, 452, 402))
    text(d, (56, 138), 'A 2-D ARITHMETIC EXAMPLE', 11, MUTED, True)
    text(d, (56, 160), 'Target vector [1.00, 0.20]', 15, INK, True)
    rows = [('L0 codeword', '[0.80, 0.00]', 'residual [0.20, 0.20]'),
            ('L1 codeword', '[0.15, 0.15]', 'residual [0.05, 0.05]'),
            ('L2 codeword', '[0.05, 0.05]', 'residual [0.00, 0.00]')]
    for j, (name, code, res) in enumerate(rows):
        k = reveal(t, 2.4 + j * 0.9, 0.5)
        y = 188 + j * 46
        d.rectangle((56, y, 436, y + 40), fill=fade(PANEL, '#eef4f0', k))
        text(d, (66, y + 5), name, 12, fade(BG, MUTED, k), True)
        text(d, (66, y + 21), code, 15, fade(BG, INK, k), True)
        text(d, (250, y + 13), res, 13, fade(BG, C_ACT, k))
    k_sum = reveal(t, 5.0, 0.5)
    text(d, (56, 332), 'sum = [0.80+0.15+0.05, 0.00+0.15+0.05] = [1.00, 0.20]',
         14, fade(BG, INK, k_sum), True)
    text(d, (56, 356), 'Additive: one path, three corrections.', 12, fade(BG, MUTED, k_sum))
    text(d, (56, 376), 'Real quantization usually keeps a small remainder.', 11, MUTED)

    # left bottom: decode chain
    panel(d, (40, 414, 452, 560), fill='#eef3ee', outline='#d7e2d7')
    text(d, (56, 422), 'THE SAME STEPS ON A REAL CHUNK', 11, MUTED, True)
    chain = ['e_k0 + e_k1 + e_k2', '/ jacobian weights', 'reshape (10, 3)', 'compose_to_abs']
    for j, s in enumerate(chain):
        k = reveal(t, 5.8 + j * 0.45, 0.3)
        text(d, (56, 448 + j * 26), s, 14, fade(BG, INK, k), True)
        if j < len(chain) - 1:
            d.line((62, 468 + j * 26, 62, 470 + j * 26), fill=fade(BG, MUTED, k), width=2)
    text(d, (268, 448), 'code ids in', 11, MUTED)
    text(d, (268, 526), 'metres out', 11, MUTED)
    text(d, (56, 546), f'codes {rvq["codes"]}', 11, MUTED)

    # middle: three decoded paths
    wps_all = rvq['waypoints_partial'] + [rvq['waypoints_full']]
    names = ['L0 only', 'L0 + L1', 'L0 + L1 + L2']
    colours = [C_L0, C_L1, C_L2]
    allp = [(0, 0)] + [(p[0], p[1]) for p in rvq['waypoints_full']]
    plot = Frame2D.fit((470, 176, 916, 520), allp, pad=0.12)
    plot.grid(d)
    plot.origin(d)
    plot.scalebar(d, 0.5)
    robot(d, plot, 0, 0, 0, size=0.07, color=INK)
    for j, (path, colour) in enumerate(zip(wps_all, colours)):
        k = reveal(t, 5.0 + j * 1.1, 0.7)
        if k <= 0:
            continue
        pts = [plot.xy(0, 0)] + [plot.xy(p[0], p[1]) for p in path]
        d.line(pts, fill=colour, width=3 if j == 2 else 2, joint='curve')
        for p in path:
            c = plot.xy(p[0], p[1])
            d.ellipse((c[0] - 3, c[1] - 3, c[0] + 3, c[1] + 3), fill=colour)
    for j, (name, colour) in enumerate(zip(names, colours)):
        x = 474 + j * 148
        d.line((x, 156, x + 16, 156), fill=colour, width=3)
        text(d, (x + 22, 149), name, 12, INK, True)
    text(d, (474, 528), 'all three start from the same pose; only the codeword sum changes', 11, MUTED)

    # right: numbers
    panel(d, (934, 168, 1160, 400))
    text(d, (950, 176), 'PATH GAP TO THE FULL DECODE', 11, MUTED, True)
    for j, row in enumerate(rvq['refinement']):
        y = 200 + j * 48
        text(d, (950, y), row['levels'], 12, INK, True)
        text(d, (950, y + 17), f'{row["mean_position_gap_m"] * 100:.1f} cm', 20, colours[j], True)
        text(d, (1046, y + 21), f'{row["mean_heading_gap_rad"]:.3f} rad', 10, MUTED)
    text(d, (950, 346), 'mean per-point distance from', 10, MUTED)
    text(d, (950, 360), 'the full decode; same start', 10, MUTED)
    k_ref = reveal(t, 7.8, 0.6)
    text(d, (950, 380), 'a correction, not a route', 11, fade(BG, INK, k_ref), True)

    panel(d, (934, 412, 1160, 560))
    text(d, (950, 420), 'ONE CODEWORD PER LEVEL', 11, MUTED, True)
    for j, row in enumerate(rvq['level_stats']):
        y = 444 + j * 26
        text(d, (950, y), f'L{j}', 12, colours[j], True)
        text(d, (976, y), f'step {row["per_step_xy_mean_m"] * 100:.1f} cm', 11, INK)
        text(d, (1074, y), f'net {row["chunk_net_mean_m"] * 100:.0f} cm', 11, MUTED)
    text(d, (950, 528), 'mean over all 256 codewords;', 10, MUTED)
    text(d, (950, 542), 'not an execution error.', 10, MUTED)

    # bottom: stop
    panel(d, (40, 568, 1160, 596), fill='#fdf3ec', outline='#f0d9c8')
    stop = rvq['stop']
    text(d, (56, 573), f'STOP = codes {stop["codes"]}: the L0 codeword is only near zero '
                       f'(net {stop["codeword_net_m"] * 100:.2f} cm), so stop is matched by code, '
                       f'not by path length', 12, C_APOS, True)

    text(d, (40, 604), beat(beats_03(rvq), t), 19, INK, True)
    text(d, (40, 640), 'Offline decode of the released bundle, codes '
                       f'{rvq["codes"]}: a teaching example, not a measured execution error.', 12, MUTED)
    return im


# ── 04 SE(2) composition ─────────────────────────────────────────────────────
BEATS_04 = [
    (0.0, 'Each step is written in the frame of the previous pose, so the deltas cannot just be added.'),
    (2.6, 'Step 1: 0.5 m forward from the identity pose lands on (0.5, 0, 0).'),
    (5.4, 'Step 2: turn left 90 deg in place. The position does not change; the heading does.'),
    (8.2, 'Step 3: 0.5 m along the NEW heading. World y grows, because the nose now faces +y.'),
    (11.4, 'Ten steps of 0.15 m compose to 1.50 m: the points accumulate, they do not restart.'),
    (13.8, 'Ten accumulated points in the chunk-start frame; a world path needs one more transform.'),
]


def anim_se2(sec, scenes, rvq, cfg, poster=False):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    t = 15.8 if poster else sec
    header(d, '04 / DELTAS BECOME POSES', 'From per-step motion to ten poses',
           'compose_to_abs: local (dx, dy, dtheta) in the previous frame -> '
           '(x, y, theta) in the chunk-start frame')

    plot = Frame2D.fit((58, 168, 448, 536), [(0, 0), (0.5, 0), (0.5, 0.5)], pad=0.20,
                       min_range=0.0)
    plot.grid(d, 0.25)
    plot.origin(d)

    robot(d, plot, 0, 0, 0, size=0.045, color='#9fb3ae')
    k1 = reveal(t, 2.6, 0.9)
    k2 = reveal(t, 5.4, 0.9)
    k3 = reveal(t, 8.2, 0.9)
    if k1 > 0:
        p0, p1 = plot.xy(0, 0), plot.xy(0.5, 0)
        d.line((*p0, *p1), fill=C_ACT, width=4)
        robot(d, plot, 0.5, 0, 0, size=0.045, color=C_ACT)
    if k2 > 0:
        p1, p2 = plot.xy(0.5, 0), plot.xy(0.5, 0.5)
        d.line((*p1, *p2), fill=C_ACT, width=4)
        robot(d, plot, 0.5, 0, math.pi / 2, size=0.045, color=C_ACT)
        c = plot.xy(0.5, 0)
        r = 30 * k2
        d.arc((c[0] - r, c[1] - r, c[0] + r, c[1] + r), start=-88, end=-4, fill=C_APOS, width=3)
        ex, ey = c[0] + r * math.cos(math.radians(-6)), c[1] + r * math.sin(math.radians(-6))
        d.polygon([(ex + 7, ey + 2), (ex - 3, ey - 7), (ex - 1, ey + 9)], fill=C_APOS)
        text(d, (c[0] - 92, c[1] + 32), 'left turn, in place', 11, C_APOS, True)
    if k3 > 0:
        robot(d, plot, 0.5, 0.5, math.pi / 2, size=0.045, color=C_ACT)

    labels = [((0, 0), '(0, 0, 0)', (16, -4)), ((0.5, 0), '(0.5, 0, 0)', (16, 18)),
              ((0.5, 0.5), '(0.5, 0.5, pi/2)', (-14, -26))]
    for j, (pos, label, off) in enumerate(labels):
        k = reveal(t, 2.6 + j * 2.7, 0.5)
        if k <= 0:
            continue
        c = plot.xy(*pos)
        d.ellipse((c[0] - 5, c[1] - 5, c[0] + 5, c[1] + 5), fill='white', outline=INK, width=2)
        text(d, (c[0] + off[0], c[1] + off[1]), label, 13, fade(BG, INK, k), True)
    text(d, (58, 546), 'square plot: one dotted cell is 0.25 m on both axes', 11, MUTED)
    k_note = reveal(t, 5.8, 0.6)
    text(d, (58, 566), 'in-place turn: position unchanged, heading +90 deg', 12,
         fade(BG, C_APOS, k_note))

    panel(d, (470, 152, 1160, 312))
    text(d, (486, 160), 'THE COMPOSITION', 11, MUTED, True)
    text(d, (486, 186), 'x_j = x_j-1 + cos(th_j-1) dx_j - sin(th_j-1) dy_j', 15, INK, True)
    text(d, (486, 214), 'y_j = y_j-1 + sin(th_j-1) dx_j + cos(th_j-1) dy_j', 15, INK, True)
    text(d, (486, 242), 'th_j = wrap(th_j-1 + dth_j)', 15, INK, True)
    text(d, (486, 276), 'The previous heading rotates each delta out of its local frame.', 13, MUTED)

    panel(d, (470, 326, 1160, 546))
    text(d, (486, 334), 'TEN STEPS OF 0.15 m, NO TURN', 11, MUTED, True)
    for label, x in (('j', 492), ('dx', 556), ('x_j', 660), ('y_j', 800), ('theta_j', 940)):
        text(d, (x, 360), label, 12, MUTED, True)
    n = 10 if poster else max(0, min(10, int((t - 11.4) * 5) + 1))
    if n == 0:
        text(d, (486, 420), 'the ten rows appear once the deltas start composing',
             12, '#b9c6c2', True)
    for j in range(n):
        y = 384 + j * 13
        text(d, (492, y), str(j + 1), 11, INK)
        text(d, (556, y), '0.150', 11, MUTED)
        text(d, (660, y), f'{0.15 * (j + 1):.3f}', 11, C_ACT, True)
        text(d, (800, y), '0.000', 11, MUTED)
        text(d, (940, y), '0.000', 11, MUTED)
    k_end = reveal(t, 12.6, 0.6)
    text(d, (486, 518), 'ten increments -> ten accumulated points -> 1.50 m', 14,
         fade(BG, INK, k_end), True)

    k_rep = reveal(t, 13.8, 0.8)
    for j, (name, colour) in enumerate((('per-step delta (previous step frame)', C_APOS),
                                       ('10 accumulated points (chunk-start frame)', C_ACT),
                                       ('world path (needs the capture root pose)', C_OPOS))):
        x = 44 + j * 372
        panel(d, (x, 598, x + 356, 622), fill=fade('#eef1ee', colour, 0.12 * k_rep),
              outline=fade(EDGE, colour, k_rep))
        text(d, (x + 12, 602), name, 11, fade(BG, INK, k_rep), True)

    text(d, (40, 632), beat(BEATS_04, t), 18, INK, True)
    text(d, (40, 656), 'Hand-worked example on a metric plane. The ten points are a local path, '
                       'not world coordinates.', 11, MUTED)
    return im


# ── 05 training stages ───────────────────────────────────────────────────────
BEATS_05 = [
    (0.0, 'Three stages, three different training signals; only the last one looks at outcomes.'),
    (1.8, 'Stage 1 adds spatial and embodied reasoning tasks on top of the pretrained VLM.'),
    (5.2, 'Stage 2 fits reference pointing and RVQ-coded trajectories with token cross-entropy.'),
    (9.2, 'DAgger adds corrections on the states the policy itself reaches, not only expert states.'),
    (11.6, 'Stage 3 runs the policy in simulation and ranks whole rollouts with a group baseline.'),
    (14.2, 'Discrete tokens keep pointing and trajectories inside the language-model interface.'),
]


def anim_training(sec, scenes, rvq, cfg, poster=False):
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    t = 15.8 if poster else sec
    header(d, '05 / HOW THE MODEL LEARNS IT', 'Embodied reasoning, imitation, then online RL',
           'Paper-level description; this course runs the released weights and does not retrain '
           'on the Jetson')

    stages = [
        ('STAGE 1', 'Embodied reasoning', 'Add spatial skills first',
         ['spatial reasoning', 'image pointing', 'video QA', 'multi-view linking',
          'feasibility / failure'], C_ACT),
        ('STAGE 2', 'SFT + DAgger', 'Fit reference pointing and actions',
         ['observation + instruction', '-> APOS, OPOS, act_l0..l2',
          'cross-entropy on those positions', 'DAgger: correct on own states'], C_OPOS),
        ('STAGE 3', 'Online RL (GRPO)', 'Rank whole rollouts against a baseline',
         ['run the policy in simulation', 'score the whole outcome',
          'normalise within the group', 'clipped update + KL to reference'], C_RL),
    ]
    for j, (eyebrow, title, sub, bullets, colour) in enumerate(stages):
        x = 40 + j * 380
        k = reveal(t, 1.8 + j * 3.4, 0.45)
        panel(d, (x, 136, x + 360, 556), fill=fade('#f6f8f4', '#ffffff', 0.55 + 0.45 * k),
              outline=fade(EDGE, colour, k), width=2 if k > .5 else 1)
        d.rectangle((x + 1, 136, x + 359, 142), fill=fade(EDGE, colour, k))
        text(d, (x + 18, 154), eyebrow, 11, fade(MUTED, colour, k), True)
        text(d, (x + 18, 176), title, 21, fade(BG, INK, k), True)
        text(d, (x + 18, 206), sub, 12, fade(BG, MUTED, k))
        for i, b in enumerate(bullets):
            kb = reveal(t, 2.4 + j * 3.4 + i * 0.25, 0.3)
            y = 234 + i * 22
            if kb > 0:
                d.ellipse((x + 19, y + 5, x + 25, y + 11), fill=fade(BG, colour, kb))
                text(d, (x + 34, y), b, 12, fade(BG, INK, kb))

    # stage 1 lower: the extra task families
    x1 = 40
    for i, name in enumerate(('spatial', 'pointing', 'video QA', 'multi-view', 'failure')):
        k = reveal(t, 2.6 + i * 0.22, 0.3)
        cx = x1 + 18 + (i % 3) * 112
        cy = 372 + (i // 3) * 42
        chip(d, (cx, cy, cx + 104, cy + 32), name, fill=fade('#eef1ee', C_ACT, 0.25 * k),
             color=fade(MUTED, INK, k), size=11, bold=False, outline=fade(EDGE, C_ACT, k))
    text(d, (x1 + 18, 466), 'Points and videos reuse the same', 11, MUTED)
    text(d, (x1 + 18, 482), 'autoregressive interface; no rule', 11, MUTED)
    text(d, (x1 + 18, 498), 'navigator is added.', 11, MUTED)
    text(d, (x1 + 18, 530), 'Signal: task supervision.', 11, C_ACT, True)

    # stage 2 lower: sample card, CE numbers, DAgger
    x2 = 40 + 380
    k = reveal(t, 5.6, 0.6)
    panel(d, (x2 + 18, 368, x2 + 196, 460), fill=fade('#eef3ee', '#dbeae3', k),
          outline=fade(EDGE, C_OPOS, k))
    text(d, (x2 + 30, 374), 'training sample', 10, MUTED, True)
    text(d, (x2 + 30, 390), 'obs + instruction', 11, fade(BG, INK, k), True)
    text(d, (x2 + 30, 408), '-> APOS, OPOS', 11, fade(BG, INK, k), True)
    text(d, (x2 + 30, 426), '-> act_l0, l1, l2', 11, fade(BG, INK, k), True)
    text(d, (x2 + 30, 444), 'reference, not invention', 10, MUTED)
    panel(d, (x2 + 208, 368, x2 + 342, 460), fill='#fdf3ec', outline='#f0d9c8')
    text(d, (x2 + 220, 374), 'token CE', 10, MUTED, True)
    text(d, (x2 + 220, 394), 'p 0.2 -> 0.8', 11, INK, True)
    text(d, (x2 + 220, 412), '1.609', 12, C_APOS, True)
    text(d, (x2 + 220, 430), '-> 0.223', 12, C_APOS, True)
    kd = reveal(t, 9.2, 0.6)
    text(d, (x2 + 18, 472), 'DAgger loop', 11, fade(MUTED, C_OPOS, kd), True)
    text(d, (x2 + 18, 490), 'policy runs -> own states', 10, fade(BG, MUTED, kd))
    text(d, (x2 + 18, 506), '-> reference fix -> data', 10, fade(BG, MUTED, kd))
    text(d, (x2 + 18, 530), 'Signal: reference tokens.', 11, C_OPOS, True)

    # stage 3 lower: rollout bars
    x3 = 40 + 760
    rewards = cfg['grpo_rewards']
    adv = cfg['grpo_advantages']
    k3 = reveal(t, 12.0, 0.6)
    for i, (r, a) in enumerate(zip(rewards, adv)):
        bx = x3 + 24 + i * 72
        h = 14 + r * 30
        d.rectangle((bx, 400 - h, bx + 48, 400), fill=fade('#e7ece8', C_RL, 0.35 + 0.65 * k3))
        text(d, (bx + 24, 404), f'R={r}', 10, fade(BG, INK, k3), True, 'ma')
        text(d, (bx + 24, 422), f'A={a:+.2f}', 9, fade(BG, C_RL, k3), True, 'ma')
    text(d, (x3 + 18, 444), 'one task, 3 rollouts', 10, MUTED)
    text(d, (x3 + 18, 460), 'A = (R - mean) / std', 10, MUTED)
    text(d, (x3 + 18, 482), 'equal rewards give no', 10, MUTED)
    text(d, (x3 + 18, 496), 'group-relative signal', 10, MUTED)
    text(d, (x3 + 18, 524), 'Signal: outcomes.', 11, C_RL, True)

    k_sum = reveal(t, 14.2, 0.8)
    panel(d, (40, 578, 1160, 604), fill=fade('#eef1ee', '#dce9e3', k_sum),
          outline=fade(EDGE, C_OPOS, k_sum))
    text(d, (56, 584), 'Three signals, in order: spatial clues -> reference outputs -> actual '
                       'outcomes. The RVQ codes carry probabilities, so they train like any other token.',
         13, fade(BG, INK, k_sum), True)

    text(d, (40, 614), beat(BEATS_05, t), 18, INK, True)
    text(d, (40, 644), 'Stage descriptions follow the paper. Privileged training labels and rewards '
                       'are not model inputs at run time.', 11, MUTED)
    return im


# ── encode ───────────────────────────────────────────────────────────────────
def encode_gif(frames, path):
    # A shared palette avoids flashing colours between GIF frames.
    palette = frames[-1].quantize(colors=192, method=Image.Quantize.MEDIANCUT)
    quantized = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    quantized[0].save(path, save_all=True, append_images=quantized[1:], duration=100, loop=0,
                      optimize=True, disposal=1)
    with Image.open(path) as gif:
        duration = sum(f.info.get('duration', 0) for f in ImageSequence.Iterator(gif))
        assert duration == SECONDS * 1000, (path, duration)
    return duration


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--assets', type=Path, required=True, help='output directory')
    ap.add_argument('--scene-atlas', type=Path, default=None,
                    help='scene-atlas.jpg from docs/.../assets/slowfast (default: sibling assets)')
    args = ap.parse_args()
    assets = args.assets
    assets.mkdir(parents=True, exist_ok=True)
    cfg, rvq, ref = load_data()

    atlas = args.scene_atlas
    if atlas is None:
        atlas = HERE.parents[2] / '7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast/scene-atlas.jpg'
    scenes = load_scenes(atlas, cfg)

    jobs = [
        ('01-decision-loop', lambda s, p=False: anim_loop(s, scenes, rvq, cfg, p)),
        ('02-pointing-grid', lambda s, p=False: anim_pointing(s, scenes, rvq, cfg, ref, p)),
        ('03-rvq-residual', lambda s, p=False: anim_rvq(s, scenes, rvq, cfg, p)),
        ('04-se2-compose', lambda s, p=False: anim_se2(s, scenes, rvq, cfg, p)),
        ('05-training-stages', lambda s, p=False: anim_training(s, scenes, rvq, cfg, p)),
    ]
    outputs = []
    for name, fn in jobs:
        frames = [fn(i / FPS) for i in range(FPS * SECONDS)]
        path = assets / (name + '.gif')
        duration = encode_gif(frames, path)
        poster = fn(15.8, True)
        poster.save(assets / (name + '.png'), optimize=True)
        with Image.open(path) as gif:
            n_frames = getattr(gif, 'n_frames', 0)
        outputs.append({'file': path.name, 'dimensions': [W, H], 'duration_ms': duration,
                        'sampling_fps': FPS, 'frames_drawn': FPS * SECONDS, 'gif_frames': n_frames,
                        'hold_frames': FPS * SECONDS - n_frames, 'size': path.stat().st_size,
                        'poster': name + '.png'})
        print(f'{name}: {path.stat().st_size} bytes, {n_frames} stored frames')

    (assets / 'animation-manifest.json').write_text(json.dumps({
        'outputs': outputs,
        'format': '1200x675, 10 fps sampling, 16 s loop, shared palette per animation',
        'hold_frames_note': 'Consecutive identical frames are merged into hold frames by the encoder; '
                            'that is not a dropped frame and not a runtime rate.',
        'source_config': 'example-config.json',
        'rvq_example': 'rvq-teaching-example.json (offline decode of the released bundle)',
        'renderer': 'build_visuals.py; every number is computed or read from the frozen files',
        'scene_source': 'docs/M07-Vision-Language-Navigation/7.2_Semantic_Grounding_and_Learned_VLN/assets/slowfast/scene-atlas.jpg '
                        '(built-in image_gen; same crops as 7.2.3)',
        'not_runtime_evidence': True,
        'english_only_labels': True,
    }, indent=2) + '\n')

    (assets / 'teaching-numbers.json').write_text(json.dumps({
        'point_grid': {'w': cfg['point_grid_w'], 'h': cfg['point_grid_h'],
                       'cells': cfg['point_grid_w'] * cfg['point_grid_h'],
                       'cell_px_on_demo_frame': [cfg['demo_frame_wh'][0] / cfg['point_grid_w'],
                                                 cfg['demo_frame_wh'][1] / cfg['point_grid_h']]},
        'pointing_example': {'opos_pixel': cfg['demo_pixel'], 'opos_id': cfg['demo_opos_id'],
                             'apos_pixel': cfg['demo_apos_pixel'], 'apos_id': cfg['demo_apos_id'],
                             'row': cfg['demo_pixel'][1] // (cfg['demo_frame_wh'][1] / cfg['point_grid_h']),
                             'col': cfg['demo_pixel'][0] // (cfg['demo_frame_wh'][0] / cfg['point_grid_w'])},
        'special_ids': {'apos': [cfg['point_grid_w'] * cfg['point_grid_h'] + i for i in (1, 2, 3)],
                        'apos_vocab': cfg['point_grid_w'] * cfg['point_grid_h'] + 4,
                        'opos_vocab': cfg['point_grid_w'] * cfg['point_grid_h'] + 1},
        'rvq': {'codes': rvq['codes'], 'tokens': rvq['token_literals'],
                'levels': rvq['levels'], 'horizon': rvq['horizon'],
                'combinations': rvq['combo_count'], 'refinement': rvq['refinement'],
                'level_stats': rvq['level_stats'], 'stop': rvq['stop']},
        'se2': {'worked_example': [[0, 0, 0], [0.5, 0, 0], [0.5, 0, math.pi / 2], [0.5, 0.5, math.pi / 2]],
                'straight_ten_steps': {'step_m': 0.15, 'total_m': 1.5}},
        'training': {'grpo_rewards': cfg['grpo_rewards'], 'grpo_advantages': cfg['grpo_advantages'],
                     'ce_example': cfg['ce_example']},
        'teaching_only': True,
    }, indent=2) + '\n')
    print(json.dumps([{k: o[k] for k in ('file', 'size', 'gif_frames', 'hold_frames')} for o in outputs], indent=2))


if __name__ == '__main__':
    main()
