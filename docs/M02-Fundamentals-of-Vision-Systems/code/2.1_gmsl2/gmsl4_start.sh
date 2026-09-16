#!/usr/bin/env bash
# Four-camera GMSL FSYNC preflight, HDMI preview and transient RTSP launcher.
set -Eeuo pipefail

MODE="${1:-preview}"
SECONDS_ARG="${2:-0}"
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDP_PORT="${GMSL4_UDP_PORT:-5600}"
RTSP_PORT="${GMSL4_RTSP_PORT:-8554}"
MEDIAMTX_VERSION="${MEDIAMTX_VERSION:-1.20.0}"
MEDIAMTX_DIR="$BASE_DIR/mediamtx"
MEDIAMTX_BIN="$MEDIAMTX_DIR/mediamtx"
PIDFILE="$BASE_DIR/gmsl4_stream.pids"

log()  { echo -e "\\e[36m[gmsl4]\\e[0m $*"; }
die()  { echo -e "\\e[31m[gmsl4]\\e[0m $*" >&2; exit 1; }
usage() {
  cat <<'EOF'
Usage: gmsl4_start.sh {preview|stream|verify|stop|status} [seconds]
  preview [seconds]  display a real-time 2x2 HDMI mosaic and write diagnostics.
  stream  [seconds]  display HDMI and publish rtsp://<Jetson-IP>:8554/gmsl4.
  verify  [seconds]  run GMSL/FSYNC preflight and timestamp diagnostics only.
  stop               stop the script-managed capture and RTSP relay.
  status             show stream process and RTSP listener state.
EOF
}

case "$MODE" in preview|stream|verify|stop|status) ;; *) usage; exit 2;; esac
if [[ "$MODE" == stop || "$MODE" == status ]]; then
  if [[ "$MODE" == stop ]]; then
    if [[ -f "$PIDFILE" ]]; then
      read -r capture_pid mtx_pid < "$PIDFILE" || true
      [[ -n "${capture_pid:-}" ]] && kill -- -"$capture_pid" 2>/dev/null || true
      [[ -n "${mtx_pid:-}" && "$mtx_pid" != "-" ]] && kill -- -"$mtx_pid" 2>/dev/null || true
      rm -f "$PIDFILE"
      log "stop requested"
    else
      log "no script-managed stream is running"
    fi
  else
    [[ -f "$PIDFILE" ]] && { log "processes: $(<"$PIDFILE")"; } || log "no script-managed stream"
    ss -ltnp 2>/dev/null | grep -E ":${RTSP_PORT}\\b" || true
  fi
  exit 0
fi

if [[ $EUID -ne 0 ]]; then exec sudo -E "$BASE_DIR/gmsl4_start.sh" "$@"; fi
RUN_AS="${SUDO_USER:-root}"
for dev in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do [[ -c "$dev" ]] || die "missing $dev"; done
for cmd in media-ctl v4l2-ctl dtc fuser; do command -v "$cmd" >/dev/null || die "missing command: $cmd"; done
python3 -c 'import gi' || die "python3 GStreamer bindings are required"

OVERLAY=/boot/tegra234-seeed-gmsl2x1x4-3g-overlay.dtbo
[[ -r "$OVERLAY" ]] || die "missing active GMSL overlay: $OVERLAY"
overlay_dts=$(mktemp)
dtc -I dtb -O dts "$OVERLAY" > "$overlay_dts" 2>/dev/null || die "cannot inspect GMSL overlay"
grep -q fsync_mfp_in "$overlay_dts" || die "FSYNC input is absent from the active overlay"
for index in 0 1 2 3; do grep -q "ser_${index}_mfp7_fsync" "$overlay_dts" || die "FSYNC output for camera $index is absent"; done
rm -f "$overlay_dts"
log "FSYNC overlay preflight passed"

busy=$(fuser /dev/video0 /dev/video1 /dev/video2 /dev/video3 2>/dev/null || true)
[[ -z "$busy" ]] || die "camera devices are busy: $busy"
for index in 0 1 2 3; do
  media-ctl -d /dev/media0 --set-v4l2 "\"ser_${index}_ch_${index}\":1[fmt:YUYV8_1X16/1920x1536]" || true
  media-ctl -d /dev/media0 --set-v4l2 "\"des_0_ch_${index}\":0[fmt:YUYV8_1X16/1920x1536]" || true
  v4l2-ctl -d "/dev/video${index}" --set-fmt-video=width=1920,height=1536 -c sensor_mode=0 >/dev/null
done

if [[ "$MODE" != verify ]]; then
  [[ -n "${DISPLAY:-}" ]] || [[ ! -e /tmp/.X11-unix/X0 ]] || export DISPLAY=:0
  [[ -n "${DISPLAY:-}" ]] || die "no local X11 display; use verify in headless mode"
fi
[[ "$SECONDS_ARG" =~ ^[0-9]+$ ]] || die "seconds must be a non-negative integer"
outdir="$BASE_DIR/out_${MODE}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$outdir"
[[ "$RUN_AS" == root ]] || chown -R "$RUN_AS":"$RUN_AS" "$outdir"
mtx_pid=""
capture_pid=""
cleanup() {
  local rc=$?
  [[ -n "$capture_pid" ]] && kill -- -"$capture_pid" 2>/dev/null || true
  [[ -n "$mtx_pid" ]] && kill -- -"$mtx_pid" 2>/dev/null || true
  rm -f "$PIDFILE"
  exit "$rc"
}
trap cleanup INT TERM EXIT

install_mediamtx() {
  [[ -x "$MEDIAMTX_BIN" ]] && return
  command -v curl >/dev/null || die "curl is required for first RTSP relay install"
  temp=$(mktemp -d)
  archive="mediamtx_v${MEDIAMTX_VERSION}_linux_arm64.tar.gz"
  base="https://github.com/bluenviron/mediamtx/releases/download/v${MEDIAMTX_VERSION}"
  log "downloading MediaMTX v${MEDIAMTX_VERSION} for ARM64"
  curl --fail --location --silent --show-error "$base/$archive" -o "$temp/$archive"
  curl --fail --location --silent --show-error "$base/checksums.sha256" -o "$temp/checksums.sha256"
  (cd "$temp" && grep "\*$archive$" checksums.sha256 | sha256sum --check -) || die "MediaMTX checksum verification failed"
  mkdir -p "$MEDIAMTX_DIR"
  tar -xzf "$temp/$archive" -C "$MEDIAMTX_DIR" mediamtx
  chmod 0755 "$MEDIAMTX_BIN"
  rm -rf "$temp"
}

if [[ "$MODE" == stream ]]; then
  install_mediamtx
  cat > "$outdir/mediamtx.yml" <<EOF
logLevel: info
rtsp: true
rtspAddress: :${RTSP_PORT}
hls: false
webrtc: false
paths:
  gmsl4:
    source: udp+mpegts://127.0.0.1:${UDP_PORT}
EOF
  [[ "$RUN_AS" == root ]] || chown "$RUN_AS":"$RUN_AS" "$outdir/mediamtx.yml"
  setsid sudo -u "$RUN_AS" "$MEDIAMTX_BIN" "$outdir/mediamtx.yml" > "$outdir/mediamtx.log" 2>&1 &
  mtx_pid=$!
  sleep 1
  kill -0 "$mtx_pid" 2>/dev/null || { cat "$outdir/mediamtx.log" >&2; die "MediaMTX did not start"; }
  log "RTSP: rtsp://$(hostname -I | awk '{print $1}'):${RTSP_PORT}/gmsl4"
fi

embedded_capture() {
  awk 'found { if ($0 == "__GMSL4_CAPTURE_END__") exit; print } $0 == "# __GMSL4_CAPTURE_BEGIN__" { found=1 }' "$0"
}

embedded_capture | setsid sudo -u "$RUN_AS" env DISPLAY="${DISPLAY:-}" HOME="/home/$RUN_AS" GMSL4_SINK="${GMSL4_SINK:-autovideosink}" \
  python3 - --mode "$MODE" --seconds "$SECONDS_ARG" --outdir "$outdir" --udp-port "$UDP_PORT" &
capture_pid=$!
printf '%s %s\n' "$capture_pid" "${mtx_pid:--}" > "$PIDFILE"
wait "$capture_pid"
capture_pid=""
log "output: $outdir"
exit 0

: <<'__GMSL4_CAPTURE_END__'
# __GMSL4_CAPTURE_BEGIN__
#!/usr/bin/env python3
"""Four GMSL cameras in one pipeline: verify, HDMI mosaic, and RTSP ingest."""
import argparse
import csv
import json
import os
import signal
import sys
import time

import gi
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst

CAMERAS = 4
WIDTH, HEIGHT, FPS = 1920, 1536, 30
CELL_W, CELL_H = WIDTH // 2, HEIGHT // 2


def add(pipe, *elements):
    for element in elements:
        if element is None:
            raise RuntimeError("required GStreamer element is unavailable")
        pipe.add(element)


def link(*elements):
    for left, right in zip(elements, elements[1:]):
        if not left.link(right):
            raise RuntimeError("cannot link %s -> %s" % (left.name, right.name))


def set_prop(element, name, value):
    if element.find_property(name) is not None:
        element.set_property(name, value)


def queue(name, buffers=4, leaky=True):
    item = Gst.ElementFactory.make("queue", name)
    item.set_property("max-size-buffers", buffers)
    item.set_property("max-size-bytes", 0)
    item.set_property("max-size-time", 0)
    if leaky:
        item.set_property("leaky", 2)  # downstream: keep the newest frame
    return item


class CameraStats:
    def __init__(self, camera_id, output_dir):
        self.camera_id = camera_id
        self.path = os.path.join(output_dir, "cam%d_timestamps.csv" % camera_id)
        self.file = open(self.path, "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(("sequence", "capture_pts_ns", "buffer_offset", "arrival_mono_ns"))
        self.rows = []
        self.sequence = 0

    def sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        pts = -1 if buffer.pts == Gst.CLOCK_TIME_NONE else int(buffer.pts)
        offset = -1 if buffer.offset == Gst.BUFFER_OFFSET_NONE else int(buffer.offset)
        arrival = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.writer.writerow((self.sequence, pts, offset, arrival))
        self.rows.append((pts, offset, arrival))
        self.sequence += 1
        if self.sequence % 120 == 0:
            self.file.flush()
        return Gst.FlowReturn.OK

    def close(self):
        self.file.flush()
        self.file.close()


def build(mode, stats, udp_port):
    pipe = Gst.Pipeline.new("gmsl4")
    mosaic_mode = mode in ("preview", "stream")
    comp = None
    if mosaic_mode:
        comp = Gst.ElementFactory.make("compositor", "mosaic")
        comp.set_property("background", 1)
        add(pipe, comp)
    source_caps = Gst.Caps.from_string(
        "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1" % (WIDTH, HEIGHT, FPS))
    tile_caps = Gst.Caps.from_string(
        "video/x-raw,width=%d,height=%d,framerate=%d/1" % (CELL_W, CELL_H, FPS))

    for camera_id in range(CAMERAS):
        source = Gst.ElementFactory.make("v4l2src", "source%d" % camera_id)
        source.set_property("device", "/dev/video%d" % camera_id)
        set_prop(source, "io-mode", 2)  # mmap on Tegra VI
        caps = Gst.ElementFactory.make("capsfilter", "source_caps%d" % camera_id)
        caps.set_property("caps", source_caps)
        source_q = queue("source_q%d" % camera_id, 6, False)
        tee = Gst.ElementFactory.make("tee", "tee%d" % camera_id)
        tee.set_property("allow-not-linked", True)
        add(pipe, source, caps, source_q, tee)
        link(source, caps, source_q, tee)

        stats_q = queue("stats_q%d" % camera_id, 16)
        sink = Gst.ElementFactory.make("appsink", "stats%d" % camera_id)
        sink.set_property("emit-signals", True)
        sink.set_property("sync", False)
        sink.set_property("max-buffers", 16)
        sink.set_property("drop", True)
        add(pipe, stats_q, sink)
        link(tee, stats_q, sink)
        sink.connect("new-sample", stats[camera_id].sample)

        if mosaic_mode:
            tile_q = queue("tile_q%d" % camera_id, 3)
            convert = Gst.ElementFactory.make("videoconvert", "convert%d" % camera_id)
            label = Gst.ElementFactory.make("textoverlay", "label%d" % camera_id)
            label.set_property("text", "CAM %d" % camera_id)
            label.set_property("valignment", "top")
            label.set_property("halignment", "left")
            label.set_property("font-desc", "Sans Bold 28")
            scale = Gst.ElementFactory.make("videoscale", "scale%d" % camera_id)
            filt = Gst.ElementFactory.make("capsfilter", "tile_caps%d" % camera_id)
            filt.set_property("caps", tile_caps)
            tile_out = queue("tile_out%d" % camera_id, 2)
            add(pipe, tile_q, convert, label, scale, filt, tile_out)
            link(tee, tile_q, convert, label, scale, filt, tile_out)
            pad = comp.get_request_pad("sink_%d" % camera_id)
            pad.set_property("xpos", (camera_id % 2) * CELL_W)
            pad.set_property("ypos", (camera_id // 2) * CELL_H)
            if tile_out.get_static_pad("src").link(pad) != Gst.PadLinkReturn.OK:
                raise RuntimeError("cannot add CAM %d to mosaic" % camera_id)

    if not mosaic_mode:
        return pipe

    output_caps = Gst.ElementFactory.make("capsfilter", "mosaic_caps")
    output_caps.set_property("caps", Gst.Caps.from_string(
        "video/x-raw,width=%d,height=%d,framerate=%d/1" % (WIDTH, HEIGHT, FPS)))
    convert = Gst.ElementFactory.make("videoconvert", "mosaic_convert")
    tee = Gst.ElementFactory.make("tee", "mosaic_tee")
    add(pipe, output_caps, convert, tee)
    link(comp, output_caps, convert, tee)

    display_q = queue("display_q", 2)
    display_name = os.environ.get("GMSL4_SINK", "autovideosink")
    display = Gst.ElementFactory.make(display_name, "display")
    if display is None:
        raise RuntimeError("display sink unavailable: %s" % display_name)
    set_prop(display, "sync", False)
    add(pipe, display_q, display)
    link(tee, display_q, display)

    if mode == "stream":
        encode_q = queue("encode_q", 3)
        nvconv = Gst.ElementFactory.make("nvvidconv", "nvvidconv")
        nvmm = Gst.ElementFactory.make("capsfilter", "nvmm_caps")
        nvmm.set_property("caps", Gst.Caps.from_string(
            "video/x-raw(memory:NVMM),format=NV12,width=%d,height=%d,framerate=%d/1" %
            (WIDTH, HEIGHT, FPS)))
        encoder = Gst.ElementFactory.make("nvv4l2h265enc", "h265")
        for name, value in (("bitrate", 12000000), ("iframeinterval", FPS),
                            ("idrinterval", FPS), ("insert-sps-pps", True),
                            ("maxperf-enable", True)):
            set_prop(encoder, name, value)
        parser = Gst.ElementFactory.make("h265parse", "h265parse")
        set_prop(parser, "config-interval", -1)
        muxer = Gst.ElementFactory.make("mpegtsmux", "mpegtsmux")
        set_prop(muxer, "alignment", 7)
        udp = Gst.ElementFactory.make("udpsink", "rtsp_ingest")
        udp.set_property("host", "127.0.0.1")
        udp.set_property("port", udp_port)
        udp.set_property("sync", False)
        udp.set_property("async", False)
        add(pipe, encode_q, nvconv, nvmm, encoder, parser, muxer, udp)
        link(tee, encode_q, nvconv, nvmm, encoder, parser, muxer, udp)
    return pipe


def report(stats, mode, output_dir, duration):
    data = {"mode": mode, "duration_s": round(duration, 3), "width": WIDTH,
            "height": HEIGHT, "fps_target": FPS,
            "sync_note": "Capture PTS is diagnostic only. Verify hardware FSYNC with a common LED or moving target."}
    cameras = {}
    for camera in stats:
        arrivals = [row[2] for row in camera.rows]
        elapsed = (arrivals[-1] - arrivals[0]) / 1e9 if len(arrivals) > 1 else 0
        cameras["cam%d" % camera.camera_id] = {
            "frames": len(camera.rows), "fps": round(len(arrivals) / elapsed, 2) if elapsed else 0}
    data["cameras"] = cameras
    data["pts_offset_vs_cam0_ms"] = {}
    ref = stats[0].rows
    for camera in stats[1:]:
        offsets = [(camera.rows[i][0] - ref[i][0]) / 1e6
                   for i in range(min(len(ref), len(camera.rows)))
                   if ref[i][0] >= 0 and camera.rows[i][0] >= 0]
        if offsets:
            data["pts_offset_vs_cam0_ms"]["cam%d" % camera.camera_id] = {
                "mean": round(sum(offsets) / len(offsets), 3),
                "peak_to_peak": round(max(offsets) - min(offsets), 3), "samples": len(offsets)}
    with open(os.path.join(output_dir, "stats.json"), "w") as handle:
        json.dump(data, handle, indent=2)
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("verify", "preview", "stream", "rec"), default="verify")
    parser.add_argument("--seconds", type=int, default=10)
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--udp-port", type=int, default=5600)
    args = parser.parse_args()
    if args.mode == "verify" and args.seconds <= 0:
        args.seconds = 10
    os.makedirs(args.outdir, exist_ok=True)
    Gst.init(None)
    stats = [CameraStats(i, args.outdir) for i in range(CAMERAS)]
    pipe = build(args.mode, stats, args.udp_port)
    loop = GLib.MainLoop()
    running = [True]

    def stop(*_args):
        if running[0]:
            running[0] = False
            pipe.set_state(Gst.State.NULL)
            loop.quit()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if args.seconds > 0:
        GLib.timeout_add_seconds(args.seconds, stop)
    bus = pipe.get_bus()
    bus.add_signal_watch()
    def bus_message(_bus, message):
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            print("GStreamer ERROR: %s (%s)" % (error.message, debug), file=sys.stderr)
            stop()
        elif message.type == Gst.MessageType.EOS:
            stop()
    bus.connect("message", bus_message)
    started = time.monotonic()
    print("[gmsl4] %s started" % args.mode, flush=True)
    pipe.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    finally:
        pipe.set_state(Gst.State.NULL)
        for camera in stats:
            camera.close()
    summary = report(stats, args.mode, args.outdir, time.monotonic() - started)
    print("[gmsl4] stats: %s" % os.path.join(args.outdir, "stats.json"), flush=True)
    for name, value in summary["cameras"].items():
        print("[gmsl4] %s: %d frames, %.2f fps" % (name, value["frames"], value["fps"]), flush=True)


if __name__ == "__main__":
    main()
__GMSL4_CAPTURE_END__
