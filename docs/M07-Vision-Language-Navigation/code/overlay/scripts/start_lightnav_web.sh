#!/usr/bin/env bash
set -Eeuo pipefail

# One-command LAN launcher for the simulation Web UI.
# This starts only the simulation Demo. It never starts CAN or the real SDK.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCHER="$ROOT/scripts/jetson_sim_demo.py"
HOST="${LIGHTNAV_WEB_HOST:-0.0.0.0}"
PORT="${LIGHTNAV_WEB_PORT:-8091}"
RESOLUTION="${LIGHTNAV_WEB_RESOLUTION:-960x540}"
HEALTH_URL="http://127.0.0.1:${PORT}/api/health"

ACTION="${1:-start}"
case "$ACTION" in
  start) ;;
  stop) exec python3 "$LAUNCHER" stop ;;
  status) exec python3 "$LAUNCHER" status ;;
  *) echo "用法：$0 [stop|status]" >&2; exit 2 ;;
esac

if [[ ! -x "$ROOT/mujoco_demo/.venv-cpu/bin/python" ]]; then
  echo "错误：找不到 MuJoCo CPU 环境：$ROOT/mujoco_demo/.venv-cpu/bin/python" >&2
  exit 1
fi

if python3 "$LAUNCHER" status >/tmp/lightnav-sim-status.json 2>/tmp/lightnav-sim-status.err; then
  running=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("running",False))' /tmp/lightnav-sim-status.json)
  if [[ "$running" == "True" ]]; then
    bound_host=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("process",{}).get("host","127.0.0.1"))' /tmp/lightnav-sim-status.json)
    if [[ "$bound_host" == "0.0.0.0" || "$bound_host" == "::" ]]; then
      echo "LightNav Demo 已在运行，复用现有 LAN 服务。"
    else
      active=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("health",{}).get("simulation",{}).get("active",False))' /tmp/lightnav-sim-status.json)
      if [[ "$active" == "True" ]]; then
        echo "错误：已有 Demo 正在执行导航，拒绝自动重启以免中断任务。请先在网页点击停止，再重新执行本脚本。" >&2
        exit 2
      fi
      echo "现有 Demo 只绑定本机地址，正在空闲状态下切换为 LAN 服务。"
      python3 "$LAUNCHER" stop
      python3 "$LAUNCHER" start --host "$HOST" --port "$PORT" --resolution "$RESOLUTION"
    fi
  else
    python3 "$LAUNCHER" start --host "$HOST" --port "$PORT" --resolution "$RESOLUTION"
  fi
else
  cat /tmp/lightnav-sim-status.err >&2 || true
  python3 "$LAUNCHER" start --host "$HOST" --port "$PORT" --resolution "$RESOLUTION"
fi

ready=0
for _ in $(seq 1 60); do
  if body="$(curl -fsS --max-time 2 "$HEALTH_URL" 2>/dev/null)"; then
    if BODY="$body" python3 -c 'import json,os; h=json.loads(os.environ["BODY"]); raise SystemExit(0 if h.get("ok") is True and h.get("model_ready") is True else 1)'; then
      ready=1
      break
    fi
  fi
  sleep 1
done

if [[ "$ready" != 1 ]]; then
  echo "错误：8091 在 60 秒内没有通过 health 检查。" >&2
  echo "查看最近启动目录中的 stdout.log，或执行：python3 $LAUNCHER status" >&2
  exit 1
fi

echo
echo "LightNav Demo Web 服务已就绪。"
echo "访问：http://<Jetson-IP>:${PORT}/"
echo "本机健康检查：$HEALTH_URL"
echo "停止服务：bash $0 stop"

if command -v hostname >/dev/null 2>&1; then
  addresses=$(hostname -I 2>/dev/null || true)
  [[ -n "$addresses" ]] && echo "本机 IP：$addresses"
fi
