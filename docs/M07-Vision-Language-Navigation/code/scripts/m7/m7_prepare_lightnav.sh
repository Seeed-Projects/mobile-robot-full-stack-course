#!/usr/bin/env bash
# Prepare a pinned LightNav-0 checkout with the M7 course overlay and, optionally,
# the separately supplied DM-H65 CAD bundle.
#
#   m7_prepare_lightnav.sh [--project DIR] [--upstream URL|PATH] [--bundle TARBALL]
#                           [--apply] [--allow-external-project]
#
# Dry-run by default: the overlay installer and the asset preparer are both called
# without --apply, so nothing is written until --apply is given. The deployed device
# project is never prepared implicitly - writing outside the prepared-clone root
# requires --allow-external-project.
set -euo pipefail

M7_ROOT="${M7_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M7_ROOT/../../.." && pwd)}"

PINNED_COMMIT="3015508b70fb6e30bbd66701e94c9b916dcb8df8"
UPSTREAM="${LIGHTNAV_UPSTREAM:-https://github.com/lightorigins/LightNav-0.git}"
RUNTIME_ROOT="${LIGHTNAV_RUNTIME_ROOT:-$WS_ROOT/shared/lightnav}"
PROJECT="${LIGHTNAV_PROJECT:-$RUNTIME_ROOT/LightNav-0}"
BUNDLE=""
PYTHON="${PYTHON:-python3}"
APPLY=0
ALLOW_EXTERNAL=0

die() { printf '[m7] %s\n' "$*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --apply) APPLY=1 ;;
    --project) PROJECT="${2:?--project needs a directory}"; shift ;;
    --bundle) BUNDLE="${2:?--bundle needs a tarball}"; shift ;;
    --upstream) UPSTREAM="${2:?--upstream needs a URL or path}"; shift ;;
    --allow-external-project) ALLOW_EXTERNAL=1 ;;
    -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

printf '[m7] module:    %s\n' "$M7_ROOT"
printf '[m7] workspace: %s\n' "$WS_ROOT"
printf '[m7] project:   %s\n' "$PROJECT"
printf '[m7] pinned:    %s\n' "$PINNED_COMMIT"
if [ "$APPLY" = 1 ]; then printf '[m7] mode:      apply\n'; else printf '[m7] mode:      dry-run (add --apply to write)\n'; fi

if [ "$APPLY" = 1 ] && [ "$ALLOW_EXTERNAL" = 0 ]; then
  parent="$(cd "$(dirname "$PROJECT")" 2>/dev/null && pwd -P || true)"
  case "${parent:-$(dirname "$PROJECT")}/$(basename "$PROJECT")" in
    "$RUNTIME_ROOT"/*) ;;
    *) die "refusing to write outside $RUNTIME_ROOT (pass --allow-external-project to override)" ;;
  esac
fi

if [ ! -d "$PROJECT/.git" ]; then
  [ "$APPLY" = 1 ] || die "no project at $PROJECT (dry-run: re-run with --apply to clone)"
  mkdir -p "$(dirname "$PROJECT")"
  printf '[m7] cloning %s -> %s\n' "$UPSTREAM" "$PROJECT"
  git clone "$UPSTREAM" "$PROJECT"
fi

git -C "$PROJECT" rev-parse --verify --quiet "$PINNED_COMMIT^{commit}" >/dev/null \
  || die "pinned commit $PINNED_COMMIT is not present in $PROJECT"

head="$(git -C "$PROJECT" rev-parse HEAD)"
if [ "$head" != "$PINNED_COMMIT" ]; then
  [ "$APPLY" = 1 ] || die "HEAD is $head, expected $PINNED_COMMIT (re-run with --apply to check out)"
  git -C "$PROJECT" checkout --quiet --detach "$PINNED_COMMIT"
  head="$(git -C "$PROJECT" rev-parse HEAD)"
fi
[ "$head" = "$PINNED_COMMIT" ] || die "could not reach the pinned commit (HEAD=$head)"
printf '[m7] upstream:  %s\n' "$head"

if [ -n "$(git -C "$PROJECT" status --porcelain)" ]; then
  printf '[m7] note:      project has local changes; the overlay installer rejects files that match neither the upstream nor the course copy\n'
fi

printf '[m7] step 1/3:  install course overlay\n'
if [ "$APPLY" = 1 ]; then
  "$PYTHON" "$M7_ROOT/tools/install_overlay.py" --project "$PROJECT" --apply
else
  "$PYTHON" "$M7_ROOT/tools/install_overlay.py" --project "$PROJECT"
fi

if [ -n "$BUNDLE" ]; then
  [ -f "$BUNDLE" ] || die "bundle not found: $BUNDLE"
  printf '[m7] step 2/3:  prepare CAD assets from %s\n' "$BUNDLE"
  if [ "$APPLY" = 1 ]; then
    "$PYTHON" "$M7_ROOT/tools/prepare_assets.py" --project "$PROJECT" --bundle "$BUNDLE" --apply
  else
    "$PYTHON" "$M7_ROOT/tools/prepare_assets.py" --project "$PROJECT" --bundle "$BUNDLE"
  fi
else
  printf '[m7] step 2/3:  CAD assets skipped (pass --bundle <dm-h65-course-assets-*.tar.gz>)\n'
fi

chassis="$PROJECT/course-assets/dm-h65/config.json"
task="$PROJECT/course-assets/task.json"
printf '[m7] step 3/3:  preflight\n'
if [ -f "$chassis" ] && [ -f "$task" ]; then
  set +e
  "$PYTHON" "$M7_ROOT/tools/preflight.py" --project "$PROJECT" \
    --chassis-config "$chassis" --task-config "$task"
  rc=$?
  set -e
  [ "$rc" = 0 ] || printf '[m7] preflight gaps above (exit %s) are expected on a clone without the two uv environments\n' "$rc"
else
  printf '[m7] preflight skipped: %s not prepared yet\n' "$PROJECT/course-assets"
fi

printf '[m7] next:      %s/scripts/m7/m7_sim_demo.sh status --project %s\n' "$M7_ROOT" "$PROJECT"