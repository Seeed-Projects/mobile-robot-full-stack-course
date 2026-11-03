#!/usr/bin/env bash
set -euo pipefail
m6_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${m6_repo_dir}/modules/m06_navigation/run_navigation.sh" mode:=mapping "$@"
