#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./tools/skip-zip-from-gitworktree.sh on   # ignore .zip and .zip.meta changes (skip-worktree)
#   ./tools/skip-zip-from-gitworktree.sh off  # resume tracking those files
#
# Run this from the repo root. It only touches files already tracked by git.

MODE="${1:-}"
if [[ "$MODE" != "on" && "$MODE" != "off" ]]; then
  echo "Usage: $0 [on|off]" >&2
  exit 1
fi

ROOT=$(git rev-parse --show-toplevel)
cd "$ROOT"

COUNT=$(git ls-files -- '*.zip' '*.zip.meta' | wc -l | tr -d ' ')
if [[ "$COUNT" == "0" ]]; then
  echo "No tracked .zip or .zip.meta files found."
  exit 0
fi

if [[ "$MODE" == "on" ]]; then
  git ls-files -z -- '*.zip' '*.zip.meta' | xargs -0 git update-index --skip-worktree
  echo "Marked ${COUNT} file(s) as skip-worktree."
else
  git ls-files -z -- '*.zip' '*.zip.meta' | xargs -0 git update-index --no-skip-worktree
  echo "Cleared skip-worktree on ${COUNT} file(s)."
fi
