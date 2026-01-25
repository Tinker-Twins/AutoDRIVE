#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   ./scripts/unzip_and_clean.sh [root_dir]
# Unzips every .zip under the root (default: repo root), then deletes the .zip
# and matching .zip.meta so Unity only tracks the extracted assets.

ROOT="${1:-.}"

if ! command -v unzip >/dev/null 2>&1; then
  echo "unzip is required but not installed" >&2
  exit 1
fi

if [ ! -d "$ROOT" ]; then
  echo "Root directory not found: $ROOT" >&2
  exit 1
fi

while IFS= read -r -d '' zip_path; do
  target_dir=$(dirname "$zip_path")
  echo "Unzipping: $zip_path"
  unzip -nq "$zip_path" -d "$target_dir"

  echo "Removing archive: $zip_path"
  rm -f "$zip_path"

  meta_path="${zip_path}.meta"
  if [ -f "$meta_path" ]; then
    echo "Removing meta: $meta_path"
    rm -f "$meta_path"
  fi
done < <(find "$ROOT" -type f -name '*.zip' -print0)

echo "Done."
