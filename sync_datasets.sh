#!/usr/bin/env bash
# Sync instance files between this machine and orcd.
# Usage:
#   ./sync_datasets.sh pull   # orcd -> local
#   ./sync_datasets.sh push   # local -> orcd

REMOTE="orcd:~/DARP.jl/experiments/"
LOCAL="$(cd "$(dirname "$0")" && pwd)/experiments/"

RSYNC_OPTS=(-avz --include="*/" --include="instances/***" --exclude="*")

case "${1:-}" in
  pull)
    echo "Pulling instance files from orcd..."
    rsync "${RSYNC_OPTS[@]}" "$REMOTE" "$LOCAL"
    ;;
  push)
    echo "Pushing instance files to orcd..."
    rsync "${RSYNC_OPTS[@]}" "$LOCAL" "$REMOTE"
    ;;
  *)
    echo "Usage: $0 {pull|push}"
    exit 1
    ;;
esac
