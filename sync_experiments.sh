#!/usr/bin/env bash
# Sync experiment results between this machine and orcd.
# Syncs run_*/ result CSVs; excludes SLURM logs and archived/ by default.
#
# Usage:
#   ./sync_experiments.sh pull [--logs]   # orcd -> local
#   ./sync_experiments.sh push [--logs]   # local -> orcd
#
# Flags:
#   --logs   also sync slurm_logs/ directories

REMOTE="orcd:~/DARP.jl/experiments/"
LOCAL="$(cd "$(dirname "$0")" && pwd)/experiments/"

INCLUDE_LOGS=false
DIRECTION=""
for arg in "$@"; do
  case "$arg" in
    pull|push) DIRECTION="$arg" ;;
    --logs)    INCLUDE_LOGS=true ;;
  esac
done

if [ -z "$DIRECTION" ]; then
  echo "Usage: $0 {pull|push} [--logs]"
  exit 1
fi

RSYNC_OPTS=(-avz
  --exclude="archived/"
  --exclude="instances/"
  --exclude="jobs.txt"
)

if [ "$INCLUDE_LOGS" = false ]; then
  RSYNC_OPTS+=(--exclude="slurm_logs/")
fi

case "$DIRECTION" in
  pull)
    echo "Pulling experiment results from orcd..."
    [ "$INCLUDE_LOGS" = true ] && echo "  (including slurm_logs/)"
    rsync "${RSYNC_OPTS[@]}" "$REMOTE" "$LOCAL"
    ;;
  push)
    echo "Pushing experiment results to orcd..."
    [ "$INCLUDE_LOGS" = true ] && echo "  (including slurm_logs/)"
    rsync "${RSYNC_OPTS[@]}" "$LOCAL" "$REMOTE"
    ;;
esac
