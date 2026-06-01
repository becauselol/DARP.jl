#!/bin/bash
#SBATCH --job-name=darp_single
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=32G
#SBATCH --time=01:00:00
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null

# Generic single-instance SLURM array job.
# Each task reads one line from the job list and solves exactly one (solver, instance) pair.
# Wall time per task ≤ DARP_TIME_LIMIT + ~10 min overhead.
#
# Usage (submit from repo root):
#   sbatch --array=1-<N> \
#          --output=<exp_dir>/slurm_logs/%A_%a.out \
#          --error=<exp_dir>/slurm_logs/%A_%a.err \
#          scripts/sbatch_single_instance.sh <jobs_file> <base_outdir>
#
# Arguments (positional, passed after the script name):
#   $1 = jobs_file   — path to the tab-separated job list (solver TAB instance_path)
#   $2 = base_outdir — base experiment output directory
#
# The array size must match the number of lines in jobs_file.

JOBS_FILE="$1"
BASE_OUTDIR="$2"
TASK=${SLURM_ARRAY_TASK_ID}
PROJECT_ROOT="$SLURM_SUBMIT_DIR"

if [ -z "$JOBS_FILE" ] || [ -z "$BASE_OUTDIR" ]; then
    echo "ERROR: Usage: sbatch_single_instance.sh <jobs_file> <base_outdir>"
    exit 1
fi

# Read this task's (solver, instance_path) from the job list
JOB_LINE=$(sed -n "${TASK}p" "$JOBS_FILE")
if [ -z "$JOB_LINE" ]; then
    echo "ERROR: No job found for task $TASK in $JOBS_FILE"
    exit 1
fi

SOLVER=$(echo "$JOB_LINE" | cut -f1)
INST_PATH=$(echo "$JOB_LINE" | cut -f2)
INST_NAME=$(basename "$INST_PATH" .txt)

echo "=========================================="
echo "DARP Single-Instance Benchmark"
echo "Array job:   ${SLURM_ARRAY_JOB_ID}  task: ${TASK}"
echo "Solver:      ${SOLVER}"
echo "Instance:    ${INST_NAME}"
echo "Node:        ${SLURM_NODELIST}"
echo "Started:     $(date)"
echo "Project root:${PROJECT_ROOT}"
echo "Output:      ${BASE_OUTDIR}/${SOLVER}/${INST_NAME}.csv"
echo "=========================================="
echo ""

# Load modules
echo "===== Loading modules ====="
module load julia/1.10
module load gurobi/12
julia --version
echo ""

# Julia depot: rsync to local scratch
echo "===== Setting up Julia depot ====="
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_${SLURM_ARRAY_JOB_ID}_${TASK}"
fi
mkdir -p "$JULIA_DEPOT_PATH"
rsync -a ~/.julia/ "$JULIA_DEPOT_PATH/"
echo "Depot ready: $JULIA_DEPOT_PATH"
echo ""

cd "$PROJECT_ROOT"

export DARP_TIME_LIMIT="${DARP_TIME_LIMIT:-1200}"
export DARP_PRICING_TIME="${DARP_PRICING_TIME:-30}"
export DARP_CG_PATIENCE="${DARP_CG_PATIENCE:-3}"

echo "===== Settings ====="
echo "  DARP_TIME_LIMIT   = ${DARP_TIME_LIMIT}s"
echo "  DARP_PRICING_TIME = ${DARP_PRICING_TIME}s"
echo "  DARP_CG_PATIENCE  = ${DARP_CG_PATIENCE}"
echo ""

echo "===== Running ====="
julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      "$PROJECT_ROOT/scripts/run_single_instance.jl" \
      "$BASE_OUTDIR" "$SOLVER" "$INST_PATH"
EXIT_CODE=$?

echo ""
echo "=========================================="
echo "Finished: $(date)  exit=$EXIT_CODE"
echo "=========================================="
exit $EXIT_CODE
