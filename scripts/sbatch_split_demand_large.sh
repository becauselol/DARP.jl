#!/bin/bash
#SBATCH --job-name=darp_sd_large
#SBATCH --partition=mit_normal
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=12:00:00
#SBATCH --array=1-7
#SBATCH --output=experiments/split_demand_large/slurm_logs/%A_%a.out
#SBATCH --error=experiments/split_demand_large/slurm_logs/%A_%a.err

# Array task → n-size mapping (SplitDemandCGSolver only):
#   1 → n=150    (3 types × 3 seeds = 9 instances, ~minutes)
#   2 → n=200
#   3 → n=300
#   4 → n=400
#   5 → n=500
#   6 → n=750
#   7 → n=1000   (may approach time limit on harder instances)
#
# Submit from repo root:
#   sbatch scripts/sbatch_split_demand_large.sh
#
# Rerun a single size:
#   sbatch --array=7 scripts/sbatch_split_demand_large.sh

N_SIZES=(150 200 300 400 500 750 1000)
FILTER_N=${N_SIZES[$((SLURM_ARRAY_TASK_ID - 1))]}

PROJECT_ROOT="$SLURM_SUBMIT_DIR"
BASE_OUTDIR="${PROJECT_ROOT}/experiments/split_demand_large/${SLURM_ARRAY_JOB_ID}"

echo "=========================================="
echo "DARP Large-Instance Benchmark (DemandCG)"
echo "Array job:   ${SLURM_ARRAY_JOB_ID}  task: ${SLURM_ARRAY_TASK_ID}/7"
echo "Filter n:    ${FILTER_N}"
echo "Node:        ${SLURM_NODELIST}"
echo "Started:     $(date)"
echo "Project root:${PROJECT_ROOT}"
echo "Experiment:  ${BASE_OUTDIR}"
echo "=========================================="
echo ""

# Load modules
echo "===== Loading modules ====="
module load julia/1.10
module load gurobi/12
echo "Loaded modules:"
module list
julia --version
echo ""

# Julia depot: rsync to local scratch
echo "===== Setting up Julia depot ====="
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}"
fi
mkdir -p "$JULIA_DEPOT_PATH"
echo "JULIA_DEPOT_PATH=$JULIA_DEPOT_PATH"
echo "Copying Julia depot from ~/.julia ..."
rsync -a ~/.julia/ "$JULIA_DEPOT_PATH/"
echo "Finished copying Julia depot."
echo ""

cd "$PROJECT_ROOT"
echo "Working directory: $(pwd)"
echo ""

export DARP_TIME_LIMIT="${DARP_TIME_LIMIT:-1200}"
export DARP_SEEDS="${DARP_SEEDS:-42,123,999}"
export DARP_MAX_CG_ITERS="${DARP_MAX_CG_ITERS:-1000}"

echo "===== Benchmark settings ====="
echo "  FILTER_N          = ${FILTER_N}"
echo "  DARP_TIME_LIMIT   = ${DARP_TIME_LIMIT}s"
echo "  DARP_SEEDS        = ${DARP_SEEDS}"
echo "  DARP_MAX_CG_ITERS = ${DARP_MAX_CG_ITERS}"
echo ""

echo "===== Running benchmark ====="
julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      --threads="${SLURM_CPUS_PER_TASK:-1}" \
      "$PROJECT_ROOT/scripts/run_benchmark_large.jl" \
      "$BASE_OUTDIR" "$FILTER_N"
EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo "ERROR: benchmark failed with exit code $EXIT_CODE"
    exit $EXIT_CODE
fi

echo ""
echo "=========================================="
echo "Finished: $(date)"
echo "Results:  ${BASE_OUTDIR}/DemandCG/n${FILTER_N}/"
echo "=========================================="

exit 0
