#!/bin/bash
#SBATCH --job-name=darp_sd_3600
#SBATCH --partition=mit_preemptable
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=64G
#SBATCH --time=12:00:00
#SBATCH --array=1-20
#SBATCH --output=experiments/split_demand_3600s/slurm_logs/%A_%a.out
#SBATCH --error=experiments/split_demand_3600s/slurm_logs/%A_%a.err

# 3600s-per-instance variant of sbatch_split_demand.sh.
# Intended for comparison against the 1200s run to measure improvement.
#
# Array task → (solver, n-size) mapping:
#  Tasks  1– 7 : DemandCG   on n = 4, 8, 16, 32, 48, 64, 100
#  Tasks  8–14 : NoCapCG    on n = 4, 8, 16, 32, 48, 64, 100
#  Tasks 15–17 : DemandIP   on n = 4, 8, 16
#  Tasks 18–20 : NoCapIP    on n = 4, 8, 16
#
# Submit from repo root:
#   sbatch scripts/sbatch_split_demand_3600s.sh

CG_SIZES=(4 8 16 32 48 64 100)
IP_SIZES=(4 8 16)
TASK=${SLURM_ARRAY_TASK_ID}

if   [ "$TASK" -le 7 ];  then SOLVER="DemandCG"; FILTER_N=${CG_SIZES[$((TASK - 1))]}
elif [ "$TASK" -le 14 ]; then SOLVER="NoCapCG";  FILTER_N=${CG_SIZES[$((TASK - 8))]}
elif [ "$TASK" -le 17 ]; then SOLVER="DemandIP"; FILTER_N=${IP_SIZES[$((TASK - 15))]}
else                          SOLVER="NoCapIP";  FILTER_N=${IP_SIZES[$((TASK - 18))]}
fi

PROJECT_ROOT="$SLURM_SUBMIT_DIR"
BASE_OUTDIR="${PROJECT_ROOT}/experiments/split_demand_3600s/${SLURM_ARRAY_JOB_ID}"

echo "=========================================="
echo "DARP Split-Demand Benchmark (3600s)"
echo "Array job:   ${SLURM_ARRAY_JOB_ID}  task: ${TASK}/20"
echo "Solver:      ${SOLVER}"
echo "Filter n:    ${FILTER_N}"
echo "Node:        ${SLURM_NODELIST}"
echo "Started:     $(date)"
echo "Project root:${PROJECT_ROOT}"
echo "Experiment:  ${BASE_OUTDIR}"
echo "=========================================="
echo ""

echo "===== Loading modules ====="
module load julia/1.10
module load gurobi/12
echo "Loaded modules:"
module list
julia --version
echo ""

echo "===== Setting up Julia depot ====="
if [ -n "${SLURM_TMPDIR:-}" ]; then
    export JULIA_DEPOT_PATH="$SLURM_TMPDIR/julia_depot"
else
    export JULIA_DEPOT_PATH="/tmp/$USER/julia_depot_${SLURM_ARRAY_JOB_ID}_${TASK}"
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

export DARP_TIME_LIMIT=3600
export DARP_PRICING_TIME=60
export DARP_CG_PATIENCE=3
export DARP_IP_MAX_N="${DARP_IP_MAX_N:-20}"
export DARP_CG_MAX_N="${DARP_CG_MAX_N:-9999}"
export DARP_SEEDS="${DARP_SEEDS:-42,123,999}"

echo "===== Benchmark settings ====="
echo "  SOLVER            = ${SOLVER}"
echo "  FILTER_N          = ${FILTER_N}"
echo "  DARP_TIME_LIMIT   = ${DARP_TIME_LIMIT}s"
echo "  DARP_PRICING_TIME = ${DARP_PRICING_TIME}s"
echo "  DARP_CG_PATIENCE  = ${DARP_CG_PATIENCE}"
echo "  DARP_IP_MAX_N     = ${DARP_IP_MAX_N}"
echo "  DARP_CG_MAX_N     = ${DARP_CG_MAX_N}"
echo "  DARP_SEEDS        = ${DARP_SEEDS}"
echo ""

echo "===== Running benchmark ====="
julia --startup-file=no \
      --project="$PROJECT_ROOT" \
      --threads="${SLURM_CPUS_PER_TASK:-1}" \
      "$PROJECT_ROOT/scripts/run_benchmark_split_demand.jl" \
      "$BASE_OUTDIR" "$SOLVER" "$FILTER_N"
EXIT_CODE=$?

if [ $EXIT_CODE -ne 0 ]; then
    echo "ERROR: benchmark failed with exit code $EXIT_CODE"
    exit $EXIT_CODE
fi

echo ""
echo "=========================================="
echo "Finished: $(date)"
echo "Results:  ${BASE_OUTDIR}/${SOLVER}/n${FILTER_N}/"
echo "=========================================="

exit 0
