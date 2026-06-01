"""
    scripts/run_benchmark_split_demand.jl

Benchmark runner for SplitDemand and SplitDeliveryNoCap solvers.
Designed to be called by a SLURM job array — one task per (solver, n-size).

Usage:
    julia --project=. scripts/run_benchmark_split_demand.jl <base_outdir> <solver> [filter_n]

Arguments:
    base_outdir  — shared experiment directory (e.g. experiments/split_demand/<job_id>)
    solver       — DemandCG | DemandIP | NoCapCG | NoCapIP | all
    filter_n     — if given, only solve instances with exactly this many requests

Output written to:
    <base_outdir>/instances/              — generated instance files (shared)
    <base_outdir>/<solver>/n<filter_n>/   — per-task results

Environment variables:
    DARP_TIME_LIMIT     — per-solve wall-time limit in seconds (default: 1200)
    DARP_IP_MAX_N       — skip IP solvers for n > this (default: 20)
    DARP_CG_MAX_N       — skip CG solvers for n > this (default: 9999)
    DARP_SEEDS          — comma-separated random seeds (default: "42,123,999")
    DARP_MAX_CG_ITERS   — max CG pricing iterations (default: 1000)
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using DARP, Printf
include(joinpath(@__DIR__, "generate_instances.jl"))

# ── Configuration ─────────────────────────────────────────────────────────────

const BASE_OUTDIR  = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "experiments", "split_demand", "standalone")
const SOLVER_LABEL = length(ARGS) >= 2 ? ARGS[2] : "all"
const FILTER_N     = length(ARGS) >= 3 ? parse(Int, ARGS[3]) : 0   # 0 = no filter

const INST_DIR     = joinpath(BASE_OUTDIR, "instances")
const OUTDIR       = FILTER_N > 0 ? joinpath(BASE_OUTDIR, SOLVER_LABEL, "n$(FILTER_N)") : joinpath(BASE_OUTDIR, SOLVER_LABEL)

const TIME_LIMIT         = parse(Float64, get(ENV, "DARP_TIME_LIMIT",         "1200"))
const IP_MAX_N           = parse(Int,     get(ENV, "DARP_IP_MAX_N",           "20"))
const CG_MAX_N           = parse(Int,     get(ENV, "DARP_CG_MAX_N",           "9999"))
const RUN_SEEDS          = [parse(Int, s) for s in split(get(ENV, "DARP_SEEDS", "42,123,999"), ",")]
const MAX_CG_ITERS       = parse(Int,     get(ENV, "DARP_MAX_CG_ITERS",       "1000"))
const PRICING_TIME       = parse(Float64, get(ENV, "DARP_PRICING_TIME",       "30.0"))
const CG_PATIENCE        = parse(Int,     get(ENV, "DARP_CG_PATIENCE",        "3"))

const VALID_SOLVERS = ("DemandCG", "DemandIP", "NoCapCG", "NoCapIP", "all")
SOLVER_LABEL in VALID_SOLVERS || error("Unknown solver '$SOLVER_LABEL'. Choose from: $(join(VALID_SOLVERS, ", "))")

# ── Solvers ───────────────────────────────────────────────────────────────────

const DEMAND_IP_SOLVER = SplitDemandIPSolver(time_limit_sec=TIME_LIMIT, verbose=false)
const DEMAND_CG_SOLVER = SplitDemandCGSolver(
    time_limit_sec=TIME_LIMIT, max_cg_iters=MAX_CG_ITERS,
    verbose=false, solve_ip=true, max_routes_per_iter=20,
    pricing_time_per_iter=PRICING_TIME, patience=CG_PATIENCE
)
const NOCAP_IP_SOLVER = SplitDeliveryNoCapIPSolver(time_limit_sec=TIME_LIMIT, verbose=false)
const NOCAP_CG_SOLVER = SplitDeliveryNoCapCGSolver(
    time_limit_sec=TIME_LIMIT, max_cg_iters=MAX_CG_ITERS,
    verbose=false, solve_ip=true, max_routes_per_iter=20
)

# label → (solver, max_n)
const SOLVER_MAP = Dict(
    "DemandCG" => (DEMAND_CG_SOLVER, CG_MAX_N),
    "DemandIP" => (DEMAND_IP_SOLVER, IP_MAX_N),
    "NoCapCG"  => (NOCAP_CG_SOLVER,  CG_MAX_N),
    "NoCapIP"  => (NOCAP_IP_SOLVER,  IP_MAX_N),
)

const RUN_LABELS = SOLVER_LABEL == "all" ? ["DemandCG", "DemandIP", "NoCapCG", "NoCapIP"] : [SOLVER_LABEL]

# ── CPU time helper ───────────────────────────────────────────────────────────

function _cpu_time() :: Float64
    try
        ru = zeros(Int64, 18)
        ccall(:getrusage, Cint, (Cint, Ptr{Cvoid}), 0, pointer(ru))
        return ru[1] + ru[2] / 1_000_000.0 + ru[3] + ru[4] / 1_000_000.0
    catch
        return time()
    end
end

# ── Solve helpers ─────────────────────────────────────────────────────────────

function _make_result(inst_name, sol, cpu_elapsed)
    BenchmarkResult(
        inst_name, sol.solver_name, sol.status,
        sol.objective_value,
        isnan(sol.lp_bound) ? Inf : sol.lp_bound,
        sol.solve_time_sec, cpu_elapsed,
        sol.is_feasible, sol.instance.n, sol.instance.K, sol.n_cg_iters
    )
end

function _error_result(inst_name, solver_name, inst)
    BenchmarkResult(inst_name, solver_name, :error, Inf, Inf, 0.0, 0.0,
                    false, inst.n, inst.K, 0)
end

function _run_solver(label, solver, inst, inst_name, results, cg_sols)
    print("  [$label] $inst_name (n=$(inst.n), K=$(inst.K)) ... ")
    flush(stdout)
    cpu0 = _cpu_time()
    try
        sol  = solve(solver, inst)
        cpu1 = _cpu_time()
        r    = _make_result(inst_name, sol, cpu1 - cpu0)
        push!(results, r)
        push!(cg_sols, sol)
        @printf("%-10s  obj=%-10s  lp=%-10s  wall=%6.1fs  cpu=%6.1fs  iters=%d\n",
                r.status,
                isfinite(r.objective_value) ? @sprintf("%.4f", r.objective_value) : "Inf",
                isfinite(r.lp_bound)        ? @sprintf("%.4f", r.lp_bound)        : "Inf",
                r.solve_time_sec, r.cpu_time_sec, r.n_cg_iters)
    catch e
        println("ERROR: $e")
        push!(results, _error_result(inst_name, string(typeof(solver)), inst))
    end
    flush(stdout)
end

# ── Step 1: Generate instances (idempotent; concurrent writes are safe) ───────

println("=== Generating instances → $INST_DIR ===")
paths = generate_suite(outdir=INST_DIR, seeds=RUN_SEEDS)
println("  $(length(paths)) instances written.\n")

# ── Step 2: Apply n-filter ────────────────────────────────────────────────────

if FILTER_N > 0
    paths = filter(p -> begin
        inst = read_instance(p)
        inst.n == FILTER_N
    end, paths)
    println("  Filtered to n=$FILTER_N: $(length(paths)) instances.\n")
end

if isempty(paths)
    println("No instances to solve — exiting.")
    exit(0)
end

# ── Step 3: Run benchmark for selected solver(s) ──────────────────────────────

println("=== Running benchmarks: $(join(RUN_LABELS, ", ")) ===")
println("  time_limit=$(TIME_LIMIT)s  ip_max_n=$IP_MAX_N  cg_max_n=$CG_MAX_N  filter_n=$(FILTER_N > 0 ? FILTER_N : "none")  seeds=$(RUN_SEEDS)")
println()

all_results = BenchmarkResult[]
all_cg_sols = DARPSolution[]

for label in RUN_LABELS
    solver, max_n = SOLVER_MAP[label]
    eligible = filter(p -> read_instance(p).n <= max_n, paths)
    if isempty(eligible)
        println("--- Solver: $label — no eligible instances (max_n=$max_n) ---\n")
        continue
    end
    println("--- Solver: $label ($(length(eligible)) instances) ---")
    for fpath in sort(eligible)
        inst      = read_instance(fpath)
        inst_name = splitext(basename(fpath))[1]
        _run_solver(label, solver, inst, inst_name, all_results, all_cg_sols)
    end
    println()
end

# ── Step 4: Write results ─────────────────────────────────────────────────────

mkpath(OUTDIR)
csv_path  = joinpath(OUTDIR, "benchmark_results.csv")
iter_path = joinpath(OUTDIR, "cg_iter_log.csv")

write_benchmark_csv(all_results, csv_path)
write_iter_log_csv(all_cg_sols, iter_path)

println("\n=== Results written to $OUTDIR ===")
println("  Summary : $csv_path")
println("  Iter log: $iter_path")
println()
print_benchmark_table(all_results)
