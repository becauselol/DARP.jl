"""
    scripts/run_benchmark_large.jl

Large-instance benchmark for SplitDemandCGSolver (n = 150 → 1000).
One SLURM array task per n-size: runs all 3 instance types × 3 seeds = 9 solves.

Usage:
    julia --project=. scripts/run_benchmark_large.jl <base_outdir> <filter_n>

Arguments:
    base_outdir  — shared experiment dir (e.g. experiments/split_demand_large/<job_id>)
    filter_n     — n value for this task (150, 200, 300, 400, 500, 750, or 1000)

Environment variables:
    DARP_TIME_LIMIT     — per-solve wall-time limit in seconds (default: 1200)
    DARP_SEEDS          — comma-separated random seeds (default: "42,123,999")
    DARP_MAX_CG_ITERS   — max CG pricing iterations (default: 1000)
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using DARP, Printf
include(joinpath(@__DIR__, "generate_large_instances.jl"))

# ── Configuration ─────────────────────────────────────────────────────────────

const BASE_OUTDIR  = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "experiments", "split_demand_large", "standalone")
const FILTER_N     = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : 0
const INST_DIR     = joinpath(BASE_OUTDIR, "instances")
const OUTDIR       = FILTER_N > 0 ? joinpath(BASE_OUTDIR, "DemandCG", "n$(FILTER_N)") : joinpath(BASE_OUTDIR, "DemandCG")

const TIME_LIMIT   = parse(Float64, get(ENV, "DARP_TIME_LIMIT",   "1200"))
const RUN_SEEDS    = [parse(Int, s) for s in split(get(ENV, "DARP_SEEDS", "42,123,999"), ",")]
const MAX_CG_ITERS = parse(Int,     get(ENV, "DARP_MAX_CG_ITERS", "1000"))
const PRICING_TIME = parse(Float64, get(ENV, "DARP_PRICING_TIME", "30.0"))
const CG_PATIENCE  = parse(Int,     get(ENV, "DARP_CG_PATIENCE",  "3"))

println("=== Large-instance DemandCG Benchmark ===")
println("  base_outdir = $BASE_OUTDIR")
println("  filter_n    = $FILTER_N")
println("  time_limit  = $(TIME_LIMIT)s")
println("  seeds       = $RUN_SEEDS")
println("  max_iters   = $MAX_CG_ITERS")
println()

# ── Solver ────────────────────────────────────────────────────────────────────

const SOLVER = SplitDemandCGSolver(
    time_limit_sec        = TIME_LIMIT,
    max_cg_iters          = MAX_CG_ITERS,
    verbose               = false,
    solve_ip              = true,
    max_routes_per_iter   = 20,
    pricing_time_per_iter = PRICING_TIME,
    patience              = CG_PATIENCE
)

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

# ── Helpers ───────────────────────────────────────────────────────────────────

function _make_result(inst_name, sol, cpu_elapsed)
    BenchmarkResult(
        inst_name, sol.solver_name, sol.status,
        sol.objective_value,
        isnan(sol.lp_bound) ? Inf : sol.lp_bound,
        sol.solve_time_sec, cpu_elapsed,
        sol.is_feasible, sol.instance.n, sol.instance.K, sol.n_cg_iters
    )
end

function _error_result(inst_name, inst)
    BenchmarkResult(inst_name, "SplitDemandCGSolver", :error, Inf, Inf, 0.0, 0.0,
                    false, inst.n, inst.K, 0)
end

# ── Step 1: Generate instances ────────────────────────────────────────────────

println("=== Generating instances → $INST_DIR ===")
paths = generate_large_suite(outdir=INST_DIR, seeds=RUN_SEEDS)
println("  $(length(paths)) instances written.")

if FILTER_N > 0
    paths = filter(p -> read_instance(p).n == FILTER_N, paths)
    println("  Filtered to n=$FILTER_N: $(length(paths)) instances.")
end
println()

isempty(paths) && (println("No instances to solve — exiting."); exit(0))

# ── Step 2: Run benchmark ─────────────────────────────────────────────────────

println("=== Solving $(length(paths)) instances with DemandCG ===")
all_results = BenchmarkResult[]
all_cg_sols = DARPSolution[]

for fpath in sort(paths)
    inst      = read_instance(fpath)
    inst_name = splitext(basename(fpath))[1]
    print("  [DemandCG] $inst_name (n=$(inst.n), K=$(inst.K)) ... ")
    flush(stdout)
    cpu0 = _cpu_time()
    try
        sol  = solve(SOLVER, inst)
        cpu1 = _cpu_time()
        r    = _make_result(inst_name, sol, cpu1 - cpu0)
        push!(all_results, r)
        push!(all_cg_sols, sol)
        @printf("%-10s  obj=%-12s  lp=%-12s  wall=%7.1fs  iters=%d\n",
                r.status,
                isfinite(r.objective_value) ? @sprintf("%.2f", r.objective_value) : "Inf",
                isfinite(r.lp_bound)        ? @sprintf("%.2f", r.lp_bound)        : "Inf",
                r.solve_time_sec, r.n_cg_iters)
    catch e
        cpu1 = _cpu_time()
        println("ERROR: $e")
        push!(all_results, _error_result(inst_name, inst))
    end
    flush(stdout)
end

# ── Step 3: Write results ─────────────────────────────────────────────────────

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
