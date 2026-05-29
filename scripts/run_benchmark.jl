"""
    scripts/run_benchmark.jl

End-to-end benchmark runner. Generates all instances (if not already present),
solves them with the configured solvers, and writes results to CSV.

Usage:
    julia --project=. scripts/run_benchmark.jl [outdir]

Outputs (written to outdir, default: results/):
    benchmark_results.csv   — one row per solver/instance: objective (IP upper bound),
                              lp_bound (LP lower bound), wall time, CPU time, CG iters
    cg_iter_log.csv         — per-CG-iteration LP objective and columns added
    instances/              — generated .txt instance files

Environment variables:
    DARP_TIME_LIMIT         — per-solve time limit in seconds (default: 300)
    DARP_IP_MAX_N           — skip IP solver for instances with n > this (default: 16)
    DARP_SEEDS              — comma-separated seeds (default: "42,123,999")
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using DARP, Printf
include(joinpath(@__DIR__, "generate_instances.jl"))

# ── Configuration ─────────────────────────────────────────────────────────────

const OUTDIR     = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "results")
const INST_DIR   = joinpath(OUTDIR, "instances")
const TIME_LIMIT = parse(Float64, get(ENV, "DARP_TIME_LIMIT", "300"))
const IP_MAX_N   = parse(Int,     get(ENV, "DARP_IP_MAX_N",   "16"))
const RUN_SEEDS  = [parse(Int, s) for s in split(get(ENV, "DARP_SEEDS", "42,123,999"), ",")]

# ── CPU time helper (user+sys via getrusage, falls back to wall time) ─────────

function _script_cpu_time() :: Float64
    try
        ru = zeros(Int64, 18)
        ccall(:getrusage, Cint, (Cint, Ptr{Cvoid}), 0, pointer(ru))
        return ru[1] + ru[2] / 1_000_000.0 + ru[3] + ru[4] / 1_000_000.0
    catch
        return time()
    end
end

# ── Solvers ───────────────────────────────────────────────────────────────────

const IP_SOLVER = SplitDemandIPSolver(
    time_limit_sec = TIME_LIMIT,
    verbose        = false
)

const CG_SOLVER = SplitDemandCGSolver(
    time_limit_sec      = TIME_LIMIT,
    max_cg_iters        = 1000,
    verbose             = false,
    solve_ip            = true,
    max_routes_per_iter = 20
)

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

function _error_result(inst_name, solver_name, inst)
    BenchmarkResult(inst_name, solver_name, :error, Inf, Inf, 0.0, 0.0,
                    false, inst.n, inst.K, 0)
end

# ── Step 1: Generate instances ────────────────────────────────────────────────

println("=== Generating instances → $INST_DIR ===")
paths = generate_suite(outdir=INST_DIR, seeds=RUN_SEEDS)
println("  $(length(paths)) instances written.\n")

# ── Step 2: Run benchmarks ────────────────────────────────────────────────────

println("=== Running benchmarks  time_limit=$(TIME_LIMIT)s  ip_max_n=$IP_MAX_N ===\n")

all_results = BenchmarkResult[]
all_cg_sols = DARPSolution[]          # only CG solutions (carry iter_log)

for fpath in sort(paths)
    inst      = read_instance(fpath)
    inst_name = splitext(basename(fpath))[1]

    # CG solver ────────────────────────────────────────────────────────────────
    print("  [CG] $inst_name (n=$(inst.n), K=$(inst.K)) ... ")
    cpu0 = _script_cpu_time()
    try
        sol  = solve(CG_SOLVER, inst)
        cpu1 = _script_cpu_time()
        r    = _make_result(inst_name, sol, cpu1 - cpu0)
        push!(all_results, r)
        push!(all_cg_sols, sol)
        @printf("%-10s  obj=%-10s  lp=%-10s  wall=%6.1fs  cpu=%6.1fs  iters=%d\n",
                r.status,
                isfinite(r.objective_value) ? @sprintf("%.4f", r.objective_value) : "Inf",
                isfinite(r.lp_bound)        ? @sprintf("%.4f", r.lp_bound)        : "Inf",
                r.solve_time_sec, r.cpu_time_sec, r.n_cg_iters)
    catch e
        println("ERROR: $e")
        push!(all_results, _error_result(inst_name, "SplitDemandCGSolver", inst))
    end

    # IP solver (small instances only) ─────────────────────────────────────────
    inst.n > IP_MAX_N && continue
    print("  [IP] $inst_name (n=$(inst.n), K=$(inst.K)) ... ")
    cpu0 = _script_cpu_time()
    try
        sol  = solve(IP_SOLVER, inst)
        cpu1 = _script_cpu_time()
        r    = _make_result(inst_name, sol, cpu1 - cpu0)
        push!(all_results, r)
        @printf("%-10s  obj=%-10s  lp=%-10s  wall=%6.1fs  cpu=%6.1fs\n",
                r.status,
                isfinite(r.objective_value) ? @sprintf("%.4f", r.objective_value) : "Inf",
                isfinite(r.lp_bound)        ? @sprintf("%.4f", r.lp_bound)        : "Inf",
                r.solve_time_sec, r.cpu_time_sec)
    catch e
        println("ERROR: $e")
        push!(all_results, _error_result(inst_name, "SplitDemandIPSolver", inst))
    end
end

# ── Step 3: Write results ─────────────────────────────────────────────────────

mkpath(OUTDIR)
csv_path  = joinpath(OUTDIR, "benchmark_results.csv")
iter_path = joinpath(OUTDIR, "cg_iter_log.csv")

write_benchmark_csv(all_results, csv_path)
write_iter_log_csv(all_cg_sols, iter_path)

println("\n=== Results written ===")
println("  Summary : $csv_path")
println("  Iter log: $iter_path")
println()
print_benchmark_table(all_results)
