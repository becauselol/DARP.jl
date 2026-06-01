"""
    scripts/run_single_instance.jl

Solve exactly one (solver, instance) pair and write a single-row result CSV.
Designed to be called by a SLURM job array via sbatch_single_instance.sh.

Usage:
    julia --project=. scripts/run_single_instance.jl <base_outdir> <solver> <instance_path>

Output:
    <base_outdir>/<solver>/<instance_name>.csv     — one-row benchmark result
    <base_outdir>/<solver>/<instance_name>_iters.csv — CG iteration log (CG solvers only)

Environment variables:
    DARP_TIME_LIMIT       — solve time limit in seconds (default: 1200)
    DARP_MAX_CG_ITERS     — max CG iterations (default: 10000)
    DARP_PRICING_TIME     — per-iteration pricing budget in seconds (default: 30)
    DARP_CG_PATIENCE      — consecutive exhausted-zero iterations to declare LP optimal (default: 3)
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using DARP, Printf

length(ARGS) >= 3 || error("Usage: run_single_instance.jl <base_outdir> <solver> <instance_path>")

const BASE_OUTDIR   = ARGS[1]
const SOLVER_LABEL  = ARGS[2]
const INST_PATH     = ARGS[3]

const TIME_LIMIT    = parse(Float64, get(ENV, "DARP_TIME_LIMIT",   "1200"))
const MAX_CG_ITERS  = parse(Int,     get(ENV, "DARP_MAX_CG_ITERS", "10000"))
const PRICING_TIME  = parse(Float64, get(ENV, "DARP_PRICING_TIME", "30.0"))
const CG_PATIENCE   = parse(Int,     get(ENV, "DARP_CG_PATIENCE",  "3"))

const VALID = ("DemandCG", "DemandIP", "NoCapCG", "NoCapIP")
SOLVER_LABEL in VALID || error("Unknown solver '$SOLVER_LABEL'. Choose from: $(join(VALID, ", "))")

isfile(INST_PATH) || error("Instance file not found: $INST_PATH")
const INST_NAME = splitext(basename(INST_PATH))[1]
const OUTDIR    = joinpath(BASE_OUTDIR, SOLVER_LABEL)
const CSV_PATH  = joinpath(OUTDIR, "$(INST_NAME).csv")

# Skip if result already exists with a data row (header + ≥1 result line)
if isfile(CSV_PATH) && countlines(CSV_PATH) >= 2
    println("=== Skipping $SOLVER_LABEL / $INST_NAME — result already exists ===")
    println("  $CSV_PATH")
    exit(0)
end

mkpath(OUTDIR)

println("=== Single-Instance Benchmark ===")
println("  solver   = $SOLVER_LABEL")
println("  instance = $INST_NAME")
println("  outdir   = $OUTDIR")
println("  time_lim = $(TIME_LIMIT)s")
println()

# ── Build solver ──────────────────────────────────────────────────────────────

function make_solver(label)
    if label == "DemandCG"
        return SplitDemandCGSolver(
            time_limit_sec        = TIME_LIMIT,
            max_cg_iters          = MAX_CG_ITERS,
            verbose               = false,
            solve_ip              = true,
            max_routes_per_iter   = 20,
            pricing_time_per_iter = PRICING_TIME,
            patience              = CG_PATIENCE
        )
    elseif label == "DemandIP"
        return SplitDemandIPSolver(time_limit_sec=TIME_LIMIT, verbose=false)
    elseif label == "NoCapCG"
        return SplitDeliveryNoCapCGSolver(
            time_limit_sec      = TIME_LIMIT,
            max_cg_iters        = MAX_CG_ITERS,
            verbose             = false,
            solve_ip            = true,
            max_routes_per_iter = 20
        )
    elseif label == "NoCapIP"
        return SplitDeliveryNoCapIPSolver(time_limit_sec=TIME_LIMIT, verbose=false)
    end
end

# ── CPU time helper ───────────────────────────────────────────────────────────

function _cpu_time()
    try
        ru = zeros(Int64, 18)
        ccall(:getrusage, Cint, (Cint, Ptr{Cvoid}), 0, pointer(ru))
        return ru[1] + ru[2] / 1_000_000.0 + ru[3] + ru[4] / 1_000_000.0
    catch
        return time()
    end
end

# ── Solve ─────────────────────────────────────────────────────────────────────

solver = make_solver(SOLVER_LABEL)
inst   = read_instance(INST_PATH)

println("Solving $INST_NAME (n=$(inst.n), K=$(inst.K))...")
cpu0 = _cpu_time()
sol  = solve(solver, inst)
cpu1 = _cpu_time()

result = BenchmarkResult(
    INST_NAME, sol.solver_name, sol.status,
    sol.objective_value,
    isnan(sol.lp_bound) ? Inf : sol.lp_bound,
    sol.solve_time_sec, cpu1 - cpu0,
    sol.is_feasible, inst.n, inst.K, sol.n_cg_iters
)

obj_str = isfinite(result.objective_value) ? @sprintf("%.4f", result.objective_value) : "Inf"
lp_str  = isfinite(result.lp_bound)        ? @sprintf("%.4f", result.lp_bound)        : "Inf"
@printf("%-10s  obj=%-12s  lp=%-12s  wall=%7.1fs  cpu=%7.1fs  iters=%d\n",
        result.status, obj_str, lp_str,
        result.solve_time_sec, result.cpu_time_sec, result.n_cg_iters)

# ── Write results ─────────────────────────────────────────────────────────────

iter_path = joinpath(OUTDIR, "$(INST_NAME)_iters.csv")

write_benchmark_csv([result], CSV_PATH)

if !isempty(sol.iter_log)
    write_iter_log_csv([sol], iter_path)
end

println()
println("Written: $CSV_PATH")
