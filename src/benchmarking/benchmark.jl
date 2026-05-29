using Printf

"""
    BenchmarkResult

Result for a single solver/instance pair.

Extended fields:
- `lp_bound`     : LP relaxation lower bound (NaN if solver does not provide it).
- `cpu_time_sec` : CPU time (user + system via getrusage); wall time as fallback.
- `n_cg_iters`   : Number of CG pricing iterations (0 for non-CG solvers).
"""
struct BenchmarkResult
    instance_name   :: String
    solver_name     :: String
    status          :: Symbol
    objective_value :: Float64
    lp_bound        :: Float64
    solve_time_sec  :: Float64
    cpu_time_sec    :: Float64
    is_feasible     :: Bool
    n_requests      :: Int
    n_vehicles      :: Int
    n_cg_iters      :: Int
end

# CPU time via getrusage (user + sys); falls back to wall time if unavailable.
function _cpu_time_secs() :: Float64
    try
        ru = zeros(Int64, 18)
        ccall(:getrusage, Cint, (Cint, Ptr{Cvoid}), 0, pointer(ru))
        return ru[1] + ru[2] / 1_000_000.0 + ru[3] + ru[4] / 1_000_000.0
    catch
        return time()
    end
end

"""
    run_benchmark(solver, instance_dir; pattern, verbose) :: Vector{BenchmarkResult}

Solve every file in `instance_dir` matching `pattern`. Exceptions per instance are
caught so a single failure does not abort the suite.
"""
function run_benchmark(
    solver       :: AbstractDARPSolver,
    instance_dir :: String;
    pattern      :: Regex = r".*\.txt$",
    verbose      :: Bool  = true
) :: Vector{BenchmarkResult}

    files = filter(f -> occursin(pattern, f), readdir(instance_dir; join=true))
    sort!(files)

    results = BenchmarkResult[]
    for fpath in files
        inst_name = splitext(basename(fpath))[1]
        verbose && print("  Solving $inst_name ... ")
        cpu_before = _cpu_time_secs()
        try
            sol        = solve(solver, fpath)
            cpu_after  = _cpu_time_secs()
            push!(results, BenchmarkResult(
                inst_name,
                sol.solver_name,
                sol.status,
                sol.objective_value,
                isnan(sol.lp_bound) ? Inf : sol.lp_bound,
                sol.solve_time_sec,
                cpu_after - cpu_before,
                sol.is_feasible,
                sol.instance.n,
                sol.instance.K,
                sol.n_cg_iters
            ))
            verbose && @printf("%-12s  obj=%.2f  lp=%.2f  t=%.1fs  iters=%d\n",
                               sol.status, sol.objective_value,
                               isnan(sol.lp_bound) ? Inf : sol.lp_bound,
                               sol.solve_time_sec, sol.n_cg_iters)
        catch e
            cpu_after = _cpu_time_secs()
            push!(results, BenchmarkResult(inst_name, string(typeof(solver)),
                                           :error, Inf, Inf, 0.0,
                                           cpu_after - cpu_before, false, 0, 0, 0))
            verbose && println("ERROR: $e")
        end
    end
    return results
end

"""
    run_benchmark_suite(solvers, instance_dir; kwargs...) :: Vector{BenchmarkResult}

Run multiple solvers over the same instance directory and pool all results.
"""
function run_benchmark_suite(
    solvers      :: Vector{<:AbstractDARPSolver},
    instance_dir :: String;
    kwargs...
) :: Vector{BenchmarkResult}

    all_results = BenchmarkResult[]
    for solver in solvers
        append!(all_results, run_benchmark(solver, instance_dir; kwargs...))
    end
    return all_results
end

"""
    write_benchmark_csv(results, filepath)

Write summary benchmark results to a CSV file (one row per solver/instance pair).
Columns: instance, solver, status, objective (IP upper bound), lp_bound (LP lower bound),
         solve_time_sec, cpu_time_sec, feasible, n_requests, n_vehicles, n_cg_iters.
"""
function write_benchmark_csv(results::Vector{BenchmarkResult}, filepath::String)
    mkpath(dirname(filepath))
    open(filepath, "w") do io
        println(io, "instance,solver,status,objective,lp_bound,solve_time_sec,cpu_time_sec,feasible,n_requests,n_vehicles,n_cg_iters")
        for r in results
            obj_str = isfinite(r.objective_value) ? @sprintf("%.6f", r.objective_value) : "Inf"
            lp_str  = isfinite(r.lp_bound)        ? @sprintf("%.6f", r.lp_bound)        : "Inf"
            @printf(io, "%s,%s,%s,%s,%s,%.4f,%.4f,%s,%d,%d,%d\n",
                    r.instance_name, r.solver_name, r.status,
                    obj_str, lp_str,
                    r.solve_time_sec, r.cpu_time_sec, r.is_feasible,
                    r.n_requests, r.n_vehicles, r.n_cg_iters)
        end
    end
end

"""
    write_iter_log_csv(sols, filepath)

Write per-CG-iteration records from a vector of DARPSolutions to a CSV file.
Rows with empty iter_log (non-CG solvers or single-iteration solves) are skipped.
Columns: instance, solver, iter, lp_obj, cols_added.
"""
function write_iter_log_csv(sols::Vector{DARPSolution}, filepath::String)
    mkpath(dirname(filepath))
    open(filepath, "w") do io
        println(io, "instance,solver,iter,lp_obj,cols_added")
        for sol in sols
            isempty(sol.iter_log) && continue
            for entry in sol.iter_log
                @printf(io, "%s,%s,%d,%.6f,%d\n",
                        sol.instance.name, sol.solver_name,
                        entry.iter, entry.lp_obj, entry.cols_added)
            end
        end
    end
end

"""
    print_benchmark_table(results)

Print a formatted ASCII table of benchmark results.
"""
function print_benchmark_table(results::Vector{BenchmarkResult})
    header = @sprintf("%-20s  %-22s  %-12s  %10s  %10s  %8s  %8s  %5s  %4s  %4s  %5s",
                      "Instance", "Solver", "Status",
                      "Objective", "LP Bound", "Wall(s)", "CPU(s)",
                      "Feas", "n", "K", "CGit")
    sep = "─"^length(header)
    println(sep)
    println(header)
    println(sep)
    for r in results
        obj_str = isfinite(r.objective_value) ? @sprintf("%.4f", r.objective_value) : "Inf"
        lp_str  = isfinite(r.lp_bound)        ? @sprintf("%.4f", r.lp_bound)        : "Inf"
        @printf("%-20s  %-22s  %-12s  %10s  %10s  %8.2f  %8.2f  %5s  %4d  %4d  %5d\n",
                r.instance_name, r.solver_name, r.status,
                obj_str, lp_str,
                r.solve_time_sec, r.cpu_time_sec, r.is_feasible,
                r.n_requests, r.n_vehicles, r.n_cg_iters)
    end
    println(sep)
end
