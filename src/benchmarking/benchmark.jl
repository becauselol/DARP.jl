using Printf

"""
    BenchmarkResult

Result for a single solver/instance pair.
"""
struct BenchmarkResult
    instance_name   :: String
    solver_name     :: String
    status          :: Symbol
    objective_value :: Float64
    solve_time_sec  :: Float64
    is_feasible     :: Bool
    n_requests      :: Int
    n_vehicles      :: Int
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
        try
            sol = solve(solver, fpath)
            push!(results, BenchmarkResult(
                inst_name,
                sol.solver_name,
                sol.status,
                sol.objective_value,
                sol.solve_time_sec,
                sol.is_feasible,
                sol.instance.n,
                sol.instance.K
            ))
            verbose && @printf("%-12s  obj=%.2f  t=%.1fs\n",
                               sol.status, sol.objective_value, sol.solve_time_sec)
        catch e
            push!(results, BenchmarkResult(inst_name, string(typeof(solver)),
                                           :error, Inf, 0.0, false, 0, 0))
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

Write benchmark results to a CSV file.
"""
function write_benchmark_csv(results::Vector{BenchmarkResult}, filepath::String)
    open(filepath, "w") do io
        println(io, "instance,solver,status,objective,solve_time_sec,feasible,n_requests,n_vehicles")
        for r in results
            @printf(io, "%s,%s,%s,%.6f,%.4f,%s,%d,%d\n",
                    r.instance_name, r.solver_name, r.status,
                    r.objective_value, r.solve_time_sec, r.is_feasible,
                    r.n_requests, r.n_vehicles)
        end
    end
end

"""
    print_benchmark_table(results)

Print a formatted ASCII table of benchmark results.
"""
function print_benchmark_table(results::Vector{BenchmarkResult})
    header = @sprintf("%-16s  %-20s  %-12s  %10s  %10s  %8s  %4s  %4s",
                      "Instance", "Solver", "Status", "Objective", "Time (s)",
                      "Feasible", "n", "K")
    sep = "─"^length(header)
    println(sep)
    println(header)
    println(sep)
    for r in results
        obj_str = isfinite(r.objective_value) ? @sprintf("%.4f", r.objective_value) : "Inf"
        @printf("%-16s  %-20s  %-12s  %10s  %10.2f  %8s  %4d  %4d\n",
                r.instance_name, r.solver_name, r.status,
                obj_str, r.solve_time_sec, r.is_feasible,
                r.n_requests, r.n_vehicles)
    end
    println(sep)
end
