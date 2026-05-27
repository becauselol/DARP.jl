using DARP

# Usage: julia --project=. examples/run_benchmarks.jl [instance_dir] [time_limit_sec]
#
# Runs the IP solver over every .txt file in instance_dir and writes results to
# benchmark_results.csv in the current directory.

instance_dir   = length(ARGS) > 0 ? ARGS[1] : joinpath(@__DIR__, "..", "data")
time_limit_sec = length(ARGS) > 1 ? parse(Float64, ARGS[2]) : 3600.0

println("Benchmark directory : $instance_dir")
println("Time limit per instance : $(time_limit_sec)s")
println()

solver = CordeauIPSolver(time_limit_sec=time_limit_sec, verbose=false)

results = run_benchmark(solver, instance_dir; verbose=true)

println()
print_benchmark_table(results)

outpath = "benchmark_results.csv"
write_benchmark_csv(results, outpath)
println("\nResults written to: $outpath")
