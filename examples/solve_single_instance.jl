using DARP

# Usage: julia --project=. examples/solve_single_instance.jl [path/to/instance.txt]

filepath = length(ARGS) > 0 ? ARGS[1] :
           joinpath(@__DIR__, "..", "test", "fixtures", "a2-16.txt")

println("Reading instance from: $filepath")
instance = read_instance(filepath)
println("Instance: $(instance.name)  (n=$(instance.n), K=$(instance.K), Q=$(instance.Q))")
println()

solver   = CordeauIPSolver(time_limit_sec=300.0, verbose=true)
solution = solve(solver, instance)

println()
println(solution_summary(solution))

# Optionally write to file
outpath = splitext(filepath)[1] * "_solution.txt"
write_solution(solution, outpath)
println("Solution written to: $outpath")
