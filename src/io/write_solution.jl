using Printf

"""
    write_solution(solution, filepath)

Write a solution to a plain-text file.
"""
function write_solution(solution::DARPSolution, filepath::String)
    open(filepath, "w") do io
        println(io, solution_summary(solution))
    end
end

"""
    solution_summary(solution) :: String

Human-readable multi-line summary of a solution.
"""
function solution_summary(solution::DARPSolution) :: String
    buf = IOBuffer()
    inst = solution.instance
    @printf(buf, "Instance    : %s  (n=%d, K=%d, Q=%d)\n",
            inst.name, inst.n, inst.K, inst.Q)
    @printf(buf, "Solver      : %s\n", solution.solver_name)
    @printf(buf, "Status      : %s\n", solution.status)
    @printf(buf, "Objective   : %.4f\n", solution.objective_value)
    @printf(buf, "Feasible    : %s\n", solution.is_feasible)
    @printf(buf, "Solve time  : %.2f s\n", solution.solve_time_sec)
    println(buf)

    for route in solution.routes
        if isempty(route.node_sequence)
            @printf(buf, "Vehicle %d: (idle)\n", route.vehicle_id)
            continue
        end
        seq_str = join(string.(route.node_sequence), " → ")
        @printf(buf, "Vehicle %d: 0 → %s → %d\n",
                route.vehicle_id, seq_str, 2*inst.n + 1)
        @printf(buf, "  Service times : %s\n",
                join([@sprintf("%.1f", t) for t in route.service_times], ", "))
        @printf(buf, "  Loads         : %s\n",
                join(string.(route.loads), ", "))
        @printf(buf, "  Distance      : %.4f\n", route.total_distance)
        @printf(buf, "  Duration      : %.4f\n", route.total_duration)
        if !isempty(route.ride_times)
            @printf(buf, "  Ride times    : %s\n",
                    join([@sprintf("%.1f", r) for r in route.ride_times], ", "))
        end
        println(buf)
    end

    return String(take!(buf))
end

function Base.show(io::IO, sol::DARPSolution)
    print(io, solution_summary(sol))
end
