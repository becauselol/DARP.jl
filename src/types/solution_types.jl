"""
    Route

The sequence of stops served by one vehicle. `node_sequence` stores Cordeau node IDs
(0-based: 1..n = pickups, n+1..2n = dropoffs). `ride_times` has one entry per pickup
in the route, in route-visit order.
"""
mutable struct Route
    vehicle_id     :: Int
    node_sequence  :: Vector{Int}     # Cordeau node IDs, excluding depot endpoints
    service_times  :: Vector{Float64} # B[i,k]: service begin time at each stop
    loads          :: Vector{Int}     # vehicle load when leaving each stop
    ride_times     :: Vector{Float64} # actual ride time for each request served
    total_distance :: Float64
    total_duration :: Float64
end

"""
    DARPSolution

Complete solution for a DARPInstance.
`status` is one of: :optimal, :feasible, :infeasible, :timeout, :error.
"""
struct DARPSolution
    instance        :: DARPInstance
    routes          :: Vector{Route}
    objective_value :: Float64
    is_feasible     :: Bool
    solver_name     :: String
    solve_time_sec  :: Float64
    status          :: Symbol
end
