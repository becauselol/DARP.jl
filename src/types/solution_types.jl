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

# Per-iteration record for CG solvers: LP objective and columns added that iteration.
const CGIterLogEntry = NamedTuple{(:iter, :lp_obj, :cols_added), Tuple{Int, Float64, Int}}

"""
    DARPSolution

Complete solution for a DARPInstance.
`status` is one of: :optimal, :feasible, :infeasible, :timeout, :error.

Extended fields (populated by solvers that support them; NaN / 0 / empty otherwise):
- `lp_bound`     : LP relaxation lower bound (objective_bound for MIP; final LP obj for CG).
- `n_cg_iters`   : Number of CG pricing iterations performed.
- `iter_log`     : Per-iteration LP objective and columns added (CG solvers only).
"""
struct DARPSolution
    instance        :: DARPInstance
    routes          :: Vector{Route}
    objective_value :: Float64
    is_feasible     :: Bool
    solver_name     :: String
    solve_time_sec  :: Float64
    status          :: Symbol
    lp_bound        :: Float64
    n_cg_iters      :: Int
    iter_log        :: Vector{CGIterLogEntry}
end

# Backward-compatible constructor used by all existing solvers.
DARPSolution(instance, routes, obj, is_feas, solver, time, status) =
    DARPSolution(instance, routes, obj, is_feas, solver, time, status,
                 NaN, 0, CGIterLogEntry[])
