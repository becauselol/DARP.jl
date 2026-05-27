"""
    FeasibleRoute

A feasible vehicle route produced by the pricing subproblem or seed generation.
Stores both Cordeau (0-based) and Julia (1-based) node indices.
"""
struct FeasibleRoute
    cordeau_seq    :: Vector{Int}       # Cordeau node IDs of non-depot stops (0-based)
    julia_seq      :: Vector{Int}       # Julia indices of same stops (cordeau_id + 1)
    request_ids    :: Vector{Int}       # sorted request indices 1..n served by this route
    cost           :: Float64           # true objective value c_r (total travel time)
    reduced_cost   :: Float64           # c̄_r from pricing; NaN for seed routes
    service_times  :: Vector{Float64}   # B[v] at each non-depot node
    loads          :: Vector{Int}       # vehicle load when leaving each non-depot node
    ride_times     :: Vector{Float64}   # one entry per pickup, in route order
    total_distance :: Float64
    total_duration :: Float64
end

"""
    CGDuals

Dual variables from the LP relaxation of the restricted master problem.
`pi[i]` is the dual of the coverage constraint for request i (i = 1..n).
`mu` is the dual of the fleet size constraint (≤ 0 at optimality for ≤ constraint).
"""
struct CGDuals
    pi :: Vector{Float64}
    mu :: Float64
end

"""
    ColumnPool

Monotonically growing pool of columns (feasible routes) for the master problem.
"""
mutable struct ColumnPool
    routes :: Vector{FeasibleRoute}
end
ColumnPool() = ColumnPool(FeasibleRoute[])

"""
    Label

A partial route label for the SPPRC pricing subproblem.
Used internally by `solve_pricing`.
"""
mutable struct Label
    node       :: Int                   # current Julia index (1..2n+2)
    rc         :: Float64               # accumulated reduced cost
    time       :: Float64               # service begin time B at current node
    load       :: Int                   # current vehicle load
    onboard    :: Vector{Int}           # sorted request IDs currently in vehicle
    ride_start :: Dict{Int,Float64}     # request_id → B at its pickup node
    path       :: Vector{Int}           # Julia indices visited so far (including current)
    distance   :: Float64               # accumulated travel distance
end
