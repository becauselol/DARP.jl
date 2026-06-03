"""
    NoDepotRoute

A feasible open-path route for the no-depot CG formulations. Routes start at a
pickup and end at the last dropoff; no depot origin or destination is included.
`alpha[i]` holds the units of demand i carried by this route.

Used by both NoDepotNoCap (alpha[i] = D_i always) and NoDepotDemand (alpha[i] ≤ min(D_i, Q)).
"""
struct NoDepotRoute
    cordeau_seq   :: Vector{Int}       # Cordeau node IDs (0-based, no depot endpoints)
    julia_seq     :: Vector{Int}       # Julia indices of the same stops
    request_ids   :: Vector{Int}       # sorted request IDs served by this route
    alpha         :: Dict{Int,Int}     # request_id → units of demand covered
    cost          :: Float64           # total travel distance (no depot legs)
    reduced_cost  :: Float64           # c̄_r from pricing; NaN for seed routes
    service_times :: Vector{Float64}   # service-begin time at each stop
    loads         :: Vector{Int}       # vehicle load after each stop
    ride_times    :: Vector{Float64}   # one entry per pickup, in route-visit order
    total_distance :: Float64
    total_duration :: Float64          # last service end − first service begin
end

"""
    NoDepotDuals

Dual variables π_i ≥ 0 for the demand-coverage constraints Σ α_{ir} λ_r ≥ D_i.
"""
struct NoDepotDuals
    pi :: Vector{Float64}
end

"""
    NoDepotPool

Monotonically growing pool of no-depot columns.
"""
mutable struct NoDepotPool
    routes :: Vector{NoDepotRoute}
end
NoDepotPool() = NoDepotPool(NoDepotRoute[])

# ── NoCap label ────────────────────────────────────────────────────────────────

"""
    NoDepotNoCapLabel

SPPRC label for the uncapped no-depot formulation. No load resource is tracked.
`alpha` is always D_i per request and is reconstructed at route-build time.
"""
mutable struct NoDepotNoCapLabel
    node       :: Int
    rc         :: Float64
    time       :: Float64
    onboard    :: Vector{Int}          # sorted request IDs currently in vehicle
    ride_start :: Dict{Int,Float64}    # request_id → service-begin time at pickup
    path       :: Vector{Int}          # Julia indices visited so far (no depot)
    distance   :: Float64
end

# ── Demand label ───────────────────────────────────────────────────────────────

"""
    NoDepotDemandLabel

SPPRC label for the capacity-constrained no-depot formulation.
Tracks load as a resource; alpha is enumerated at each pickup extension.
"""
mutable struct NoDepotDemandLabel
    node       :: Int
    rc         :: Float64
    time       :: Float64
    load       :: Int
    alpha      :: Dict{Int,Int}        # request_id → units boarded (α_ir committed)
    onboard    :: Dict{Int,Int}        # request_id → units currently on vehicle
    ride_start :: Dict{Int,Float64}
    path       :: Vector{Int}
    distance   :: Float64
end
