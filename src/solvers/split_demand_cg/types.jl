"""
    SplitDemandRoute

A feasible route for the capacity-aware split-demand CG model.
`alpha[i]` holds the units of demand i carried by this route; α_ir ≤ min(D_i, Q).
Multiple routes together satisfy Σ_r α_ir λ_r ≥ D_i.
"""
struct SplitDemandRoute
    cordeau_seq    :: Vector{Int}        # Cordeau node IDs of non-depot stops (0-based)
    julia_seq      :: Vector{Int}        # Julia indices of same stops
    request_ids    :: Vector{Int}        # sorted request IDs visited
    alpha          :: Dict{Int,Int}      # request_id → units covered (α_ir ≤ min(D_i, Q))
    cost           :: Float64            # total travel cost (distance)
    reduced_cost   :: Float64            # c̄_r from pricing; NaN for seed routes
    service_times  :: Vector{Float64}
    loads          :: Vector{Int}        # cumulative vehicle load after each stop
    ride_times     :: Vector{Float64}    # one entry per pickup, in route order
    total_distance :: Float64
    total_duration :: Float64
end

"""
    SplitDemandDuals

Dual variables from the split-demand LP relaxation.
`pi[i]` is the dual of the demand-coverage constraint for request i.
"""
struct SplitDemandDuals
    pi :: Vector{Float64}
end

"""
    SplitDemandPool

Monotonically growing pool of split-demand columns.
"""
mutable struct SplitDemandPool
    routes :: Vector{SplitDemandRoute}
end
SplitDemandPool() = SplitDemandPool(SplitDemandRoute[])

"""
    SplitDemandLabel

Partial route label for the capacity-aware split-demand SPPRC.

`load` tracks the current vehicle load (bounded by Q).
`alpha` stores the committed units for every pickup visited so far (permanent).
`onboard` is the subset of alpha that has not yet been dropped off (used for
ride-time feasibility and dropoff coupling checks).
"""
mutable struct SplitDemandLabel
    node       :: Int
    rc         :: Float64
    time       :: Float64
    load       :: Int                    # current vehicle load ∈ [0, Q]
    alpha      :: Dict{Int,Int}          # request_id → committed units (never removed)
    onboard    :: Dict{Int,Int}          # request_id → units currently on vehicle
    ride_start :: Dict{Int,Float64}      # request_id → service-begin time at pickup
    path       :: Vector{Int}            # Julia indices visited (including origin)
    distance   :: Float64
end
