"""
    SplitDeliveryNoCapRoute

A feasible route for the split-delivery CG model. `alpha[i]` holds the units
of demand i carried by this route. With uncapped vehicle capacity alpha[i] = D_i always;
the field is retained so the master LP can use it as a coefficient.
"""
struct SplitDeliveryNoCapRoute
    cordeau_seq    :: Vector{Int}        # Cordeau node IDs of non-depot stops (0-based)
    julia_seq      :: Vector{Int}        # Julia indices of same stops
    request_ids    :: Vector{Int}        # sorted request IDs visited
    alpha          :: Dict{Int,Int}      # request_id → units covered (α_ir = D_i here)
    cost           :: Float64            # total travel cost (distance)
    reduced_cost   :: Float64            # c̄_r from pricing; NaN for seed routes
    service_times  :: Vector{Float64}    # B[v] at each non-depot node
    loads          :: Vector{Int}        # cumulative load after each stop (informational)
    ride_times     :: Vector{Float64}    # one entry per pickup, in route order
    total_distance :: Float64
    total_duration :: Float64
end

"""
    SplitDeliveryNoCapDuals

Dual variables from the split-delivery LP relaxation.
`pi[i]` is the dual of the demand-coverage constraint for request i (π_i ≥ 0).
"""
struct SplitDeliveryNoCapDuals
    pi :: Vector{Float64}
end

"""
    SplitDeliveryNoCapPool

Monotonically growing pool of split-delivery columns.
"""
mutable struct SplitDeliveryNoCapPool
    routes :: Vector{SplitDeliveryNoCapRoute}
end
SplitDeliveryNoCapPool() = SplitDeliveryNoCapPool(SplitDeliveryNoCapRoute[])

"""
    SplitDeliveryNoCapLabel

Partial route label for the uncapped split-delivery SPPRC.

Vehicle capacity is not enforced, so `load` is not tracked. `alpha` is always D_i
per request (deterministic once a pickup is visited), so it is not stored in the
label — it is reconstructed at route-building time from `request_ids` and instance data.
"""
mutable struct SplitDeliveryNoCapLabel
    node       :: Int
    rc         :: Float64
    time       :: Float64
    onboard    :: Vector{Int}          # sorted request IDs currently in vehicle
    ride_start :: Dict{Int,Float64}    # request_id → service-begin time at pickup
    path       :: Vector{Int}          # Julia indices visited (including origin)
    distance   :: Float64
end
