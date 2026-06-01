"""
    solve_split_demand_pricing(instance, duals; ...) → (Vector{SplitDemandRoute}, Bool)

SPPRC pricing subproblem for the capacity-aware split-demand CG model.

At every pickup i, instead of always boarding D_i units (uncapped), the algorithm
enumerates every feasible boarding amount α = 1…min(Q − current_load, D_i), creating
one child label per value. This generates routes with varying α_ir coefficients that
are used as column coefficients in the master LP coverage constraint Σ α_ir λ_r ≥ D_i.

Returns `(routes, exhausted)`:
- `routes`    — negative-reduced-cost routes found, sorted ascending by reduced cost
- `exhausted` — true iff the DFS completed without hitting the time limit (open stack
                empty at exit). Only when exhausted=true AND routes is empty can the
                caller conclude no improving column exists and the LP is optimal.
"""
function solve_split_demand_pricing(
    instance      :: DARPInstance,
    duals         :: SplitDemandDuals;
    detour_factor :: Float64 = Inf,
    rc_tolerance  :: Float64 = -1e-6,
    max_routes    :: Int     = 10,
    time_limit    :: Float64 = 60.0
) :: Tuple{Vector{SplitDemandRoute}, Bool}
    n      = instance.n
    Q      = instance.Q
    T      = instance.T
    L      = instance.L
    N      = 2 * n + 2
    origin = 1
    dest   = N

    pp = _build_sd_demand_preprocess(instance)
    tw_s        = pp.tw_s
    tw_e        = pp.tw_e
    svc_time    = pp.svc_time
    is_pickup   = pp.is_pickup
    is_dropoff  = pp.is_dropoff
    request_of  = pp.request_of
    direct_time = pp.direct_time
    D           = pp.D

    tt = instance.travel_time
    td = instance.travel_distance

    Labels    = [SplitDemandLabel[] for _ in 1:N]
    completed = SplitDemandLabel[]

    root = SplitDemandLabel(
        origin, 0.0, tw_s[origin],
        0,
        Dict{Int,Int}(), Dict{Int,Int}(), Dict{Int,Float64}(),
        [origin], 0.0
    )
    push!(Labels[origin], root)
    open_stack = SplitDemandLabel[root]

    t_start = time()

    while !isempty(open_stack)
        time() - t_start > time_limit && break
        label = pop!(open_stack)   # LIFO = DFS
        cur   = label.node

        for j in 1:N
            j == origin && continue
            j == cur    && continue
            j in label.path && continue
            cur == dest && continue

            # Dropoff only allowed if corresponding request is onboard
            if is_dropoff[j]
                haskey(label.onboard, request_of[j]) || continue
            end

            arr_j = label.time + svc_time[cur] + tt[cur, j]
            t_j   = max(tw_s[j], arr_j)
            t_j > tw_e[j] + 1e-9 && continue

            base_rc   = label.rc + tt[cur, j]
            new_dist  = label.distance + td[cur, j]
            new_path  = vcat(label.path, j)

            if is_pickup[j]
                req_j = request_of[j]
                cap_left = Q - label.load
                max_α    = min(cap_left, D[req_j])
                max_α == 0 && continue   # vehicle full, can't board this request

                for α in 1:max_α
                    new_rc    = base_rc - α * duals.pi[req_j]
                    new_load  = label.load + α
                    new_alpha   = copy(label.alpha);   new_alpha[req_j]   = α
                    new_onboard = copy(label.onboard); new_onboard[req_j] = α
                    new_rstart  = copy(label.ride_start); new_rstart[req_j] = t_j

                    # Ride-time feasibility for all currently onboard passengers
                    feasible = true
                    for (req_i, _) in label.onboard
                        elapsed = t_j - label.ride_start[req_i] -
                                  svc_time[pp.pickup_julia[req_i]]
                        if elapsed > L + 1e-9 ||
                           (isfinite(detour_factor) &&
                            elapsed > detour_factor * direct_time[req_i] + 1e-9)
                            feasible = false; break
                        end
                    end
                    !feasible && continue

                    new_label = SplitDemandLabel(
                        j, new_rc, t_j, new_load,
                        new_alpha, new_onboard, new_rstart,
                        new_path, new_dist
                    )

                    if !_sdi_is_dominated(new_label, Labels[j])
                        _sdi_remove_dominated!(Labels[j], new_label)
                        push!(Labels[j], new_label)
                        push!(open_stack, new_label)
                    end
                end

            elseif is_dropoff[j]
                req_j   = request_of[j]
                α       = label.onboard[req_j]
                new_load    = label.load - α
                new_onboard = copy(label.onboard); delete!(new_onboard, req_j)
                new_rstart  = copy(label.ride_start)
                elapsed_j   = t_j - label.ride_start[req_j] -
                              svc_time[pp.pickup_julia[req_j]]

                # Ride-time bounds
                (elapsed_j > L + 1e-9) && continue
                (isfinite(detour_factor) &&
                 elapsed_j > detour_factor * direct_time[req_j] + 1e-9) && continue

                delete!(new_rstart, req_j)

                # Ride-time feasibility for remaining onboard passengers
                feasible = true
                for (req_i, _) in label.onboard
                    req_i == req_j && continue
                    elapsed = t_j - label.ride_start[req_i] -
                              svc_time[pp.pickup_julia[req_i]]
                    if elapsed > L + 1e-9 ||
                       (isfinite(detour_factor) &&
                        elapsed > detour_factor * direct_time[req_i] + 1e-9)
                        feasible = false; break
                    end
                end
                !feasible && continue

                new_label = SplitDemandLabel(
                    j, base_rc, t_j, new_load,
                    label.alpha, new_onboard, new_rstart,
                    new_path, new_dist
                )

                if j == dest
                    isempty(new_onboard) || continue
                    t_j - tw_s[origin] > T + 1e-9 && continue
                    push!(completed, new_label)
                    continue
                end

                if !_sdi_is_dominated(new_label, Labels[j])
                    _sdi_remove_dominated!(Labels[j], new_label)
                    push!(Labels[j], new_label)
                    push!(open_stack, new_label)
                end

            else
                # Destination depot (j == dest reached from a non-dropoff)
                # All passengers must be off
                isempty(label.onboard) || continue
                t_j - tw_s[origin] > T + 1e-9 && continue

                new_label = SplitDemandLabel(
                    j, base_rc, t_j, label.load,
                    label.alpha, Dict{Int,Int}(), Dict{Int,Float64}(),
                    new_path, new_dist
                )
                push!(completed, new_label)
            end
        end
    end

    exhausted = isempty(open_stack)  # true iff DFS ran to completion, not cut by time limit

    results = SplitDemandRoute[]
    for lbl in completed
        lbl.rc >= rc_tolerance && continue
        push!(results, _sdi_label_to_route(lbl, instance, n, N, pp))
    end

    sort!(results, by = r -> r.reduced_cost)
    return results[1:min(max_routes, length(results))], exhausted
end

# ── Internal helpers ───────────────────────────────────────────────────────────

function _build_sd_demand_preprocess(instance::DARPInstance)
    n = instance.n
    N = 2 * n + 2

    tw_s        = zeros(Float64, N)
    tw_e        = zeros(Float64, N)
    svc_time    = zeros(Float64, N)
    is_pickup   = falses(N)
    is_dropoff  = falses(N)
    request_of  = zeros(Int, N)
    pickup_julia = zeros(Int, n)
    direct_time  = zeros(Float64, n)
    D            = zeros(Int, n)

    tw_s[1]     = instance.depot_origin.tw_start
    tw_e[1]     = instance.depot_origin.tw_end
    svc_time[1] = instance.depot_origin.service_time

    for i in 1:n
        pi = i + 1
        di = n + i + 1
        pu = instance.nodes[i]
        dr = instance.nodes[n + i]

        tw_s[pi]       = pu.tw_start;  tw_e[pi]       = pu.tw_end
        svc_time[pi]   = pu.service_time
        is_pickup[pi]  = true
        request_of[pi] = i
        pickup_julia[i] = pi
        D[i]           = pu.load

        tw_s[di]        = dr.tw_start;  tw_e[di]       = dr.tw_end
        svc_time[di]    = dr.service_time
        is_dropoff[di]  = true
        request_of[di]  = i

        direct_time[i] = instance.travel_time[pi, di]
    end

    tw_s[N]     = instance.depot_destination.tw_start
    tw_e[N]     = instance.depot_destination.tw_end
    svc_time[N] = instance.depot_destination.service_time

    return (; tw_s, tw_e, svc_time, is_pickup, is_dropoff,
              request_of, pickup_julia, direct_time, D)
end

# Dominance (capacity-aware, 6 conditions):
#   A dominates B at the same node if all hold:
#     1. A.rc   ≤ B.rc
#     2. A.time ≤ B.time
#     3. A.load ≤ B.load               (more remaining capacity)
#     4. keys(A.onboard) ⊆ keys(B.onboard)
#     5. For each i in A.onboard: elapsed ride time A ≤ B
#     6. For each i in A.onboard: A.onboard[i] ≥ B.onboard[i]
#        (carrying ≥ units ensures load advantage is preserved after every dropoff:
#         A.load−B.load ≤ 0 and A.onboard[i]−B.onboard[i] ≥ 0 together imply
#         A.load_after_i ≤ B.load_after_i for all future dropoffs of i)
function _sdi_dominates(A::SplitDemandLabel, B::SplitDemandLabel)
    A.rc   > B.rc   + 1e-9 && return false
    A.time > B.time + 1e-9 && return false
    A.load > B.load         && return false

    for r in keys(A.onboard)
        !haskey(B.onboard, r)       && return false
        A.onboard[r] < B.onboard[r] && return false
        A.time - A.ride_start[r] > B.time - B.ride_start[r] + 1e-9 && return false
    end

    return true
end

function _sdi_is_dominated(lbl::SplitDemandLabel, existing::Vector{SplitDemandLabel})
    for e in existing
        _sdi_dominates(e, lbl) && return true
    end
    return false
end

function _sdi_remove_dominated!(existing::Vector{SplitDemandLabel}, lbl::SplitDemandLabel)
    filter!(e -> !_sdi_dominates(lbl, e), existing)
end

function _sdi_label_to_route(
    lbl      :: SplitDemandLabel,
    instance :: DARPInstance,
    n        :: Int,
    N        :: Int,
    pp       :: NamedTuple
) :: SplitDemandRoute
    julia_seq   = lbl.path[2:end-1]
    cordeau_seq = [j - 1 for j in julia_seq]
    request_ids = sort(unique([j - 1 for j in julia_seq if 2 <= j <= n + 1]))

    # alpha committed during pricing (stored in label, not reconstructed)
    alpha = Dict(i => lbl.alpha[i] for i in request_ids)

    svc_times = _sd_demand_replay_service_times(julia_seq, instance, pp.svc_time, n, N)
    loads_v   = _sd_demand_replay_loads(julia_seq, alpha, n)

    ride_times = Float64[]
    for (idx_pu, j) in enumerate(julia_seq)
        2 <= j <= n + 1 || continue
        req_id = j - 1
        di_idx = findfirst(==(n + req_id + 1), julia_seq)
        di_idx === nothing && continue
        rt = svc_times[di_idx] - svc_times[idx_pu] - instance.nodes[req_id].service_time
        push!(ride_times, rt)
    end

    return SplitDemandRoute(
        cordeau_seq, julia_seq, request_ids,
        alpha,
        lbl.distance,
        lbl.rc,
        svc_times, loads_v, ride_times,
        lbl.distance,
        lbl.time - instance.depot_origin.tw_start
    )
end

function _sd_demand_replay_service_times(
    julia_seq :: Vector{Int},
    instance  :: DARPInstance,
    svc_time  :: Vector{Float64},
    n         :: Int,
    N         :: Int
) :: Vector{Float64}
    tt       = instance.travel_time
    cur_time = instance.depot_origin.tw_start
    prev     = 1
    times    = Float64[]
    for j in julia_seq
        arr   = cur_time + svc_time[prev] + tt[prev, j]
        t_j   = max(_sd_demand_tw_start(j, instance, n, N), arr)
        push!(times, t_j)
        cur_time = t_j
        prev     = j
    end
    return times
end

function _sd_demand_tw_start(j::Int, instance::DARPInstance, n::Int, N::Int)::Float64
    j == 1          && return instance.depot_origin.tw_start
    j == N          && return instance.depot_destination.tw_start
    2 <= j <= n + 1   && return instance.nodes[j - 1].tw_start
    n+2 <= j <= 2n+1  && return instance.nodes[j - 1].tw_start
    return 0.0
end

function _sd_demand_replay_loads(
    julia_seq :: Vector{Int},
    alpha     :: Dict{Int,Int},
    n         :: Int
) :: Vector{Int}
    load  = 0
    loads = Int[]
    for j in julia_seq
        if 2 <= j <= n + 1
            load += get(alpha, j - 1, 0)
        elseif n + 2 <= j <= 2*n + 1
            load -= get(alpha, j - n - 1, 0)
        end
        push!(loads, load)
    end
    return loads
end
