"""
    solve_nodepot_nocap_pricing(instance, duals; ...) → (Vector{NoDepotRoute}, Bool)

SPPRC pricing for the uncapped no-depot CG model.

No depot nodes. Each of the n pickups spawns a root label; the vehicle materialises
at pickup i at time tw_s[i] and immediately boards D_i units. A label is complete
when its onboard set becomes empty (all boarded passengers have been dropped off).
Route cost = travel distance between consecutive stops only (no depot legs).

Returns `(routes, exhausted)`:
- `routes`    — negative-reduced-cost routes sorted ascending by reduced cost
- `exhausted` — true iff DFS completed without hitting the time limit
"""
function solve_nodepot_nocap_pricing(
    instance      :: DARPInstance,
    duals         :: NoDepotDuals;
    detour_factor :: Float64 = Inf,
    rc_tolerance  :: Float64 = -1e-6,
    max_routes    :: Int     = 10,
    time_limit    :: Float64 = 60.0
) :: Tuple{Vector{NoDepotRoute}, Bool}
    n  = instance.n
    T  = instance.T
    L  = instance.L
    N  = 2 * n + 2

    pp          = _build_nd_preprocess(instance)
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

    # Per-node label sets (indices 2..2n+1; depots 1 and N unused)
    Labels    = [NoDepotNoCapLabel[] for _ in 1:N]
    completed = NoDepotNoCapLabel[]

    open_stack = NoDepotNoCapLabel[]

    # One root label per pickup: vehicle materialises at pickup i at tw_s[i]
    for i in 1:n
        pi  = pp.pickup_julia[i]
        rc0 = -Float64(D[i]) * duals.pi[i]
        root = NoDepotNoCapLabel(pi, rc0, tw_s[pi], [i],
                                 Dict(i => tw_s[pi]), [pi], 0.0)
        if !_nd_nocap_is_dominated(root, Labels[pi])
            _nd_nocap_remove_dominated!(Labels[pi], root)
            push!(Labels[pi], root)
            push!(open_stack, root)
        end
    end

    t_start = time()

    while !isempty(open_stack)
        time() - t_start > time_limit && break
        label = pop!(open_stack)
        cur   = label.node

        for j in 2:(N - 1)   # only pickup/dropoff nodes, no depots
            j == cur          && continue
            j in label.path   && continue

            # Dropoff only allowed if corresponding request is onboard
            if is_dropoff[j]
                request_of[j] ∉ label.onboard && continue
            end

            arr_j = label.time + svc_time[cur] + tt[cur, j]
            t_j   = max(tw_s[j], arr_j)
            t_j > tw_e[j] + 1e-9 && continue

            new_dist     = label.distance + td[cur, j]
            new_path     = vcat(label.path, j)
            new_onboard  = copy(label.onboard)
            new_rstart   = copy(label.ride_start)
            new_rc       = label.rc + tt[cur, j]

            if is_pickup[j]
                req_j  = request_of[j]
                new_rc -= Float64(D[req_j]) * duals.pi[req_j]
                push!(new_onboard, req_j)
                sort!(new_onboard)
                new_rstart[req_j] = t_j
            else
                # Dropoff
                req_j = request_of[j]
                filter!(r -> r != req_j, new_onboard)
                delete!(new_rstart, req_j)

                # Ride-time check for the passenger being dropped off
                elapsed_j = t_j - label.ride_start[req_j] -
                            svc_time[pp.pickup_julia[req_j]]
                elapsed_j > L + 1e-9 && continue
                isfinite(detour_factor) &&
                    elapsed_j > detour_factor * direct_time[req_j] + 1e-9 && continue
            end

            # Ride-time check for all passengers still onboard after this stop
            feasible = true
            for req_i in label.onboard
                is_dropoff[j] && req_i == request_of[j] && continue
                elapsed = t_j - label.ride_start[req_i] -
                          svc_time[pp.pickup_julia[req_i]]
                if elapsed > L + 1e-9 ||
                   (isfinite(detour_factor) &&
                    elapsed > detour_factor * direct_time[req_i] + 1e-9)
                    feasible = false; break
                end
            end
            !feasible && continue

            new_label = NoDepotNoCapLabel(j, new_rc, t_j,
                                          new_onboard, new_rstart,
                                          new_path, new_dist)

            if isempty(new_onboard)
                # Route complete: all passengers dropped off
                push!(completed, new_label)
                # Do not push to open_stack — route terminates here
                continue
            end

            if !_nd_nocap_is_dominated(new_label, Labels[j])
                _nd_nocap_remove_dominated!(Labels[j], new_label)
                push!(Labels[j], new_label)
                push!(open_stack, new_label)
            end
        end
    end

    exhausted = isempty(open_stack)

    results = NoDepotRoute[]
    for lbl in completed
        lbl.rc >= rc_tolerance && continue
        push!(results, _nd_nocap_label_to_route(lbl, instance, n, pp))
    end

    sort!(results, by = r -> r.reduced_cost)
    return results[1:min(max_routes, length(results))], exhausted
end

# ── NoDepot Demand pricing ─────────────────────────────────────────────────────

"""
    solve_nodepot_demand_pricing(instance, duals; ...) → (Vector{NoDepotRoute}, Bool)

SPPRC pricing for the capacity-constrained no-depot CG model.

Identical to the NoCap version except vehicle capacity Q is enforced and α is
enumerated at each pickup extension (α = 1…min(Q − load, D_i)), spawning one
child label per feasible boarding amount.
"""
function solve_nodepot_demand_pricing(
    instance      :: DARPInstance,
    duals         :: NoDepotDuals;
    detour_factor :: Float64 = Inf,
    rc_tolerance  :: Float64 = -1e-6,
    max_routes    :: Int     = 10,
    time_limit    :: Float64 = 60.0
) :: Tuple{Vector{NoDepotRoute}, Bool}
    n  = instance.n
    Q  = instance.Q
    L  = instance.L
    N  = 2 * n + 2

    pp          = _build_nd_preprocess(instance)
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

    Labels    = [NoDepotDemandLabel[] for _ in 1:N]
    completed = NoDepotDemandLabel[]
    open_stack = NoDepotDemandLabel[]

    # One root label per (pickup i, boarding amount α)
    for i in 1:n
        pi      = pp.pickup_julia[i]
        max_α   = min(Q, D[i])
        for α in 1:max_α
            rc0  = -Float64(α) * duals.pi[i]
            root = NoDepotDemandLabel(
                pi, rc0, tw_s[pi], α,
                Dict(i => α), Dict(i => α),
                Dict(i => tw_s[pi]), [pi], 0.0
            )
            if !_nd_demand_is_dominated(root, Labels[pi])
                _nd_demand_remove_dominated!(Labels[pi], root)
                push!(Labels[pi], root)
                push!(open_stack, root)
            end
        end
    end

    t_start = time()

    while !isempty(open_stack)
        time() - t_start > time_limit && break
        label = pop!(open_stack)
        cur   = label.node

        for j in 2:(N - 1)
            j == cur        && continue
            j in label.path && continue

            if is_dropoff[j]
                haskey(label.onboard, request_of[j]) || continue
            end

            arr_j = label.time + svc_time[cur] + tt[cur, j]
            t_j   = max(tw_s[j], arr_j)
            t_j > tw_e[j] + 1e-9 && continue

            base_rc  = label.rc + tt[cur, j]
            new_dist = label.distance + td[cur, j]
            new_path = vcat(label.path, j)

            if is_pickup[j]
                req_j    = request_of[j]
                cap_left = Q - label.load
                max_α    = min(cap_left, D[req_j])
                max_α == 0 && continue

                for α in 1:max_α
                    new_rc      = base_rc - Float64(α) * duals.pi[req_j]
                    new_load    = label.load + α
                    new_alpha   = copy(label.alpha);   new_alpha[req_j]   = α
                    new_onboard = copy(label.onboard); new_onboard[req_j] = α
                    new_rstart  = copy(label.ride_start); new_rstart[req_j] = t_j

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

                    new_label = NoDepotDemandLabel(
                        j, new_rc, t_j, new_load,
                        new_alpha, new_onboard, new_rstart,
                        new_path, new_dist
                    )

                    if !_nd_demand_is_dominated(new_label, Labels[j])
                        _nd_demand_remove_dominated!(Labels[j], new_label)
                        push!(Labels[j], new_label)
                        push!(open_stack, new_label)
                    end
                end

            else
                # Dropoff
                req_j       = request_of[j]
                α           = label.onboard[req_j]
                new_load    = label.load - α
                new_onboard = copy(label.onboard); delete!(new_onboard, req_j)
                new_rstart  = copy(label.ride_start)

                elapsed_j = t_j - label.ride_start[req_j] -
                            svc_time[pp.pickup_julia[req_j]]
                elapsed_j > L + 1e-9 && continue
                isfinite(detour_factor) &&
                    elapsed_j > detour_factor * direct_time[req_j] + 1e-9 && continue

                delete!(new_rstart, req_j)

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

                new_label = NoDepotDemandLabel(
                    j, base_rc, t_j, new_load,
                    label.alpha, new_onboard, new_rstart,
                    new_path, new_dist
                )

                if isempty(new_onboard)
                    push!(completed, new_label)
                    continue
                end

                if !_nd_demand_is_dominated(new_label, Labels[j])
                    _nd_demand_remove_dominated!(Labels[j], new_label)
                    push!(Labels[j], new_label)
                    push!(open_stack, new_label)
                end
            end
        end
    end

    exhausted = isempty(open_stack)

    results = NoDepotRoute[]
    for lbl in completed
        lbl.rc >= rc_tolerance && continue
        push!(results, _nd_demand_label_to_route(lbl, instance, n, pp))
    end

    sort!(results, by = r -> r.reduced_cost)
    return results[1:min(max_routes, length(results))], exhausted
end

# ── Shared preprocess ──────────────────────────────────────────────────────────

function _build_nd_preprocess(instance::DARPInstance)
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

    for i in 1:n
        pi = i + 1
        di = n + i + 1
        pu = instance.nodes[i]
        dr = instance.nodes[n + i]

        tw_s[pi]        = pu.tw_start;  tw_e[pi]       = pu.tw_end
        svc_time[pi]    = pu.service_time
        is_pickup[pi]   = true
        request_of[pi]  = i
        pickup_julia[i] = pi
        D[i]            = pu.load

        tw_s[di]        = dr.tw_start;  tw_e[di]       = dr.tw_end
        svc_time[di]    = dr.service_time
        is_dropoff[di]  = true
        request_of[di]  = i

        direct_time[i] = instance.travel_time[pi, di]
    end

    return (; tw_s, tw_e, svc_time, is_pickup, is_dropoff,
              request_of, pickup_julia, direct_time, D)
end

# ── NoCap dominance ───────────────────────────────────────────────────────────
#
# A dominates B at the same node if:
#   1. A.rc   ≤ B.rc
#   2. A.time ≤ B.time
#   3. keys(A.onboard) ⊆ keys(B.onboard)
#   4. For each i in A.onboard: elapsed ride time A ≤ elapsed ride time B
#
# Identical to the depot NoCapCG dominance (no load resource).

function _nd_nocap_dominates(A::NoDepotNoCapLabel, B::NoDepotNoCapLabel)
    A.rc   > B.rc   + 1e-9 && return false
    A.time > B.time + 1e-9 && return false
    for r in A.onboard
        r ∉ B.onboard && return false
        A.time - A.ride_start[r] > B.time - B.ride_start[r] + 1e-9 && return false
    end
    return true
end

function _nd_nocap_is_dominated(lbl::NoDepotNoCapLabel, existing::Vector{NoDepotNoCapLabel})
    for e in existing
        _nd_nocap_dominates(e, lbl) && return true
    end
    return false
end

function _nd_nocap_remove_dominated!(existing::Vector{NoDepotNoCapLabel}, lbl::NoDepotNoCapLabel)
    filter!(e -> !_nd_nocap_dominates(lbl, e), existing)
end

# ── Demand dominance ──────────────────────────────────────────────────────────
#
# A dominates B at the same node if:
#   1. A.rc   ≤ B.rc
#   2. A.time ≤ B.time
#   3. A.load ≤ B.load
#   4. keys(A.onboard) ⊆ keys(B.onboard)
#   5. For each i in A.onboard: elapsed ride time A ≤ B
#   6. For each i in A.onboard: A.onboard[i] ≥ B.onboard[i]

function _nd_demand_dominates(A::NoDepotDemandLabel, B::NoDepotDemandLabel)
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

function _nd_demand_is_dominated(lbl::NoDepotDemandLabel, existing::Vector{NoDepotDemandLabel})
    for e in existing
        _nd_demand_dominates(e, lbl) && return true
    end
    return false
end

function _nd_demand_remove_dominated!(existing::Vector{NoDepotDemandLabel}, lbl::NoDepotDemandLabel)
    filter!(e -> !_nd_demand_dominates(lbl, e), existing)
end

# ── Label → Route conversion ──────────────────────────────────────────────────

function _nd_nocap_label_to_route(
    lbl      :: NoDepotNoCapLabel,
    instance :: DARPInstance,
    n        :: Int,
    pp       :: NamedTuple
) :: NoDepotRoute
    julia_seq   = lbl.path                                 # no depot at start
    cordeau_seq = [j - 1 for j in julia_seq]
    request_ids = sort(unique([j - 1 for j in julia_seq if 2 <= j <= n + 1]))
    alpha       = Dict(i => pp.D[i] for i in request_ids)

    svc_times = _nd_replay_service_times(julia_seq, instance, pp.svc_time)
    loads_v   = _nd_replay_loads(julia_seq, alpha, n)

    ride_times = Float64[]
    for (idx, j) in enumerate(julia_seq)
        2 <= j <= n + 1 || continue
        req_id = j - 1
        di_idx = findfirst(==(n + req_id + 1), julia_seq)
        di_idx === nothing && continue
        rt = svc_times[di_idx] - svc_times[idx] - instance.nodes[req_id].service_time
        push!(ride_times, rt)
    end

    duration = isempty(svc_times) ? 0.0 :
        svc_times[end] + pp.svc_time[julia_seq[end]] - svc_times[1]

    return NoDepotRoute(cordeau_seq, julia_seq, request_ids, alpha,
                        lbl.distance, lbl.rc,
                        svc_times, loads_v, ride_times,
                        lbl.distance, duration)
end

function _nd_demand_label_to_route(
    lbl      :: NoDepotDemandLabel,
    instance :: DARPInstance,
    n        :: Int,
    pp       :: NamedTuple
) :: NoDepotRoute
    julia_seq   = lbl.path
    cordeau_seq = [j - 1 for j in julia_seq]
    request_ids = sort(unique([j - 1 for j in julia_seq if 2 <= j <= n + 1]))
    alpha       = Dict(i => lbl.alpha[i] for i in request_ids)

    svc_times = _nd_replay_service_times(julia_seq, instance, pp.svc_time)
    loads_v   = _nd_replay_loads(julia_seq, alpha, n)

    ride_times = Float64[]
    for (idx, j) in enumerate(julia_seq)
        2 <= j <= n + 1 || continue
        req_id = j - 1
        di_idx = findfirst(==(n + req_id + 1), julia_seq)
        di_idx === nothing && continue
        rt = svc_times[di_idx] - svc_times[idx] - instance.nodes[req_id].service_time
        push!(ride_times, rt)
    end

    duration = isempty(svc_times) ? 0.0 :
        svc_times[end] + pp.svc_time[julia_seq[end]] - svc_times[1]

    return NoDepotRoute(cordeau_seq, julia_seq, request_ids, alpha,
                        lbl.distance, lbl.rc,
                        svc_times, loads_v, ride_times,
                        lbl.distance, duration)
end

function _nd_replay_service_times(
    julia_seq :: Vector{Int},
    instance  :: DARPInstance,
    svc_time  :: Vector{Float64}
) :: Vector{Float64}
    tt       = instance.travel_time
    n        = instance.n
    N        = 2 * n + 2
    times    = Float64[]
    isempty(julia_seq) && return times

    # First node: vehicle arrives exactly at tw_start (no depot travel)
    j0       = julia_seq[1]
    cur_time = _nd_tw_start(j0, instance, n, N)
    push!(times, cur_time)
    prev = j0

    for j in julia_seq[2:end]
        arr   = cur_time + svc_time[prev] + tt[prev, j]
        t_j   = max(_nd_tw_start(j, instance, n, N), arr)
        push!(times, t_j)
        cur_time = t_j
        prev     = j
    end
    return times
end

function _nd_tw_start(j::Int, instance::DARPInstance, n::Int, N::Int)::Float64
    2 <= j <= n + 1   && return instance.nodes[j - 1].tw_start
    n+2 <= j <= 2n+1  && return instance.nodes[j - 1].tw_start
    return 0.0
end

function _nd_replay_loads(julia_seq::Vector{Int}, alpha::Dict{Int,Int}, n::Int)::Vector{Int}
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
