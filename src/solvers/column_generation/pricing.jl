"""
    solve_pricing(instance, duals; detour_factor, rc_tolerance, max_routes) → Vector{FeasibleRoute}

SPPRC pricing subproblem for the DARP column generation solver.

Returns all routes with `reduced_cost < rc_tolerance`, sorted ascending by reduced cost.
This function is exported and can be called independently of any running model.

# Arguments
- `instance`: `DARPInstance` to price over
- `duals`: `CGDuals` with coverage duals π[i] and fleet dual μ
- `detour_factor`: δ ≥ 1; ride time ≤ δ·direct_time (Inf = disabled)
- `rc_tolerance`: threshold for considering a route improving (default -1e-6)
- `max_routes`: maximum number of routes to return (default 10)
"""
function solve_pricing(
    instance      :: DARPInstance,
    duals         :: CGDuals;
    detour_factor :: Float64 = Inf,
    rc_tolerance  :: Float64 = -1e-6,
    max_routes    :: Int     = 10,
    time_limit    :: Float64 = 60.0
) :: Vector{FeasibleRoute}
    n   = instance.n
    K   = instance.K
    Q   = instance.Q
    T   = instance.T
    L   = instance.L
    N   = 2 * n + 2
    origin = 1
    dest   = N

    # Flat arrays indexed 1..N
    pp = _build_preprocess(instance)
    tw_s      = pp.tw_s
    tw_e      = pp.tw_e
    svc_time  = pp.svc_time
    demand    = pp.demand
    is_pickup = pp.is_pickup
    request_of = pp.request_of   # Julia index j → request id (1..n), 0 if depot
    direct_time = pp.direct_time # direct_time[i] = travel_time[pickup_i, dropoff_i]

    tt = instance.travel_time
    td = instance.travel_distance

    # One Labels-at-node store for dominance: node → list of non-dominated labels
    Labels = [Label[] for _ in 1:N]

    root = Label(
        origin,
        -duals.mu,        # fleet dual subtracted once per route
        tw_s[origin],     # service time at origin
        0,                # load
        Int[],            # nothing onboard
        Dict{Int,Float64}(),
        [origin],
        0.0
    )
    push!(Labels[origin], root)

    # Use a stack (LIFO) so DFS explores long routes first — critical for large n
    # where FIFO (BFS) would exhaust the budget on short routes before reaching
    # the multi-request routes needed by vehicles with K << n.
    open_stack = Label[root]
    completed  = Label[]

    t_pricing = time()

    while !isempty(open_stack)
        time() - t_pricing > time_limit && break
        label = pop!(open_stack)   # LIFO
        cur   = label.node

        for j in 1:N
            # Basic arc validity
            j == origin && continue
            j == cur    && continue
            j in label.path && continue

            # Can't leave destination
            cur == dest && continue

            # Dropoff: only if the corresponding request is onboard
            if pp.is_dropoff[j]
                req = request_of[j]
                req ∉ label.onboard && continue
            end

            # Compute arrival and service begin time at j
            arr_j  = label.time + svc_time[cur] + tt[cur, j]
            t_j    = max(tw_s[j], arr_j)

            # Time window feasibility
            t_j > tw_e[j] + 1e-9 && continue

            # Load after visiting j
            new_load = label.load + demand[j]
            (new_load < 0 || new_load > Q) && continue

            # Check ride times for all onboard passengers
            feasible = true
            for req_i in label.onboard
                elapsed = t_j - label.ride_start[req_i] - svc_time[pp.pickup_julia[req_i]]
                elapsed > L + 1e-9 && (feasible = false; break)
                if isfinite(detour_factor)
                    elapsed > detour_factor * direct_time[req_i] + 1e-9 &&
                        (feasible = false; break)
                end
            end
            !feasible && continue

            # Accumulate reduced cost:
            # - add arc travel time
            # - subtract coverage dual if j is a pickup
            new_rc = label.rc + tt[cur, j]
            if is_pickup[j]
                new_rc -= duals.pi[request_of[j]]
            end

            new_onboard  = copy(label.onboard)
            new_ride_start = copy(label.ride_start)

            if is_pickup[j]
                req_j = request_of[j]
                push!(new_onboard, req_j)
                sort!(new_onboard)
                new_ride_start[req_j] = t_j
            elseif pp.is_dropoff[j]
                req_j = request_of[j]
                filter!(r -> r != req_j, new_onboard)
                delete!(new_ride_start, req_j)
            end

            new_path = vcat(label.path, j)
            new_dist = label.distance + td[cur, j]

            new_label = Label(
                j, new_rc, t_j, new_load,
                new_onboard, new_ride_start,
                new_path, new_dist
            )

            if j == dest
                # All passengers must be dropped off and duration within limit
                isempty(new_onboard) || continue
                dur = t_j - tw_s[origin]
                dur > T + 1e-9 && continue
                push!(completed, new_label)
                continue
            end

            # Dominance check before adding to queue
            if !_is_dominated(new_label, Labels[j], svc_time)
                _remove_dominated!(Labels[j], new_label, svc_time)
                push!(Labels[j], new_label)
                push!(open_stack, new_label)
            end
        end
    end

    # Build FeasibleRoute for each completed label with negative reduced cost
    results = FeasibleRoute[]
    for lbl in completed
        lbl.rc >= rc_tolerance && continue
        fr = _label_to_route(lbl, instance, n, N, direct_time, svc_time)
        push!(results, fr)
    end

    # Sort by reduced cost, return top max_routes
    sort!(results, by = r -> r.reduced_cost)
    return results[1:min(max_routes, length(results))]
end

# ── Internal helpers ───────────────────────────────────────────────────────────

function _build_preprocess(instance::DARPInstance)
    n  = instance.n
    N  = 2 * n + 2

    tw_s      = zeros(Float64, N)
    tw_e      = zeros(Float64, N)
    svc_time  = zeros(Float64, N)
    demand    = zeros(Int, N)
    is_pickup  = falses(N)
    is_dropoff = falses(N)
    request_of = zeros(Int, N)
    pickup_julia = zeros(Int, n)   # request i → Julia pickup index
    direct_time  = zeros(Float64, n)

    tw_s[1] = instance.depot_origin.tw_start
    tw_e[1] = instance.depot_origin.tw_end
    svc_time[1] = instance.depot_origin.service_time

    for i in 1:n
        pi = i + 1           # Julia pickup index
        di = n + i + 1       # Julia dropoff index

        pu = instance.nodes[i]
        dr = instance.nodes[n + i]

        tw_s[pi] = pu.tw_start;   tw_e[pi] = pu.tw_end
        svc_time[pi] = pu.service_time
        demand[pi] = pu.load
        is_pickup[pi] = true
        request_of[pi] = i
        pickup_julia[i] = pi

        tw_s[di] = dr.tw_start;   tw_e[di] = dr.tw_end
        svc_time[di] = dr.service_time
        demand[di] = dr.load
        is_dropoff[di] = true
        request_of[di] = i

        direct_time[i] = instance.travel_time[pi, di]
    end

    tw_s[N] = instance.depot_destination.tw_start
    tw_e[N] = instance.depot_destination.tw_end
    svc_time[N] = instance.depot_destination.service_time

    return (;
        tw_s, tw_e, svc_time, demand,
        is_pickup, is_dropoff, request_of,
        pickup_julia, direct_time
    )
end

function _is_dominated(new_lbl::Label, existing::Vector{Label}, svc_time::Vector{Float64})
    for lbl in existing
        _dominates(lbl, new_lbl, svc_time) && return true
    end
    return false
end

function _remove_dominated!(existing::Vector{Label}, new_lbl::Label, svc_time::Vector{Float64})
    filter!(lbl -> !_dominates(new_lbl, lbl, svc_time), existing)
end

function _dominates(A::Label, B::Label, svc_time::Vector{Float64})
    # A dominates B if all hold:
    A.rc   > B.rc   + 1e-9 && return false
    A.time > B.time + 1e-9 && return false
    A.load > B.load        && return false

    # A.onboard must be a subset of B.onboard
    for r in A.onboard
        r ∉ B.onboard && return false
    end

    # For each shared onboard passenger, A must have ≤ elapsed ride time
    for r in A.onboard
        r ∉ B.onboard && continue
        A_elapsed = A.time - A.ride_start[r] - svc_time[A.node]
        B_elapsed = B.time - B.ride_start[r] - svc_time[B.node]
        A_elapsed > B_elapsed + 1e-9 && return false
    end

    return true
end

function _label_to_route(
    lbl         :: Label,
    instance    :: DARPInstance,
    n           :: Int,
    N           :: Int,
    direct_time :: Vector{Float64},
    svc_time    :: Vector{Float64}
) :: FeasibleRoute
    origin = 1
    dest   = N

    # Path: [origin, ..., dest]. Strip both origin and dest.
    julia_seq = lbl.path[2:end-1]   # non-depot visited nodes

    cordeau_seq = [j - 1 for j in julia_seq]

    request_ids = sort(unique(
        [j - 1 for j in julia_seq if 2 <= j <= n + 1]
    ))

    # Service times: we need to recompute them from the path since Label only
    # stores the current node's time. Replay the path.
    svc_times = _replay_service_times(julia_seq, lbl, instance, svc_time, n, N)

    loads_v = _replay_loads(julia_seq, instance, n)

    # Ride times: one per pickup in route order
    ride_times = Float64[]
    for (idx_pu, j) in enumerate(julia_seq)
        2 <= j <= n + 1 || continue
        req_id = j - 1
        di_julia = n + req_id + 1
        idx_di = findfirst(==(di_julia), julia_seq)
        if idx_di !== nothing
            pu_svc = instance.nodes[req_id].service_time
            rt = svc_times[idx_di] - svc_times[idx_pu] - pu_svc
            push!(ride_times, rt)
        end
    end

    # lbl.distance already includes origin → ... → dest (dest was appended to path).
    total_dist = lbl.distance

    # Total duration: B[dest] - B[origin]. Service time at dest is lbl.time since lbl.node == dest.
    total_dur = lbl.time - instance.depot_origin.tw_start

    cost = total_dist

    return FeasibleRoute(
        cordeau_seq, julia_seq, request_ids,
        cost, lbl.rc,
        svc_times, loads_v, ride_times,
        total_dist, total_dur
    )
end

function _replay_service_times(
    julia_seq :: Vector{Int},
    lbl       :: Label,
    instance  :: DARPInstance,
    svc_time  :: Vector{Float64},
    n         :: Int,
    N         :: Int
) :: Vector{Float64}
    origin = 1
    tt = instance.travel_time

    cur_time = instance.depot_origin.tw_start
    prev     = origin
    times    = Float64[]

    for j in julia_seq
        arr = cur_time + svc_time[prev] + tt[prev, j]
        t_j = max(_tw_start(j, instance, n, N), arr)
        push!(times, t_j)
        cur_time = t_j
        prev = j
    end
    return times
end

function _tw_start(j::Int, instance::DARPInstance, n::Int, N::Int)::Float64
    j == 1      && return instance.depot_origin.tw_start
    j == N      && return instance.depot_destination.tw_start
    2 <= j <= n+1   && return instance.nodes[j - 1].tw_start
    n+2 <= j <= 2n+1 && return instance.nodes[j - 1].tw_start   # dropoffs: nodes[n+i], i = j-(n+1)
    return 0.0
end

function _replay_loads(julia_seq::Vector{Int}, instance::DARPInstance, n::Int)::Vector{Int}
    load = 0
    loads = Int[]
    for j in julia_seq
        if 2 <= j <= n + 1
            load += instance.nodes[j - 1].load
        elseif n + 2 <= j <= 2*n + 1
            load += instance.nodes[j - 1].load   # negative demand
        end
        push!(loads, load)
    end
    return loads
end
