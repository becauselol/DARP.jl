using JuMP
import HiGHS

"""
    SplitDeliveryNoCapIPSolver <: AbstractDARPSolver

Arc-flow MIP for split-demand DARP with uncapped vehicle capacity.

Formulation differences from CordeauIPSolver:
  - Coverage constraint: Σ_k x_{i·k} ≥ 1  (covering, not partitioning)
    Each request must be visited by at least one vehicle; with uncapped capacity
    one visit always covers the full demand D_i, so ≥ 1 visit suffices.
  - No load variables or capacity constraint (Q = ∞).

Every other constraint (time windows, ride time, route duration, flow conservation,
pickup-before-dropoff coupling) is identical to Cordeau (2006).
"""
struct SplitDeliveryNoCapIPSolver <: AbstractDARPSolver
    time_limit_sec :: Float64
    mip_gap        :: Float64
    verbose        :: Bool
end

function SplitDeliveryNoCapIPSolver(;
    time_limit_sec :: Float64 = 3600.0,
    mip_gap        :: Float64 = 1e-4,
    verbose        :: Bool    = false
) :: SplitDeliveryNoCapIPSolver
    time_limit_sec > 0 || throw(ArgumentError("time_limit_sec must be > 0"))
    return SplitDeliveryNoCapIPSolver(time_limit_sec, mip_gap, verbose)
end

function solve(solver::SplitDeliveryNoCapIPSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()

    n, K, T, L_max = instance.n, instance.K, instance.T, instance.L
    N      = 2*n + 2
    origin = 1
    dest   = N

    # Node data arrays (Julia-indexed 1..N)
    svc_time = zeros(Float64, N)
    tw_s     = zeros(Float64, N)
    tw_e     = zeros(Float64, N)
    demand   = zeros(Int, N)

    svc_time[1] = instance.depot_origin.service_time
    tw_s[1]     = instance.depot_origin.tw_start
    tw_e[1]     = instance.depot_origin.tw_end

    for i in 1:n
        pi, di = i + 1, n + i + 1
        pu, dr = instance.nodes[i], instance.nodes[n + i]

        svc_time[pi] = pu.service_time;  tw_s[pi] = pu.tw_start;  tw_e[pi] = pu.tw_end
        demand[pi]   = pu.load

        svc_time[di] = dr.service_time;  tw_s[di] = dr.tw_start;  tw_e[di] = dr.tw_end
        demand[di]   = dr.load   # negative
    end

    svc_time[N] = instance.depot_destination.service_time
    tw_s[N]     = instance.depot_destination.tw_start
    tw_e[N]     = instance.depot_destination.tw_end

    tt = instance.travel_time
    td = instance.travel_distance

    valid(i, j) = (i != j) && (i != dest) && (j != origin)

    # Per-arc tight big-M for time propagation
    M_t = [valid(i, j) ? max(0.0, tw_e[i] + svc_time[i] + tt[i,j] - tw_s[j]) : 0.0
           for i in 1:N, j in 1:N]

    model = Model(HiGHS.Optimizer)
    set_silent(model)
    set_time_limit_sec(model, solver.time_limit_sec)
    set_optimizer_attribute(model, "mip_rel_gap", solver.mip_gap)
    solver.verbose && unset_silent(model)

    # ── Variables ─────────────────────────────────────────────────────────────
    @variable(model, x[i=1:N, j=1:N, k=1:K; valid(i,j)], Bin)
    @variable(model, B[1:N, 1:K] >= 0)

    for v in 1:N, k in 1:K
        set_lower_bound(B[v, k], tw_s[v])
        set_upper_bound(B[v, k], tw_e[v])
    end

    # ── Objective: minimise total travel distance ──────────────────────────────
    @objective(model, Min,
        sum(td[i,j] * x[i,j,k]
            for i in 1:N, j in 1:N, k in 1:K if valid(i,j)))

    # ── Constraints ───────────────────────────────────────────────────────────

    # C1: Covering — each pickup visited by at least one vehicle.
    #     With uncapped capacity one visit boards all D_i units, so >= 1 suffices.
    for i in 1:n
        pi = i + 1
        @constraint(model,
            sum(x[pi, j, k] for j in 1:N if valid(pi, j) for k in 1:K) >= 1)
    end

    # C2: Flow conservation at each non-depot node (per vehicle)
    for v in 2:N-1, k in 1:K
        @constraint(model,
            sum(x[i, v, k] for i in 1:N if valid(i, v)) ==
            sum(x[v, j, k] for j in 1:N if valid(v, j)))
    end

    # C3: Each vehicle departs origin at most once
    for k in 1:K
        @constraint(model,
            sum(x[origin, j, k] for j in 1:N if valid(origin, j)) <= 1)
    end

    # C4: Same vehicle serves pickup and dropoff (pickup iff dropoff, per vehicle)
    for i in 1:n, k in 1:K
        pi, di = i + 1, n + i + 1
        @constraint(model,
            sum(x[pi, j, k] for j in 1:N if valid(pi, j)) ==
            sum(x[di, j, k] for j in 1:N if valid(di, j)))
    end

    # C5: Time propagation (big-M linearisation)
    for i in 1:N, j in 1:N, k in 1:K
        valid(i, j) || continue
        @constraint(model,
            B[j, k] >= B[i, k] + svc_time[i] + tt[i, j] -
                       M_t[i, j] * (1 - x[i, j, k]))
    end

    # C6: Ride-time bounds (per request per vehicle)
    for i in 1:n, k in 1:K
        pi, di = i + 1, n + i + 1
        s_pi   = sum(x[pi, j, k] for j in 1:N if valid(pi, j))
        M_prec = max(0.0, tw_e[pi] + svc_time[pi] + tt[pi, di] - tw_s[di])

        # Upper bound: max ride time
        @constraint(model, B[di, k] - B[pi, k] - svc_time[pi] <= L_max)
        # Lower bound: pickup must precede dropoff by at least direct travel time
        @constraint(model,
            B[di, k] - B[pi, k] - svc_time[pi] >= tt[pi, di] - M_prec * (1 - s_pi))
    end

    # C7: Max route duration per vehicle
    for k in 1:K
        @constraint(model, B[dest, k] - B[origin, k] <= T)
    end

    optimize!(model)
    elapsed = time() - t0

    # ── Extract solution ───────────────────────────────────────────────────────
    term_st   = termination_status(model)
    primal_st = primal_status(model)
    has_sol   = primal_st == JuMP.MOI.FEASIBLE_POINT

    if has_sol
        routes  = _sd_extract_routes(x, B, demand, instance, K, N, n, valid)
        obj_val = objective_value(model)
        status  = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible
    else
        routes  = [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K]
        obj_val = Inf
        status  = term_st == JuMP.MOI.INFEASIBLE ? :infeasible :
                  term_st == JuMP.MOI.TIME_LIMIT  ? :timeout   : :error
    end

    return DARPSolution(instance, routes, obj_val, has_sol, "SplitDeliveryNoCapIPSolver", elapsed, status)
end

function _sd_extract_routes(x, B, demand, instance, K, N, n, valid)
    origin = 1
    dest   = N
    td     = instance.travel_distance

    routes = Route[]
    for k in 1:K
        # Trace arc chain: follow x[cur,j,k] = 1 from origin to dest
        cur       = origin
        seq_julia = Int[]
        for _ in 1:(N + 2)
            nxt = 0
            for j in 1:N
                valid(cur, j) && value(x[cur, j, k]) > 0.5 && (nxt = j; break)
            end
            (nxt == 0 || nxt == dest) && break
            push!(seq_julia, nxt)
            cur = nxt
        end

        cordeau_seq = [j - 1 for j in seq_julia]

        # Replay service times from the arc sequence so they are always tight
        # (B variables may be non-tight when distance-only objective leaves them free).
        svc_times = _sd_replay_arc_times(seq_julia, origin, instance, N, n)

        # Cumulative load at each stop (informational; no capacity enforced)
        load = 0
        loads_v = Int[]
        for j in seq_julia
            load += demand[j]
            push!(loads_v, load)
        end

        # Ride time per pickup in route order (from replayed service times)
        ride_times = Float64[]
        for (idx_pu, pi) in enumerate(seq_julia)
            2 <= pi <= n + 1 || continue
            di     = n + (pi - 1) + 1
            idx_di = findfirst(==(di), seq_julia)
            idx_di === nothing && continue
            rt = svc_times[idx_di] - svc_times[idx_pu] - instance.nodes[pi - 1].service_time
            push!(ride_times, rt)
        end

        total_dist = 0.0
        prev = origin
        for j in seq_julia
            total_dist += td[prev, j]
            prev = j
        end
        isempty(seq_julia) || (total_dist += td[prev, dest])

        total_dur = isempty(seq_julia) ? 0.0 :
                    _sd_dest_arrival(seq_julia, svc_times, instance, N, n) -
                    instance.depot_origin.tw_start

        push!(routes, Route(k, cordeau_seq, svc_times, loads_v, ride_times,
                            total_dist, total_dur))
    end
    return routes
end

# Replay tight service-begin times along a Julia-indexed node sequence.
# Mirrors _sd_replay_service_times in the CG pricing module.
function _sd_replay_arc_times(
    seq     :: Vector{Int},
    origin  :: Int,
    instance :: DARPInstance,
    N       :: Int,
    n       :: Int
) :: Vector{Float64}
    tt       = instance.travel_time
    dep      = instance.depot_origin
    cur_time = dep.tw_start
    prev     = origin
    svc_arr  = zeros(Float64, N)   # service time at each node

    svc_arr[origin] = dep.service_time

    for i in 1:n
        pi = i + 1;  di = n + i + 1
        svc_arr[pi] = instance.nodes[i].service_time
        svc_arr[di] = instance.nodes[n + i].service_time
    end
    svc_arr[N] = instance.depot_destination.service_time

    tw_s_arr = zeros(Float64, N)
    tw_s_arr[origin] = dep.tw_start
    for i in 1:n
        tw_s_arr[i + 1]     = instance.nodes[i].tw_start
        tw_s_arr[n + i + 1] = instance.nodes[n + i].tw_start
    end
    tw_s_arr[N] = instance.depot_destination.tw_start

    times = Float64[]
    for j in seq
        arr = cur_time + svc_arr[prev] + tt[prev, j]
        t_j = max(tw_s_arr[j], arr)
        push!(times, t_j)
        cur_time = t_j
        prev     = j
    end
    return times
end

# Compute arrival time at the destination depot after the last stop.
function _sd_dest_arrival(
    seq      :: Vector{Int},
    svc_times :: Vector{Float64},
    instance :: DARPInstance,
    N        :: Int,
    n        :: Int
) :: Float64
    last_j    = seq[end]
    last_svc  = last_j == N ? instance.depot_destination.service_time :
                (last_j <= n + 1 ? instance.nodes[last_j - 1].service_time
                                 : instance.nodes[last_j - 1].service_time)
    arr_dest  = svc_times[end] + last_svc + instance.travel_time[last_j, N]
    return max(instance.depot_destination.tw_start, arr_dest)
end
