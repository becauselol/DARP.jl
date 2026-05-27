using JuMP
import HiGHS

"""
    CordeauIPSolver <: AbstractDARPSolver

Solves DARP using the Cordeau (2006) arc-based integer programming formulation
via JuMP + HiGHS.

Reference: Cordeau, J.-F. (2006). A branch-and-cut algorithm for the dial-a-ride problem.
           Operations Research, 54(3), 573–586.
"""
struct CordeauIPSolver <: AbstractDARPSolver
    time_limit_sec :: Float64
    mip_gap        :: Float64
    verbose        :: Bool
    detour_factor  :: Float64  # δ: ride time ≤ δ·t[pickup,dropoff]; Inf = disabled
end

function CordeauIPSolver(;
    time_limit_sec :: Float64 = 3600.0,
    mip_gap        :: Float64 = 1e-4,
    verbose        :: Bool    = false,
    detour_factor  :: Float64 = Inf
) :: CordeauIPSolver
    detour_factor >= 1.0 || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    return CordeauIPSolver(time_limit_sec, mip_gap, verbose, detour_factor)
end

function solve(solver::CordeauIPSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()

    n, K, Q, T, L_max = instance.n, instance.K, instance.Q, instance.T, instance.L

    # Node Julia indices (1-based):
    #   1       = origin depot   (Cordeau 0)
    #   2..n+1  = pickups        (Cordeau 1..n)
    #   n+2..2n+1 = dropoffs     (Cordeau n+1..2n)
    #   2n+2    = return depot   (Cordeau 2n+1)
    N      = 2*n + 2
    origin = 1
    dest   = N

    # Build flat node-data arrays indexed 1..N
    svc_time = zeros(Float64, N)
    tw_s     = zeros(Float64, N)
    tw_e     = zeros(Float64, N)
    demand   = zeros(Int, N)

    svc_time[1] = instance.depot_origin.service_time
    tw_s[1]     = instance.depot_origin.tw_start
    tw_e[1]     = instance.depot_origin.tw_end

    for i in 1:n
        pi = i + 1           # Julia index for pickup i
        di = n + i + 1       # Julia index for dropoff i
        pu = instance.nodes[i]
        dr = instance.nodes[n + i]

        svc_time[pi] = pu.service_time;  tw_s[pi] = pu.tw_start;  tw_e[pi] = pu.tw_end
        demand[pi]   = pu.load          # positive

        svc_time[di] = dr.service_time;  tw_s[di] = dr.tw_start;  tw_e[di] = dr.tw_end
        demand[di]   = dr.load          # negative
    end

    svc_time[N] = instance.depot_destination.service_time
    tw_s[N]     = instance.depot_destination.tw_start
    tw_e[N]     = instance.depot_destination.tw_end

    t = instance.travel_time  # N×N

    # Arc validity: no self-loops, can't leave destination, can't arrive at origin
    valid(i, j) = (i != j) && (i != dest) && (j != origin)

    # Per-arc big-M for time propagation (tighter than global)
    M_t = [valid(i, j) ? max(0.0, tw_e[i] + svc_time[i] + t[i,j] - tw_s[j]) : 0.0
           for i in 1:N, j in 1:N]

    model = Model(HiGHS.Optimizer)
    set_silent(model)
    set_time_limit_sec(model, solver.time_limit_sec)
    set_optimizer_attribute(model, "mip_rel_gap", solver.mip_gap)
    solver.verbose && unset_silent(model)

    # ── Variables ─────────────────────────────────────────────────────────────
    @variable(model, x[i=1:N, j=1:N, k=1:K; valid(i,j)], Bin)
    @variable(model, B[1:N, 1:K] >= 0)
    @variable(model, 0.0 <= Qv[1:N, 1:K] <= Q)

    for v in 1:N, k in 1:K
        set_lower_bound(B[v, k], tw_s[v])
        set_upper_bound(B[v, k], tw_e[v])
        # Tighter load bounds (Cordeau Section 3)
        set_lower_bound(Qv[v, k], max(0.0, Float64(demand[v])))
        set_upper_bound(Qv[v, k], max(0.0, min(Float64(Q), Float64(Q) + Float64(demand[v]))))
    end

    # ── Objective ─────────────────────────────────────────────────────────────
    @objective(model, Min,
        sum(t[i,j] * x[i,j,k]
            for i in 1:N, j in 1:N, k in 1:K if valid(i,j)))

    # ── Constraints ───────────────────────────────────────────────────────────

    # C1: Each pickup visited exactly once across all vehicles
    for i in 1:n
        pi = i + 1
        @constraint(model,
            sum(x[pi, j, k] for j in 1:N if valid(pi, j) for k in 1:K) == 1)
    end

    # C2: Flow conservation at each non-depot node
    for v in 2:N-1, k in 1:K
        @constraint(model,
            sum(x[i, v, k] for i in 1:N if valid(i, v)) ==
            sum(x[v, j, k] for j in 1:N if valid(v, j)))
    end

    # C3: Each vehicle departs origin depot at most once
    for k in 1:K
        @constraint(model,
            sum(x[origin, j, k] for j in 1:N if valid(origin, j)) <= 1)
    end

    # C4: Same vehicle serves pickup and dropoff for each request
    for i in 1:n, k in 1:K
        pi, di = i+1, n+i+1
        @constraint(model,
            sum(x[pi, j, k] for j in 1:N if valid(pi, j)) ==
            sum(x[di, j, k] for j in 1:N if valid(di, j)))
    end

    # C5: Time propagation (big-M linearization)
    for i in 1:N, j in 1:N, k in 1:K
        if valid(i, j)
            @constraint(model,
                B[j, k] >= B[i, k] + svc_time[i] + t[i, j] -
                           M_t[i, j] * (1 - x[i, j, k]))
        end
    end

    # C6: Load propagation (bidirectional big-M — both bounds needed for exactness)
    M_Q = Float64(Q)
    for i in 1:N, j in 1:N, k in 1:K
        if valid(i, j)
            @constraint(model,
                Qv[j, k] >= Qv[i, k] + demand[j] - M_Q * (1 - x[i, j, k]))
            @constraint(model,
                Qv[j, k] <= Qv[i, k] + demand[j] + M_Q * (1 - x[i, j, k]))
        end
    end

    # C7: Ride time bounds — upper bound (max ride time) and lower bound (precedence).
    # The lower bound B[di] >= B[pi] + d[pi] + t[pi,di] encodes pickup-before-dropoff.
    # Big-M relaxes both bounds for vehicles that don't serve request i.
    for i in 1:n, k in 1:K
        pi, di = i+1, n+i+1
        # s_pi = 1 if vehicle k serves request i (= outflow from pickup node)
        s_pi = sum(x[pi, j, k] for j in 1:N if valid(pi, j))
        M_prec = max(0.0, tw_e[pi] + svc_time[pi] + t[pi, di] - tw_s[di])
        # Absolute ride-time cap
        @constraint(model, B[di, k] - B[pi, k] - svc_time[pi] <= L_max)
        # Precedence: dropoff no earlier than direct travel time after pickup
        @constraint(model,
            B[di, k] - B[pi, k] - svc_time[pi] >= t[pi, di] - M_prec * (1 - s_pi))
    end

    # C7b: Relative detour cap — ride time ≤ δ · direct_travel_time[pickup, dropoff].
    # Tighter than L_max when direct travel is short; a complement, not a replacement.
    if isfinite(solver.detour_factor)
        δ = solver.detour_factor
        for i in 1:n, k in 1:K
            pi, di = i+1, n+i+1
            @constraint(model,
                B[di, k] - B[pi, k] - svc_time[pi] <= δ * t[pi, di])
        end
    end

    # C8: Maximum route duration for each vehicle
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
        routes  = _extract_routes(x, B, Qv, instance, K, N, n, valid)
        obj_val = objective_value(model)
        status  = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible
    else
        routes  = [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K]
        obj_val = Inf
        status  = if term_st == JuMP.MOI.INFEASIBLE
                      :infeasible
                  elseif term_st == JuMP.MOI.TIME_LIMIT
                      :timeout
                  else
                      :error
                  end
    end

    return DARPSolution(instance, routes, obj_val, has_sol, "CordeauIPSolver", elapsed, status)
end

function _extract_routes(x, B, Qv, instance, K, N, n, valid)
    origin = 1
    dest   = N

    routes = Route[]
    for k in 1:K
        # Trace arc chain from origin
        current   = origin
        seq_julia = Int[]   # Julia indices of visited non-depot nodes
        max_iter  = N + 2
        iter      = 0

        while iter < max_iter
            iter += 1
            nxt = 0
            for j in 1:N
                if valid(current, j) && value(x[current, j, k]) > 0.5
                    nxt = j
                    break
                end
            end
            (nxt == 0 || nxt == dest) && break
            push!(seq_julia, nxt)
            current = nxt
        end

        # Convert Julia indices to Cordeau node IDs (0-based)
        # Julia index j → Cordeau ID j-1
        cordeau_seq = [j - 1 for j in seq_julia]

        svc_times = [value(B[j, k]) for j in seq_julia]
        loads_v   = [round(Int, value(Qv[j, k])) for j in seq_julia]

        # Ride times: one per pickup in this route
        pickup_julia = [j for j in seq_julia if 2 <= j <= n+1]
        ride_times = Float64[]
        for pi in pickup_julia
            di = n + (pi - 1) + 1   # Julia index of matching dropoff
            rt = value(B[di, k]) - value(B[pi, k]) - instance.nodes[pi - 1].service_time
            push!(ride_times, rt)
        end

        # Total distance: depot → seq → depot
        total_dist = 0.0
        prev = origin
        for j in seq_julia
            total_dist += instance.travel_distance[prev, j]
            prev = j
        end
        if !isempty(seq_julia)
            total_dist += instance.travel_distance[prev, dest]
        end

        total_dur = isempty(seq_julia) ? 0.0 :
                    value(B[dest, k]) - value(B[origin, k])

        push!(routes, Route(k, cordeau_seq, svc_times, loads_v, ride_times,
                            total_dist, total_dur))
    end
    return routes
end
