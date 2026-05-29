using JuMP
import HiGHS

"""
    SplitDemandIPSolver <: AbstractDARPSolver

Arc-flow MIP for the split-demand DARP with vehicle capacity Q.

Each request i has total demand D_i. A vehicle that visits pickup i carries
f[i,k] ∈ [0, D_i] units, and the coverage constraint requires:
    Σ_k f[i,k] ≥ D_i
allowing D_i to be split across several vehicles when no single vehicle can
carry all D_i units (D_i > Q).

Load variables Qv[v,k] propagate along arcs with a big-M equality, bounded
by [0, Q]. No fleet constraint is imposed.
"""
struct SplitDemandIPSolver <: AbstractDARPSolver
    time_limit_sec :: Float64
    mip_gap        :: Float64
    verbose        :: Bool
    fleet_size     :: Int       # vehicle slots; 0 = use instance.K
end

function SplitDemandIPSolver(;
    time_limit_sec :: Float64 = 3600.0,
    mip_gap        :: Float64 = 1e-4,
    verbose        :: Bool    = false,
    fleet_size     :: Int     = 0
) :: SplitDemandIPSolver
    time_limit_sec > 0 || throw(ArgumentError("time_limit_sec must be > 0"))
    fleet_size >= 0    || throw(ArgumentError("fleet_size must be ≥ 0"))
    return SplitDemandIPSolver(time_limit_sec, mip_gap, verbose, fleet_size)
end

function solve(solver::SplitDemandIPSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()

    n, Q, T, L_max = instance.n, instance.Q, instance.T, instance.L
    K = solver.fleet_size > 0 ? solver.fleet_size : instance.K
    N      = 2*n + 2
    origin = 1
    dest   = N

    svc_time = zeros(Float64, N)
    tw_s     = zeros(Float64, N)
    tw_e     = zeros(Float64, N)
    D        = zeros(Int, n)

    svc_time[1] = instance.depot_origin.service_time
    tw_s[1]     = instance.depot_origin.tw_start
    tw_e[1]     = instance.depot_origin.tw_end

    for i in 1:n
        pi, di = i + 1, n + i + 1
        pu, dr = instance.nodes[i], instance.nodes[n + i]
        svc_time[pi] = pu.service_time;  tw_s[pi] = pu.tw_start;  tw_e[pi] = pu.tw_end
        svc_time[di] = dr.service_time;  tw_s[di] = dr.tw_start;  tw_e[di] = dr.tw_end
        D[i] = pu.load
    end

    svc_time[N] = instance.depot_destination.service_time
    tw_s[N]     = instance.depot_destination.tw_start
    tw_e[N]     = instance.depot_destination.tw_end

    tt = instance.travel_time
    td = instance.travel_distance

    valid(i, j)   = (i != j) && (i != dest) && (j != origin)
    is_pickup(v)  = 2 <= v <= n + 1
    is_dropoff(v) = n + 2 <= v <= 2*n + 1
    req_of(v)     = is_pickup(v) ? v - 1 : v - n - 1

    M_t = [valid(i, j) ? max(0.0, tw_e[i] + svc_time[i] + tt[i,j] - tw_s[j]) : 0.0
           for i in 1:N, j in 1:N]
    M_q = Float64(Q)

    model = Model(HiGHS.Optimizer)
    set_silent(model)
    set_time_limit_sec(model, solver.time_limit_sec)
    set_optimizer_attribute(model, "mip_rel_gap", solver.mip_gap)
    solver.verbose && unset_silent(model)

    # ── Variables ─────────────────────────────────────────────────────────────
    @variable(model, x[i=1:N, j=1:N, k=1:K; valid(i,j)], Bin)
    @variable(model, B[1:N, 1:K] >= 0)
    @variable(model, f[1:n, 1:K] >= 0)    # units of demand i carried by vehicle k
    @variable(model, Qv[1:N, 1:K] >= 0)   # load on vehicle k when leaving node v

    for v in 1:N, k in 1:K
        set_lower_bound(B[v, k], tw_s[v])
        set_upper_bound(B[v, k], tw_e[v])
    end
    for i in 1:n, k in 1:K
        set_upper_bound(f[i, k], Float64(D[i]))
    end
    for v in 1:N, k in 1:K
        set_upper_bound(Qv[v, k], M_q)
    end
    # Depots carry no passengers
    for k in 1:K
        fix(Qv[origin, k], 0.0; force=true)
        fix(Qv[dest,   k], 0.0; force=true)
    end

    # ── Objective ─────────────────────────────────────────────────────────────
    @objective(model, Min,
        sum(td[i,j] * x[i,j,k]
            for i in 1:N, j in 1:N, k in 1:K if valid(i,j)))

    # ── Constraints ───────────────────────────────────────────────────────────

    # C1: Demand coverage — total carried across all vehicles ≥ D_i
    for i in 1:n
        @constraint(model, sum(f[i, k] for k in 1:K) >= Float64(D[i]))
    end

    # C2: Flow conservation at non-depot nodes (per vehicle)
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

    # C4: Same vehicle serves pickup and dropoff (pickup iff dropoff)
    for i in 1:n, k in 1:K
        pi, di = i + 1, n + i + 1
        @constraint(model,
            sum(x[pi, j, k] for j in 1:N if valid(pi, j)) ==
            sum(x[di, j, k] for j in 1:N if valid(di, j)))
    end

    # C5: Time propagation (big-M)
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
        @constraint(model, B[di, k] - B[pi, k] - svc_time[pi] <= L_max)
        @constraint(model, B[di, k] - B[pi, k] - svc_time[pi] >= tt[pi, di] - M_prec*(1 - s_pi))
    end

    # C7: Max route duration
    for k in 1:K
        @constraint(model, B[dest, k] - B[origin, k] <= T)
    end

    # C8: f[i,k] = 0 if vehicle k does not visit pickup i
    for i in 1:n, k in 1:K
        pi = i + 1
        @constraint(model,
            f[i, k] <= Float64(D[i]) * sum(x[pi, j, k] for j in 1:N if valid(pi, j)))
    end

    # C9: Load propagation — equality along the active arc, loose otherwise.
    #     Both ≥ and ≤ are needed to pin Qv to the correct value and prevent
    #     spurious high values from propagating through subsequent arcs.
    for u in 1:N, v in 2:N-1, k in 1:K
        valid(u, v) || continue
        req = req_of(v)
        if is_pickup(v)
            @constraint(model, Qv[v,k] >= Qv[u,k] + f[req,k] - M_q*(1 - x[u,v,k]))
            @constraint(model, Qv[v,k] <= Qv[u,k] + f[req,k] + M_q*(1 - x[u,v,k]))
        else
            @constraint(model, Qv[v,k] >= Qv[u,k] - f[req,k] - M_q*(1 - x[u,v,k]))
            @constraint(model, Qv[v,k] <= Qv[u,k] - f[req,k] + M_q*(1 - x[u,v,k]))
        end
    end

    optimize!(model)
    elapsed = time() - t0

    # ── Extract solution ───────────────────────────────────────────────────────
    term_st   = termination_status(model)
    primal_st = primal_status(model)
    has_sol   = primal_st == JuMP.MOI.FEASIBLE_POINT

    lp_bound = try; objective_bound(model); catch; NaN; end

    if has_sol
        f_vals  = [value(f[i, k]) for i in 1:n, k in 1:K]
        Qv_vals = [value(Qv[v, k]) for v in 1:N, k in 1:K]
        routes  = _sdi_extract_routes(x, B, Qv_vals, f_vals, instance, K, N, n, valid)
        obj_val = objective_value(model)
        status  = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible
    else
        routes  = [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K]
        obj_val = Inf
        status  = term_st == JuMP.MOI.INFEASIBLE ? :infeasible :
                  term_st == JuMP.MOI.TIME_LIMIT  ? :timeout   : :error
    end

    return DARPSolution(instance, routes, obj_val, has_sol, "SplitDemandIPSolver", elapsed,
                        status, lp_bound, 0, CGIterLogEntry[])
end

function _sdi_extract_routes(x, B, Qv_vals, f_vals, instance, K, N, n, valid)
    origin = 1
    dest   = N
    td     = instance.travel_distance

    routes = Route[]
    for k in 1:K
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

        # Replay tight service times (B is not in objective, may not be tight)
        svc_times = _sd_replay_arc_times(seq_julia, origin, instance, N, n)

        # Load at each stop from Qv solution values
        loads_v = [round(Int, Qv_vals[v, k]) for v in seq_julia]

        # Ride time per pickup (from replayed service times)
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
