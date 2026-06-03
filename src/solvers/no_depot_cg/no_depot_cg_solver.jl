import Gurobi

# ── NoDepotNoCapCGSolver ───────────────────────────────────────────────────────

"""
    NoDepotNoCapCGSolver <: AbstractDARPSolver

Column generation solver with no depot and uncapped vehicle capacity.

Routes are open paths: a vehicle materialises at a pickup, serves some set of
requests (interleaved pickups and dropoffs), and terminates when the last
passenger is dropped off. No depot origin or destination is assumed.
Vehicle capacity is not enforced; every pickup boards the full demand D_i.

Master LP: Σ_{r:i∈r} D_i · λ_r ≥ D_i  ⟹  Σ_{r covers i} λ_r ≥ 1.
IP: binary covering MIP over the final column pool.
"""
struct NoDepotNoCapCGSolver <: AbstractDARPSolver
    time_limit_sec        :: Float64
    max_cg_iters          :: Int
    mip_gap               :: Float64
    verbose               :: Bool
    detour_factor         :: Float64
    solve_ip              :: Bool
    max_routes_per_iter   :: Int
    rc_tolerance          :: Float64
    pricing_time_per_iter :: Float64
    patience              :: Int
    timeout_patience      :: Int
    ip_time_limit_sec     :: Float64
end

function NoDepotNoCapCGSolver(;
    time_limit_sec        :: Float64 = 3600.0,
    max_cg_iters          :: Int     = 10_000,
    mip_gap               :: Float64 = 1e-4,
    verbose               :: Bool    = false,
    detour_factor         :: Float64 = Inf,
    solve_ip              :: Bool    = true,
    max_routes_per_iter   :: Int     = 10,
    rc_tolerance          :: Float64 = -1e-6,
    pricing_time_per_iter :: Float64 = 30.0,
    patience              :: Int     = 3,
    timeout_patience      :: Int     = 10,
    ip_time_limit_sec     :: Float64 = 1800.0
) :: NoDepotNoCapCGSolver
    detour_factor >= 1.0  || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    time_limit_sec > 0    || throw(ArgumentError("time_limit_sec must be > 0"))
    patience > 0          || throw(ArgumentError("patience must be > 0"))
    timeout_patience > 0  || throw(ArgumentError("timeout_patience must be > 0"))
    ip_time_limit_sec > 0 || throw(ArgumentError("ip_time_limit_sec must be > 0"))
    return NoDepotNoCapCGSolver(
        time_limit_sec, max_cg_iters, mip_gap, verbose,
        detour_factor, solve_ip, max_routes_per_iter, rc_tolerance,
        pricing_time_per_iter, patience, timeout_patience, ip_time_limit_sec
    )
end

function solve(solver::NoDepotNoCapCGSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()
    K  = instance.K

    pool = _generate_nd_nocap_seed_routes(instance)
    solver.verbose && println("[ND-NoCap-CG] Initialized with $(length(pool.routes)) seed routes")

    env   = Gurobi.Env()
    model, λ_vars, cov_cons = build_nd_rmp(instance, pool, env)

    lp_obj             = Inf
    iter_log           = CGIterLogEntry[]
    n_iters            = 0
    no_improve         = 0
    no_improve_timeout = 0
    lp_proven_optimal  = false

    for iter in 1:solver.max_cg_iters
        remaining = solver.time_limit_sec - (time() - t0)
        remaining <= 0 && break

        JuMP.set_time_limit_sec(model, remaining)
        lp_status = solve_nd_rmp!(model)

        if lp_status == :infeasible
            elapsed = time() - t0
            return DARPSolution(instance,
                [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
                Inf, false, "NoDepotNoCapCGSolver", elapsed, :infeasible,
                NaN, iter, iter_log)
        end

        lp_obj  = JuMP.objective_value(model)
        n_iters = iter
        solver.verbose && println("[ND-NoCap-CG] Iter $iter: LP obj = $(round(lp_obj, digits=4))")

        duals = extract_nd_duals(cov_cons)

        remaining      = solver.time_limit_sec - (time() - t0)
        pricing_budget = max(1.0, min(solver.pricing_time_per_iter, remaining))

        new_routes, pricing_exhausted = solve_nodepot_nocap_pricing(
            instance, duals;
            detour_factor = solver.detour_factor,
            rc_tolerance  = solver.rc_tolerance,
            max_routes    = solver.max_routes_per_iter,
            time_limit    = pricing_budget
        )

        added = 0
        for route in new_routes
            any(r.cordeau_seq == route.cordeau_seq for r in pool.routes) && continue
            add_nd_column!(model, pool, route, cov_cons, λ_vars)
            added += 1
        end

        push!(iter_log, (iter=iter, lp_obj=lp_obj, cols_added=added))
        solver.verbose && println("[ND-NoCap-CG]   Added $added new columns (pool = $(length(pool.routes)))")

        if added > 0
            no_improve         = 0
            no_improve_timeout = 0
        elseif pricing_exhausted
            no_improve += 1
            no_improve_timeout = 0
            if no_improve >= solver.patience
                lp_proven_optimal = true
                solver.verbose && println("[ND-NoCap-CG] LP proven optimal after $iter iterations")
                break
            end
        else
            no_improve_timeout += 1
            if no_improve_timeout >= solver.timeout_patience
                solver.verbose && println("[ND-NoCap-CG] Timeout patience exhausted after $iter iterations")
                break
            end
        end
    end

    elapsed = time() - t0

    if solver.solve_ip
        ip_status, ip_obj, selected = solve_nd_nocap_master_ip(
            instance, pool, env, solver.ip_time_limit_sec, solver.mip_gap, solver.verbose
        )
        elapsed = time() - t0
        if !lp_proven_optimal && ip_status == :optimal
            ip_status = :feasible
        end
        return build_nd_solution(instance, selected, ip_obj, ip_status, elapsed,
                                 "NoDepotNoCapCGSolver", lp_obj, n_iters, iter_log)
    else
        return DARPSolution(instance,
            [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
            lp_obj, false, "NoDepotNoCapCGSolver", elapsed, :lp_relaxation,
            lp_obj, n_iters, iter_log)
    end
end

# ── NoDepotDemandCGSolver ──────────────────────────────────────────────────────

"""
    NoDepotDemandCGSolver <: AbstractDARPSolver

Column generation solver with no depot and finite vehicle capacity Q.

Routes are open paths; vehicle capacity is enforced. At each pickup the solver
enumerates α = 1…min(Q − load, D_i) boarding amounts, generating one child label
per value. Coverage coefficient α_{ir} ≤ min(D_i, Q) varies per route.

Master LP: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i, λ_r ≥ 0 unbounded.
IP: nonneg-integer covering MIP.
"""
struct NoDepotDemandCGSolver <: AbstractDARPSolver
    time_limit_sec        :: Float64
    max_cg_iters          :: Int
    mip_gap               :: Float64
    verbose               :: Bool
    detour_factor         :: Float64
    solve_ip              :: Bool
    max_routes_per_iter   :: Int
    rc_tolerance          :: Float64
    pricing_time_per_iter :: Float64
    patience              :: Int
    timeout_patience      :: Int
    ip_time_limit_sec     :: Float64
end

function NoDepotDemandCGSolver(;
    time_limit_sec        :: Float64 = 3600.0,
    max_cg_iters          :: Int     = 10_000,
    mip_gap               :: Float64 = 1e-4,
    verbose               :: Bool    = false,
    detour_factor         :: Float64 = Inf,
    solve_ip              :: Bool    = true,
    max_routes_per_iter   :: Int     = 10,
    rc_tolerance          :: Float64 = -1e-6,
    pricing_time_per_iter :: Float64 = 30.0,
    patience              :: Int     = 3,
    timeout_patience      :: Int     = 10,
    ip_time_limit_sec     :: Float64 = 1800.0
) :: NoDepotDemandCGSolver
    detour_factor >= 1.0  || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    time_limit_sec > 0    || throw(ArgumentError("time_limit_sec must be > 0"))
    patience > 0          || throw(ArgumentError("patience must be > 0"))
    timeout_patience > 0  || throw(ArgumentError("timeout_patience must be > 0"))
    ip_time_limit_sec > 0 || throw(ArgumentError("ip_time_limit_sec must be > 0"))
    return NoDepotDemandCGSolver(
        time_limit_sec, max_cg_iters, mip_gap, verbose,
        detour_factor, solve_ip, max_routes_per_iter, rc_tolerance,
        pricing_time_per_iter, patience, timeout_patience, ip_time_limit_sec
    )
end

function solve(solver::NoDepotDemandCGSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()
    K  = instance.K

    pool = _generate_nd_demand_seed_routes(instance)
    solver.verbose && println("[ND-Demand-CG] Initialized with $(length(pool.routes)) seed routes")

    env   = Gurobi.Env()
    model, λ_vars, cov_cons = build_nd_rmp(instance, pool, env)

    lp_obj             = Inf
    iter_log           = CGIterLogEntry[]
    n_iters            = 0
    no_improve         = 0
    no_improve_timeout = 0
    lp_proven_optimal  = false

    for iter in 1:solver.max_cg_iters
        remaining = solver.time_limit_sec - (time() - t0)
        remaining <= 0 && break

        JuMP.set_time_limit_sec(model, remaining)
        lp_status = solve_nd_rmp!(model)

        if lp_status == :infeasible
            elapsed = time() - t0
            return DARPSolution(instance,
                [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
                Inf, false, "NoDepotDemandCGSolver", elapsed, :infeasible,
                NaN, iter, iter_log)
        end

        lp_obj  = JuMP.objective_value(model)
        n_iters = iter
        solver.verbose && println("[ND-Demand-CG] Iter $iter: LP obj = $(round(lp_obj, digits=4))")

        duals = extract_nd_duals(cov_cons)

        remaining      = solver.time_limit_sec - (time() - t0)
        pricing_budget = max(1.0, min(solver.pricing_time_per_iter, remaining))

        new_routes, pricing_exhausted = solve_nodepot_demand_pricing(
            instance, duals;
            detour_factor = solver.detour_factor,
            rc_tolerance  = solver.rc_tolerance,
            max_routes    = solver.max_routes_per_iter,
            time_limit    = pricing_budget
        )

        added = 0
        for route in new_routes
            any(r.cordeau_seq == route.cordeau_seq &&
                r.alpha == route.alpha for r in pool.routes) && continue
            add_nd_column!(model, pool, route, cov_cons, λ_vars)
            added += 1
        end

        push!(iter_log, (iter=iter, lp_obj=lp_obj, cols_added=added))
        solver.verbose && println("[ND-Demand-CG]   Added $added new columns (pool = $(length(pool.routes)))")

        if added > 0
            no_improve         = 0
            no_improve_timeout = 0
        elseif pricing_exhausted
            no_improve += 1
            no_improve_timeout = 0
            if no_improve >= solver.patience
                lp_proven_optimal = true
                solver.verbose && println("[ND-Demand-CG] LP proven optimal after $iter iterations")
                break
            end
        else
            no_improve_timeout += 1
            if no_improve_timeout >= solver.timeout_patience
                solver.verbose && println("[ND-Demand-CG] Timeout patience exhausted after $iter iterations")
                break
            end
        end
    end

    elapsed = time() - t0

    if solver.solve_ip
        ip_status, ip_obj, selected = solve_nd_demand_master_ip(
            instance, pool, env, solver.ip_time_limit_sec, solver.mip_gap, solver.verbose
        )
        elapsed = time() - t0
        if !lp_proven_optimal && ip_status == :optimal
            ip_status = :feasible
        end
        return build_nd_solution(instance, selected, ip_obj, ip_status, elapsed,
                                 "NoDepotDemandCGSolver", lp_obj, n_iters, iter_log)
    else
        return DARPSolution(instance,
            [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
            lp_obj, false, "NoDepotDemandCGSolver", elapsed, :lp_relaxation,
            lp_obj, n_iters, iter_log)
    end
end

# ── Seed route generation ──────────────────────────────────────────────────────

function _generate_nd_nocap_seed_routes(instance::DARPInstance) :: NoDepotPool
    n    = instance.n
    pool = NoDepotPool()
    tt   = instance.travel_time
    td   = instance.travel_distance

    for i in 1:n
        pi  = i + 1
        di  = n + i + 1
        pu  = instance.nodes[i]
        dr  = instance.nodes[n + i]
        D_i = pu.load

        t_pi    = pu.tw_start
        arr_di  = t_pi + pu.service_time + tt[pi, di]
        t_di    = max(dr.tw_start, arr_di)
        t_di > dr.tw_end + 1e-9 &&
            error("NoDepot seed route for request $i infeasible: dropoff time window violated")

        ride_t = t_di - t_pi - pu.service_time
        ride_t > instance.L + 1e-9 &&
            error("NoDepot seed route for request $i infeasible: ride time violated")

        dist     = td[pi, di]
        duration = t_di + dr.service_time - t_pi

        push!(pool.routes, NoDepotRoute(
            [i, n + i],
            [pi, di],
            [i],
            Dict(i => D_i),
            dist,
            NaN,
            [t_pi, t_di],
            [D_i, 0],
            [ride_t],
            dist,
            duration
        ))
    end
    return pool
end

function _generate_nd_demand_seed_routes(instance::DARPInstance) :: NoDepotPool
    n    = instance.n
    Q    = instance.Q
    pool = NoDepotPool()
    tt   = instance.travel_time
    td   = instance.travel_distance

    for i in 1:n
        pi    = i + 1
        di    = n + i + 1
        pu    = instance.nodes[i]
        dr    = instance.nodes[n + i]
        D_i   = pu.load
        alpha = min(D_i, Q)

        t_pi    = pu.tw_start
        arr_di  = t_pi + pu.service_time + tt[pi, di]
        t_di    = max(dr.tw_start, arr_di)
        t_di > dr.tw_end + 1e-9 &&
            error("NoDepot seed route for request $i infeasible: dropoff time window violated")

        ride_t = t_di - t_pi - pu.service_time
        ride_t > instance.L + 1e-9 &&
            error("NoDepot seed route for request $i infeasible: ride time violated")

        dist     = td[pi, di]
        duration = t_di + dr.service_time - t_pi

        push!(pool.routes, NoDepotRoute(
            [i, n + i],
            [pi, di],
            [i],
            Dict(i => alpha),
            dist,
            NaN,
            [t_pi, t_di],
            [alpha, 0],
            [ride_t],
            dist,
            duration
        ))
    end
    return pool
end
