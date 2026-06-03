import Gurobi

"""
    SplitDeliveryNoCapCGSolver <: AbstractDARPSolver

Solves DARP using a split-delivery set-covering column generation formulation.

Master LP: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i for each request i, no fleet constraint.
α_{ir} = min(D_i, Q − load_before_pickup_i) — vehicles board as many units as capacity
allows, potentially leaving D_i − α_{ir} units to be served by other routes.

Pricing subproblem: forward SPPRC with greedy α computation at each pickup extension.
After LP convergence, a binary covering MIP is solved over the final column pool.
"""
struct SplitDeliveryNoCapCGSolver <: AbstractDARPSolver
    time_limit_sec        :: Float64
    max_cg_iters          :: Int
    mip_gap               :: Float64
    verbose               :: Bool
    detour_factor         :: Float64
    solve_ip              :: Bool
    max_routes_per_iter   :: Int
    rc_tolerance          :: Float64
    pricing_time_per_iter :: Float64  # fixed per-iteration pricing budget (seconds)
    patience              :: Int      # consecutive exhausted-zero iterations to declare LP optimal
    timeout_patience      :: Int      # consecutive timed-out-zero iterations before heuristic termination
    ip_time_limit_sec     :: Float64  # independent time budget for the final IP solve
end

function SplitDeliveryNoCapCGSolver(;
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
) :: SplitDeliveryNoCapCGSolver
    detour_factor >= 1.0   || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    time_limit_sec > 0     || throw(ArgumentError("time_limit_sec must be > 0"))
    max_cg_iters > 0       || throw(ArgumentError("max_cg_iters must be > 0"))
    patience > 0           || throw(ArgumentError("patience must be > 0"))
    timeout_patience > 0   || throw(ArgumentError("timeout_patience must be > 0"))
    ip_time_limit_sec > 0  || throw(ArgumentError("ip_time_limit_sec must be > 0"))
    return SplitDeliveryNoCapCGSolver(
        time_limit_sec, max_cg_iters, mip_gap, verbose,
        detour_factor, solve_ip, max_routes_per_iter, rc_tolerance,
        pricing_time_per_iter, patience, timeout_patience, ip_time_limit_sec
    )
end

function solve(solver::SplitDeliveryNoCapCGSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()
    n  = instance.n
    K  = instance.K

    pool = _generate_sd_seed_routes(instance)
    solver.verbose && println("[SD-CG] Initialized with $(length(pool.routes)) seed routes")

    env   = Gurobi.Env()
    model, λ_vars, cov_cons = build_sd_rmp(instance, pool, env)

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
        lp_status = solve_sd_rmp!(model)

        if lp_status == :infeasible
            solver.verbose && println("[SD-CG] RMP infeasible at iteration $iter")
            elapsed = time() - t0
            return DARPSolution(instance,
                [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
                Inf, false, "SplitDeliveryNoCapCGSolver", elapsed, :infeasible,
                NaN, iter, iter_log)
        end

        lp_obj  = JuMP.objective_value(model)
        n_iters = iter
        solver.verbose && println("[SD-CG] Iter $iter: LP obj = $(round(lp_obj, digits=4))")

        duals = extract_sd_duals(cov_cons)

        remaining      = solver.time_limit_sec - (time() - t0)
        pricing_budget = max(1.0, min(solver.pricing_time_per_iter, remaining))

        new_routes, pricing_exhausted = solve_nocap_pricing(
            instance, duals;
            detour_factor = solver.detour_factor,
            rc_tolerance  = solver.rc_tolerance,
            max_routes    = solver.max_routes_per_iter,
            time_limit    = pricing_budget
        )

        added = 0
        for route in new_routes
            any(r.cordeau_seq == route.cordeau_seq for r in pool.routes) && continue
            add_sd_column!(model, pool, route, cov_cons, λ_vars)
            added += 1
        end

        push!(iter_log, (iter=iter, lp_obj=lp_obj, cols_added=added))
        solver.verbose && println("[SD-CG]   Added $added new columns (pool = $(length(pool.routes)))")

        if added > 0
            no_improve         = 0
            no_improve_timeout = 0
        elseif pricing_exhausted
            # Pricing ran to completion and found no improving column — real LP optimality evidence.
            no_improve += 1
            no_improve_timeout = 0
            solver.verbose && println("[SD-CG]   Pricing exhausted, no columns ($no_improve/$(solver.patience))")
            if no_improve >= solver.patience
                lp_proven_optimal = true
                solver.verbose && println("[SD-CG] LP proven optimal after $iter iterations")
                break
            end
        else
            # Pricing hit the time limit with no columns — inconclusive, but track streak.
            no_improve_timeout += 1
            solver.verbose && println("[SD-CG]   Pricing timed out, no columns ($no_improve_timeout/$(solver.timeout_patience))")
            if no_improve_timeout >= solver.timeout_patience
                solver.verbose && println("[SD-CG] Timeout patience exhausted after $iter iterations — terminating")
                break
            end
        end
    end

    elapsed = time() - t0

    if solver.solve_ip
        ip_budget = solver.ip_time_limit_sec
        ip_status, ip_obj, selected = solve_sd_master_ip(
            instance, pool, env, ip_budget, solver.mip_gap, solver.verbose
        )
        elapsed = time() - t0
        if !lp_proven_optimal && ip_status == :optimal
            ip_status = :feasible
        end
        return build_sd_solution(instance, selected, ip_obj, ip_status, elapsed,
                                 lp_obj, n_iters, iter_log)
    else
        return DARPSolution(instance,
            [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
            lp_obj, false, "SplitDeliveryNoCapCGSolver", elapsed, :lp_relaxation,
            lp_obj, n_iters, iter_log)
    end
end

# ── Seed route generation ──────────────────────────────────────────────────────

function _generate_sd_seed_routes(instance::DARPInstance) :: SplitDeliveryNoCapPool
    n      = instance.n
    N      = 2 * n + 2
    origin = 1
    dest   = N
    pool   = SplitDeliveryNoCapPool()

    tt = instance.travel_time
    td = instance.travel_distance

    for i in 1:n
        pi  = i + 1
        di  = n + i + 1
        pu  = instance.nodes[i]
        dr  = instance.nodes[n + i]
        D_i = pu.load

        # Uncapped capacity: always board the full demand D_i.
        alpha_i = D_i

        arr_pi = instance.depot_origin.tw_start +
                 instance.depot_origin.service_time + tt[origin, pi]
        t_pi = max(pu.tw_start, arr_pi)
        t_pi > pu.tw_end + 1e-9 &&
            error("Seed route for request $i infeasible: pickup time window violated")

        arr_di = t_pi + pu.service_time + tt[pi, di]
        t_di = max(dr.tw_start, arr_di)
        t_di > dr.tw_end + 1e-9 &&
            error("Seed route for request $i infeasible: dropoff time window violated")

        ride_t = t_di - t_pi - pu.service_time
        ride_t > instance.L + 1e-9 &&
            error("Seed route for request $i infeasible: ride time violated")

        arr_dest = t_di + dr.service_time + tt[di, dest]
        t_dest = max(instance.depot_destination.tw_start, arr_dest)
        dur  = t_dest - instance.depot_origin.tw_start
        dist = td[origin, pi] + td[pi, di] + td[di, dest]

        push!(pool.routes, SplitDeliveryNoCapRoute(
                [i, n + i],
                [pi, di],
                [i],
                Dict(i => alpha_i),
                dist,
                NaN,
                [t_pi, t_di],
                [alpha_i, 0],
                [ride_t],
                dist,
                dur
            ))
    end

    return pool
end
