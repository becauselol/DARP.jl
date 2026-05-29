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
    time_limit_sec      :: Float64
    max_cg_iters        :: Int
    mip_gap             :: Float64
    verbose             :: Bool
    detour_factor       :: Float64
    solve_ip            :: Bool
    max_routes_per_iter :: Int
    rc_tolerance        :: Float64
end

function SplitDeliveryNoCapCGSolver(;
    time_limit_sec      :: Float64 = 3600.0,
    max_cg_iters        :: Int     = 500,
    mip_gap             :: Float64 = 1e-4,
    verbose             :: Bool    = false,
    detour_factor       :: Float64 = Inf,
    solve_ip            :: Bool    = true,
    max_routes_per_iter :: Int     = 10,
    rc_tolerance        :: Float64 = -1e-6
) :: SplitDeliveryNoCapCGSolver
    detour_factor >= 1.0 || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    time_limit_sec > 0   || throw(ArgumentError("time_limit_sec must be > 0"))
    max_cg_iters > 0     || throw(ArgumentError("max_cg_iters must be > 0"))
    return SplitDeliveryNoCapCGSolver(
        time_limit_sec, max_cg_iters, mip_gap, verbose,
        detour_factor, solve_ip, max_routes_per_iter, rc_tolerance
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

    lp_obj = Inf

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
                Inf, false, "SplitDeliveryNoCapCGSolver", elapsed, :infeasible)
        end

        lp_obj = JuMP.objective_value(model)
        solver.verbose && println("[SD-CG] Iter $iter: LP obj = $(round(lp_obj, digits=4))")

        duals = extract_sd_duals(cov_cons)

        pricing_budget = max(1.0, (solver.time_limit_sec - (time() - t0) - 10.0) /
                                   max(1, solver.max_cg_iters - iter + 1))

        new_routes = solve_nocap_pricing(
            instance, duals;
            detour_factor = solver.detour_factor,
            rc_tolerance  = solver.rc_tolerance,
            max_routes    = solver.max_routes_per_iter,
            time_limit    = pricing_budget
        )

        if isempty(new_routes)
            solver.verbose && println("[SD-CG] No improving columns — LP optimal after $iter iterations")
            break
        end

        added = 0
        for route in new_routes
            # Same path → same greedy alpha → skip duplicates
            any(r.cordeau_seq == route.cordeau_seq for r in pool.routes) && continue
            add_sd_column!(model, pool, route, cov_cons, λ_vars)
            added += 1
        end
        solver.verbose && println("[SD-CG]   Added $added new columns (pool = $(length(pool.routes)))")

        added == 0 && break
    end

    elapsed = time() - t0

    if solver.solve_ip
        ip_budget = max(10.0, solver.time_limit_sec - elapsed)
        ip_status, ip_obj, selected = solve_sd_master_ip(
            instance, pool, env, ip_budget, solver.mip_gap, solver.verbose
        )
        elapsed = time() - t0
        return build_sd_solution(instance, selected, ip_obj, ip_status, elapsed)
    else
        return DARPSolution(instance,
            [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
            lp_obj, false, "SplitDeliveryNoCapCGSolver", elapsed, :lp_relaxation)
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
