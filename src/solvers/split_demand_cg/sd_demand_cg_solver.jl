import Gurobi

"""
    SplitDemandCGSolver <: AbstractDARPSolver

Column generation solver for the split-demand DARP with finite vehicle capacity Q.

Master LP: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i for each request i, no fleet constraint.
α_{ir} = amount of demand i carried by route r, determined in the pricing subproblem
by enumerating α = 1…min(Q − load_before_pickup, D_i) at each pickup extension.

Pricing: forward SPPRC with exact α enumeration. Multiple child labels are spawned
per pickup (one per feasible boarding amount), allowing the same physical path to
appear as distinct columns with different load profiles and reduced costs.

After LP convergence, a binary covering MIP is solved over the final column pool.
"""
struct SplitDemandCGSolver <: AbstractDARPSolver
    time_limit_sec      :: Float64
    max_cg_iters        :: Int
    mip_gap             :: Float64
    verbose             :: Bool
    detour_factor       :: Float64
    solve_ip            :: Bool
    max_routes_per_iter :: Int
    rc_tolerance        :: Float64
end

function SplitDemandCGSolver(;
    time_limit_sec      :: Float64 = 3600.0,
    max_cg_iters        :: Int     = 500,
    mip_gap             :: Float64 = 1e-4,
    verbose             :: Bool    = false,
    detour_factor       :: Float64 = Inf,
    solve_ip            :: Bool    = true,
    max_routes_per_iter :: Int     = 10,
    rc_tolerance        :: Float64 = -1e-6
) :: SplitDemandCGSolver
    detour_factor >= 1.0 || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    time_limit_sec > 0   || throw(ArgumentError("time_limit_sec must be > 0"))
    max_cg_iters > 0     || throw(ArgumentError("max_cg_iters must be > 0"))
    return SplitDemandCGSolver(
        time_limit_sec, max_cg_iters, mip_gap, verbose,
        detour_factor, solve_ip, max_routes_per_iter, rc_tolerance
    )
end

function solve(solver::SplitDemandCGSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()
    n  = instance.n
    K  = instance.K

    pool = _generate_sd_demand_seed_routes(instance)
    solver.verbose && println("[SD-Demand-CG] Initialized with $(length(pool.routes)) seed routes")

    env   = Gurobi.Env()
    model, λ_vars, cov_cons = build_sd_demand_rmp(instance, pool, env)

    lp_obj   = Inf
    iter_log = CGIterLogEntry[]
    n_iters  = 0

    for iter in 1:solver.max_cg_iters
        remaining = solver.time_limit_sec - (time() - t0)
        remaining <= 0 && break

        JuMP.set_time_limit_sec(model, remaining)
        lp_status = solve_sd_demand_rmp!(model)

        if lp_status == :infeasible
            solver.verbose && println("[SD-Demand-CG] RMP infeasible at iteration $iter")
            elapsed = time() - t0
            return DARPSolution(instance,
                [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
                Inf, false, "SplitDemandCGSolver", elapsed, :infeasible,
                NaN, iter, iter_log)
        end

        lp_obj  = JuMP.objective_value(model)
        n_iters = iter
        solver.verbose && println("[SD-Demand-CG] Iter $iter: LP obj = $(round(lp_obj, digits=4))")

        duals = extract_sd_demand_duals(cov_cons)

        pricing_budget = max(1.0, (solver.time_limit_sec - (time() - t0) - 10.0) /
                                   max(1, solver.max_cg_iters - iter + 1))

        new_routes = solve_split_demand_pricing(
            instance, duals;
            detour_factor = solver.detour_factor,
            rc_tolerance  = solver.rc_tolerance,
            max_routes    = solver.max_routes_per_iter,
            time_limit    = pricing_budget
        )

        added = 0
        if !isempty(new_routes)
            for route in new_routes
                # Same path AND same alpha → identical column; skip duplicates
                any(r.cordeau_seq == route.cordeau_seq &&
                    r.alpha == route.alpha for r in pool.routes) && continue
                add_sd_demand_column!(model, pool, route, cov_cons, λ_vars)
                added += 1
            end
        end

        push!(iter_log, (iter=iter, lp_obj=lp_obj, cols_added=added))
        solver.verbose && println("[SD-Demand-CG]   Added $added new columns (pool = $(length(pool.routes)))")

        (isempty(new_routes) || added == 0) && begin
            solver.verbose && println("[SD-Demand-CG] No improving columns — LP optimal after $iter iterations")
            break
        end
    end

    elapsed = time() - t0

    if solver.solve_ip
        ip_budget = max(10.0, solver.time_limit_sec - elapsed)
        ip_status, ip_obj, selected = solve_sd_demand_master_ip(
            instance, pool, env, ip_budget, solver.mip_gap, solver.verbose
        )
        elapsed = time() - t0
        return build_sd_demand_solution(instance, selected, ip_obj, ip_status, elapsed,
                                        lp_obj, n_iters, iter_log)
    else
        return DARPSolution(instance,
            [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
            lp_obj, false, "SplitDemandCGSolver", elapsed, :lp_relaxation,
            lp_obj, n_iters, iter_log)
    end
end

# ── Seed route generation ──────────────────────────────────────────────────────

function _generate_sd_demand_seed_routes(instance::DARPInstance) :: SplitDemandPool
    n      = instance.n
    Q      = instance.Q
    N      = 2 * n + 2
    origin = 1
    dest   = N
    pool   = SplitDemandPool()

    tt = instance.travel_time
    td = instance.travel_distance

    for i in 1:n
        pi  = i + 1
        di  = n + i + 1
        pu  = instance.nodes[i]
        dr  = instance.nodes[n + i]
        D_i = pu.load

        # Board as many units as capacity allows (min(D_i, Q))
        alpha_i = min(D_i, Q)

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

        push!(pool.routes, SplitDemandRoute(
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
