import Gurobi

"""
    ColumnGenerationSolver <: AbstractDARPSolver

Solves DARP using a route-based (set partitioning) formulation with column generation.

The LP relaxation of the restricted master problem is solved with Gurobi (dual simplex,
warm-started re-solves). The pricing subproblem is solved via a forward SPPRC labeling
algorithm. After LP convergence the master IP is solved over the final column pool.

`solve_pricing` is exported and callable independently of a running model.
"""
struct ColumnGenerationSolver <: AbstractDARPSolver
    time_limit_sec      :: Float64
    max_cg_iters        :: Int
    mip_gap             :: Float64
    verbose             :: Bool
    detour_factor       :: Float64
    solve_ip            :: Bool
    max_routes_per_iter :: Int
    rc_tolerance        :: Float64
end

function ColumnGenerationSolver(;
    time_limit_sec      :: Float64 = 3600.0,
    max_cg_iters        :: Int     = 500,
    mip_gap             :: Float64 = 1e-4,
    verbose             :: Bool    = false,
    detour_factor       :: Float64 = Inf,
    solve_ip            :: Bool    = true,
    max_routes_per_iter :: Int     = 10,
    rc_tolerance        :: Float64 = -1e-6
) :: ColumnGenerationSolver
    detour_factor >= 1.0 || throw(ArgumentError("detour_factor must be ≥ 1.0"))
    time_limit_sec > 0   || throw(ArgumentError("time_limit_sec must be > 0"))
    max_cg_iters > 0     || throw(ArgumentError("max_cg_iters must be > 0"))
    return ColumnGenerationSolver(
        time_limit_sec, max_cg_iters, mip_gap, verbose,
        detour_factor, solve_ip, max_routes_per_iter, rc_tolerance
    )
end

function solve(solver::ColumnGenerationSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    t0 = time()
    n  = instance.n
    K  = instance.K

    # ── Seed pool ─────────────────────────────────────────────────────────────
    pool = _generate_seed_routes(instance)
    solver.verbose && println("[CG] Initialized with $(length(pool.routes)) seed routes")

    env   = Gurobi.Env()
    model, λ_vars, a_vars, cov_cons, fleet_con = build_rmp(instance, pool, env)

    lp_obj = Inf

    # ── CG loop ───────────────────────────────────────────────────────────────
    for iter in 1:solver.max_cg_iters
        remaining = solver.time_limit_sec - (time() - t0)
        remaining <= 0 && break

        JuMP.set_time_limit_sec(model, remaining)
        lp_status = solve_rmp!(model)

        if lp_status == :infeasible
            solver.verbose && println("[CG] RMP infeasible at iteration $iter")
            elapsed = time() - t0
            return DARPSolution(instance,
                [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
                Inf, false, "ColumnGenerationSolver", elapsed, :infeasible)
        end

        lp_obj = JuMP.objective_value(model)

        art_sum = sum(JuMP.value(a_vars[i]) for i in 1:n)
        if solver.verbose
            art_str = art_sum > 1e-4 ? "  (artificials = $(round(art_sum, digits=4)))" : ""
            println("[CG] Iter $iter: LP obj = $(round(lp_obj, digits=4))$art_str")
        end

        duals  = extract_duals(cov_cons, fleet_con)

        # Budget pricing: leave at least 10s for the IP after CG convergence
        pricing_budget = max(1.0, (solver.time_limit_sec - (time() - t0) - 10.0) /
                                   max(1, solver.max_cg_iters - iter + 1))

        new_routes = solve_pricing(
            instance, duals;
            detour_factor = solver.detour_factor,
            rc_tolerance  = solver.rc_tolerance,
            max_routes    = solver.max_routes_per_iter,
            time_limit    = pricing_budget
        )

        if isempty(new_routes)
            solver.verbose && println("[CG] No improving columns — LP optimal after $iter iterations")
            break
        end

        added = 0
        for route in new_routes
            any(r.cordeau_seq == route.cordeau_seq for r in pool.routes) && continue
            add_column!(model, pool, route, cov_cons, fleet_con, λ_vars)
            added += 1
        end
        solver.verbose && println("[CG]   Added $added new columns (pool size = $(length(pool.routes)))")

        # All found routes are already in the pool — LP is optimal w.r.t. current columns
        added == 0 && break
    end

    elapsed = time() - t0

    # ── Final IP ──────────────────────────────────────────────────────────────
    if solver.solve_ip
        # Always give the IP at least 10s; CG budget should leave room via time_limit_sec
        ip_budget = max(10.0, solver.time_limit_sec - elapsed)
        ip_status, ip_obj, selected = solve_master_ip(
            instance, pool, env, ip_budget, solver.mip_gap, solver.verbose
        )
        elapsed = time() - t0
        return build_solution(instance, selected, ip_obj, ip_status, elapsed)
    else
        # LP-only mode: report the LP lower bound with no primal integer solution.
        # Subtract artificial contribution so the bound reflects real travel cost.
        art_sum = sum(JuMP.value(a_vars[i]) for i in 1:n)
        lp_bound = lp_obj - _ARTIFICIAL_M * art_sum
        return DARPSolution(instance,
            [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0) for k in 1:K],
            lp_bound, false, "ColumnGenerationSolver", elapsed, :lp_relaxation)
    end
end

# ── Seed route generation ──────────────────────────────────────────────────────

function _generate_seed_routes(instance::DARPInstance) :: ColumnPool
    n      = instance.n
    N      = 2 * n + 2
    origin = 1
    dest   = N
    pool   = ColumnPool()

    tt = instance.travel_time
    td = instance.travel_distance

    for i in 1:n
        pi = i + 1       # Julia pickup
        di = n + i + 1   # Julia dropoff

        pu = instance.nodes[i]
        dr = instance.nodes[n + i]

        # Service time at pickup
        arr_pi = instance.depot_origin.tw_start +
                 instance.depot_origin.service_time + tt[origin, pi]
        t_pi = max(pu.tw_start, arr_pi)
        t_pi > pu.tw_end + 1e-9 &&
            error("Seed route for request $i infeasible: pickup time window violated")

        # Service time at dropoff
        arr_di = t_pi + pu.service_time + tt[pi, di]
        t_di = max(dr.tw_start, arr_di)
        t_di > dr.tw_end + 1e-9 &&
            error("Seed route for request $i infeasible: dropoff time window violated")

        # Ride time check
        ride_t = t_di - t_pi - pu.service_time
        ride_t > instance.L + 1e-9 &&
            error("Seed route for request $i infeasible: ride time $ride_t > L=$(instance.L)")

        # Duration
        arr_dest = t_di + dr.service_time + tt[di, dest]
        t_dest = max(instance.depot_destination.tw_start, arr_dest)
        dur = t_dest - instance.depot_origin.tw_start

        # Distances
        dist = td[origin, pi] + td[pi, di] + td[di, dest]
        cost = dist

        fr = FeasibleRoute(
            [i, n + i],           # cordeau_seq (1-based Cordeau: pickup=i, dropoff=n+i)
            [pi, di],             # julia_seq
            [i],                  # request_ids
            cost,
            NaN,                  # reduced_cost: not from pricing
            [t_pi, t_di],         # service_times
            [pu.load, 0],         # loads: +load at pickup, back to 0 at dropoff
            [ride_t],             # ride_times
            dist,
            dur
        )
        push!(pool.routes, fr)
    end

    return pool
end

