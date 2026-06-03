"""
    build_nd_solution(instance, selected, obj_val, status, solve_time,
                      solver_name, lp_bound, n_cg_iters, iter_log) → DARPSolution

Convert selected NoDepotRoutes into a DARPSolution. Pads with idle routes up to K.
Routes have no depot endpoints; total_duration spans first pickup to last dropoff.
"""
function build_nd_solution(
    instance   :: DARPInstance,
    selected   :: Vector{NoDepotRoute},
    obj_val    :: Float64,
    status     :: Symbol,
    solve_time :: Float64,
    solver_name :: String,
    lp_bound   :: Float64               = NaN,
    n_cg_iters :: Int                   = 0,
    iter_log   :: Vector{CGIterLogEntry} = CGIterLogEntry[]
) :: DARPSolution
    K       = instance.K
    is_feas = status in (:optimal, :feasible)

    routes = Route[]
    for (vid, r) in enumerate(selected)
        push!(routes, Route(
            vid,
            r.cordeau_seq,
            r.service_times,
            r.loads,
            r.ride_times,
            r.total_distance,
            r.total_duration
        ))
    end

    for vid in (length(selected) + 1):K
        push!(routes, Route(vid, Int[], Float64[], Int[], Float64[], 0.0, 0.0))
    end

    return DARPSolution(instance, routes, obj_val, is_feas,
                        solver_name, solve_time, status,
                        lp_bound, n_cg_iters, iter_log)
end
