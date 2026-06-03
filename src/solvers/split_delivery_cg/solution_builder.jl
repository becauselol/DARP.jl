"""
    build_sd_solution(instance, selected, obj_val, status, solve_time,
                      lp_bound, n_cg_iters, iter_log) → DARPSolution

Convert a vector of selected `SplitDeliveryNoCapRoute`s into a `DARPSolution`.
Pads with idle routes for unused vehicle slots up to K.
"""
function build_sd_solution(
    instance   :: DARPInstance,
    selected   :: Vector{SplitDeliveryNoCapRoute},
    obj_val    :: Float64,
    status     :: Symbol,
    solve_time :: Float64,
    lp_bound   :: Float64               = NaN,
    n_cg_iters :: Int                   = 0,
    iter_log   :: Vector{CGIterLogEntry} = CGIterLogEntry[]
) :: DARPSolution
    K       = instance.K
    is_feas = status in (:optimal, :feasible)

    routes = Route[]
    for (vid, fr) in enumerate(selected)
        push!(routes, Route(
            vid,
            fr.cordeau_seq,
            fr.service_times,
            fr.loads,
            fr.ride_times,
            fr.total_distance,
            fr.total_duration
        ))
    end

    for vid in (length(selected) + 1):K
        push!(routes, Route(vid, Int[], Float64[], Int[], Float64[], 0.0, 0.0))
    end

    return DARPSolution(instance, routes, obj_val, is_feas,
                        "SplitDeliveryNoCapCGSolver", solve_time, status,
                        lp_bound, n_cg_iters, iter_log)
end
