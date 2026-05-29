"""
    solve_sd_demand_master_ip(instance, pool, env, time_limit_sec, mip_gap, verbose)
        → (status::Symbol, obj::Float64, selected::Vector{SplitDemandRoute})

Binary covering MIP over the final split-demand column pool.
Coverage: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i  (integer coefficients α_{ir} ≤ min(D_i, Q)).
"""
function solve_sd_demand_master_ip(
    instance       :: DARPInstance,
    pool           :: SplitDemandPool,
    env            :: Gurobi.Env,
    time_limit_sec :: Float64,
    mip_gap        :: Float64,
    verbose        :: Bool
) :: Tuple{Symbol, Float64, Vector{SplitDemandRoute}}
    n = instance.n
    D = [instance.nodes[i].load for i in 1:n]
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "TimeLimit",  time_limit_sec)
    JuMP.set_optimizer_attribute(model, "MIPGap",     mip_gap)
    JuMP.set_optimizer_attribute(model, "OutputFlag", verbose ? 1 : 0)

    @variable(model, λ[1:R], Bin)

    for i in 1:n
        @constraint(model,
            sum(Float64(pool.routes[r].alpha[i]) * λ[r]
                for r in 1:R if haskey(pool.routes[r].alpha, i)) >= Float64(D[i]))
    end

    @objective(model, Min, sum(pool.routes[r].cost * λ[r] for r in 1:R))

    JuMP.optimize!(model)
    term_st   = JuMP.termination_status(model)
    primal_st = JuMP.primal_status(model)

    if primal_st != JuMP.MOI.FEASIBLE_POINT
        status = term_st == JuMP.MOI.INFEASIBLE ? :infeasible :
                 term_st == JuMP.MOI.TIME_LIMIT  ? :timeout   : :error
        return status, Inf, SplitDemandRoute[]
    end

    obj      = JuMP.objective_value(model)
    selected = [pool.routes[r] for r in 1:R if JuMP.value(λ[r]) > 0.5]
    status   = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible

    return status, obj, selected
end
