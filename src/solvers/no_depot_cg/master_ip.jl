"""
    solve_nd_nocap_master_ip(instance, pool, env, time_limit_sec, mip_gap, verbose)
        → (status, obj, selected)

Binary covering MIP for the NoDepotNoCap formulation.
α_{ir} = D_i always, so coverage reduces to Σ_{r covers i} λ_r ≥ 1.
Binary variables: each route used at most once.
"""
function solve_nd_nocap_master_ip(
    instance       :: DARPInstance,
    pool           :: NoDepotPool,
    env            :: Gurobi.Env,
    time_limit_sec :: Float64,
    mip_gap        :: Float64,
    verbose        :: Bool
) :: Tuple{Symbol, Float64, Vector{NoDepotRoute}}
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
        return status, Inf, NoDepotRoute[]
    end

    obj      = JuMP.objective_value(model)
    selected = [pool.routes[r] for r in 1:R if JuMP.value(λ[r]) > 0.5]
    status   = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible
    return status, obj, selected
end

"""
    solve_nd_demand_master_ip(instance, pool, env, time_limit_sec, mip_gap, verbose)
        → (status, obj, selected)

Nonneg-integer covering MIP for the NoDepotDemand formulation.
α_{ir} ≤ min(D_i, Q), so multiple uses of the same route may be needed.
"""
function solve_nd_demand_master_ip(
    instance       :: DARPInstance,
    pool           :: NoDepotPool,
    env            :: Gurobi.Env,
    time_limit_sec :: Float64,
    mip_gap        :: Float64,
    verbose        :: Bool
) :: Tuple{Symbol, Float64, Vector{NoDepotRoute}}
    n = instance.n
    D = [instance.nodes[i].load for i in 1:n]
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "TimeLimit",  time_limit_sec)
    JuMP.set_optimizer_attribute(model, "MIPGap",     mip_gap)
    JuMP.set_optimizer_attribute(model, "OutputFlag", verbose ? 1 : 0)

    @variable(model, λ[1:R] >= 0, Int)

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
        return status, Inf, NoDepotRoute[]
    end

    obj      = JuMP.objective_value(model)
    selected = [pool.routes[r] for r in 1:R if JuMP.value(λ[r]) > 0.5]
    status   = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible
    return status, obj, selected
end
