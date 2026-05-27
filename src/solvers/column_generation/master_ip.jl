"""
    solve_master_ip(instance, pool, env, time_limit_sec, mip_gap, verbose)
        → (status::Symbol, obj::Float64, selected::Vector{FeasibleRoute})

Build and solve a binary set-partitioning MIP over the final column pool.
Uses the shared `Gurobi.Env` to avoid license acquisition overhead.
"""
function solve_master_ip(
    instance       :: DARPInstance,
    pool           :: ColumnPool,
    env            :: Gurobi.Env,
    time_limit_sec :: Float64,
    mip_gap        :: Float64,
    verbose        :: Bool
) :: Tuple{Symbol, Float64, Vector{FeasibleRoute}}
    n = instance.n
    K = instance.K
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "TimeLimit",  time_limit_sec)
    JuMP.set_optimizer_attribute(model, "MIPGap",     mip_gap)
    JuMP.set_optimizer_attribute(model, "OutputFlag", verbose ? 1 : 0)

    @variable(model, λ[1:R], Bin)

    for i in 1:n
        @constraint(model,
            sum(λ[r] for r in 1:R if i in pool.routes[r].request_ids) == 1)
    end

    @constraint(model, sum(λ[r] for r in 1:R) <= Float64(K))

    @objective(model, Min, sum(pool.routes[r].cost * λ[r] for r in 1:R))

    JuMP.optimize!(model)
    term_st   = JuMP.termination_status(model)
    primal_st = JuMP.primal_status(model)

    has_sol = primal_st == JuMP.MOI.FEASIBLE_POINT
    if !has_sol
        status = term_st == JuMP.MOI.INFEASIBLE ? :infeasible :
                 term_st == JuMP.MOI.TIME_LIMIT  ? :timeout   : :error
        return status, Inf, FeasibleRoute[]
    end

    obj = JuMP.objective_value(model)
    selected = [pool.routes[r] for r in 1:R if JuMP.value(λ[r]) > 0.5]
    status = term_st == JuMP.MOI.OPTIMAL ? :optimal : :feasible

    return status, obj, selected
end
