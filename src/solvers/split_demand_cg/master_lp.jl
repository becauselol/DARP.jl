import Gurobi

"""
    build_sd_demand_rmp(instance, pool, env) → (model, λ_vars, cov_cons)

LP relaxation of the split-demand restricted master problem.

Coverage: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i  (α_{ir} ≤ min(D_i, Q)).
λ_r is unbounded above (no ≤ 1 constraint): with α_ir < D_i possible, the LP
needs fractional λ_r > 1 on a single seed route to remain feasible before
CG generates routes that collectively cover D_i at integer λ values.
"""
function build_sd_demand_rmp(
    instance :: DARPInstance,
    pool     :: SplitDemandPool,
    env      :: Gurobi.Env
)
    n = instance.n
    D = [instance.nodes[i].load for i in 1:n]
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "Method",     1)
    JuMP.set_optimizer_attribute(model, "Presolve",   0)
    JuMP.set_optimizer_attribute(model, "OutputFlag", 0)

    λ_vars = @variable(model, λ[1:R] >= 0)   # unbounded above

    cov_cons = [@constraint(model,
        sum(Float64(pool.routes[r].alpha[i]) * λ[r]
            for r in 1:R if haskey(pool.routes[r].alpha, i)) >= Float64(D[i]))
        for i in 1:n]

    @objective(model, Min,
        sum(pool.routes[r].cost * λ[r] for r in 1:R))

    return model, λ_vars, cov_cons
end

function solve_sd_demand_rmp!(model::JuMP.Model) :: Symbol
    JuMP.optimize!(model)
    st = JuMP.termination_status(model)
    st == JuMP.MOI.OPTIMAL    && return :optimal
    st == JuMP.MOI.INFEASIBLE && return :infeasible
    st == JuMP.MOI.TIME_LIMIT && return :timeout
    return :error
end

function extract_sd_demand_duals(
    cov_cons :: Vector{<:JuMP.ConstraintRef}
) :: SplitDemandDuals
    return SplitDemandDuals([JuMP.dual(c) for c in cov_cons])
end

"""
    add_sd_demand_column!(model, pool, route, cov_cons, λ_vars)

Hot-add a split-demand column. Coverage coefficient is `route.alpha[i]` (may be < D_i).
New λ variable is unbounded above to match the LP relaxation.
"""
function add_sd_demand_column!(
    model    :: JuMP.Model,
    pool     :: SplitDemandPool,
    route    :: SplitDemandRoute,
    cov_cons :: Vector{<:JuMP.ConstraintRef},
    λ_vars   :: Vector{JuMP.VariableRef}
) :: JuMP.VariableRef
    push!(pool.routes, route)

    new_λ = @variable(model, lower_bound = 0.0)   # unbounded above
    push!(λ_vars, new_λ)

    JuMP.set_objective_coefficient(model, new_λ, route.cost)

    for i in route.request_ids
        JuMP.set_normalized_coefficient(cov_cons[i], new_λ, Float64(route.alpha[i]))
    end

    return new_λ
end
