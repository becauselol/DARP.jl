import Gurobi

"""
    build_sd_rmp(instance, pool, env) → (model, λ_vars, cov_cons)

Build the LP relaxation of the split-delivery restricted master problem.

Coverage constraint: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i for each request i.
The coefficient α_{ir} (units of demand i carried by route r) varies per column
and is stored in `SplitDeliveryNoCapRoute.alpha`.
"""
function build_sd_rmp(
    instance :: DARPInstance,
    pool     :: SplitDeliveryNoCapPool,
    env      :: Gurobi.Env
)
    n = instance.n
    D = [instance.nodes[i].load for i in 1:n]
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "Method",     1)  # dual simplex
    JuMP.set_optimizer_attribute(model, "Presolve",   0)
    JuMP.set_optimizer_attribute(model, "OutputFlag", 0)

    λ_vars = @variable(model, 0 <= λ[1:R] <= 1)

    cov_cons = [@constraint(model,
        sum(Float64(pool.routes[r].alpha[i]) * λ[r]
            for r in 1:R if haskey(pool.routes[r].alpha, i)) >= Float64(D[i]))
        for i in 1:n]

    @objective(model, Min,
        sum(pool.routes[r].cost * λ[r] for r in 1:R))

    return model, λ_vars, cov_cons
end

"""
    solve_sd_rmp!(model) → Symbol
"""
function solve_sd_rmp!(model::JuMP.Model) :: Symbol
    JuMP.optimize!(model)
    st = JuMP.termination_status(model)
    st == JuMP.MOI.OPTIMAL    && return :optimal
    st == JuMP.MOI.INFEASIBLE && return :infeasible
    st == JuMP.MOI.TIME_LIMIT && return :timeout
    return :error
end

"""
    extract_sd_duals(cov_cons) → SplitDeliveryNoCapDuals

π_i ≥ 0 (dual of a ≥ constraint in minimization).
"""
function extract_sd_duals(
    cov_cons :: Vector{<:JuMP.ConstraintRef}
) :: SplitDeliveryNoCapDuals
    return SplitDeliveryNoCapDuals([JuMP.dual(c) for c in cov_cons])
end

"""
    add_sd_column!(model, pool, route, cov_cons, λ_vars)

Hot-add a split-delivery column. The coefficient for coverage constraint i is
`route.alpha[i]` (integer units), not 1.0 as in the unit-demand model.
"""
function add_sd_column!(
    model    :: JuMP.Model,
    pool     :: SplitDeliveryNoCapPool,
    route    :: SplitDeliveryNoCapRoute,
    cov_cons :: Vector{<:JuMP.ConstraintRef},
    λ_vars   :: Vector{JuMP.VariableRef}
) :: JuMP.VariableRef
    push!(pool.routes, route)

    new_λ = @variable(model, lower_bound = 0.0, upper_bound = 1.0)
    push!(λ_vars, new_λ)

    JuMP.set_objective_coefficient(model, new_λ, route.cost)

    for i in route.request_ids
        JuMP.set_normalized_coefficient(cov_cons[i], new_λ, Float64(route.alpha[i]))
    end

    return new_λ
end
