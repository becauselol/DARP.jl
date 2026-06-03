import Gurobi

"""
    build_nd_rmp(instance, pool, env) → (model, λ_vars, cov_cons)

LP relaxation of the no-depot restricted master problem.
Coverage: Σ_{r:i∈r} α_{ir} · λ_r ≥ D_i for each request i.
λ_r ≥ 0, unbounded above (needed when α_{ir} < D_i as in the demand variant).
"""
function build_nd_rmp(
    instance :: DARPInstance,
    pool     :: NoDepotPool,
    env      :: Gurobi.Env
)
    n = instance.n
    D = [instance.nodes[i].load for i in 1:n]
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "Method",     1)
    JuMP.set_optimizer_attribute(model, "Presolve",   0)
    JuMP.set_optimizer_attribute(model, "OutputFlag", 0)

    λ_vars = @variable(model, λ[1:R] >= 0)

    cov_cons = [@constraint(model,
        sum(Float64(pool.routes[r].alpha[i]) * λ[r]
            for r in 1:R if haskey(pool.routes[r].alpha, i)) >= Float64(D[i]))
        for i in 1:n]

    @objective(model, Min,
        sum(pool.routes[r].cost * λ[r] for r in 1:R))

    return model, λ_vars, cov_cons
end

function solve_nd_rmp!(model::JuMP.Model) :: Symbol
    JuMP.optimize!(model)
    st = JuMP.termination_status(model)
    st == JuMP.MOI.OPTIMAL    && return :optimal
    st == JuMP.MOI.INFEASIBLE && return :infeasible
    st == JuMP.MOI.TIME_LIMIT && return :timeout
    return :error
end

function extract_nd_duals(cov_cons::Vector{<:JuMP.ConstraintRef}) :: NoDepotDuals
    return NoDepotDuals([JuMP.dual(c) for c in cov_cons])
end

"""
    add_nd_column!(model, pool, route, cov_cons, λ_vars)

Hot-add a no-depot column. Coverage coefficient is `route.alpha[i]`.
New λ variable is unbounded above.
"""
function add_nd_column!(
    model    :: JuMP.Model,
    pool     :: NoDepotPool,
    route    :: NoDepotRoute,
    cov_cons :: Vector{<:JuMP.ConstraintRef},
    λ_vars   :: Vector{JuMP.VariableRef}
) :: JuMP.VariableRef
    push!(pool.routes, route)

    new_λ = @variable(model, lower_bound = 0.0)
    push!(λ_vars, new_λ)

    JuMP.set_objective_coefficient(model, new_λ, route.cost)

    for i in route.request_ids
        JuMP.set_normalized_coefficient(cov_cons[i], new_λ, Float64(route.alpha[i]))
    end

    return new_λ
end
