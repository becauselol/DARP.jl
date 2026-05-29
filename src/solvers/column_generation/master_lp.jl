import Gurobi

"""
    build_rmp(instance, pool, env) → (model, λ_vars, cov_cons)

Build the LP relaxation of the restricted master problem using Gurobi.

Uses the covering formulation (≥ 1) with no fleet constraint. With single-request seed
routes the LP is always feasible.

`env` is a shared `Gurobi.Env` to avoid per-model license acquisition overhead.
"""
function build_rmp(
    instance :: DARPInstance,
    pool     :: ColumnPool,
    env      :: Gurobi.Env
)
    n = instance.n
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "Method",      1)  # dual simplex for warm re-solves
    JuMP.set_optimizer_attribute(model, "Presolve",    0)
    JuMP.set_optimizer_attribute(model, "OutputFlag",  0)

    # LP relaxation: λ ∈ [0,1]
    λ_vars = @variable(model, 0 <= λ[1:R] <= 1)

    # Covering: Σ_{r:i∈r} λ_r ≥ 1 for each request i
    cov_cons = [@constraint(model,
        sum(λ[r] for r in 1:R if i in pool.routes[r].request_ids) >= 1.0)
        for i in 1:n]

    @objective(model, Min,
        sum(pool.routes[r].cost * λ[r] for r in 1:R))

    return model, λ_vars, cov_cons
end

"""
    solve_rmp!(model) → Symbol

Solve the LP relaxation. Returns `:optimal`, `:infeasible`, `:timeout`, or `:error`.
"""
function solve_rmp!(model::JuMP.Model) :: Symbol
    JuMP.optimize!(model)
    st = JuMP.termination_status(model)
    st == JuMP.MOI.OPTIMAL          && return :optimal
    st == JuMP.MOI.INFEASIBLE       && return :infeasible
    st == JuMP.MOI.TIME_LIMIT       && return :timeout
    return :error
end

"""
    extract_duals(cov_cons) → CGDuals

Extract dual variables from the LP relaxation after solving.
`pi[i]` is the coverage dual (≥ 0 for a ≥ constraint in minimization).
`mu` is always 0.0 (no fleet constraint).
"""
function extract_duals(
    cov_cons :: Vector{<:JuMP.ConstraintRef}
) :: CGDuals
    pi = [JuMP.dual(c) for c in cov_cons]
    return CGDuals(pi, 0.0)
end

"""
    add_column!(model, pool, route, cov_cons, λ_vars)

Hot-add a column to the existing model without rebuilding.
Gurobi preserves the simplex basis enabling warm-start dual simplex re-solves.
"""
function add_column!(
    model    :: JuMP.Model,
    pool     :: ColumnPool,
    route    :: FeasibleRoute,
    cov_cons :: Vector{<:JuMP.ConstraintRef},
    λ_vars   :: Vector{JuMP.VariableRef}
) :: JuMP.VariableRef
    push!(pool.routes, route)

    new_λ = @variable(model, lower_bound = 0.0, upper_bound = 1.0)
    push!(λ_vars, new_λ)

    JuMP.set_objective_coefficient(model, new_λ, route.cost)

    for i in route.request_ids
        JuMP.set_normalized_coefficient(cov_cons[i], new_λ, 1.0)
    end

    return new_λ
end
