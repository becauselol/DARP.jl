import Gurobi

# Large-M penalty for artificial variables. Ensures any real feasible route is
# preferred over an artificial; must exceed the cost of the most expensive route.
const _ARTIFICIAL_M = 1e6

"""
    build_rmp(instance, pool, env) → (model, λ_vars, a_vars, cov_cons, fleet_con)

Build the LP relaxation of the restricted master problem using Gurobi.

Uses the partitioning formulation (= 1) + fleet constraint (≤ K). An artificial
variable a_i ≥ 0 at cost M is added to each coverage constraint so the LP is
always feasible regardless of fleet size and initial column pool. If at LP
optimality Σ a_i > 0, the DARP instance is infeasible for the given K.

`env` is a shared `Gurobi.Env` to avoid per-model license acquisition overhead.
"""
function build_rmp(
    instance :: DARPInstance,
    pool     :: ColumnPool,
    env      :: Gurobi.Env
)
    n = instance.n
    K = instance.K
    R = length(pool.routes)

    model = JuMP.Model(() -> Gurobi.Optimizer(env))
    JuMP.set_optimizer_attribute(model, "Method",      1)  # dual simplex for warm re-solves
    JuMP.set_optimizer_attribute(model, "Presolve",    0)
    JuMP.set_optimizer_attribute(model, "OutputFlag",  0)

    # LP relaxation: λ ∈ [0,1]
    λ_vars = @variable(model, 0 <= λ[1:R] <= 1)

    # Artificial variables a_i ≥ 0 at cost M (one per coverage constraint).
    # Initial LP basis: all λ = 0, all a_i = 1 → partitioning satisfied trivially.
    a_vars = @variable(model, 0 <= a[1:n])

    # Partitioning: Σ_{r:i∈r} λ_r + a_i = 1 for each request i
    cov_cons = [@constraint(model,
        sum(λ[r] for r in 1:R if i in pool.routes[r].request_ids) + a[i] == 1.0)
        for i in 1:n]

    # Fleet: total routes activated ≤ K
    fleet_con = @constraint(model, sum(λ[r] for r in 1:R) <= Float64(K))

    # Objective: minimize real travel cost + M·artificials
    @objective(model, Min,
        sum(pool.routes[r].cost * λ[r] for r in 1:R) +
        _ARTIFICIAL_M * sum(a[i] for i in 1:n))

    return model, λ_vars, a_vars, cov_cons, fleet_con
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
    extract_duals(cov_cons, fleet_con) → CGDuals

Extract dual variables from the LP relaxation after solving.
`pi[i]` is the coverage dual (unrestricted in sign).
`mu` is the fleet dual (≤ 0 at optimality for a minimization ≤ constraint).
"""
function extract_duals(
    cov_cons  :: Vector{<:JuMP.ConstraintRef},
    fleet_con :: JuMP.ConstraintRef
) :: CGDuals
    pi = [JuMP.dual(c) for c in cov_cons]
    mu = JuMP.dual(fleet_con)
    return CGDuals(pi, mu)
end

"""
    add_column!(model, pool, route, cov_cons, fleet_con, λ_vars)

Hot-add a column to the existing model without rebuilding.
Gurobi preserves the simplex basis enabling warm-start dual simplex re-solves.
"""
function add_column!(
    model     :: JuMP.Model,
    pool      :: ColumnPool,
    route     :: FeasibleRoute,
    cov_cons  :: Vector{<:JuMP.ConstraintRef},
    fleet_con :: JuMP.ConstraintRef,
    λ_vars    :: Vector{JuMP.VariableRef}
) :: JuMP.VariableRef
    push!(pool.routes, route)

    new_λ = @variable(model, lower_bound = 0.0, upper_bound = 1.0)
    push!(λ_vars, new_λ)

    JuMP.set_objective_coefficient(model, new_λ, route.cost)

    for i in route.request_ids
        JuMP.set_normalized_coefficient(cov_cons[i], new_λ, 1.0)
    end
    JuMP.set_normalized_coefficient(fleet_con, new_λ, 1.0)

    return new_λ
end
