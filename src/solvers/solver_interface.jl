"""
    AbstractDARPSolver

Abstract supertype for all DARP solvers. To add a new solver:

    struct MySolver <: AbstractDARPSolver
        # solver-specific parameters
    end

    function DARP.solve(solver::MySolver, instance::DARPInstance; kwargs...) :: DARPSolution
        # implementation
    end

No changes to any existing file are required.
"""
abstract type AbstractDARPSolver end

"""
    solve(solver, instance; kwargs...) :: DARPSolution

Primary dispatch point. Every concrete solver must implement this method.
"""
function solve(solver::AbstractDARPSolver, instance::DARPInstance; kwargs...) :: DARPSolution
    error("solve not implemented for solver type $(typeof(solver))")
end

"""
    solve(solver, filepath; kwargs...) :: DARPSolution

Convenience overload: parse `filepath` as a DARPInstance, then solve.
"""
function solve(solver::AbstractDARPSolver, filepath::String; kwargs...) :: DARPSolution
    return solve(solver, read_instance(filepath); kwargs...)
end
