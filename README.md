# DARP.jl

A Julia package for solving the **Dial-a-Ride Problem (DARP)** using multiple methods.

Given a set of transportation requests (each specifying a pickup location, dropoff location, and time windows), a fleet of vehicles departing from depots, the DARP asks for vehicle routes that serve all requests while minimizing total travel distance subject to capacity, time window, ride time, and route duration constraints.

## Installation

```julia
using Pkg
Pkg.activate(".")
Pkg.instantiate()
```

## Quick Start

```julia
using DARP

# Parse a Cordeau-format instance
instance = read_instance("data/a2-16.txt")

# Solve with the integer programming formulation
solver   = CordeauIPSolver(time_limit_sec=300.0, verbose=true)
solution = solve(solver, instance)

# Inspect and save
println(solution_summary(solution))
write_solution(solution, "a2-16_solution.txt")
```

## Package Structure

```
src/
  DARP.jl               # module entry, exports
  types/                # Node, Request, Vehicle, DARPInstance, Route, DARPSolution
  io/                   # read_instance, write_solution
  solvers/
    solver_interface.jl # AbstractDARPSolver — extend this to add new methods
    ip_solver.jl        # CordeauIPSolver (JuMP + HiGHS)
  benchmarking/         # run_benchmark, run_benchmark_suite, write_benchmark_csv
data/                   # place Cordeau benchmark .txt files here (see data/README.md)
examples/               # runnable scripts
test/                   # test suite with one committed fixture instance
```

## Core API

### Input types

| Type | Description |
|------|-------------|
| `Node` / `Station` | A pickup or dropoff stop (id, coords, time window, load) |
| `DepotNode` | Vehicle origin/destination depot |
| `Request` | Pairs one pickup with one dropoff; carries max ride time |
| `Vehicle` | Capacity, origin depot, destination depot, max route duration |
| `DARPInstance` | All problem data + precomputed travel matrices |

### Solving

```julia
solver   = CordeauIPSolver(time_limit_sec=3600.0, mip_gap=1e-4, verbose=false)
solution = solve(solver, instance)     # from DARPInstance
solution = solve(solver, "file.txt")   # convenience: parse then solve
```

### Output types

| Type | Description |
|------|-------------|
| `Route` | Node sequence, service times, loads, ride times, distance for one vehicle |
| `DARPSolution` | All routes + objective value + status (`:optimal`, `:feasible`, …) |

### Benchmarking

```julia
results = run_benchmark(solver, "data/"; verbose=true)
print_benchmark_table(results)
write_benchmark_csv(results, "results.csv")

# Multiple solvers
results = run_benchmark_suite([solver1, solver2], "data/")
```

## Adding a New Solver

Subtype `AbstractDARPSolver` and implement `DARP.solve`:

```julia
struct MyHeuristic <: AbstractDARPSolver
    max_iter::Int
end

function DARP.solve(s::MyHeuristic, inst::DARPInstance; kwargs...) :: DARPSolution
    # ... your implementation
end
```

No changes to any existing file required.

## Instance Format

See `data/README.md` for the Cordeau benchmark format and download instructions.

## References

- Cordeau, J.-F. & Laporte, G. (2003). A tabu search heuristic for the static multi-vehicle dial-a-ride problem. *Transportation Research Part B*, 37(6), 579–594.
- Cordeau, J.-F. (2006). A branch-and-cut algorithm for the dial-a-ride problem. *Operations Research*, 54(3), 573–586.
