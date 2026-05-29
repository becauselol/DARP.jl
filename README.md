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
    solver_interface.jl       # AbstractDARPSolver — extend this to add new methods
    ip_solver.jl              # CordeauIPSolver (arc-flow MIP, JuMP + HiGHS)
    split_delivery_ip.jl      # SplitDeliveryNoCapIPSolver (split delivery, no cap)
    split_demand_ip.jl        # SplitDemandIPSolver (split demand, Q enforced)
    column_generation/        # ColumnGenerationSolver (CG + MIP, unit demand)
    split_delivery_cg/        # SplitDeliveryNoCapCGSolver (CG, split delivery)
    split_demand_cg/          # SplitDemandCGSolver (CG, split demand, Q enforced)
  benchmarking/               # run_benchmark, run_benchmark_suite, write_benchmark_csv
scripts/
  generate_instances.jl # Parameterised random instance generator (Cordeau format)
  run_benchmark.jl      # End-to-end benchmark runner → CSV output
data/                   # place Cordeau benchmark .txt files here (see data/README.md)
test/
  fixtures/             # committed fixture instances (small.txt, a2-16.txt, …)
  fixtures/generated/   # auto-generated instances (gitignored; recreated by script)
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

# Multiple solvers over the same directory
results = run_benchmark_suite([solver1, solver2], "data/")
```

`BenchmarkResult` records: `instance_name`, `solver_name`, `status`, `objective_value`
(IP upper bound), `lp_bound` (LP relaxation lower bound), `solve_time_sec`, `cpu_time_sec`,
`is_feasible`, `n_requests`, `n_vehicles`, `n_cg_iters`.

The iter-log CSV (written by `write_iter_log_csv`) captures per-iteration LP objective and
columns-added for CG solvers, enabling convergence analysis.

---

## Benchmarking

### Solvers

| Solver | Formulation | Notes |
|--------|-------------|-------|
| `CordeauIPSolver` | Arc-flow MIP (HiGHS) | Cordeau (2006) formulation; unit demands |
| `SplitDeliveryNoCapIPSolver` | Arc-flow MIP, split delivery | No vehicle capacity constraint |
| `SplitDeliveryNoCapCGSolver` | Column generation + MIP | No capacity; SPPRC pricing |
| `SplitDemandIPSolver` | Arc-flow MIP (HiGHS) | Capacity enforced; `f[i,k]` demand allocation variables |
| `SplitDemandCGSolver` | Column generation + MIP (Gurobi) | Capacity enforced; α-enumeration SPPRC pricing |
| `ColumnGenerationSolver` | Column generation + MIP | Unit demand CG |

### Generated dataset overview

`scripts/generate_instances.jl` produces instances in three demand tiers across seven size classes.
Run `julia --project=. scripts/generate_instances.jl` to recreate them (output goes to
`test/fixtures/generated/`, which is gitignored).

**Demand tiers**

| Tier | D_max | Q | Splitting behaviour |
|------|-------|---|---------------------|
| `unit`  | 1 | 6 | All D_i = 1; no splitting possible |
| `multi` | 3 | 6 | D_i ∈ {1,2,3} ≤ Q; splitting possible but never required |
| `split` | 5 | 3 | D_i ∈ {1…5}; requests with D_i ∈ {4,5} **must** be split across vehicles |

**Instance sizes** (each generated with seeds 42, 123, 999 → 3 files per row)

| Name prefix | n | K | Q | D_max | Grid | T | L |
|-------------|---|---|---|-------|------|---|---|
| `n4_k2_q3_unit/multi` / `n4_k3_q3_split`     |   4 |  2–3  | 3/6 | 1/3/5 | 10×10 |  60 |  30 |
| `n8_k3_q6_unit/multi` / `n8_k5_q3_split`     |   8 |  3–5  | 3/6 | 1/3/5 | 10×10 |  60 |  30 |
| `n16_k6_q6_unit/multi` / `n16_k10_q3_split`  |  16 |  6–10 | 3/6 | 1/3/5 | 20×20 | 110 |  55 |
| `n32_k11_q6_unit/multi` / `n32_k18_q3_split` |  32 | 11–18 | 3/6 | 1/3/5 | 20×20 | 110 |  55 |
| `n48_k16_q6_unit/multi` / `n48_k26_q3_split` |  48 | 16–26 | 3/6 | 1/3/5 | 30×30 | 170 |  85 |
| `n64_k22_q6_unit/multi` / `n64_k35_q3_split` |  64 | 22–35 | 3/6 | 1/3/5 | 30×30 | 170 |  85 |
| `n100_k34_q6_unit/multi` / `n100_k55_q3_split` | 100 | 34–55 | 3/6 | 1/3/5 | 40×40 | 230 | 115 |

Total: 21 presets × 3 seeds = **63 instances**.

**Generation conditions**

- **Coordinates**: pickup and dropoff nodes placed independently, uniformly at random in a
  square grid. Depot fixed at (0, 0). Grid side scales with n to keep node density roughly
  constant (~1 node per 4 grid units²).
- **Travel times / distances**: Euclidean distance, unit speed (time = distance).
- **Time windows**: all nodes use `[0, T]` (wide-open). This guarantees feasibility by
  construction and stresses routing/capacity rather than scheduling.
- **T**: ≈ 4 × diagonal of the grid, rounded to the nearest 10.
- **L (max ride time)**: T ÷ 2.
- **Service time**: 2 time units at every stop.
- **Demands**: `unit` — all 1; `multi`/`split` — drawn i.i.d. uniform integer from `[1, D_max]`.

### Running the full benchmark on a server

```bash
# Optional configuration via environment variables
export DARP_TIME_LIMIT=300      # seconds per solve (default 300)
export DARP_IP_MAX_N=16         # skip arc-IP for instances with n > this (default 16)
export DARP_SEEDS=42,123,999    # seeds to generate (default 42,123,999)

julia --project=. scripts/run_benchmark.jl /path/to/results
```

Outputs written to `/path/to/results/`:

| File | Contents |
|------|----------|
| `benchmark_results.csv` | One row per solver/instance: `instance`, `solver`, `status`, `objective` (IP upper bound), `lp_bound` (LP lower bound), `solve_time_sec`, `cpu_time_sec`, `feasible`, `n`, `K`, `n_cg_iters` |
| `cg_iter_log.csv` | One row per CG iteration: `instance`, `solver`, `iter`, `lp_obj`, `cols_added` |
| `instances/` | Generated `.txt` instance files |

The script runs `SplitDemandCGSolver` on every instance and `SplitDemandIPSolver` on
instances with n ≤ `DARP_IP_MAX_N`, allowing direct comparison of LP lower bound, IP upper
bound, CG convergence speed, and CPU time as n scales.

---

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
