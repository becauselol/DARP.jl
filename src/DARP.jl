module DARP

using JuMP

# ── Types ─────────────────────────────────────────────────────────────────────
export AbstractNode, Node, Station, DepotNode
export Request
export Vehicle
export DARPInstance
export Route, DARPSolution

# ── I/O ───────────────────────────────────────────────────────────────────────
export read_instance, write_solution, solution_summary

# ── Solver interface ───────────────────────────────────────────────────────────
export AbstractDARPSolver, solve

# ── Solvers ───────────────────────────────────────────────────────────────────
export CordeauIPSolver
export ColumnGenerationSolver
export FeasibleRoute, CGDuals, ColumnPool
export solve_pricing
export SplitDeliveryNoCapCGSolver
export SplitDeliveryNoCapRoute, SplitDeliveryNoCapDuals, SplitDeliveryNoCapPool
export solve_nocap_pricing
export SplitDeliveryNoCapIPSolver
export SplitDemandIPSolver
export SplitDemandCGSolver
export SplitDemandRoute, SplitDemandDuals, SplitDemandPool
export solve_split_demand_pricing

# ── Benchmarking ──────────────────────────────────────────────────────────────
export BenchmarkResult, run_benchmark, run_benchmark_suite
export write_benchmark_csv, print_benchmark_table

include("types/node_types.jl")
include("types/request_types.jl")
include("types/vehicle_types.jl")
include("types/instance_types.jl")
include("types/solution_types.jl")

include("io/read_instance.jl")
include("io/write_solution.jl")

include("solvers/solver_interface.jl")
include("solvers/ip_solver.jl")

include("solvers/column_generation/types.jl")
include("solvers/column_generation/pricing.jl")
include("solvers/column_generation/master_lp.jl")
include("solvers/column_generation/master_ip.jl")
include("solvers/column_generation/solution_builder.jl")
include("solvers/column_generation/cg_solver.jl")

include("solvers/split_delivery_cg/types.jl")
include("solvers/split_delivery_cg/pricing.jl")
include("solvers/split_delivery_cg/master_lp.jl")
include("solvers/split_delivery_cg/master_ip.jl")
include("solvers/split_delivery_cg/solution_builder.jl")
include("solvers/split_delivery_cg/sd_cg_solver.jl")
include("solvers/split_delivery_ip.jl")
include("solvers/split_demand_ip.jl")

include("solvers/split_demand_cg/types.jl")
include("solvers/split_demand_cg/pricing.jl")
include("solvers/split_demand_cg/master_lp.jl")
include("solvers/split_demand_cg/master_ip.jl")
include("solvers/split_demand_cg/solution_builder.jl")
include("solvers/split_demand_cg/sd_demand_cg_solver.jl")

include("benchmarking/benchmark.jl")

end # module DARP
