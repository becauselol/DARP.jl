"""
    scripts/generate_job_list.jl

Generates benchmark instances and writes a job list for single-instance SLURM arrays.

Usage:
    julia --project=. scripts/generate_job_list.jl <inst_dir> <jobs_file> [large]

Arguments:
    inst_dir   — directory to write instance .txt files into
    jobs_file  — output path for the job list (one line per task)
    large      — pass "large" as third arg to use the large-instance presets

Job list format (tab-separated, one row per (solver, instance)):
    solver\\tinstance_path

Solver eligibility:
    DemandCG, NoCapCG  — all instances
    DemandIP, NoCapIP  — only instances with n ≤ DARP_IP_MAX_N (default 20)
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using Printf

inst_dir   = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "experiments", "split_demand_single", "instances")
jobs_file  = length(ARGS) >= 2 ? ARGS[2] : joinpath(@__DIR__, "..", "experiments", "split_demand_single", "jobs.txt")
use_large  = length(ARGS) >= 3 && ARGS[3] == "large"

IP_MAX_N   = parse(Int, get(ENV, "DARP_IP_MAX_N", "20"))

if use_large
    include(joinpath(@__DIR__, "generate_large_instances.jl"))
    paths = generate_large_suite(outdir=inst_dir)
else
    include(joinpath(@__DIR__, "generate_instances.jl"))
    paths = generate_suite(outdir=inst_dir)
end

println("Generated $(length(paths)) instances → $inst_dir")

cg_solvers = ["DemandCG", "NoCapCG"]
ip_solvers = ["DemandIP", "NoCapIP"]

mkpath(dirname(jobs_file))
open(jobs_file, "w") do io
    for solver in cg_solvers, path in sort(paths)
        println(io, "$solver\t$path")
    end
    # IP solvers only for small instances
    if !use_large
        for solver in ip_solvers, path in sort(paths)
            # Peek at the first line to get n without loading DARP
            first_line = readline(path)
            n = parse(Int, split(first_line)[2])
            n > IP_MAX_N && continue
            println(io, "$solver\t$path")
        end
    end
end

n_jobs = countlines(jobs_file)
println("Job list written: $jobs_file ($n_jobs tasks)")
println()
println("Submit with:")
println("  sbatch --array=1-$n_jobs scripts/sbatch_single_instance.sh $jobs_file <base_outdir>")
