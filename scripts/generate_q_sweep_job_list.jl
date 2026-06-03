"""
    scripts/generate_q_sweep_job_list.jl

Generates instances and writes a DemandCG-only job list for the Q-sweep experiment.

Usage:
    julia --project=. scripts/generate_q_sweep_job_list.jl <inst_dir> <jobs_file>

Filtering (via environment variables):
    DARP_Q_VALUES  — comma-separated Q values to include (default: "3,6,9,12")
    DARP_N_VALUES  — comma-separated n values to include (default: "8,16,32,64")
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

inst_dir  = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "experiments", "cg_q_sweep", "instances")
jobs_file = length(ARGS) >= 2 ? ARGS[2] : joinpath(@__DIR__, "..", "experiments", "cg_q_sweep", "jobs.txt")

Q_STR = get(ENV, "DARP_Q_VALUES", "3,6,9,12")
N_STR = get(ENV, "DARP_N_VALUES", "8,16,32,64")

allowed_Q = Set(parse(Int, s) for s in split(Q_STR, ","))
allowed_N = Set(parse(Int, s) for s in split(N_STR, ","))

include(joinpath(@__DIR__, "generate_q_sweep_instances.jl"))
paths = generate_q_sweep_suite(outdir=inst_dir)

println("Generated $(length(paths)) instances → $inst_dir")
println("Filtering: Q=$(Q_STR)  n=$(N_STR)")

# Extract Q and n from filename: n8_q3_d6_42.txt
function qsweep_q(path)
    m = match(r"_q(\d+)_", basename(path))
    m === nothing ? -1 : parse(Int, m.captures[1])
end
function qsweep_n(path)
    m = match(r"^n(\d+)_", basename(path))
    m === nothing ? -1 : parse(Int, m.captures[1])
end

keep(path) = qsweep_q(path) in allowed_Q && qsweep_n(path) in allowed_N
filtered = filter(keep, paths)
println("Kept $(length(filtered)) instances after filtering")

mkpath(dirname(jobs_file))
open(jobs_file, "w") do io
    for path in filtered
        println(io, "DemandCG\t$path")
    end
end

n_jobs = countlines(jobs_file)
println("Job list written: $jobs_file ($n_jobs tasks)")
println()
println("Submit with:")
println("  sbatch --array=1-$n_jobs \\")
println("         --output=<exp_dir>/slurm_logs/%A_%a.out \\")
println("         --error=<exp_dir>/slurm_logs/%A_%a.err \\")
println("         scripts/sbatch_single_instance.sh $jobs_file <base_outdir>")
