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

Filtering (via environment variables):
    DARP_IP_MAX_N   — max n for IP solvers (default: 20)
    DARP_VARIANTS   — comma-separated allowed demand variants: unit,multi,split (default: "multi,split")
    DARP_SIZES      — comma-separated allowed n values (default: "4,8,16,32,64,100"; use "all" to keep every size)

Solver eligibility:
    DemandCG, NoCapCG  — all instances passing variant/size filters
    DemandIP, NoCapIP  — instances with n ≤ DARP_IP_MAX_N passing variant/size filters
"""

import Pkg
Pkg.activate(joinpath(@__DIR__, ".."))

using Printf

inst_dir   = length(ARGS) >= 1 ? ARGS[1] : joinpath(@__DIR__, "..", "experiments", "cg_vs_ip", "instances")
jobs_file  = length(ARGS) >= 2 ? ARGS[2] : joinpath(@__DIR__, "..", "experiments", "cg_vs_ip", "jobs.txt")
use_large  = length(ARGS) >= 3 && ARGS[3] == "large"

IP_MAX_N      = parse(Int, get(ENV, "DARP_IP_MAX_N", "20"))
VARIANTS_STR  = get(ENV, "DARP_VARIANTS", "multi,split")
SIZES_STR     = get(ENV, "DARP_SIZES", "4,8,16,32,64,100")

allowed_variants = Set(strip.(split(VARIANTS_STR, ",")))
allowed_sizes    = SIZES_STR == "all" ? nothing : Set(parse(Int, s) for s in split(SIZES_STR, ","))

# Extract the demand variant (unit/multi/split) from a filename like n32_k11_q6_multi_42.txt
function instance_variant(path)
    b = basename(path)
    occursin("_unit_", b)  && return "unit"
    occursin("_multi_", b) && return "multi"
    occursin("_split_", b) && return "split"
    return "unknown"
end

# Extract n from the first line of an instance file (Cordeau format: "K  n  T  Q  L")
function instance_n(path)
    parse(Int, split(readline(path))[2])
end

function keep(path)
    instance_variant(path) in allowed_variants || return false
    allowed_sizes === nothing && return true
    instance_n(path) in allowed_sizes
end

if use_large
    include(joinpath(@__DIR__, "generate_large_instances.jl"))
    paths = generate_large_suite(outdir=inst_dir)
else
    include(joinpath(@__DIR__, "generate_instances.jl"))
    paths = generate_suite(outdir=inst_dir)
end

println("Generated $(length(paths)) instances → $inst_dir")
println("Filtering: variants=$(VARIANTS_STR)  sizes=$(SIZES_STR)")

filtered = filter(keep, sort(paths))
println("Kept $(length(filtered)) instances after filtering")

cg_solvers = ["DemandCG", "NoCapCG"]
ip_solvers = ["DemandIP", "NoCapIP"]

mkpath(dirname(jobs_file))
open(jobs_file, "w") do io
    for solver in cg_solvers, path in filtered
        println(io, "$solver\t$path")
    end
    if !use_large
        for solver in ip_solvers, path in filtered
            instance_n(path) > IP_MAX_N && continue
            println(io, "$solver\t$path")
        end
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
