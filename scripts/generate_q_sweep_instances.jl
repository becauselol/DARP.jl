"""
    scripts/generate_q_sweep_instances.jl

Generates DARP instances for the Q-sweep experiment.

Fixed: D_max = 6 across all presets (demands drawn uniformly from {1..6}).
Varied: Q ∈ {3, 6, 9, 12} — vehicle capacity.

This means:
  Q=3  → demands 4–6 exceed capacity, mandatory splitting for ~50% of requests
  Q=6  → D_max = Q, splitting is possible but never required
  Q=9  → all demands fit comfortably, capacity rarely tight
  Q=12 → ample capacity, splitting essentially never useful

K is sized as ceil(n × D_max / Q × 1.2) to guarantee the seed routes can cover
all demand (with slack). K is not a binding constraint in the CG/IP formulations.

File naming: n<N>_q<Q>_d6_<seed>.txt
"""

using Random, Printf

function generate_q_sweep_instance(;
    n            :: Int,
    K            :: Int,
    Q            :: Int,
    D_max        :: Int     = 6,
    T            :: Union{Int,Nothing} = nothing,
    L            :: Union{Int,Nothing} = nothing,
    service_time :: Int     = 2,
    seed         :: Int     = 42,
    grid_size    :: Int     = 10
) :: String
    rng   = MersenneTwister(seed)
    T_val = isnothing(T) ? ceil(Int, 4 * grid_size * sqrt(2) / 10) * 10 : T
    L_val = isnothing(L) ? T_val ÷ 2 : L

    xs_p    = rand(rng, Float64, n) .* grid_size
    ys_p    = rand(rng, Float64, n) .* grid_size
    xs_d    = rand(rng, Float64, n) .* grid_size
    ys_d    = rand(rng, Float64, n) .* grid_size
    demands = rand(rng, 1:D_max, n)

    io = IOBuffer()
    println(io, "$K  $n  $T_val  $Q  $L_val")
    @printf(io, "0   0.000   0.000  0   0  %d  0\n", T_val)
    for i in 1:n
        @printf(io, "%d   %.3f   %.3f  %d   0  %d  %d\n",
                i, xs_p[i], ys_p[i], service_time, T_val, demands[i])
    end
    for i in 1:n
        @printf(io, "%d   %.3f   %.3f  %d   0  %d  %d\n",
                n + i, xs_d[i], ys_d[i], service_time, T_val, -demands[i])
    end
    @printf(io, "%d   0.000   0.000  0   0  %d  0\n", 2*n + 1, T_val)
    return String(take!(io))
end

# K = ceil(n × D_max / Q × 1.2), floored at n
_qsweep_K(n, Q; D_max=6) = max(n, ceil(Int, n * D_max / Q * 1.2))

# Presets: all combinations of (n, Q), D_max=6 fixed
const Q_SWEEP_N      = [8, 16, 32, 64]
const Q_SWEEP_Q      = [3, 6, 9, 12]
const Q_SWEEP_DMAX   = 6
const Q_SWEEP_GRIDS  = Dict(8 => 10, 16 => 20, 32 => 20, 64 => 30)
const Q_SWEEP_SEEDS  = [42, 123, 999]

"""
    generate_q_sweep_suite(; outdir, seeds) → Vector{String}

Generate all Q-sweep instances and write them to `outdir`.
Returns sorted list of written file paths.
"""
function generate_q_sweep_suite(;
    outdir :: String      = joinpath(@__DIR__, "..", "experiments", "cg_q_sweep", "instances"),
    seeds  :: Vector{Int} = Q_SWEEP_SEEDS
) :: Vector{String}
    mkpath(outdir)
    paths = String[]
    for n in Q_SWEEP_N, Q in Q_SWEEP_Q, seed in seeds
        K         = _qsweep_K(n, Q)
        grid_size = Q_SWEEP_GRIDS[n]
        content   = generate_q_sweep_instance(;
            n=n, K=K, Q=Q, D_max=Q_SWEEP_DMAX, grid_size=grid_size, seed=seed
        )
        fname = "n$(n)_q$(Q)_d6_$(seed).txt"
        path  = joinpath(outdir, fname)
        write(path, content)
        push!(paths, path)
    end
    sort!(paths)
    return paths
end

if abspath(PROGRAM_FILE) == @__FILE__
    paths = generate_q_sweep_suite()
    println("Generated $(length(paths)) Q-sweep instances")
    for p in paths; println("  $(basename(p))"); end
end
