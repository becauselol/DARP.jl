using Random, Printf

"""
    generate_instance(; n, K, Q, D_max, T, L, service_time, seed, grid_size) → String

Generate a random DARP instance in Cordeau format and return it as a string.

- Depot is fixed at (0, 0); pickup/dropoff coordinates are uniform in [0, grid_size]².
- Wide time windows [0, T] on every node guarantee feasibility by construction.
- T defaults to 4 × grid_size × √2 (≈4 diagonal trips), rounded up to nearest 10.
- L defaults to T ÷ 2.
- D_max = 1 → unit demands; D_max > 1 → demands drawn uniformly from 1:D_max.
"""
function generate_instance(;
    n            :: Int,
    K            :: Int,
    Q            :: Int,
    D_max        :: Int     = 1,
    T            :: Union{Int,Nothing} = nothing,
    L            :: Union{Int,Nothing} = nothing,
    service_time :: Int     = 2,
    seed         :: Int     = 42,
    grid_size    :: Int     = 10
) :: String
    rng   = MersenneTwister(seed)
    # T: enough for ~4 full diagonal trips; L: allow rides up to half the route budget
    T_val = isnothing(T) ? ceil(Int, 4 * grid_size * sqrt(2) / 10) * 10 : T
    L_val = isnothing(L) ? T_val ÷ 2 : L

    xs_p = rand(rng, Float64, n) .* grid_size
    ys_p = rand(rng, Float64, n) .* grid_size
    xs_d = rand(rng, Float64, n) .* grid_size
    ys_d = rand(rng, Float64, n) .* grid_size
    demands = D_max == 1 ? ones(Int, n) : rand(rng, 1:D_max, n)

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

"""
    write_instance(path, content)

Write `content` to `path`, creating parent directories as needed.
"""
function write_instance(path::String, content::String)
    mkpath(dirname(path))
    write(path, content)
end

# ── Preset table ──────────────────────────────────────────────────────────────

# ── Preset groups ─────────────────────────────────────────────────────────────
#
# "unit"   : D_max=1 (all demands fit in one vehicle; no splitting possible)
# "multi"  : D_max=3, Q=6 (multi-unit demands; splitting possible but never required)
# "split"  : D_max=5, Q=3 (D_i ∈ {4,5} > Q forces mandatory demand splitting)
#
# Fleet sizing: K ≈ ceil(n × avg_D / (Q × avg_stops_per_vehicle))
#   unit/multi: avg_D ≈ 2, ~3 stops/veh → K ≈ ceil(n/3)
#   split:      avg_D ≈ 3, Q=3, ~2 stops/veh → K ≈ ceil(n/2); +slack for split routes
#
const PRESETS = [
    # ── Small (arc-IP tractable) ─────────────────────────────────────────────
    (name="n4_k2_q3_unit",    n=4,   K=2,  Q=3, D_max=1, grid_size=10),
    (name="n4_k2_q3_multi",   n=4,   K=2,  Q=3, D_max=3, grid_size=10),
    (name="n4_k3_q3_split",   n=4,   K=3,  Q=3, D_max=5, grid_size=10),
    (name="n8_k3_q6_unit",    n=8,   K=3,  Q=6, D_max=1, grid_size=10),
    (name="n8_k3_q6_multi",   n=8,   K=3,  Q=6, D_max=3, grid_size=10),
    (name="n8_k5_q3_split",   n=8,   K=5,  Q=3, D_max=5, grid_size=10),
    # ── Medium ───────────────────────────────────────────────────────────────
    (name="n16_k6_q6_unit",   n=16,  K=6,  Q=6, D_max=1, grid_size=20),
    (name="n16_k6_q6_multi",  n=16,  K=6,  Q=6, D_max=3, grid_size=20),
    (name="n16_k10_q3_split", n=16,  K=10, Q=3, D_max=5, grid_size=20),
    (name="n32_k11_q6_unit",  n=32,  K=11, Q=6, D_max=1, grid_size=20),
    (name="n32_k11_q6_multi", n=32,  K=11, Q=6, D_max=3, grid_size=20),
    (name="n32_k18_q3_split", n=32,  K=18, Q=3, D_max=5, grid_size=20),
    # ── Large (CG territory) ─────────────────────────────────────────────────
    (name="n48_k16_q6_unit",  n=48,  K=16, Q=6, D_max=1, grid_size=30),
    (name="n48_k16_q6_multi", n=48,  K=16, Q=6, D_max=3, grid_size=30),
    (name="n48_k26_q3_split", n=48,  K=26, Q=3, D_max=5, grid_size=30),
    (name="n64_k22_q6_unit",  n=64,  K=22, Q=6, D_max=1, grid_size=30),
    (name="n64_k22_q6_multi", n=64,  K=22, Q=6, D_max=3, grid_size=30),
    (name="n64_k35_q3_split", n=64,  K=35, Q=3, D_max=5, grid_size=30),
    (name="n100_k34_q6_unit", n=100, K=34, Q=6, D_max=1, grid_size=40),
    (name="n100_k34_q6_multi",n=100, K=34, Q=6, D_max=3, grid_size=40),
    (name="n100_k55_q3_split",n=100, K=55, Q=3, D_max=5, grid_size=40),
]

const DEFAULT_SEEDS = [42, 123, 999]

"""
    generate_suite(; outdir, seeds) → Vector{String}

Generate all preset instances for each seed and write them to `outdir`.
Returns a sorted list of written file paths.
"""
function generate_suite(;
    outdir :: String        = joinpath(@__DIR__, "..", "test", "fixtures", "generated"),
    seeds  :: Vector{Int}   = DEFAULT_SEEDS
) :: Vector{String}
    paths = String[]
    for preset in PRESETS, seed in seeds
        content = generate_instance(;
            n         = preset.n,
            K         = preset.K,
            Q         = preset.Q,
            D_max     = preset.D_max,
            grid_size = preset.grid_size,
            seed      = seed
        )
        fname = "$(preset.name)_$(seed).txt"
        path  = joinpath(outdir, fname)
        write_instance(path, content)
        push!(paths, path)
    end
    sort!(paths)
    return paths
end

# ── Run when invoked directly ─────────────────────────────────────────────────

if abspath(PROGRAM_FILE) == @__FILE__
    paths = generate_suite()
    println("Generated $(length(paths)) instances → $(dirname(paths[1]))")
    for p in paths
        println("  $(basename(p))")
    end
end
