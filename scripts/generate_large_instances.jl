using Random, Printf

"""
    generate_instance(; n, K, Q, D_max, T, L, service_time, seed, grid_size) → String

Generate a random DARP instance in Cordeau format. Identical to the small-instance
generator but called from generate_large_instances.jl.
"""
function generate_instance_large(;
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

function write_instance_large(path::String, content::String)
    mkpath(dirname(path))
    write(path, content)
end

# ── Large instance presets ────────────────────────────────────────────────────
#
# Fleet sizing (matching small-instance rationale):
#   unit/multi : K = ceil(n / 3)        Q=6   D_max=1 or 3
#   split      : K = ceil(n * 0.55)     Q=3   D_max=5  (forces mandatory splits)
#
# grid_size scales as ~sqrt(n/100) * 40 to keep spatial density constant.

const LARGE_PRESETS = [
    (name="n150_k50_q6_unit",    n=150,  K=50,  Q=6, D_max=1, grid_size=45),
    (name="n150_k50_q6_multi",   n=150,  K=50,  Q=6, D_max=3, grid_size=45),
    (name="n150_k83_q3_split",   n=150,  K=83,  Q=3, D_max=5, grid_size=45),
    (name="n200_k67_q6_unit",    n=200,  K=67,  Q=6, D_max=1, grid_size=50),
    (name="n200_k67_q6_multi",   n=200,  K=67,  Q=6, D_max=3, grid_size=50),
    (name="n200_k110_q3_split",  n=200,  K=110, Q=3, D_max=5, grid_size=50),
    (name="n300_k100_q6_unit",   n=300,  K=100, Q=6, D_max=1, grid_size=62),
    (name="n300_k100_q6_multi",  n=300,  K=100, Q=6, D_max=3, grid_size=62),
    (name="n300_k165_q3_split",  n=300,  K=165, Q=3, D_max=5, grid_size=62),
    (name="n400_k134_q6_unit",   n=400,  K=134, Q=6, D_max=1, grid_size=72),
    (name="n400_k134_q6_multi",  n=400,  K=134, Q=6, D_max=3, grid_size=72),
    (name="n400_k220_q3_split",  n=400,  K=220, Q=3, D_max=5, grid_size=72),
    (name="n500_k167_q6_unit",   n=500,  K=167, Q=6, D_max=1, grid_size=80),
    (name="n500_k167_q6_multi",  n=500,  K=167, Q=6, D_max=3, grid_size=80),
    (name="n500_k275_q3_split",  n=500,  K=275, Q=3, D_max=5, grid_size=80),
    (name="n750_k250_q6_unit",   n=750,  K=250, Q=6, D_max=1, grid_size=98),
    (name="n750_k250_q6_multi",  n=750,  K=250, Q=6, D_max=3, grid_size=98),
    (name="n750_k413_q3_split",  n=750,  K=413, Q=3, D_max=5, grid_size=98),
    (name="n1000_k334_q6_unit",  n=1000, K=334, Q=6, D_max=1, grid_size=113),
    (name="n1000_k334_q6_multi", n=1000, K=334, Q=6, D_max=3, grid_size=113),
    (name="n1000_k550_q3_split", n=1000, K=550, Q=3, D_max=5, grid_size=113),
]

const LARGE_DEFAULT_SEEDS = [42, 123, 999]

"""
    generate_large_suite(; outdir, seeds) → Vector{String}
"""
function generate_large_suite(;
    outdir :: String      = joinpath(@__DIR__, "..", "experiments", "split_demand_large", "instances"),
    seeds  :: Vector{Int} = LARGE_DEFAULT_SEEDS
) :: Vector{String}
    paths = String[]
    for preset in LARGE_PRESETS, seed in seeds
        content = generate_instance_large(;
            n         = preset.n,
            K         = preset.K,
            Q         = preset.Q,
            D_max     = preset.D_max,
            grid_size = preset.grid_size,
            seed      = seed
        )
        fname = "$(preset.name)_$(seed).txt"
        path  = joinpath(outdir, fname)
        write_instance_large(path, content)
        push!(paths, path)
    end
    sort!(paths)
    return paths
end
