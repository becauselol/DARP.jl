using Test
using DARP

const FIXTURE_DIR = joinpath(@__DIR__, "fixtures")
const A2_16 = joinpath(FIXTURE_DIR, "a2-16.txt")
const SMALL  = joinpath(FIXTURE_DIR, "small.txt")   # 4 requests, 2 vehicles — fast CI solve

@testset "DARP.jl" begin
    include("test_types.jl")
    include("test_io.jl")
    include("test_solver_ip.jl")
    include("test_solver_cg.jl")
    include("test_solver_sd_cg.jl")
end
