@testset "I/O" begin

    inst = read_instance(A2_16)

    @testset "Header parsing" begin
        @test inst.n == 16
        @test inst.K == 2
        @test inst.Q == 6
        @test inst.T ≈ 480.0
        @test inst.L ≈ 90.0
        @test inst.name == "a2-16"
    end

    @testset "Node counts" begin
        @test length(inst.nodes)    == 2 * inst.n      # 32 nodes
        @test length(inst.requests) == inst.n           # 16 requests
        @test length(inst.vehicles) == inst.K           # 2 vehicles
    end

    @testset "Travel matrix dimensions" begin
        N = 2*inst.n + 2
        @test size(inst.travel_time)     == (N, N)
        @test size(inst.travel_distance) == (N, N)
        # Diagonal must be zero
        @test all(inst.travel_distance[i,i] == 0.0 for i in 1:N)
        # Off-diagonal must be non-negative (co-located depots can have distance 0)
        @test all(inst.travel_distance[i,j] >= 0.0 for i in 1:N, j in 1:N if i != j)
        # Non-depot nodes must be at distinct locations from each other
        @test all(inst.travel_distance[i,j] > 0.0
                  for i in 2:N-1, j in 2:N-1 if i != j)
    end

    @testset "Load sign convention" begin
        # Pickups are nodes[1..n], dropoffs are nodes[n+1..2n]
        @test all(inst.nodes[i].load > 0 for i in 1:inst.n)
        @test all(inst.nodes[inst.n + i].load < 0 for i in 1:inst.n)
    end

    @testset "Time windows valid" begin
        for node in inst.nodes
            @test node.tw_start <= node.tw_end
        end
        @test inst.depot_origin.tw_start <= inst.depot_origin.tw_end
        @test inst.depot_destination.tw_start <= inst.depot_destination.tw_end
    end

    @testset "Request pairing" begin
        for req in inst.requests
            @test req.pickup.load  == -req.dropoff.load
            @test req.pickup.load  > 0
            @test req.dropoff.load < 0
        end
    end

    @testset "write_solution smoke test" begin
        # Build a dummy solution and write it without error
        empty_routes = [Route(k, Int[], Float64[], Int[], Float64[], 0.0, 0.0)
                        for k in 1:inst.K]
        sol = DARPSolution(inst, empty_routes, Inf, false, "test", 0.0, :infeasible)
        tmpfile = tempname() * ".txt"
        @test_nowarn write_solution(sol, tmpfile)
        @test isfile(tmpfile)
        rm(tmpfile)
    end

end
