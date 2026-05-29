@testset "Types" begin

    @testset "Node" begin
        node = Node(1, 2.5, 3.7, 3.0, 10.0, 40.0, 1)
        @test node.id == 1
        @test node.x ≈ 2.5
        @test node.load == 1
        @test Station === Node
    end

    @testset "DepotNode" begin
        depot = DepotNode(0, 0.0, 0.0, 0.0, 0.0, 480.0)
        @test depot.id == 0
        @test depot.tw_end ≈ 480.0
    end

    @testset "Request" begin
        pu = Node(1, 1.0, 2.0, 3.0, 0.0, 100.0,  1)
        dr = Node(2, 4.0, 5.0, 3.0, 0.0, 100.0, -1)
        req = Request(1, pu, dr, 90.0)
        @test req.id == 1
        @test req.pickup.load == 1
        @test req.dropoff.load == -1
        @test req.max_ride_time ≈ 90.0
    end

    @testset "Vehicle" begin
        dep = DepotNode(0, 0.0, 0.0, 0.0, 0.0, 480.0)
        v = Vehicle(1, 6, dep, dep, 480.0)
        @test v.capacity == 6
        @test v.max_route_duration ≈ 480.0
    end

    @testset "Route" begin
        r = Route(1, [1, 17], [5.0, 20.0], [1, 0], [15.0], 12.3, 25.0)
        @test r.vehicle_id == 1
        @test length(r.node_sequence) == 2
        @test r.total_distance ≈ 12.3
    end

    @testset "DARPSolution fields" begin
        # Minimal check that the struct is accessible; content tested in test_solver_ip.jl
        @test fieldnames(DARPSolution) == (
            :instance, :routes, :objective_value, :is_feasible,
            :solver_name, :solve_time_sec, :status,
            :lp_bound, :n_cg_iters, :iter_log)
    end

end
