@testset "IP Solver" begin

    # Use the small 4-request fixture for fast, deterministic CI.
    # The a2-16 fixture is reserved for examples/ and manual benchmarking.
    inst   = read_instance(SMALL)
    solver = CordeauIPSolver(time_limit_sec=60.0, verbose=false)
    sol    = solve(solver, inst)

    @testset "Solution structure" begin
        @test sol isa DARPSolution
        @test sol.instance === inst
        @test length(sol.routes) == inst.K
        @test sol.solver_name == "CordeauIPSolver"
        @test sol.solve_time_sec > 0
    end

    @testset "Status and feasibility" begin
        @test sol.status in (:optimal, :feasible)
        @test sol.is_feasible
        @test isfinite(sol.objective_value)
        @test sol.objective_value > 0
    end

    @testset "Route coverage" begin
        served = Int[]
        for route in sol.routes
            for node_id in route.node_sequence
                1 <= node_id <= inst.n && push!(served, node_id)
            end
        end
        @test sort(served) == collect(1:inst.n)
    end

    @testset "Feasibility checks per route" begin
        n = inst.n
        Q = inst.Q
        T = inst.T
        L = inst.L

        for route in sol.routes
            isempty(route.node_sequence) && continue

            # Capacity never exceeded
            @test all(0 <= ld <= Q for ld in route.loads)

            # Service times non-negative and increasing
            @test all(bt >= 0 for bt in route.service_times)
            @test issorted(route.service_times)

            # Time windows respected
            for (pos, node_id) in enumerate(route.node_sequence)
                node = inst.nodes[node_id]
                @test node.tw_start <= route.service_times[pos] <= node.tw_end
            end

            # Ride times are non-negative (pickup before dropoff) and within limit
            @test all(rt >= -1e-6 for rt in route.ride_times)
            @test all(rt <= L + 1e-6 for rt in route.ride_times)

            # Route duration within limit
            @test route.total_duration <= T + 1e-6
        end
    end

    @testset "Precedence: every pickup before its dropoff" begin
        n = inst.n
        for route in sol.routes
            isempty(route.node_sequence) && continue
            seq = route.node_sequence
            for i in 1:n
                pu_pos = findfirst(==(i),     seq)   # Cordeau ID 1..n
                dr_pos = findfirst(==(n + i), seq)   # Cordeau ID n+1..2n
                (pu_pos === nothing || dr_pos === nothing) && continue
                @test pu_pos < dr_pos
            end
        end
    end

    @testset "Relative detour cap" begin
        # With detour_factor=1.5, ride time ≤ 1.5 × direct travel time per request.
        sol_d = solve(CordeauIPSolver(time_limit_sec=60.0, detour_factor=1.5), inst)
        @test sol_d.is_feasible
        n_d = inst.n
        for route in sol_d.routes
            isempty(route.node_sequence) && continue
            for (idx, pi_c) in enumerate(route.node_sequence)
                1 <= pi_c <= n_d || continue   # only pickups
                i = pi_c                        # request index
                pi_j = i + 1                   # Julia pickup index
                di_j = n_d + i + 1             # Julia dropoff index
                direct_t = inst.travel_time[pi_j, di_j]
                ride_t = route.ride_times[findfirst(==(i), [c for c in route.node_sequence if 1 <= c <= n_d])]
                @test ride_t <= 1.5 * direct_t + 1e-6
            end
        end
    end

    @testset "detour_factor validation" begin
        @test_throws ArgumentError CordeauIPSolver(detour_factor=0.5)
    end

    @testset "Convenience filepath overload" begin
        sol2 = solve(CordeauIPSolver(time_limit_sec=60.0), SMALL)
        @test sol2 isa DARPSolution
    end

end
