@testset "CG Solver" begin

    inst   = read_instance(SMALL)
    solver = ColumnGenerationSolver(time_limit_sec=120.0, verbose=false)
    sol    = solve(solver, inst)

    @testset "CGDuals type" begin
        d = CGDuals(fill(1.0, inst.n), -0.5)
        @test d.pi isa Vector{Float64}
        @test length(d.pi) == inst.n
        @test d.mu isa Float64
    end

    @testset "solve_pricing standalone — zero duals" begin
        d = CGDuals(zeros(inst.n), 0.0)
        routes = solve_pricing(inst, d; max_routes=5)
        # Zero duals → rc = travel cost ≥ 0 → all routes non-improving
        @test all(r.reduced_cost >= -1e-6 for r in routes)
    end

    @testset "solve_pricing standalone — large duals" begin
        d = CGDuals(fill(50.0, inst.n), -1.0)
        routes = solve_pricing(inst, d; max_routes=5)
        @test !isempty(routes)
        @test all(r.reduced_cost < -1e-6 for r in routes)
        @test issorted(r.reduced_cost for r in routes)
    end

    @testset "FeasibleRoute structure" begin
        d = CGDuals(fill(50.0, inst.n), -1.0)
        routes = solve_pricing(inst, d; max_routes=5)
        for fr in routes
            @test !isempty(fr.cordeau_seq)
            @test length(fr.julia_seq)  == length(fr.cordeau_seq)
            @test !isempty(fr.request_ids)
            @test issorted(fr.request_ids)
            @test fr.cost >= 0.0
            @test fr.total_distance >= 0.0
            @test fr.total_duration >= 0.0
            @test length(fr.service_times) == length(fr.cordeau_seq)
            @test length(fr.loads)         == length(fr.cordeau_seq)
        end
    end

    @testset "Solution structure" begin
        @test sol isa DARPSolution
        @test sol.instance === inst
        @test length(sol.routes) == inst.K
        @test sol.solver_name == "ColumnGenerationSolver"
        @test sol.solve_time_sec > 0
    end

    @testset "Status and feasibility" begin
        @test sol.status in (:optimal, :feasible)
        @test sol.is_feasible
        @test isfinite(sol.objective_value)
        @test sol.objective_value > 0
    end

    @testset "Request coverage" begin
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

            @test all(0 <= ld <= Q for ld in route.loads)
            @test all(bt >= 0 for bt in route.service_times)
            @test issorted(route.service_times)

            for (pos, node_id) in enumerate(route.node_sequence)
                node = inst.nodes[node_id]
                @test node.tw_start <= route.service_times[pos] <= node.tw_end
            end

            @test all(rt >= -1e-6 for rt in route.ride_times)
            @test all(rt <= L + 1e-6 for rt in route.ride_times)

            @test route.total_duration <= T + 1e-6
        end
    end

    @testset "Precedence: every pickup before its dropoff" begin
        n = inst.n
        for route in sol.routes
            isempty(route.node_sequence) && continue
            seq = route.node_sequence
            for i in 1:n
                pu_pos = findfirst(==(i),     seq)
                dr_pos = findfirst(==(n + i), seq)
                (pu_pos === nothing || dr_pos === nothing) && continue
                @test pu_pos < dr_pos
            end
        end
    end

    @testset "Objective agrees with IP solver" begin
        sol_ip = solve(CordeauIPSolver(time_limit_sec=120.0), inst)
        # CG LP lower bound ≤ IP objective; both should agree on small instance
        @test sol.objective_value <= sol_ip.objective_value + 1e-3
    end

    @testset "Detour factor" begin
        sol_d = solve(ColumnGenerationSolver(time_limit_sec=120.0, detour_factor=1.5), inst)
        @test sol_d.is_feasible
        n_d = inst.n
        for route in sol_d.routes
            isempty(route.node_sequence) && continue
            pickup_idx = 0
            for (pos, node_id) in enumerate(route.node_sequence)
                1 <= node_id <= n_d || continue
                pickup_idx += 1
                direct_t = inst.travel_time[node_id + 1, n_d + node_id + 1]
                rt = route.ride_times[pickup_idx]
                @test rt <= 1.5 * direct_t + 1e-6
            end
        end
    end

    @testset "LP-only mode" begin
        sol_lp = solve(ColumnGenerationSolver(time_limit_sec=120.0, solve_ip=false), inst)
        @test sol_lp.status == :lp_relaxation
        sol_ip = solve(CordeauIPSolver(time_limit_sec=120.0), inst)
        # LP relaxation objective ≤ IP optimal (LP is a relaxation)
        @test sol_lp.objective_value <= sol_ip.objective_value + 1e-3
    end

    @testset "Constructor validation" begin
        @test_throws ArgumentError ColumnGenerationSolver(detour_factor=0.5)
        @test_throws ArgumentError ColumnGenerationSolver(time_limit_sec=-1.0)
        @test_throws ArgumentError ColumnGenerationSolver(max_cg_iters=0)
    end

    @testset "Convenience filepath overload" begin
        sol2 = solve(ColumnGenerationSolver(time_limit_sec=120.0), SMALL)
        @test sol2 isa DARPSolution
    end

    @testset "Benchmark compatibility" begin
        results = run_benchmark(ColumnGenerationSolver(time_limit_sec=120.0), FIXTURE_DIR;
                                pattern=r"small\.txt$", verbose=false)
        @test length(results) == 1
        @test results[1] isa BenchmarkResult
        @test results[1].solver_name == "ColumnGenerationSolver"
        @test results[1].is_feasible
    end

end
