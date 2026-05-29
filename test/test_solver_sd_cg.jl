@testset "Split-Delivery CG Solver" begin

    inst   = read_instance(SMALL)
    solver = SplitDeliveryNoCapCGSolver(time_limit_sec=120.0, verbose=false)
    sol    = solve(solver, inst)

    @testset "SplitDeliveryNoCapDuals type" begin
        d = SplitDeliveryNoCapDuals(fill(1.0, inst.n))
        @test d.pi isa Vector{Float64}
        @test length(d.pi) == inst.n
    end

    @testset "solve_nocap_pricing — zero duals" begin
        d = SplitDeliveryNoCapDuals(zeros(inst.n))
        routes = solve_nocap_pricing(inst, d; max_routes=5)
        # Zero duals → rc = travel cost ≥ 0 → no improving columns
        @test all(r.reduced_cost >= -1e-6 for r in routes)
    end

    @testset "solve_nocap_pricing — large duals" begin
        d = SplitDeliveryNoCapDuals(fill(50.0, inst.n))
        routes = solve_nocap_pricing(inst, d; max_routes=5)
        @test !isempty(routes)
        @test all(r.reduced_cost < -1e-6 for r in routes)
        @test issorted(r.reduced_cost for r in routes)
    end

    @testset "SplitDeliveryNoCapRoute structure" begin
        d = SplitDeliveryNoCapDuals(fill(50.0, inst.n))
        routes = solve_nocap_pricing(inst, d; max_routes=5)
        for fr in routes
            @test !isempty(fr.cordeau_seq)
            @test length(fr.julia_seq)     == length(fr.cordeau_seq)
            @test !isempty(fr.request_ids)
            @test issorted(fr.request_ids)
            @test fr.cost >= 0.0
            @test fr.total_distance >= 0.0
            @test fr.total_duration >= 0.0
            @test length(fr.service_times) == length(fr.cordeau_seq)
            @test length(fr.loads)         == length(fr.cordeau_seq)
            # Uncapped capacity: alpha_i = D_i exactly for every visited request
            for i in fr.request_ids
                @test haskey(fr.alpha, i)
                @test fr.alpha[i] == inst.nodes[i].load
            end
            # Reduced cost: c_r − Σ D_i · π_i
            expected_rc = fr.total_distance -
                sum(fr.alpha[i] * 50.0 for i in fr.request_ids)
            @test abs(fr.reduced_cost - expected_rc) < 1e-6
        end
    end

    @testset "Solution structure" begin
        @test sol isa DARPSolution
        @test sol.instance === inst
        @test !isempty(sol.routes)
        @test sol.solver_name == "SplitDeliveryNoCapCGSolver"
        @test sol.solve_time_sec > 0
    end

    @testset "Status and feasibility" begin
        @test sol.status in (:optimal, :feasible)
        @test sol.is_feasible
        @test isfinite(sol.objective_value)
        @test sol.objective_value > 0
    end

    @testset "Demand coverage" begin
        # For each request i, the selected routes must collectively cover ≥ D_i units.
        # With unit demand (D_i=1) this reduces to: each request appears in exactly one route.
        n = inst.n
        D = [inst.nodes[i].load for i in 1:n]
        covered = zeros(Int, n)
        for route in sol.routes
            isempty(route.node_sequence) && continue
            for node_id in route.node_sequence
                1 <= node_id <= n && (covered[node_id] += 1)
            end
        end
        for i in 1:n
            @test covered[i] >= 1   # at least served once (covering)
        end
    end

    @testset "Feasibility checks per route" begin
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

    @testset "Objective vs IP solver (unit demand)" begin
        # For unit demand, split-delivery ≡ regular covering → obj ≤ arc IP obj
        sol_ip = solve(CordeauIPSolver(time_limit_sec=120.0), inst)
        @test sol.objective_value <= sol_ip.objective_value + 1e-3
    end

    @testset "LP-only mode" begin
        sol_lp = solve(SplitDeliveryNoCapCGSolver(time_limit_sec=120.0, solve_ip=false), inst)
        @test sol_lp.status == :lp_relaxation
        sol_ip = solve(CordeauIPSolver(time_limit_sec=120.0), inst)
        @test sol_lp.objective_value <= sol_ip.objective_value + 1e-3
    end

    @testset "Constructor validation" begin
        @test_throws ArgumentError SplitDeliveryNoCapCGSolver(detour_factor=0.5)
        @test_throws ArgumentError SplitDeliveryNoCapCGSolver(time_limit_sec=-1.0)
        @test_throws ArgumentError SplitDeliveryNoCapCGSolver(max_cg_iters=0)
    end

    @testset "Convenience filepath overload" begin
        sol2 = solve(SplitDeliveryNoCapCGSolver(time_limit_sec=120.0), SMALL)
        @test sol2 isa DARPSolution
    end

    @testset "Benchmark compatibility" begin
        results = run_benchmark(SplitDeliveryNoCapCGSolver(time_limit_sec=120.0), FIXTURE_DIR;
                                pattern=r"small\.txt$", verbose=false)
        @test length(results) == 1
        @test results[1] isa BenchmarkResult
        @test results[1].solver_name == "SplitDeliveryNoCapCGSolver"
        @test results[1].is_feasible
    end

end
