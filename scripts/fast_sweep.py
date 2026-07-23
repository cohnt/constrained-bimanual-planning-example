import sys
import os
import time
import numpy as np

repo_root = os.path.dirname(os.path.abspath(__file__))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)
cpp_py_path = os.path.join(repo_root, "cpp_parameterization", "python")
if cpp_py_path not in sys.path:
    sys.path.insert(0, cpp_py_path)

from pydrake.all import (
    StartMeshcat,
    CollisionCheckerParams,
    RobotDiagramBuilder,
    MeshcatVisualizerParams,
    Role,
    MeshcatVisualizer,
    Parser,
    LoadModelDirectives,
    ProcessModelDirectives,
    SceneGraphCollisionChecker,
    IrisNp2Options,
    SnoptSolver,
    IpoptSolver,
    MathematicalProgram,
    HPolyhedron,
    IrisNp2,
    Hyperellipsoid,
    GcsTrajectoryOptimization,
    Point,
    GraphOfConvexSetsOptions,
    PiecewisePolynomial,
    CompositeTrajectory,
    BsplineBasis,
    BsplineTrajectory,
    KinematicTrajectoryOptimization,
    MinimumDistanceLowerBoundConstraint,
    SolverOptions,
    CommonSolverOption,
)

from iiwa_ik import (
    MakeParameterization,
    IiwaBimanualReachableConstraint,
    IiwaBimanualJointLimitConstraint,
    FullFeasibilityConstraint,
    IiwaBimanualPathCost,
)

from src.iiwa_analytic_ik import iiwa_limits_lower, iiwa_limits_upper
import src.common as common
import src.rrt as rrt
import src.shortcut as shortcut

def build_trajopt_problem():
    print("Building Trajectory Optimization Problem (one-time setup)...")
    meshcat = StartMeshcat()
    directives_file = os.path.join(common.RepoDir(), "models/old_shelves.dmd.yaml")
    grasp_distance = 0.6
    shoulder_up = True
    elbow_up = True
    wrist_up = False

    q_tilde_bottom = np.array([-0.6430910102907225, 1.9156121024586796, -1.7968254667817805, 1.2945447141185198, -0.023834531305537934, -0.876966810663043, -1.7041643160834519, 1.45])
    q_tilde_middle = np.array([-0.5997312520566763, 1.489780849654964, -1.4739679827359913, 1.2905366081785483, -0.04421061906813227, -0.8793712572715165, -1.1603461715511334, 1.45])
    q_tilde_top = np.array([-0.1994994216078726, 0.9140739951190965, -2.236618320862171, 0.5238879195899456, 0.7998441913611017, -1.3575398006936048, -1.0153092816310436, 2.41])

    seeds = [
        q_tilde_bottom,
        np.array([-0.7341522021700233, 1.9192492722970935, -1.849050540687353, 1.4690188979347225, -0.022913995470214974, -0.7839567180379224, -1.735834076048031, 1.45]),
        np.array([-0.816394667473979, 1.9228828117510568, -1.9042766014076622, 1.6254903325102958, -0.020884458583263387, -0.6994788210824544, -1.773950224396859, 1.45]),
        np.array([-0.9076736984240236, 1.7999568628541147, -1.8278258357789336, 1.8976493299850326, -0.032028511314404574, -0.5492230012012871, -1.624933169711267, 1.45]),
        np.array([-0.90780384835653, 1.5443282072400564, -1.480882097408486, 1.9741581801564516, -0.07059018895327443, -0.5065618808846135, -1.1610690777465094, 1.45]),
        np.array([-0.877792385473089, 1.283945692440691, -1.1673903163525974, 1.7986279782674526, -0.08798686286997325, -0.605914842625335, -0.7496023024205761, 1.45]),
        np.array([-0.7363360141869535, 1.0835790623705088, -1.102219288049605, 1.3727471630916555, -0.07210415656873102, -0.8362237374759414, -0.6008766712030682, 1.45]),
        np.array([-0.7093225760311644, 0.8650840325295542, -1.4794100092984808, 1.2099934253928784, 0.44173726212402287, -0.9673197772349095, -0.9450827150678346, 2.0]),
        np.array([-0.5237049267440886, 0.7086764066165658, -1.9872212610757156, 1.045742737284787, 0.8594286107005795, -1.171705603794283, -1.1435157398017397, 2.41]),
        np.array([-0.37540312953312194, 0.7958305227244739, -2.112215906760149, 0.8433434932970723, 0.8316630398644385, -1.2430896040746857, -1.1077155278001196, 2.41]),
        q_tilde_top,
        np.array([-0.7686406052800139, 1.504938625148829, -1.4584578152597332, 1.655937158932382, -0.055175677810583384, -0.6834840454669682, -1.1418310479792013, 1.45]),
        q_tilde_middle,
    ]

    params = CollisionCheckerParams()
    builder = RobotDiagramBuilder(time_step=0.0)

    meshcat_visual_params = MeshcatVisualizerParams()
    meshcat_visual_params.delete_on_initialization_event = False
    meshcat_visual_params.role = Role.kIllustration
    meshcat_visual_params.prefix = "visual"
    meshcat_visual = MeshcatVisualizer.AddToBuilder(builder.builder(), builder.scene_graph(), meshcat, meshcat_visual_params)

    plant = builder.plant()
    parser = Parser(plant)
    package_xml_path = os.path.join(common.RepoDir(), "package.xml")
    parser.package_map().AddPackageXml(package_xml_path)
    directives = LoadModelDirectives(directives_file)
    ProcessModelDirectives(directives, parser)

    params.robot_model_instances = [plant.GetModelInstanceByName("iiwa_left"), plant.GetModelInstanceByName("iiwa_right")]
    plant.Finalize()

    builder.builder().ExportInput(plant.get_actuation_input_port(), "actuation")
    builder.builder().ExportOutput(plant.get_state_output_port(), "state")

    diagram = builder.Build()
    params.model = diagram
    params.edge_step_size = 0.01
    checker = SceneGraphCollisionChecker(params)

    iris_np2_options = IrisNp2Options()
    iris_np2_options.parameterization = MakeParameterization(shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_np2_options.sampled_iris_options.random_seed = 2
    iris_np2_options.sampled_iris_options.max_iterations = 1
    iris_np2_options.sampled_iris_options.relax_margin = True
    iris_np2_options.sampled_iris_options.epsilon = 0.01
    iris_np2_options.sampled_iris_options.delta = 0.01
    iris_np2_options.add_hyperplane_if_solve_fails = True

    iris_prog = MathematicalProgram()
    q_tilde_vars = iris_prog.NewContinuousVariables(8, "q_tilde")
    iris_np2_options.sampled_iris_options.prog_with_additional_constraints = iris_prog
    reachability_constraint = IiwaBimanualReachableConstraint(shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_prog.AddConstraint(reachability_constraint, q_tilde_vars)
    subordinate_arm_joint_limit_constraint = IiwaBimanualJointLimitConstraint(iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_prog.AddConstraint(subordinate_arm_joint_limit_constraint, q_tilde_vars)

    domain_lower = np.hstack((iiwa_limits_lower, [0.0]))
    domain_upper = np.hstack((iiwa_limits_upper, [2.0 * np.pi]))
    domain = HPolyhedron.MakeBox(domain_lower, domain_upper)

    regions = [IrisNp2(checker, Hyperellipsoid.MakeHypersphere(1e-2, seed), domain, iris_np2_options).ReduceInequalities() for seed in seeds]

    start = q_tilde_bottom.copy()
    goal = q_tilde_top.copy()

    def RandomConfig(): return np.random.uniform(low=domain_lower, high=domain_upper)
    def ValidityChecker(q_tilde):
        if np.any(reachability_constraint.Eval(q_tilde) > np.ones(4)) or np.any(reachability_constraint.Eval(q_tilde) < -np.ones(4)): return False
        q_full = iris_np2_options.parameterization.get_parameterization_double()(q_tilde)
        q_sub = q_full[7:]
        if np.any(q_sub < iiwa_limits_lower) or np.any(q_sub > iiwa_limits_upper): return False
        if np.any(np.abs(q_sub[[1, 3, 5]]) < 1e-2): return False
        return checker.CheckConfigCollisionFree(q_full)

    rrt_options = rrt.RRTOptions(step_size=2e-1, check_size=1e-2, max_vertices=1e4, max_iters=1e6, goal_sample_frequency=0.01, always_swap=False)
    rrt_planner = rrt.BiRRT(RandomConfig, ValidityChecker)
    np.random.seed(0)
    path = rrt_planner.plan(start, goal, rrt_options)
    np.random.seed(0)
    shortcut_path = shortcut.shortcut(path.copy(), ValidityChecker, num_tries=1e2, check_size=rrt_options.check_size)

    spline_order = 4
    control_points_matrix = np.array([point for point in shortcut_path]).T
    basis = BsplineBasis(spline_order, control_points_matrix.shape[1], initial_parameter_value=0.0, final_parameter_value=float(len(shortcut_path)-1))
    initial_traj = BsplineTrajectory(basis, control_points_matrix)

    def create_trajopt():
        trajopt = KinematicTrajectoryOptimization(initial_traj)
        trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.start_time()), initial_traj.value(initial_traj.start_time()), 0)
        trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.end_time()), initial_traj.value(initial_traj.end_time()), 1)

        minimum_distance_lower_bound_constraint = MinimumDistanceLowerBoundConstraint(
            plant, 0.001, plant.GetMyContextFromRoot(diagram.CreateDefaultContext()), None, 0.049
        )
        full_feasibility_constraint = FullFeasibilityConstraint(
            iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up, grasp_distance, minimum_distance_lower_bound_constraint
        )

        for s in np.linspace(0, 1, 100):
            trajopt.AddPathPositionConstraint(full_feasibility_constraint, s)

        parameterized_path_energy = IiwaBimanualPathCost(trajopt.num_positions(), trajopt.num_control_points(), shoulder_up, elbow_up, wrist_up, grasp_distance, True)
        trajopt.prog().AddCost(parameterized_path_energy, trajopt.control_points().flatten())
        return trajopt

    return create_trajopt

def eval_solution(prog, result):
    vars = prog.decision_variables()
    x_sol = result.GetSolution(vars)
    costs = prog.GetAllCosts()
    cost_val = 0.0
    for c_binding in costs:
        indices = prog.FindDecisionVariableIndices(c_binding.variables())
        cost_val += np.sum(c_binding.evaluator().Eval(x_sol[indices]))

    constraints = prog.GetAllConstraints()
    feas = True
    max_v = 0.0
    for b in constraints:
        indices = prog.FindDecisionVariableIndices(b.variables())
        val = b.evaluator().Eval(x_sol[indices])
        lb, ub = b.evaluator().lower_bound(), b.evaluator().upper_bound()
        if np.any(val < lb - 1e-4) or np.any(val > ub + 1e-4):
            feas = False
            m1 = np.max(lb - val) if np.any(val < lb) else 0.0
            m2 = np.max(val - ub) if np.any(val > ub) else 0.0
            max_v = max(max_v, m1, m2)
    return cost_val, feas, max_v

def run_fast_sweeps():
    create_trajopt = build_trajopt_problem()

    results = []

    # 1. SNOPT Standard Sweeps
    print("\n" + "="*80)
    print("SWEEP 1: Standard SNOPT Optimality Tolerance")
    print("="*80)
    for opt_tol in [1e-1, 1e-2, 1e-3, 1e-4, 1e-6]:
        trajopt = create_trajopt()
        prog = trajopt.prog()
        solver = SnoptSolver()
        opts = SolverOptions()
        opts.SetOption(CommonSolverOption.kPrintToConsole, False)
        opts.SetOption(SnoptSolver().solver_id(), "Major print level", 0)
        opts.SetOption(SnoptSolver().solver_id(), "Time Limit", 60)
        opts.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", opt_tol)

        t0 = time.time()
        res = solver.Solve(prog, None, opts)
        t_solve = time.time() - t0
        cost, feas, max_v = eval_solution(prog, res)

        results.append({
            "category": "SNOPT Standard",
            "params": f"opt_tol={opt_tol}",
            "success": res.is_success(),
            "feasible": feas,
            "cost": cost,
            "solve_time": t_solve
        })
        print(f"SNOPT opt_tol={opt_tol:<6} | Succ={res.is_success()!s:<5} | Feas={feas!s:<5} | Cost={cost:<7.4f} | Time={t_solve:<6.2f}s")

    # 2. IPOPT Standard Sweeps
    print("\n" + "="*80)
    print("SWEEP 2: Standard IPOPT Configurations")
    print("="*80)
    ipopt_configs = [
        ("special_penalty_1e2", {"print_level": 0, "max_wall_time": 60, "acceptable_tol": 1e-2, "acceptable_iter": 5, "line_search_method": "penalty"}),
        ("special_penalty_1e3", {"print_level": 0, "max_wall_time": 60, "acceptable_tol": 1e-3, "acceptable_iter": 5, "line_search_method": "penalty"}),
        ("default_strict", {"print_level": 0, "max_wall_time": 60}),
    ]

    for name, opt_dict in ipopt_configs:
        trajopt = create_trajopt()
        prog = trajopt.prog()
        solver = IpoptSolver()
        opts = SolverOptions()
        opts.SetOption(CommonSolverOption.kPrintToConsole, False)
        for k, v in opt_dict.items():
            opts.SetOption(IpoptSolver().solver_id(), k, v)

        t0 = time.time()
        res = solver.Solve(prog, None, opts)
        t_solve = time.time() - t0
        cost, feas, max_v = eval_solution(prog, res)

        results.append({
            "category": "IPOPT Standard",
            "params": name,
            "success": res.is_success(),
            "feasible": feas,
            "cost": cost,
            "solve_time": t_solve
        })
        print(f"IPOPT {name:<20} | Succ={res.is_success()!s:<5} | Feas={feas!s:<5} | Cost={cost:<7.4f} | Time={t_solve:<6.2f}s")

    # 3. SNOPT Trust Region Sweeps
    print("\n" + "="*80)
    print("SWEEP 3: SNOPT Trust Region (TR) Hyperparameters")
    print("="*80)
    tr_configs = [
        (0.25, 1e-1, 5),
        (0.25, 1e-2, 5),
        (0.25, 1e-3, 5),
        (0.50, 1e-2, 5),
        (0.50, 1e-3, 5),
    ]

    for delta_0, opt_tol, max_it in tr_configs:
        trajopt = create_trajopt()
        prog = trajopt.prog()
        solver = SnoptSolver()
        opts = SolverOptions()
        opts.SetOption(CommonSolverOption.kPrintToConsole, False)
        opts.SetOption(SnoptSolver().solver_id(), "Major print level", 0)
        opts.SetOption(SnoptSolver().solver_id(), "Time Limit", 60)
        opts.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", opt_tol)

        from main_cpp import solve_trust_region_loop
        res, t_solve = solve_trust_region_loop(prog, solver, opts, max_iters=max_it, delta_0=delta_0)
        cost, feas, max_v = eval_solution(prog, res)

        params_str = f"delta={delta_0}, tol={opt_tol}, it={max_it}"
        results.append({
            "category": "SNOPT Trust Region",
            "params": params_str,
            "success": res.is_success(),
            "feasible": feas,
            "cost": cost,
            "solve_time": t_solve
        })
        print(f"TR {params_str:<30} | Succ={res.is_success()!s:<5} | Feas={feas!s:<5} | Cost={cost:<7.4f} | Time={t_solve:<6.2f}s", flush=True)

    summary_file = "sweep_summary.txt"
    with open(summary_file, "w") as f:
        f.write("="*98 + "\n")
        f.write("BEST HYPERPARAMETERS PERFORMANCE COMPARISON SUMMARY\n")
        f.write("="*98 + "\n")
        f.write(f"{'Category':<22} | {'Parameters':<32} | {'Success':<8} | {'Feasible':<8} | {'Cost':<8} | {'Time (s)':<10}\n")
        f.write("-" * 98 + "\n")
        for r in results:
            line = f"{r['category']:<22} | {r['params']:<32} | {str(r['success']):<8} | {str(r['feasible']):<8} | {r['cost']:<8.4f} | {r['solve_time']:<10.2f}\n"
            f.write(line)

    print(f"\nSummary saved to {summary_file}", flush=True)

if __name__ == "__main__":
    run_fast_sweeps()
