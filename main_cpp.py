import sys
import os
import time
import numpy as np
from tqdm import tqdm

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
    IrisZoOptions,
    SnoptSolver,
    IpoptSolver,
    MathematicalProgram,
    HPolyhedron,
    IrisNp2,
    Hyperellipsoid,
    GcsTrajectoryOptimization,
    Point,
    GraphOfConvexSetsOptions,
    FunctionHandleTrajectory,
    InitializeAutoDiff,
    ExtractGradient,
    Toppra,
    PathParameterizedTrajectory,
    PiecewisePolynomial,
    CompositeTrajectory,
    CalcGridPointsOptions,
    BsplineBasis,
    BsplineTrajectory,
    KinematicTrajectoryOptimization,
    MinimumDistanceLowerBoundConstraint,
    SolverOptions,
    CommonSolverOption,
    DiagramBuilder,
    TrajectorySource,
    InverseDynamicsController,
    Demultiplexer,
    Simulator,
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

def run_pipeline(strip_special_optimizer_settings=False, use_solver="snopt"):
    print("=" * 80)
    print(f"RUNNING MAIN_CPP PIPELINE (strip_special_optimizer_settings={strip_special_optimizer_settings}, use_solver={use_solver})")
    print("=" * 80)

    # 1. Meshcat & Scene Setup
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
    meshcat_visual = MeshcatVisualizer.AddToBuilder(
        builder.builder(), builder.scene_graph(), meshcat, meshcat_visual_params)

    meshcat_collision_params = MeshcatVisualizerParams()
    meshcat_collision_params.delete_on_initialization_event = False
    meshcat_collision_params.role = Role.kProximity
    meshcat_collision_params.prefix = "collision"
    meshcat_collision_params.visible_by_default = False
    meshcat_collision = MeshcatVisualizer.AddToBuilder(
        builder.builder(), builder.scene_graph(), meshcat, meshcat_collision_params)

    plant = builder.plant()
    parser = Parser(plant)
    package_xml_path = os.path.join(common.RepoDir(), "package.xml")
    parser.package_map().AddPackageXml(package_xml_path)
    directives = LoadModelDirectives(directives_file)
    ProcessModelDirectives(directives, parser)

    params.robot_model_instances = [
        plant.GetModelInstanceByName("iiwa_left"),
        plant.GetModelInstanceByName("iiwa_right")
    ]

    plant.Finalize()

    builder.builder().ExportInput(plant.get_actuation_input_port(), "actuation")
    builder.builder().ExportOutput(plant.get_state_output_port(), "state")

    diagram = builder.Build()

    params.model = diagram
    params.edge_step_size = 0.01
    checker = SceneGraphCollisionChecker(params)

    # 2. Iris Parameterization & Region Generation
    iris_np2_options = IrisNp2Options()
    iris_np2_options.parameterization = MakeParameterization(shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_np2_options.sampled_iris_options.random_seed = 2
    iris_np2_options.sampled_iris_options.verbose = False
    iris_np2_options.sampled_iris_options.max_iterations = 1
    iris_np2_options.sampled_iris_options.relax_margin = True
    iris_np2_options.sampled_iris_options.epsilon = 0.01
    iris_np2_options.sampled_iris_options.delta = 0.01
    iris_np2_options.sampled_iris_options.sample_particles_in_parallel = False
    iris_np2_options.add_hyperplane_if_solve_fails = True

    iris_prog = MathematicalProgram()
    q_tilde_vars = iris_prog.NewContinuousVariables(8, "q_tilde")
    iris_np2_options.sampled_iris_options.prog_with_additional_constraints = iris_prog

    reachability_constraint = IiwaBimanualReachableConstraint(shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_prog.AddConstraint(reachability_constraint, q_tilde_vars)

    subordinate_arm_joint_limit_constraint = IiwaBimanualJointLimitConstraint(
        iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up, grasp_distance
    )
    iris_prog.AddConstraint(subordinate_arm_joint_limit_constraint, q_tilde_vars)

    domain_lower = np.hstack((iiwa_limits_lower, [0.0]))
    domain_upper = np.hstack((iiwa_limits_upper, [2.0 * np.pi]))
    domain = HPolyhedron.MakeBox(domain_lower, domain_upper)

    def grow_region(seed):
        return IrisNp2(checker, Hyperellipsoid.MakeHypersphere(1e-2, seed), domain, iris_np2_options)

    print("Growing Iris Regions...")
    start_time = time.time()
    regions = []
    for i in range(len(seeds)):
        region = grow_region(seeds[i])
        regions.append(region.ReduceInequalities())
    print(f"Region generation complete in {time.time() - start_time:.2f}s. Avg faces: {np.mean([len(r.b()) for r in regions]):.1f}")

    # 3. GCS Trajectory Optimization
    start = q_tilde_bottom.copy()
    goal = q_tilde_top.copy()

    gcs = GcsTrajectoryOptimization(8)
    gcs.AddPathContinuityConstraints(1)

    main_graph = gcs.AddRegions(regions, order=2, h_min=0.1, h_max=100, name="")
    start_graph = gcs.AddRegions([Point(start)], 0)
    goal_graph = gcs.AddRegions([Point(goal)], 0)
    gcs.AddEdges(start_graph, main_graph)
    gcs.AddEdges(main_graph, goal_graph)

    gcs.AddPathLengthCost()
    gcs.AddTimeCost()
    gcs.AddVelocityBounds(-np.ones(8), np.ones(8))

    gcs_options = GraphOfConvexSetsOptions()
    gcs_options.max_rounding_trials = 100
    gcs_options.max_rounded_paths = 100
    gcs_options.convex_relaxation = True

    gcs_traj, gcs_result = gcs.SolvePath(start_graph, goal_graph, gcs_options)
    print(f"GCS Path Solve Success: {gcs_result.is_success()}")

    # 4. RRT & Shortcutting
    def RandomConfig():
        return np.random.uniform(low=domain_lower, high=domain_upper)

    def ValidityChecker(q_tilde):
        if np.any(reachability_constraint.Eval(q_tilde) > np.ones(4)) or np.any(reachability_constraint.Eval(q_tilde) < -np.ones(4)):
            return False
        q_full = iris_np2_options.parameterization.get_parameterization_double()(q_tilde)
        q_sub = q_full[7:]
        if np.any(q_sub < iiwa_limits_lower) or np.any(q_sub > iiwa_limits_upper):
            return False
        if np.any(np.abs(q_sub[[1, 3, 5]]) < 1e-2):
            return False
        return checker.CheckConfigCollisionFree(q_full)

    rrt_options = rrt.RRTOptions(
        step_size=2e-1,
        check_size=1e-2,
        max_vertices=1e4,
        max_iters=1e6,
        goal_sample_frequency=0.01,
        always_swap=False
    )
    rrt_planner = rrt.BiRRT(RandomConfig, ValidityChecker)
    np.random.seed(0)
    path = rrt_planner.plan(start, goal, rrt_options)

    np.random.seed(0)
    shortcut_path = shortcut.shortcut(path.copy(), ValidityChecker, num_tries=1e2, check_size=rrt_options.check_size)

    # 5. Kinematic Trajectory Optimization Setup
    spline_order = 4
    t0 = 0.0
    t1 = float(len(shortcut_path) - 1)

    control_points_matrix = np.array([point for point in shortcut_path]).T
    basis = BsplineBasis(spline_order, control_points_matrix.shape[1], initial_parameter_value=t0, final_parameter_value=t1)
    initial_traj = BsplineTrajectory(basis, control_points_matrix)

    trajopt = KinematicTrajectoryOptimization(initial_traj)
    trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.start_time()), initial_traj.value(initial_traj.start_time()), 0)
    trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.end_time()), initial_traj.value(initial_traj.end_time()), 1)

    minimum_distance = 0.001
    influence_distance = 0.05
    minimum_distance_constraint_diagram_context = diagram.CreateDefaultContext()
    minimum_distance_constraint_plant_context = plant.GetMyContextFromRoot(minimum_distance_constraint_diagram_context)
    minimum_distance_lower_bound_constraint = MinimumDistanceLowerBoundConstraint(
        plant, minimum_distance, minimum_distance_constraint_plant_context, None, influence_distance - minimum_distance
    )

    full_feasibility_constraint = FullFeasibilityConstraint(
        iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up, grasp_distance, minimum_distance_lower_bound_constraint
    )

    num_constraints_to_apply = 100
    for s in np.linspace(0, 1, num_constraints_to_apply):
        trajopt.AddPathPositionConstraint(full_feasibility_constraint, s)

    parameterized_path_energy = IiwaBimanualPathCost(
        trajopt.num_positions(), trajopt.num_control_points(), shoulder_up, elbow_up, wrist_up, grasp_distance, True
    )
    trajopt.prog().AddCost(parameterized_path_energy, trajopt.control_points().flatten())

    # Check initial guess satisfaction
    satisfied_init = True
    for binding in trajopt.prog().GetAllConstraints():
        c = binding.evaluator()
        val = c.Eval(trajopt.prog().GetInitialGuess(binding.variables()))
        lb, ub = c.lower_bound(), c.upper_bound()
        ok = np.all(val >= lb - 1e-4) and np.all(val <= ub + 1e-4)
        satisfied_init &= ok

    print("All constraints satisfied at initial guess?", satisfied_init)

    # 6. Configure Solver Options
    trajopt_options = SolverOptions()

    if use_solver == "ipopt":
        trajopt_options.SetOption(CommonSolverOption.kPrintToConsole, True)
        trajopt_options.SetOption(IpoptSolver().solver_id(), "print_level", 5)
        trajopt_options.SetOption(IpoptSolver().solver_id(), "max_wall_time", 60)
        if not strip_special_optimizer_settings:
            trajopt_options.SetOption(IpoptSolver().solver_id(), "acceptable_tol", 1e-2)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "acceptable_iter", 5)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "acceptable_constr_viol_tol", 1e-6)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "bound_relax_factor", 1e-12)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "honor_original_bounds", "yes")
            trajopt_options.SetOption(IpoptSolver().solver_id(), "line_search_method", "penalty")
            trajopt_options.SetOption(IpoptSolver().solver_id(), "watchdog_shortened_iter_trigger", 0)
        else:
            print("NOTE: Special IPOPT optimizer settings REMOVED (logging/timing kept)!")
    elif use_solver == "snopt":
        trajopt_options.SetOption(CommonSolverOption.kPrintToConsole, False)
        trajopt_options.SetOption(CommonSolverOption.kPrintFileName, "snopt.log")
        trajopt_options.SetOption(SnoptSolver().solver_id(), "Major print level", 1)
        trajopt_options.SetOption(SnoptSolver().solver_id(), "Timing level", 3)
        trajopt_options.SetOption(SnoptSolver().solver_id(), "Time Limit", 60)
        if opt_tolerance is not None:
            trajopt_options.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", opt_tolerance)
            print(f"Setting SNOPT Major optimality tolerance to {opt_tolerance}")
        elif not strip_special_optimizer_settings:
            trajopt_options.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", 1e-1)
        else:
            print("NOTE: Special SNOPT optimizer settings REMOVED (logging/timing kept)!")

def solve_trust_region_loop(prog, solver, solver_options, max_iters=20, delta_0=0.25, delta_min=1e-5, delta_max=5.0, opt_tolerance=None):
    """
    Hacky Trust Region loop around Drake MathematicalProgram.
    """
    vars = prog.decision_variables()
    x_k = prog.GetInitialGuess(vars)
    delta_k = delta_0

    costs = prog.GetAllCosts()
    def eval_cost(x):
        total = 0.0
        for c_binding in costs:
            indices = prog.FindDecisionVariableIndices(c_binding.variables())
            total += np.sum(c_binding.evaluator().Eval(x[indices]))
        return total

    def eval_feasibility(x):
        constraints = prog.GetAllConstraints()
        satisfied = True
        max_viol = 0.0
        for b in constraints:
            indices = prog.FindDecisionVariableIndices(b.variables())
            val = b.evaluator().Eval(x[indices])
            lb, ub = b.evaluator().lower_bound(), b.evaluator().upper_bound()
            if np.any(val < lb - 1e-4) or np.any(val > ub + 1e-4):
                satisfied = False
                m1 = np.max(lb - val) if np.any(val < lb) else 0.0
                m2 = np.max(val - ub) if np.any(val > ub) else 0.0
                max_viol = max(max_viol, m1, m2)
        return satisfied, max_viol

    # Add bounding box constraint for trust region
    tr_binding = prog.AddBoundingBoxConstraint(x_k - delta_k, x_k + delta_k, vars)

    best_x = x_k.copy()
    best_cost = eval_cost(x_k)
    print(f"[TR Loop Start] Initial Cost: {best_cost:.4f}, Initial Radius: {delta_k:.4f}")

    total_solve_time = 0.0
    current_tol = opt_tolerance if opt_tolerance is not None else 1e-3
    solver_options.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", current_tol)

    for it in range(max_iters):
        # Update bounds on the trust region constraint
        tr_binding.evaluator().UpdateLowerBound(x_k - delta_k)
        tr_binding.evaluator().UpdateUpperBound(x_k + delta_k)

        t0 = time.time()
        sub_result = solver.Solve(prog, x_k, solver_options)
        t_sub = time.time() - t0
        total_solve_time += t_sub

        x_cand = sub_result.GetSolution(vars)
        cost_cand = eval_cost(x_cand)
        is_feas, max_v = eval_feasibility(x_cand)

        cost_diff = best_cost - cost_cand
        step_norm = np.linalg.norm(x_cand - x_k)

        print(f"TR Iter {it+1:02d} | t={t_sub:.3f}s | Delta={delta_k:.5f} | Tol={current_tol:.1e} | Step={step_norm:.5f} | Cost={cost_cand:.4f} (diff={cost_diff:+.4f}) | Feas={is_feas}")

        if is_feas and cost_diff > 1e-4:
            # Accept step
            x_k = x_cand.copy()
            best_x = x_cand.copy()
            best_cost = cost_cand
            # Expand radius to allow taking larger strides
            delta_k = min(delta_k * 1.5, delta_max)
        else:
            # Step rejected or zero progress: shrink radius
            delta_k = delta_k * 0.5
            print(f"  -> Shrinking radius to Delta = {delta_k:.5f}")

        if delta_k < delta_min or cost_diff <= 1e-4:
            print(f"TR Loop converged/stopped at iteration {it+1}!")
            break

    # Remove bounding box constraint from prog after loop so outside evaluation works on original prog
    prog.RemoveConstraint(tr_binding)
    prog.SetInitialGuess(vars, best_x)

    # Re-solve once at final best_x without trust region box to produce a standard MathematicalProgramResult
    final_result = solver.Solve(prog, best_x, solver_options)
    return final_result, total_solve_time

def run_pipeline(strip_special_optimizer_settings=False, use_solver="snopt", use_trust_region=False, opt_tolerance=None, delta_0=0.25, max_iters=20):
    print("=" * 80)
    print(f"RUNNING MAIN_CPP PIPELINE (strip_special_optimizer_settings={strip_special_optimizer_settings}, use_solver={use_solver}, use_trust_region={use_trust_region}, opt_tolerance={opt_tolerance}, delta_0={delta_0}, max_iters={max_iters})")
    print("=" * 80)

    # 1. Meshcat & Scene Setup
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
    meshcat_visual = MeshcatVisualizer.AddToBuilder(
        builder.builder(), builder.scene_graph(), meshcat, meshcat_visual_params)

    meshcat_collision_params = MeshcatVisualizerParams()
    meshcat_collision_params.delete_on_initialization_event = False
    meshcat_collision_params.role = Role.kProximity
    meshcat_collision_params.prefix = "collision"
    meshcat_collision_params.visible_by_default = False
    meshcat_collision = MeshcatVisualizer.AddToBuilder(
        builder.builder(), builder.scene_graph(), meshcat, meshcat_collision_params)

    plant = builder.plant()
    parser = Parser(plant)
    package_xml_path = os.path.join(common.RepoDir(), "package.xml")
    parser.package_map().AddPackageXml(package_xml_path)
    directives = LoadModelDirectives(directives_file)
    ProcessModelDirectives(directives, parser)

    params.robot_model_instances = [
        plant.GetModelInstanceByName("iiwa_left"),
        plant.GetModelInstanceByName("iiwa_right")
    ]

    plant.Finalize()

    builder.builder().ExportInput(plant.get_actuation_input_port(), "actuation")
    builder.builder().ExportOutput(plant.get_state_output_port(), "state")

    diagram = builder.Build()

    params.model = diagram
    params.edge_step_size = 0.01
    checker = SceneGraphCollisionChecker(params)

    # 2. Iris Parameterization & Region Generation
    iris_np2_options = IrisNp2Options()
    iris_np2_options.parameterization = MakeParameterization(shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_np2_options.sampled_iris_options.random_seed = 2
    iris_np2_options.sampled_iris_options.verbose = False
    iris_np2_options.sampled_iris_options.max_iterations = 1
    iris_np2_options.sampled_iris_options.relax_margin = True
    iris_np2_options.sampled_iris_options.epsilon = 0.01
    iris_np2_options.sampled_iris_options.delta = 0.01
    iris_np2_options.sampled_iris_options.sample_particles_in_parallel = False
    iris_np2_options.add_hyperplane_if_solve_fails = True

    iris_prog = MathematicalProgram()
    q_tilde_vars = iris_prog.NewContinuousVariables(8, "q_tilde")
    iris_np2_options.sampled_iris_options.prog_with_additional_constraints = iris_prog

    reachability_constraint = IiwaBimanualReachableConstraint(shoulder_up, elbow_up, wrist_up, grasp_distance)
    iris_prog.AddConstraint(reachability_constraint, q_tilde_vars)

    subordinate_arm_joint_limit_constraint = IiwaBimanualJointLimitConstraint(
        iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up, grasp_distance
    )
    iris_prog.AddConstraint(subordinate_arm_joint_limit_constraint, q_tilde_vars)

    domain_lower = np.hstack((iiwa_limits_lower, [0.0]))
    domain_upper = np.hstack((iiwa_limits_upper, [2.0 * np.pi]))
    domain = HPolyhedron.MakeBox(domain_lower, domain_upper)

    def grow_region(seed):
        return IrisNp2(checker, Hyperellipsoid.MakeHypersphere(1e-2, seed), domain, iris_np2_options)

    print("Growing Iris Regions...")
    start_time = time.time()
    regions = []
    for i in range(len(seeds)):
        region = grow_region(seeds[i])
        regions.append(region.ReduceInequalities())
    print(f"Region generation complete in {time.time() - start_time:.2f}s. Avg faces: {np.mean([len(r.b()) for r in regions]):.1f}")

    # 3. GCS Trajectory Optimization
    start = q_tilde_bottom.copy()
    goal = q_tilde_top.copy()

    gcs = GcsTrajectoryOptimization(8)
    gcs.AddPathContinuityConstraints(1)

    main_graph = gcs.AddRegions(regions, order=2, h_min=0.1, h_max=100, name="")
    start_graph = gcs.AddRegions([Point(start)], 0)
    goal_graph = gcs.AddRegions([Point(goal)], 0)
    gcs.AddEdges(start_graph, main_graph)
    gcs.AddEdges(main_graph, goal_graph)

    gcs.AddPathLengthCost()
    gcs.AddTimeCost()
    gcs.AddVelocityBounds(-np.ones(8), np.ones(8))

    gcs_options = GraphOfConvexSetsOptions()
    gcs_options.max_rounding_trials = 100
    gcs_options.max_rounded_paths = 100
    gcs_options.convex_relaxation = True

    gcs_traj, gcs_result = gcs.SolvePath(start_graph, goal_graph, gcs_options)
    print(f"GCS Path Solve Success: {gcs_result.is_success()}")

    # 4. RRT & Shortcutting
    def RandomConfig():
        return np.random.uniform(low=domain_lower, high=domain_upper)

    def ValidityChecker(q_tilde):
        if np.any(reachability_constraint.Eval(q_tilde) > np.ones(4)) or np.any(reachability_constraint.Eval(q_tilde) < -np.ones(4)):
            return False
        q_full = iris_np2_options.parameterization.get_parameterization_double()(q_tilde)
        q_sub = q_full[7:]
        if np.any(q_sub < iiwa_limits_lower) or np.any(q_sub > iiwa_limits_upper):
            return False
        if np.any(np.abs(q_sub[[1, 3, 5]]) < 1e-2):
            return False
        return checker.CheckConfigCollisionFree(q_full)

    rrt_options = rrt.RRTOptions(
        step_size=2e-1,
        check_size=1e-2,
        max_vertices=1e4,
        max_iters=1e6,
        goal_sample_frequency=0.01,
        always_swap=False
    )
    rrt_planner = rrt.BiRRT(RandomConfig, ValidityChecker)
    np.random.seed(0)
    path = rrt_planner.plan(start, goal, rrt_options)

    np.random.seed(0)
    shortcut_path = shortcut.shortcut(path.copy(), ValidityChecker, num_tries=1e2, check_size=rrt_options.check_size)

    # 5. Kinematic Trajectory Optimization Setup
    spline_order = 4
    t0 = 0.0
    t1 = float(len(shortcut_path) - 1)

    control_points_matrix = np.array([point for point in shortcut_path]).T
    basis = BsplineBasis(spline_order, control_points_matrix.shape[1], initial_parameter_value=t0, final_parameter_value=t1)
    initial_traj = BsplineTrajectory(basis, control_points_matrix)

    trajopt = KinematicTrajectoryOptimization(initial_traj)
    trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.start_time()), initial_traj.value(initial_traj.start_time()), 0)
    trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.end_time()), initial_traj.value(initial_traj.end_time()), 1)

    minimum_distance = 0.001
    influence_distance = 0.05
    minimum_distance_constraint_diagram_context = diagram.CreateDefaultContext()
    minimum_distance_constraint_plant_context = plant.GetMyContextFromRoot(minimum_distance_constraint_diagram_context)
    minimum_distance_lower_bound_constraint = MinimumDistanceLowerBoundConstraint(
        plant, minimum_distance, minimum_distance_constraint_plant_context, None, influence_distance - minimum_distance
    )

    full_feasibility_constraint = FullFeasibilityConstraint(
        iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up, grasp_distance, minimum_distance_lower_bound_constraint
    )

    num_constraints_to_apply = 100
    for s in np.linspace(0, 1, num_constraints_to_apply):
        trajopt.AddPathPositionConstraint(full_feasibility_constraint, s)

    parameterized_path_energy = IiwaBimanualPathCost(
        trajopt.num_positions(), trajopt.num_control_points(), shoulder_up, elbow_up, wrist_up, grasp_distance, True
    )
    trajopt.prog().AddCost(parameterized_path_energy, trajopt.control_points().flatten())

    # Check initial guess satisfaction
    satisfied_init = True
    for binding in trajopt.prog().GetAllConstraints():
        c = binding.evaluator()
        val = c.Eval(trajopt.prog().GetInitialGuess(binding.variables()))
        lb, ub = c.lower_bound(), c.upper_bound()
        ok = np.all(val >= lb - 1e-4) and np.all(val <= ub + 1e-4)
        satisfied_init &= ok

    print("All constraints satisfied at initial guess?", satisfied_init)

    # 6. Configure Solver Options
    trajopt_options = SolverOptions()

    if use_solver == "ipopt":
        trajopt_options.SetOption(CommonSolverOption.kPrintToConsole, True)
        trajopt_options.SetOption(IpoptSolver().solver_id(), "print_level", 5)
        trajopt_options.SetOption(IpoptSolver().solver_id(), "max_wall_time", 60)
        if not strip_special_optimizer_settings:
            trajopt_options.SetOption(IpoptSolver().solver_id(), "acceptable_tol", 1e-2)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "acceptable_iter", 5)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "acceptable_constr_viol_tol", 1e-6)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "bound_relax_factor", 1e-12)
            trajopt_options.SetOption(IpoptSolver().solver_id(), "honor_original_bounds", "yes")
            trajopt_options.SetOption(IpoptSolver().solver_id(), "line_search_method", "penalty")
            trajopt_options.SetOption(IpoptSolver().solver_id(), "watchdog_shortened_iter_trigger", 0)
        else:
            print("NOTE: Special IPOPT optimizer settings REMOVED (logging/timing kept)!")
    elif use_solver == "snopt":
        trajopt_options.SetOption(CommonSolverOption.kPrintToConsole, False)
        trajopt_options.SetOption(CommonSolverOption.kPrintFileName, "snopt.log")
        trajopt_options.SetOption(SnoptSolver().solver_id(), "Major print level", 1)
        trajopt_options.SetOption(SnoptSolver().solver_id(), "Timing level", 3)
        trajopt_options.SetOption(SnoptSolver().solver_id(), "Time Limit", 60)
        if not strip_special_optimizer_settings:
            trajopt_options.SetOption(SnoptSolver().solver_id(), "Major optimality tolerance", 1e-1)
        else:
            print("NOTE: Special SNOPT optimizer settings REMOVED (logging/timing kept)!")

    if use_solver == "snopt":
        solver = SnoptSolver()
    elif use_solver == "ipopt":
        solver = IpoptSolver()
    else:
        raise ValueError(f"Unknown solver {use_solver}")

    print(f"Solving trajectory optimization using {solver.solver_id().name()} (use_trust_region={use_trust_region})...")
    
    if use_trust_region:
        result, t_solve = solve_trust_region_loop(trajopt.prog(), solver, trajopt_options, delta_0=delta_0, opt_tolerance=opt_tolerance, max_iters=max_iters)
    else:
        t_start = time.time()
        result = solver.Solve(trajopt.prog(), None, trajopt_options)
        t_solve = time.time() - t_start

    print(f"Solve completed in {t_solve:.2f}s.")
    print(f"Solve success: {result.is_success()}")
    print(f"Solution result status code: {result.get_solution_result()}")

    # Check solution satisfaction
    satisfied_sol = True
    max_violation = 0.0
    for binding in trajopt.prog().GetAllConstraints():
        c = binding.evaluator()
        val = c.Eval(result.GetSolution(binding.variables()))
        lb, ub = c.lower_bound(), c.upper_bound()
        ok = np.all(val >= lb - 1e-4) and np.all(val <= ub + 1e-4)
        if not ok:
            mag1 = np.max(lb - val) if np.any(val < lb) else 0.0
            mag2 = np.max(val - ub) if np.any(val > ub) else 0.0
            mag = max(mag1, mag2)
            max_violation = max(max_violation, mag)
        satisfied_sol &= ok

    # Evaluate final solution cost
    vars_all = trajopt.prog().decision_variables()
    x_sol = result.GetSolution(vars_all)
    sol_cost = 0.0
    for c_binding in trajopt.prog().GetAllCosts():
        indices = trajopt.prog().FindDecisionVariableIndices(c_binding.variables())
        sol_cost += np.sum(c_binding.evaluator().Eval(x_sol[indices]))
    print(f"Final Solution Cost: {sol_cost:.4f}")

    print("All constraints satisfied at solution?", satisfied_sol)
    if not satisfied_sol:
        print(f"Max violation magnitude at solution: {max_violation:.2e}")

    return result.is_success(), satisfied_sol, t_solve, sol_cost

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--strip-options", action="store_true", help="Strip special optimizer options")
    parser.add_argument("--trust-region", action="store_true", help="Use custom Trust Region loop")
    parser.add_argument("--solver", type=str, default="snopt", choices=["snopt", "ipopt"])
    parser.add_argument("--opt-tolerance", type=float, default=None, help="Custom SNOPT Major optimality tolerance")
    parser.add_argument("--delta-0", type=float, default=0.25, help="Initial trust region radius")
    parser.add_argument("--max-iters", type=int, default=20, help="Max TR iterations")
    args = parser.parse_args()

    run_pipeline(
        strip_special_optimizer_settings=args.strip_options,
        use_solver=args.solver,
        use_trust_region=args.trust_region,
        opt_tolerance=args.opt_tolerance,
        delta_0=args.delta_0,
        max_iters=args.max_iters
    )

