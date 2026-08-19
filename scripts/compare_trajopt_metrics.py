"""Compare trajectory optimization objectives by post-TOPPRA trajectory duration.

The notebooks optimize a path in the 8-dimensional parameterized space and then
lift it to the 14-dimensional configuration space, where TOPPRA retimes it
subject to the robot's velocity, acceleration and torque limits. Which objective
the trajectory optimizer minimizes is a free choice, and this script measures
what that choice costs in trajectory duration. Three objectives are compared,
all of them sums over consecutive B-spline control points, differing only in the
metric tensor:

  parameterized  sum_i ||q~_i - q~_{i-1}||^2                       (tensor: I)
  cspace         sum_i ||f(q~_i) - f(q~_{i-1})||^2                 (tensor: J^T J)
  kinetic        sum_i (df_i)^T M(f(q~_mid,i)) (df_i)              (tensor: J^T M J)

where f is the analytic-IK parameterization, df_i = f(q~_i) - f(q~_{i-1}), and
M is the mass matrix. The parameterized metric is what the RRT and shortcutter
optimize, but it is physically meaningless. The C-space metric is what the
notebooks currently use. The kinetic energy metric follows Kyaw & Kelly,
"Geometry-Aware Sampling-Based Motion Planning on Riemannian Manifolds"
(arXiv:2602.00992), where the mass matrix is the Riemannian metric tensor and
geodesics minimize kinetic energy for a given traversal time.

For each of the 6 ordered pairs of the three key configurations, this runs 10
BiRRT plans, shortcuts each, and then optimizes each of those initial guesses
under each of the three objectives. Since the three objectives share an initial
guess, the comparison is paired. Post-TOPPRA duration is the headline number;
solve time, success rate, cross-metric costs and limit utilization are also
recorded.

This uses the C++ parameterization for speed, so build it first -- see
cpp_parameterization/README.md.

Several variations on the experiment can be run independently of each other, to
find out what the comparison is sensitive to:

  --toppra-limits no-acceleration   retime under velocity and torque limits only
  --no-shortcut                     initialize from the raw RRT path
  --solver ipopt                    use IPOPT instead of SNOPT
  --payload-mass 28                 have the arms carry a heavy grasped object
                                    (28kg = the combined rated payload of the two
                                    IIWA-14 arms, i.e. as heavy as this hardware goes)

Usage:
    python3 scripts/compare_trajopt_metrics.py               # the full run
    python3 scripts/compare_trajopt_metrics.py --smoke       # one pair, one trial
    python3 scripts/compare_trajopt_metrics.py --analyze-only results/....csv
"""

import argparse
import csv
import multiprocessing
import os
import sys
import time
import traceback
import zlib

import numpy as np

repo_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(repo_dir)
sys.path.append(os.path.join(repo_dir, "cpp_parameterization/python"))

from pydrake.all import (
    BsplineBasis,
    BsplineTrajectory,
    CalcGridPointsOptions,
    CollisionCheckerParams,
    CommonSolverOption,
    CompositeTrajectory,
    IpoptSolver,
    RigidTransform,
    SpatialInertia,
    UnitInertia,
    ExtractGradient,
    FunctionHandleTrajectory,
    InitializeAutoDiff,
    KinematicTrajectoryOptimization,
    LoadModelDirectives,
    MinimumDistanceLowerBoundConstraint,
    MultibodyForces,
    Parser,
    PathParameterizedTrajectory,
    PiecewisePolynomial,
    ProcessModelDirectives,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    SnoptSolver,
    SolverOptions,
    Toppra,
)

from iiwa_ik import (
    FullFeasibilityConstraint,
    IiwaBimanualKineticEnergyPathCost,
    IiwaBimanualPathCost,
    IiwaBimanualReachableConstraint,
    MakeParameterization,
)

import src.common as common
import src.rrt as rrt
import src.shortcut as shortcut
from src.iiwa_analytic_ik import iiwa_limits_lower, iiwa_limits_upper

# --------------------------------------------------
# Parameters. These match notebooks/main_cpp.ipynb.

directives_file = os.path.join(common.RepoDir(), "models/old_shelves.dmd.yaml")
grasp_distance = 0.6
shoulder_up = True
elbow_up = True
wrist_up = False

q_tilde_bottom = np.array([-0.6430910102907225, 1.9156121024586796, -1.7968254667817805, 1.2945447141185198, -0.023834531305537934, -0.876966810663043, -1.7041643160834519, 1.45])
q_tilde_middle = np.array([-0.5997312520566763, 1.489780849654964, -1.4739679827359913, 1.2905366081785483, -0.04421061906813227, -0.8793712572715165, -1.1603461715511334, 1.45])
q_tilde_top = np.array([-0.1994994216078726, 0.9140739951190965, -2.236618320862171, 0.5238879195899456, 0.7998441913611017, -1.3575398006936048, -1.0153092816310436, 2.41])

key_configurations = {
    "bottom": q_tilde_bottom,
    "middle": q_tilde_middle,
    "top": q_tilde_top,
}
# All 6 ordered pairs, i.e. both directions of each of the three connections.
config_pairs = [(a, b) for a in key_configurations for b in key_configurations if a != b]

domain_lower = np.hstack((iiwa_limits_lower, [0.0]))
domain_upper = np.hstack((iiwa_limits_upper, [2.0 * np.pi]))

rrt_options = rrt.RRTOptions(
    step_size=2e-1,
    check_size=1e-2,
    max_vertices=1e4,
    max_iters=1e6,
    goal_sample_frequency=0.01,
    always_swap=False,
)

spline_order = 4  # Cubic spline
num_feasibility_constraints = 100
minimum_distance = 0.001  # 1mm
influence_distance = 0.05  # 5cm

metrics = ["parameterized", "cspace", "kinetic"]

# The notebook's SNOPT settings are loose enough that a solve may stop on the
# iteration budget rather than on optimality, which would make the comparison a
# measure of the stopping rule rather than of the objective. Both settings are
# run so that this can be checked rather than assumed. The IPOPT settings mirror
# the ones commented into notebooks/main_cpp.ipynb.
solver_settings = {
    "snopt": {
        "notebook": {"Major optimality tolerance": 1e-1, "Time Limit": 60},
        "tight": {"Major optimality tolerance": 1e-4, "Time Limit": 120},
    },
    "ipopt": {
        "notebook": {
            "print_level": 0,
            "max_wall_time": 60.0,
            "acceptable_tol": 1e-2,
            "acceptable_iter": 5,
            "acceptable_constr_viol_tol": 1e-6,
            "bound_relax_factor": 1e-12,
            "honor_original_bounds": "yes",
            "line_search_method": "penalty",
            "watchdog_shortened_iter_trigger": 0,
        },
        "tight": {
            "print_level": 0,
            "max_wall_time": 120.0,
            "tol": 1e-6,
            "acceptable_tol": 1e-4,
            "acceptable_iter": 15,
            "acceptable_constr_viol_tol": 1e-6,
            "bound_relax_factor": 1e-12,
            "honor_original_bounds": "yes",
            "line_search_method": "penalty",
            "watchdog_shortened_iter_trigger": 0,
        },
    },
}

# A solve counts if the trajectory it returns is actually feasible, not if the
# solver flagged success. SNOPT exit codes 3 ("accuracy not achieved") and 41
# ("point cannot be improved") both return usable points, and excluding them
# throws away good data and biases the comparison. These tolerances are far
# above the numerical slop seen in successful solves (order 1e-5 rad) and far
# below a genuine failure (order 1e-1 and up).
joint_limit_tolerance = 1e-3
reachability_tolerance = 1e-3
# The optimizer only enforces the constraints at num_feasibility_constraints
# sample points, so the dense check can catch brief corner-cutting in between.
minimum_feasible_fraction = 0.99

# Set from the command line, and passed to each worker process.
CONFIG = {
    "solver": "snopt",
    "shortcut": True,
    "toppra_limits": "all",
    "payload_mass": 0.0,
    "torque_limit_scale": 1.0,
}

num_trials = 10

# --------------------------------------------------
# Environment.


class Environment:
    """Everything that has to be built once per process: the plant, the
    collision checker, the parameterization, and the constraints and costs that
    depend on them."""

    def __init__(self, config):
        self.config = config
        builder = RobotDiagramBuilder(time_step=0.0)
        self.plant = builder.plant()
        parser = Parser(self.plant)
        parser.package_map().AddPackageXml(os.path.join(common.RepoDir(), "package.xml"))
        ProcessModelDirectives(LoadModelDirectives(directives_file), parser)

        params = CollisionCheckerParams()
        params.robot_model_instances = [
            self.plant.GetModelInstanceByName("iiwa_left"),
            self.plant.GetModelInstanceByName("iiwa_right"),
        ]
        if config["payload_mass"] > 0.0:
            self.AddPayload(config["payload_mass"])
        self.plant.Finalize()

        self.diagram = builder.Build()
        params.model = self.diagram
        params.edge_step_size = 0.01
        self.checker = SceneGraphCollisionChecker(params)

        self.parameterization = MakeParameterization(shoulder_up, elbow_up, wrist_up, grasp_distance)
        self.parameterization_double = self.parameterization.get_parameterization_double()
        self.parameterization_autodiff = self.parameterization.get_parameterization_autodiff()

        self.reachability_constraint = IiwaBimanualReachableConstraint(
            shoulder_up, elbow_up, wrist_up, grasp_distance)

        # Context used to build the minimum distance constraint below, and a
        # separate one for evaluating inverse dynamics.
        self.constraint_context = self.plant.GetMyContextFromRoot(self.diagram.CreateDefaultContext())
        self.minimum_distance_lower_bound_constraint = MinimumDistanceLowerBoundConstraint(
            self.plant, minimum_distance, self.constraint_context, None,
            influence_distance - minimum_distance)
        self.full_feasibility_constraint = FullFeasibilityConstraint(
            iiwa_limits_lower, iiwa_limits_upper, shoulder_up, elbow_up, wrist_up,
            grasp_distance, self.minimum_distance_lower_bound_constraint)

        self.dynamics_context = self.plant.CreateDefaultContext()

        self.velocity_limits = self.plant.GetVelocityUpperLimits()
        self.acceleration_limits = self.plant.GetAccelerationUpperLimits()
        # Derating the torque limits is a way to ask what happens when torque,
        # rather than the purely kinematic velocity and acceleration limits, is
        # what constrains the motion. The scaled limits are used both by TOPPRA
        # and by the utilization diagnostics, so utilization stays relative to
        # the limit actually enforced.
        self.torque_limits = self.plant.GetEffortUpperLimits() * config["torque_limit_scale"]
        self.torque_limits_lower = self.plant.GetEffortLowerLimits() * config["torque_limit_scale"]

    def AddPayload(self, payload_mass):
        """Models a heavy object held by the two grippers, which shows up in the
        mass matrix and in the torque required to move.

        The object is rigidly held by both arms, so the true system is a closed
        kinematic chain whose constrained mass matrix is not block diagonal and
        cannot be reproduced by welding bodies to the two arms independently.
        This approximates it by welding half the mass to each gripper, at the
        object's centre. That centre is a fixed point in each gripper's frame
        precisely because the grasp is rigid, so it is measured from the
        kinematics (see LocatePayload) rather than hard-coded.

        The payload carries no collision geometry, so the RRT and the collision
        constraints are unaffected and the paths stay comparable to a run
        without a payload."""
        X_gripper_payload = self.LocatePayload()
        for side in ["left", "right"]:
            gripper = self.plant.GetBodyByName("body", self.plant.GetModelInstanceByName(f"wsg_{side}"))
            inertia = SpatialInertia(
                mass=payload_mass / 2.0,
                p_PScm_E=np.zeros(3),
                # A 20cm sphere, i.e. a compact object rather than a long bar.
                G_SP_E=UnitInertia.SolidSphere(0.1),
            )
            body = self.plant.AddRigidBody(f"payload_{side}", gripper.model_instance(), inertia)
            self.plant.WeldFrames(gripper.body_frame(), body.body_frame(), X_gripper_payload)

    def LocatePayload(self):
        """The grasped object's centre, expressed in the left gripper's frame.
        Taken as the midpoint between the two grippers, which is a fixed point
        in either gripper's frame since the grasp is rigid. Verified against a
        second configuration, since that invariance is the whole premise.

        This needs a finalized plant, but the payload has to be welded on before
        finalizing, so it measures against a throwaway copy of the scene."""
        scratch_builder = RobotDiagramBuilder(time_step=0.0)
        scratch_plant = scratch_builder.plant()
        scratch_parser = Parser(scratch_plant)
        scratch_parser.package_map().AddPackageXml(os.path.join(common.RepoDir(), "package.xml"))
        ProcessModelDirectives(LoadModelDirectives(directives_file), scratch_parser)
        scratch_plant.Finalize()
        context = scratch_plant.CreateDefaultContext()
        parameterization_double = MakeParameterization(
            shoulder_up, elbow_up, wrist_up, grasp_distance).get_parameterization_double()

        def Midpoint(q_tilde):
            scratch_plant.SetPositions(context, parameterization_double(q_tilde))
            X_W_left = scratch_plant.EvalBodyPoseInWorld(
                context, scratch_plant.GetBodyByName("body", scratch_plant.GetModelInstanceByName("wsg_left")))
            X_W_right = scratch_plant.EvalBodyPoseInWorld(
                context, scratch_plant.GetBodyByName("body", scratch_plant.GetModelInstanceByName("wsg_right")))
            p_W_middle = 0.5 * (X_W_left.translation() + X_W_right.translation())
            return X_W_left.inverse() @ p_W_middle

        p_gripper_payload = Midpoint(q_tilde_middle)
        check = Midpoint(q_tilde_top)
        assert np.allclose(p_gripper_payload, check, atol=1e-9), (
            f"the grasp is not rigid: object centre moves from {p_gripper_payload} to {check}")
        return RigidTransform(p_gripper_payload)

    def RandomConfig(self):
        return np.random.uniform(low=domain_lower, high=domain_upper)

    def ValidityChecker(self, q_tilde):
        """Copied from notebooks/main_cpp.ipynb: reachability, then the
        subordinate arm's joint limits, then a guard against the analytic IK
        singularities, then collisions."""
        if np.any(self.reachability_constraint.Eval(q_tilde) > np.ones(4)):
            return False
        if np.any(self.reachability_constraint.Eval(q_tilde) < -np.ones(4)):
            return False

        q_full = self.parameterization_double(q_tilde)
        q_sub = q_full[7:]
        if np.any(q_sub < iiwa_limits_lower):
            return False
        if np.any(q_sub > iiwa_limits_upper):
            return False

        if np.any(np.abs(q_sub[[1, 3, 5]]) < 1e-2):
            return False

        return self.checker.CheckConfigCollisionFree(q_full)


environment = None


def InitializeWorker(config):
    global environment, CONFIG
    CONFIG = config
    environment = Environment(config)


# --------------------------------------------------
# Costs.


def ParameterizedPathEnergyMatrix(num_positions, num_control_points):
    """The matrix D for which sum_i ||q~_i - q~_{i-1}||^2 == ||D x||^2, where x
    is the control point matrix flattened in row-major (numpy) order, matching
    the convention of the C++ costs."""
    num_segments = num_control_points - 1
    D = np.zeros((num_positions * num_segments, num_positions * num_control_points))
    for i in range(num_segments):
        for r in range(num_positions):
            D[i * num_positions + r, r * num_control_points + i] = -1.0
            D[i * num_positions + r, r * num_control_points + i + 1] = 1.0
    return D


def ParameterizedPathEnergy(control_points):
    """sum_i ||q~_i - q~_{i-1}||^2 for an 8 x num_control_points matrix."""
    return float(np.sum(np.diff(control_points, axis=1) ** 2))


def MakeCost(metric, num_positions, num_control_points, plant, scale=1.0):
    """The C++ cost object for a metric, or None for the parameterized metric,
    which is quadratic in the decision variables and so is added directly to the
    program instead."""
    if metric == "parameterized":
        return None
    if metric == "cspace":
        return IiwaBimanualPathCost(num_positions, num_control_points, shoulder_up,
                                    elbow_up, wrist_up, grasp_distance, True, scale)
    if metric == "kinetic":
        return IiwaBimanualKineticEnergyPathCost(num_positions, num_control_points,
                                                 shoulder_up, elbow_up, wrist_up,
                                                 grasp_distance, plant, scale)
    raise ValueError(f"Unknown metric {metric}")


def EvaluateMetric(metric, control_points, cost_objects):
    if metric == "parameterized":
        return ParameterizedPathEnergy(control_points)
    return float(cost_objects[metric].Eval(control_points.flatten())[0])


# --------------------------------------------------
# Lifting and retiming.


def LiftTrajectory(traj_tilde, env):
    """Wraps an 8-dimensional trajectory in a 14-dimensional
    FunctionHandleTrajectory via the parameterization, as the notebooks do. The
    first derivative is analytic (the parameterization Jacobian times the
    parameterized velocity) and the second comes from a five-point stencil,
    since FunctionHandleTrajectory has no analytic derivative."""
    traj_function = lambda t: env.parameterization_double(traj_tilde.value(t).flatten())
    full_traj = FunctionHandleTrajectory(traj_function, 14, 1,
                                         traj_tilde.start_time(), traj_tilde.end_time())

    def full_traj_derivative(t, order, dt=1e-6):
        if order == 1:
            x_val = traj_tilde.value(t).flatten()
            xdot_val = traj_tilde.EvalDerivative(t, 1).flatten()
            x_ad = InitializeAutoDiff(x_val).flatten()
            y_ad = env.parameterization_autodiff(x_ad)
            J = ExtractGradient(y_ad)
            return J @ xdot_val.reshape(-1, 1)
        elif order == 2:
            f1 = full_traj_derivative(t + dt, order=1)
            f2 = full_traj_derivative(t - dt, order=1)
            f3 = full_traj_derivative(t + 2 * dt, order=1)
            f4 = full_traj_derivative(t - 2 * dt, order=1)
            return (f4 - 8 * f2 + 8 * f1 - f3) / (12 * dt)
        else:
            raise RuntimeError(f"Only first and second derivatives supported (requested order={order})")

    full_traj.set_derivative(full_traj_derivative)
    return full_traj


def Retime(full_traj, env):
    """TOPPRA, exactly as in the notebooks. Returns (retimed trajectory, note),
    where note is empty on success."""
    gridpoints = Toppra.CalcGridPoints(full_traj, CalcGridPointsOptions(max_iter=2, min_points=200))
    toppra = Toppra(full_traj, env.plant, gridpoints)
    toppra.AddJointVelocityLimit(env.plant.GetVelocityLowerLimits(), env.plant.GetVelocityUpperLimits())
    if env.config["toppra_limits"] != "no-acceleration":
        toppra.AddJointAccelerationLimit(env.plant.GetAccelerationLowerLimits(),
                                         env.plant.GetAccelerationUpperLimits())
    toppra.AddJointTorqueLimit(env.torque_limits_lower, env.torque_limits)
    time_traj = toppra.SolvePathParameterization()
    if time_traj is None:
        return None, "TOPPRA returned no path parameterization"
    retimed = PathParameterizedTrajectory(full_traj, time_traj)
    duration = retimed.end_time() - retimed.start_time()
    if not np.isfinite(duration):
        return retimed, "TOPPRA returned an infinite duration"
    return retimed, ""


# --------------------------------------------------
# Evaluation of a resulting trajectory.


def CheckFeasibility(traj_tilde, env, num_samples=500):
    """Densely checks the constraints that the optimizer only enforced at 100
    sample points, so that a shorter trajectory that is actually infeasible
    cannot masquerade as an improvement. Collisions are reported separately from
    the two analytic violations, since a violation of a few 1e-5 rad is solver
    slop while a collision is not."""
    num_feasible = 0
    num_collision_free = 0
    worst_reachability = 0.0
    worst_joint_limit = 0.0
    for s in np.linspace(traj_tilde.start_time(), traj_tilde.end_time(), num_samples):
        q_tilde = traj_tilde.value(s).flatten()
        unclipped = env.reachability_constraint.Eval(q_tilde)
        reachability_violation = np.max(np.abs(unclipped) - 1.0)
        worst_reachability = max(worst_reachability, reachability_violation)

        q_full = env.parameterization_double(q_tilde)
        q_sub = q_full[7:]
        joint_limit_violation = max(np.max(iiwa_limits_lower - q_sub), np.max(q_sub - iiwa_limits_upper))
        worst_joint_limit = max(worst_joint_limit, joint_limit_violation)

        collision_free = env.checker.CheckConfigCollisionFree(q_full)
        num_collision_free += int(collision_free)
        # Tolerances, not exact satisfaction: a converged solve routinely sits a
        # few 1e-5 rad outside a joint limit, which is solver slop rather than
        # an infeasible trajectory.
        if (reachability_violation <= reachability_tolerance
                and joint_limit_violation <= joint_limit_tolerance and collision_free):
            num_feasible += 1
    return {
        "feasible_fraction": num_feasible / num_samples,
        "collision_free_fraction": num_collision_free / num_samples,
        "worst_reachability_violation": worst_reachability,
        "worst_joint_limit_violation": worst_joint_limit,
    }


def MeasureExecution(retimed_traj, env, num_samples=400):
    """What executing the retimed trajectory actually costs.

    Two families of number come out of this. The first is the peak fraction of
    each limit used: TOPPRA drives at least one of them to 1, and which one it
    is says what is actually setting the duration. The second is energy, which
    is a downstream measure of plan quality independent of duration -- a plan
    can be fast and wasteful, or slow and cheap.

    Torques come from inverse dynamics and include gravity, so lifting the
    payload counts as work, as it should.

    Energy is reported four ways because they answer different questions:
      mechanical_work    integral of sum_i |tau_i * qdot_i| dt. Total mechanical
                         work through the joints, treating every joint as
                         consuming energy whether it drives or brakes (i.e. no
                         regeneration). The closest thing to "energy usage".
      positive_work      integral of sum_i max(tau_i * qdot_i, 0) dt. What the
                         motors must supply if braking is free.
      thermal_integral   integral of sum_i tau_i^2 dt. Resistive heating in the
                         motors goes as current squared, and current goes as
                         torque, so this tracks how hot the arms get.
      kinetic_energy_integral
                         integral of qdot^T M qdot dt. The Dirichlet energy of
                         the paper's Riemannian metric (their eq. 2, up to the
                         factor of 1/2), but measured on the retimed trajectory
                         in real time rather than on the path. This is what the
                         kinetic energy objective is a discrete proxy for.
    """
    peak = {"velocity": 0.0, "acceleration": 0.0, "torque": 0.0}
    # Peak utilization only says a limit is touched *somewhere*. What matters
    # for duration is how much of the path each limit is actually active for,
    # since that is the fraction over which it is setting the speed.
    utilization = {"velocity": np.zeros(num_samples), "acceleration": np.zeros(num_samples),
                   "torque": np.zeros(num_samples)}
    times = np.linspace(retimed_traj.start_time(), retimed_traj.end_time(), num_samples)
    mechanical_power = np.zeros(num_samples)
    positive_power = np.zeros(num_samples)
    squared_torque = np.zeros(num_samples)
    twice_kinetic_energy = np.zeros(num_samples)

    for i, t in enumerate(times):
        q = retimed_traj.value(t).flatten()
        v = retimed_traj.EvalDerivative(t, 1).flatten()
        a = retimed_traj.EvalDerivative(t, 2).flatten()

        env.plant.SetPositions(env.dynamics_context, q)
        env.plant.SetVelocities(env.dynamics_context, v)
        forces = MultibodyForces(env.plant)
        env.plant.CalcForceElementsContribution(env.dynamics_context, forces)
        tau = env.plant.CalcInverseDynamics(env.dynamics_context, a, forces)
        mass_matrix = env.plant.CalcMassMatrix(env.dynamics_context)

        utilization["velocity"][i] = np.max(np.abs(v) / env.velocity_limits)
        utilization["acceleration"][i] = np.max(np.abs(a) / env.acceleration_limits)
        utilization["torque"][i] = np.max(np.abs(tau) / env.torque_limits)
        for name in peak:
            peak[name] = max(peak[name], utilization[name][i])

        power = tau * v
        mechanical_power[i] = np.sum(np.abs(power))
        positive_power[i] = np.sum(np.maximum(power, 0.0))
        squared_torque[i] = np.sum(tau ** 2)
        twice_kinetic_energy[i] = v @ mass_matrix @ v

    measurements = {f"peak_{name}_utilization": value for name, value in peak.items()}
    # Median utilization is robust to the finite difference stencil's occasional
    # outliers in a way the peak is not.
    for name, values in utilization.items():
        measurements[f"median_{name}_utilization"] = float(np.median(values))
        measurements[f"{name}_active_fraction"] = float(np.mean(values >= 0.99))
    measurements["binding_limit"] = max(peak, key=peak.get)
    measurements["mechanical_work"] = float(np.trapezoid(mechanical_power, times))
    measurements["positive_work"] = float(np.trapezoid(positive_power, times))
    measurements["thermal_integral"] = float(np.trapezoid(squared_torque, times))
    measurements["kinetic_energy_integral"] = float(np.trapezoid(twice_kinetic_energy, times))
    measurements["peak_kinetic_energy"] = float(0.5 * np.max(twice_kinetic_energy))
    return measurements


# --------------------------------------------------
# One unit of work: one (pair, trial), i.e. one RRT plan optimized under each
# metric with each solver setting.


def PlanAndOptimize(job):
    start_name, goal_name, trial = job
    env = environment
    rows = []
    start = key_configurations[start_name].copy()
    goal = key_configurations[goal_name].copy()

    # A distinct but reproducible seed per (pair, trial). Python's hash() is
    # salted per process, so it would not be reproducible across runs.
    seed = zlib.crc32(f"{start_name}->{goal_name}:{trial}".encode()) % (2 ** 31)
    np.random.seed(seed)

    rrt_start_time = time.time()
    rrt_planner = rrt.BiRRT(env.RandomConfig, env.ValidityChecker)
    path = rrt_planner.plan(start, goal, rrt_options)
    rrt_time = time.time() - rrt_start_time
    if len(path) == 0:
        print(f"[{start_name}->{goal_name} trial {trial}] RRT FAILED", flush=True)
        return [{"start": start_name, "goal": goal_name, "trial": trial, "metric": "",
                 "solver_setting": "", "rrt_success": False}]

    shortcut_start_time = time.time()
    if env.config["shortcut"]:
        shortcut_path = shortcut.shortcut(path.copy(), env.ValidityChecker, num_tries=1e2,
                                          check_size=rrt_options.check_size)
    else:
        # Feed the optimizer the raw RRT path, to see how much of the result is
        # owed to the shortcutter rather than to the objective.
        shortcut_path = path.copy()
    shortcut_time = time.time() - shortcut_start_time

    control_points_matrix = np.array([point for point in shortcut_path]).T
    num_positions, num_control_points = control_points_matrix.shape
    if num_control_points < spline_order:
        print(f"[{start_name}->{goal_name} trial {trial}] shortcut path has only "
              f"{num_control_points} waypoints, fewer than the spline order; skipping",
              flush=True)
        return [{"start": start_name, "goal": goal_name, "trial": trial, "metric": "",
                 "solver_setting": "", "rrt_success": True, "note": "path too short"}]

    # Unscaled cost objects, used both to normalize and to evaluate every
    # metric on every result.
    cost_objects = {m: MakeCost(m, num_positions, num_control_points, env.plant)
                    for m in metrics}
    initial_metric_values = {m: EvaluateMetric(m, control_points_matrix, cost_objects)
                             for m in metrics}

    # The un-optimized shortcut path, retimed, as a per-trial baseline.
    rrt_traj_segments = [
        PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            np.array([float(i) - 1.0, float(i)]),
            np.array([shortcut_path[i - 1], shortcut_path[i]]).T,
            np.zeros(8),
            np.zeros(8),
        )
        for i in range(1, len(shortcut_path))
    ]
    rrt_traj = CompositeTrajectory(rrt_traj_segments)
    try:
        rrt_retimed, rrt_note = Retime(LiftTrajectory(rrt_traj, env), env)
        rrt_duration = (rrt_retimed.end_time() - rrt_retimed.start_time()) if rrt_retimed else np.nan
    except Exception as e:
        rrt_duration, rrt_note = np.nan, f"exception: {e}"
    if rrt_note:
        print(f"[{start_name}->{goal_name} trial {trial}] shortcut path baseline: {rrt_note}",
              flush=True)

    basis = BsplineBasis(spline_order, num_control_points, initial_parameter_value=0.0,
                         final_parameter_value=float(num_control_points - 1))
    initial_traj = BsplineTrajectory(basis, control_points_matrix)

    for setting_name, setting in solver_settings[env.config["solver"]].items():
        for metric in metrics:
            row = {
                "start": start_name,
                "goal": goal_name,
                "trial": trial,
                "metric": metric,
                "solver_setting": setting_name,
                "solver": env.config["solver"],
                "toppra_limits": env.config["toppra_limits"],
                "shortcut": env.config["shortcut"],
                "payload_mass": env.config["payload_mass"],
                "torque_limit_scale": env.config["torque_limit_scale"],
                "seed": seed,
                "rrt_success": True,
                "rrt_time": rrt_time,
                "rrt_waypoints": len(path),
                "shortcut_time": shortcut_time,
                "shortcut_waypoints": len(shortcut_path),
                "num_control_points": num_control_points,
                "shortcut_duration": rrt_duration,
                "shortcut_toppra_note": rrt_note,
            }
            for m in metrics:
                row[f"initial_{m}"] = initial_metric_values[m]
            try:
                row.update(Optimize(initial_traj, metric, setting, env, cost_objects,
                                    initial_metric_values, f"{start_name}->{goal_name} trial {trial}"))
            except Exception:
                row["note"] = "exception"
                print(f"[{start_name}->{goal_name} trial {trial} {metric} {setting_name}] "
                      f"EXCEPTION:\n{traceback.format_exc()}", flush=True)
            rows.append(row)
            print(f"[{start_name}->{goal_name} trial {trial}] {setting_name:8s} {metric:13s} "
                  f"duration {row.get('duration', float('nan')):8.3f} "
                  f"solve {row.get('solve_time', float('nan')):6.2f}s "
                  f"success {row.get('trajopt_success')}", flush=True)

    return rows


def Optimize(initial_traj, metric, setting, env, cost_objects, initial_metric_values, label):
    """Runs trajectory optimization under one metric, then lifts and retimes the
    result. Returns a dict of measurements."""
    row = {}
    trajopt = KinematicTrajectoryOptimization(initial_traj)
    num_positions = trajopt.num_positions()
    num_control_points = trajopt.num_control_points()

    trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.start_time()),
                                      initial_traj.value(initial_traj.start_time()), 0)
    trajopt.AddPathPositionConstraint(initial_traj.value(initial_traj.end_time()),
                                      initial_traj.value(initial_traj.end_time()), 1)
    for s in np.linspace(0, 1, num_feasibility_constraints):
        trajopt.AddPathPositionConstraint(env.full_feasibility_constraint, s)

    # Normalize so that every objective starts at 1.0. This does not change the
    # minimizer, but it puts the solver's relative tolerances and merit function
    # on the same footing for objectives with different physical units.
    scale = 1.0 / initial_metric_values[metric] if initial_metric_values[metric] > 0 else 1.0
    row["cost_scale"] = scale
    if metric == "parameterized":
        D = ParameterizedPathEnergyMatrix(num_positions, num_control_points)
        Q = 2.0 * scale * (D.T @ D)
        trajopt.prog().AddQuadraticCost(Q, np.zeros(Q.shape[0]), 0.0,
                                        trajopt.control_points().flatten(), is_convex=True)
    else:
        cost = MakeCost(metric, num_positions, num_control_points, env.plant, scale)
        trajopt.prog().AddCost(cost, trajopt.control_points().flatten())

    solver = SnoptSolver() if env.config["solver"] == "snopt" else IpoptSolver()
    options = SolverOptions()
    options.SetOption(CommonSolverOption.kPrintToConsole, False)
    if env.config["solver"] == "snopt":
        options.SetOption(solver.solver_id(), "Major print level", 0)
    for key, value in setting.items():
        options.SetOption(solver.solver_id(), key, value)

    solve_start_time = time.time()
    result = solver.Solve(trajopt.prog(), None, options)
    row["solve_time"] = time.time() - solve_start_time
    row["trajopt_success"] = result.is_success()
    row["solution_result"] = str(result.get_solution_result())
    details = result.get_solver_details()
    if env.config["solver"] == "snopt":
        row["solver_exit_code"] = int(details.info)
        row["snopt_solve_time"] = details.solve_time
    else:
        row["solver_exit_code"] = int(details.status)
    row["final_cost"] = result.get_optimal_cost()

    trajopt_traj = trajopt.ReconstructTrajectory(result)
    control_points = np.column_stack(
        [np.asarray(point).flatten() for point in trajopt_traj.control_points()])
    for m in metrics:
        row[f"final_{m}"] = EvaluateMetric(m, control_points, cost_objects)

    feasibility = CheckFeasibility(trajopt_traj, env)
    row.update(feasibility)

    try:
        retimed, note = Retime(LiftTrajectory(trajopt_traj, env), env)
    except Exception as e:
        retimed, note = None, f"exception: {e}"
    row["toppra_note"] = note
    if retimed is None:
        row["duration"] = np.nan
    else:
        row["duration"] = retimed.end_time() - retimed.start_time()
    if note:
        # TOPPRA should not fail unless the optimizer returned something
        # infeasible, so this is flagged rather than quietly retried.
        print(f"\n!!! TOPPRA PROBLEM [{label} {metric}]: {note}\n"
              f"    trajopt success: {result.is_success()} ({result.get_solution_result()})\n"
              f"    densely feasible fraction: {feasibility['feasible_fraction']:.3f}, "
              f"worst reachability violation: {feasibility['worst_reachability_violation']:.3e}, "
              f"worst joint limit violation: {feasibility['worst_joint_limit_violation']:.3e}\n"
              f"    control points dumped for investigation\n", flush=True)
        DumpFailure(label, metric, control_points)

    if retimed is not None and np.isfinite(row["duration"]):
        row.update(MeasureExecution(retimed, env))

    # Keeping the solution means any further downstream measure can be computed
    # later without re-solving. Stripped out of the row before the CSV is
    # written, and saved alongside it instead.
    row["control_points"] = control_points

    return row


def DumpFailure(label, metric, control_points):
    directory = os.path.join(common.RepoDir(), "results", "toppra_failures")
    os.makedirs(directory, exist_ok=True)
    name = label.replace(" ", "_").replace("->", "_to_") + "_" + metric + ".npy"
    np.save(os.path.join(directory, name), control_points)


# --------------------------------------------------
# Analysis.


def Summarize(rows):
    import scipy.stats

    rows = [r for r in rows if r.get("metric")]

    def Number(row, key):
        value = row.get(key, "")
        if value in ("", None):
            return np.nan
        return float(value)

    def ExitCode(row):
        """The solver's exit code, or -1 if the result file predates it being
        recorded under this name."""
        for key in ["solver_exit_code", "snopt_info"]:
            value = Number(row, key)
            if np.isfinite(value):
                return int(value)
        return -1

    def Usable(row):
        """Whether the trajectory the solver returned is one we can measure.

        This deliberately does not use the solver's success flag. SNOPT exit
        codes 3 ("requested accuracy could not be achieved") and 41 ("current
        point cannot be improved") are not failures -- the solver returns a
        point, it just could not certify optimality to the requested tolerance,
        and that point is normally feasible and near-optimal. What matters is
        whether the returned trajectory actually satisfies the constraints, so
        that is what gets checked."""
        if not np.isfinite(Number(row, "duration")):
            return False
        if Number(row, "worst_joint_limit_violation") > joint_limit_tolerance:
            return False
        if Number(row, "worst_reachability_violation") > reachability_tolerance:
            return False
        # Collisions are the part that tolerances cannot excuse.
        collision_free = Number(row, "collision_free_fraction")
        if not np.isfinite(collision_free):
            collision_free = Number(row, "feasible_fraction")  # older result files
        return collision_free >= minimum_feasible_fraction

    print("\n" + "=" * 78)
    print("POST-TOPPRA TRAJECTORY DURATION BY OBJECTIVE")
    print("=" * 78)

    setting_names = []
    for r in rows:
        if r["solver_setting"] not in setting_names:
            setting_names.append(r["solver_setting"])

    for setting_name in setting_names:
        subset = [r for r in rows if r["solver_setting"] == setting_name]
        if not subset:
            continue
        solver_name = subset[0].get("solver", "snopt")
        setting = solver_settings[solver_name][setting_name]
        print(f"\n--- {solver_name}, {setting_name} settings: "
              + ", ".join(f"{k} {v}" for k, v in setting.items()) + " ---")

        # A solve that returned an infeasible trajectory still produces a
        # duration, and usually a badly inflated one, so those rows would
        # otherwise dominate the statistics. They are counted, listed below,
        # and excluded from the averages.
        print(f"\nDurations over usable solves only (the returned trajectory is feasible,"
              f"\nregardless of what the solver's exit code says):")
        print(f"\n{'metric':14s} {'n':>4s} {'unusable':>9s} {'mean':>8s} {'median':>8s} {'std':>8s} "
              f"{'min':>8s} {'max':>8s} {'solve s':>8s} {'snopt s':>7s} {'feasible':>9s}")
        for metric in metrics:
            metric_rows = [r for r in subset if r["metric"] == metric]
            usable = [r for r in metric_rows if Usable(r)]
            durations = np.array([Number(r, "duration") for r in usable])
            durations = durations[np.isfinite(durations)]
            solve_times = np.array([Number(r, "solve_time") for r in metric_rows])
            snopt_times = np.array([Number(r, "snopt_solve_time") for r in metric_rows])
            feasible = np.array([Number(r, "feasible_fraction") for r in usable])
            if len(durations) == 0:
                print(f"{metric:14s} {0:4d} {len(metric_rows):9d}   (no usable solves)")
                continue
            print(f"{metric:14s} {len(durations):4d} {len(metric_rows) - len(usable):9d} "
                  f"{np.mean(durations):8.3f} "
                  f"{np.median(durations):8.3f} {np.std(durations):8.3f} "
                  f"{np.min(durations):8.3f} {np.max(durations):8.3f} "
                  f"{np.nanmedian(solve_times):8.2f} {np.nanmedian(snopt_times):7.2f} "
                  f"{np.nanmean(feasible) * 100:8.1f}%")

        # Energy is a downstream measure of plan quality that is independent of
        # duration: a plan can be fast and wasteful, or slow and cheap. It is
        # also the quantity the kinetic energy objective is a proxy for, so it
        # is the fairest place to look for that objective paying off.
        energy_keys = [("mechanical_work", "work J"), ("positive_work", "pos work J"),
                       ("thermal_integral", "int tau^2"), ("kinetic_energy_integral", "int KE"),
                       ("peak_kinetic_energy", "peak KE J")]
        if any(np.isfinite(Number(r, "mechanical_work")) for r in subset):
            print(f"\nEnergy of the executed trajectory (means over usable solves, "
                  f"and % vs cspace):")
            print(f"{'metric':14s}" + "".join(f"{label:>22s}" for _, label in energy_keys))
            baseline_energy = {}
            for key, _ in energy_keys:
                values = [Number(r, key) for r in subset if r["metric"] == "cspace" and Usable(r)]
                values = [v for v in values if np.isfinite(v)]
                baseline_energy[key] = np.mean(values) if values else np.nan
            for metric in metrics:
                line = f"{metric:14s}"
                for key, _ in energy_keys:
                    values = [Number(r, key) for r in subset if r["metric"] == metric and Usable(r)]
                    values = [v for v in values if np.isfinite(v)]
                    if not values:
                        line += f"{'-':>22s}"
                        continue
                    mean = np.mean(values)
                    percent = 100.0 * (mean - baseline_energy[key]) / baseline_energy[key]
                    line += f"{mean:14.1f}{percent:+7.1f}%"
                print(line)

        # The point of this table is to show that the exit code is a poor proxy
        # for whether the answer is usable.
        print(f"\nSolver exit code vs. usability (SNOPT: 1 = optimal, 3 = accuracy not "
              f"achieved,\n41 = point cannot be improved, 30s = resource limit / numerical "
              f"trouble):")
        print(f"  {'code':>6s} {'n':>5s} {'usable':>8s}")
        codes = {}
        for r in subset:
            codes.setdefault(ExitCode(r), []).append(Usable(r))
        for code in sorted(codes):
            print(f"  {code:6d} {len(codes[code]):5d} {sum(codes[code]):8d}")

        unusable = [r for r in subset if not Usable(r)]
        if unusable:
            print(f"\n{len(unusable)} unusable solves (these are the ones worth investigating):")
            for r in unusable:
                print(f"  {r['start']}->{r['goal']} t{r['trial']} {r['metric']:13s} "
                      f"code {ExitCode(r):3d} "
                      f"feasible {Number(r, 'feasible_fraction'):.3f} "
                      f"reach {Number(r, 'worst_reachability_violation'):.1e} "
                      f"joint {Number(r, 'worst_joint_limit_violation'):.1e} "
                      f"duration {Number(r, 'duration'):.2f}")

        # Paired comparison against the C-space objective, which is what the
        # notebooks currently use. Only pairs in which both solves are usable
        # are compared, since otherwise the difference measures a solver
        # failure rather than the objective.
        print(f"\nPaired against 'cspace' (same RRT seed, both solves usable):")
        print(f"{'metric':14s} {'n':>4s} {'mean delta':>11s} {'mean %':>8s} {'median %':>9s} {'wins':>7s} {'p':>9s}")
        baseline = {(r["start"], r["goal"], r["trial"]): Number(r, "duration")
                    for r in subset if r["metric"] == "cspace" and Usable(r)}
        for metric in metrics:
            if metric == "cspace":
                continue
            pairs = []
            for r in subset:
                if r["metric"] != metric or not Usable(r):
                    continue
                key = (r["start"], r["goal"], r["trial"])
                if key in baseline and np.isfinite(baseline[key]) and np.isfinite(Number(r, "duration")):
                    pairs.append((Number(r, "duration"), baseline[key]))
            if not pairs:
                continue
            mine = np.array([p[0] for p in pairs])
            theirs = np.array([p[1] for p in pairs])
            delta = mine - theirs
            percent = 100.0 * delta / theirs
            try:
                p_value = scipy.stats.wilcoxon(mine, theirs).pvalue
            except ValueError:
                p_value = np.nan
            print(f"{metric:14s} {len(pairs):4d} {np.mean(delta):11.3f} {np.mean(percent):7.1f}% "
                  f"{np.median(percent):8.1f}% {np.mean(delta < 0) * 100:6.0f}% {p_value:9.2e}")

        # Also compare against the un-optimized shortcut path.
        print(f"\nAgainst the un-optimized shortcut path:")
        for metric in metrics:
            pairs = [(Number(r, "duration"), Number(r, "shortcut_duration"))
                     for r in subset if r["metric"] == metric and Usable(r)]
            pairs = [p for p in pairs if np.isfinite(p[0]) and np.isfinite(p[1])]
            if not pairs:
                continue
            percent = 100.0 * (np.array([p[0] for p in pairs]) - np.array([p[1] for p in pairs])) / np.array([p[1] for p in pairs])
            print(f"  {metric:14s} mean {np.mean(percent):6.1f}%  median {np.median(percent):6.1f}%")

        print(f"\nDuration by configuration pair (mean):")
        header = f"{'pair':20s}" + "".join(f"{m:>14s}" for m in metrics)
        print(header)
        for start_name, goal_name in config_pairs:
            line = f"{start_name + '->' + goal_name:20s}"
            for metric in metrics:
                durations = [Number(r, "duration") for r in subset
                             if r["metric"] == metric and Usable(r)
                             and r["start"] == start_name and r["goal"] == goal_name]
                durations = [d for d in durations if np.isfinite(d)]
                line += f"{np.mean(durations):14.3f}" if durations else f"{'-':>14s}"
            print(line)

        print(f"\nCross-metric costs (mean value of each metric on each solution, "
              f"normalized by the initial guess):")
        header = f"{'optimized for':16s}" + "".join(f"{m:>16s}" for m in metrics)
        print(header)
        for metric in metrics:
            line = f"{metric:16s}"
            for evaluated in metrics:
                ratios = [Number(r, f"final_{evaluated}") / Number(r, f"initial_{evaluated}")
                          for r in subset if r["metric"] == metric and Usable(r)
                          and np.isfinite(Number(r, f"initial_{evaluated}"))
                          and Number(r, f"initial_{evaluated}") > 0]
                ratios = [x for x in ratios if np.isfinite(x)]
                line += f"{np.mean(ratios):16.3f}" if ratios else f"{'-':>16s}"
            print(line)

        # TOPPRA enforces the limits at its grid points, and the path's second
        # derivative here comes from a finite difference stencil, so a peak
        # sampled between grid points can come out slightly above 1.
        print(f"\nWhich limit binds after retiming (peak utilization, and how often it is the max):")
        for metric in metrics:
            metric_rows = [r for r in subset if r["metric"] == metric and Usable(r)]
            binding = [r.get("binding_limit", "") for r in metric_rows]
            counts = {limit: binding.count(limit) for limit in ["velocity", "acceleration", "torque"]}
            # Medians, not means: the second derivative comes from a finite
            # difference stencil, which throws occasional large outliers that
            # make a mean uninformative. The fraction of solves in which a
            # limit is saturated is the robust signal.
            peaks = {limit: np.nanmedian([Number(r, f"peak_{limit}_utilization") for r in metric_rows])
                     for limit in ["velocity", "acceleration", "torque"]}
            saturated = {limit: np.nanmean([Number(r, f"peak_{limit}_utilization") >= 0.99
                                            for r in metric_rows])
                         for limit in ["velocity", "acceleration", "torque"]}
            print(f"  {metric:14s} " + "  ".join(
                f"{limit}: median {peaks[limit]:.2f}, saturated {100 * saturated[limit]:3.0f}%"
                for limit in counts))

        # KNOWN ISSUE, deferred: the feasibility constraints are imposed only at
        # num_feasibility_constraints sample points, so the trajectory can clip
        # an obstacle between two of them. The excursions are shallow (under 1%
        # of the path) and do not affect the simulated results, so they are not
        # corrected here -- but they must be dealt with before any of this runs
        # on hardware. Denser constraint sampling is the fix.
        print(f"\nCorner-cutting between the {num_feasibility_constraints} constraint samples "
              f"(HARDWARE CAVEAT, harmless in simulation):")
        for metric in metrics:
            metric_rows = [r for r in subset if r["metric"] == metric and Usable(r)]
            collision_free = np.array([Number(r, "collision_free_fraction") for r in metric_rows])
            collision_free = collision_free[np.isfinite(collision_free)]
            if len(collision_free) == 0:
                continue
            print(f"  {metric:14s} {100 * np.mean(collision_free < 1.0):3.0f}% of solves clip "
                  f"somewhere; worst is {100 * (1 - np.min(collision_free)):.1f}% of the path")

        toppra_problems = [r for r in subset if r.get("toppra_note")]
        if toppra_problems:
            print(f"\n!!! {len(toppra_problems)} TOPPRA failures needing investigation:")
            for r in toppra_problems:
                print(f"    {r['start']}->{r['goal']} trial {r['trial']} {r['metric']}: "
                      f"{r['toppra_note']} (trajopt success {r.get('trajopt_success')}, "
                      f"densely feasible {r.get('feasible_fraction')})")
        else:
            print(f"\nNo TOPPRA failures.")


# --------------------------------------------------


def CompareRuns(paths):
    """Puts the headline numbers from several runs side by side, so that the
    effect of each variation on the kinetic-vs-C-space comparison is visible in
    one table."""
    import scipy.stats

    print("\n" + "=" * 110)
    print("VARIATIONS SIDE BY SIDE")
    print("=" * 110)
    print(f"\n{'run':26s} {'setting':9s} {'metric':14s} {'n':>4s} {'median':>8s} {'mean':>8s} "
          f"{'vs cspace':>10s} {'wins':>6s} {'p':>9s} {'binds':>13s}")

    for path in paths:
        rows = [r for r in ReadCsv(path) if r.get("metric")]
        if not rows:
            print(f"{os.path.basename(path):26s}  (no rows)")
            continue
        label = os.path.splitext(os.path.basename(path))[0]

        def Number(row, key):
            value = row.get(key, "")
            return np.nan if value in ("", None) else float(value)

        def Usable(row):
            if not np.isfinite(Number(row, "duration")):
                return False
            if Number(row, "worst_joint_limit_violation") > joint_limit_tolerance:
                return False
            if Number(row, "worst_reachability_violation") > reachability_tolerance:
                return False
            collision_free = Number(row, "collision_free_fraction")
            if not np.isfinite(collision_free):
                collision_free = Number(row, "feasible_fraction")
            return collision_free >= minimum_feasible_fraction

        setting_names = []
        for r in rows:
            if r["solver_setting"] not in setting_names:
                setting_names.append(r["solver_setting"])

        for setting_name in setting_names:
            subset = [r for r in rows if r["solver_setting"] == setting_name and Usable(r)]
            baseline = {(r["start"], r["goal"], r["trial"]): Number(r, "duration")
                        for r in subset if r["metric"] == "cspace"}
            for metric in metrics:
                mine = [r for r in subset if r["metric"] == metric]
                durations = np.array([Number(r, "duration") for r in mine])
                if len(durations) == 0:
                    continue
                pairs = [(Number(r, "duration"), baseline[(r["start"], r["goal"], r["trial"])])
                         for r in mine if (r["start"], r["goal"], r["trial"]) in baseline]
                binding = [r.get("binding_limit", "") for r in mine]
                most_binding = max(["velocity", "acceleration", "torque"], key=binding.count)
                if metric == "cspace" or not pairs:
                    comparison = f"{'--':>10s} {'--':>6s} {'--':>9s}"
                else:
                    a = np.array([x[0] for x in pairs])
                    b = np.array([x[1] for x in pairs])
                    try:
                        p_value = scipy.stats.wilcoxon(a, b).pvalue
                    except ValueError:
                        p_value = np.nan
                    comparison = (f"{100 * np.median((a - b) / b):+9.1f}% "
                                  f"{100 * np.mean(a < b):5.0f}% {p_value:9.2e}")
                print(f"{label:26s} {setting_name:9s} {metric:14s} {len(durations):4d} "
                      f"{np.median(durations):8.3f} {np.mean(durations):8.3f} {comparison} "
                      f"{most_binding:>13s}")
        print()


def SaveTrajectories(rows, path):
    """Writes the optimized control points to an .npz beside the CSV, so that
    any further downstream measure can be computed from the finished plans
    without re-solving. Keys match the CSV's identifying columns."""
    trajectories = {}
    for row in rows:
        control_points = row.pop("control_points", None)
        if control_points is None:
            continue
        key = f"{row['start']}_to_{row['goal']}_t{row['trial']}_{row['metric']}_{row['solver_setting']}"
        trajectories[key] = control_points
    if trajectories:
        np.savez_compressed(path, **trajectories)
    return len(trajectories)


def WriteCsv(rows, path):
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def ReadCsv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--jobs", type=int, default=min(os.cpu_count() or 1, 20),
                        help="number of worker processes")
    parser.add_argument("--trials", type=int, default=num_trials,
                        help="RRT plans per configuration pair")
    parser.add_argument("--smoke", action="store_true",
                        help="one pair, one trial, to check the pipeline before a full run")
    parser.add_argument("--solver", choices=["snopt", "ipopt"], default="snopt",
                        help="which nonlinear solver runs the trajectory optimization")
    parser.add_argument("--no-shortcut", action="store_true",
                        help="initialize from the raw RRT path instead of the shortcut one")
    parser.add_argument("--toppra-limits", choices=["all", "no-acceleration"], default="all",
                        help="which limits TOPPRA retimes against; 'no-acceleration' leaves "
                             "only the velocity and torque limits")
    parser.add_argument("--payload-mass", type=float, default=0.0,
                        help="mass in kg of an object held between the two grippers, split "
                             "evenly between them; changes the mass matrix and the torque "
                             "needed to move, but carries no collision geometry. 28kg is the "
                             "combined rated payload of the two IIWA-14 arms.")
    parser.add_argument("--torque-limit-scale", type=float, default=1.0,
                        help="scales the joint torque limits TOPPRA retimes against, e.g. 0.5 "
                             "to derate the hardware by half. Velocity and acceleration limits "
                             "are purely kinematic and bind on essentially every trajectory, so "
                             "derating torque is how to make it the binding constraint.")
    parser.add_argument("--label", default=None,
                        help="names the output CSV, so variations do not overwrite each other")
    parser.add_argument("--output", default=None)
    parser.add_argument("--analyze-only", default=None,
                        help="skip the experiment and re-print the summary from a CSV")
    parser.add_argument("--compare", nargs="+", default=None,
                        help="skip the experiment and put several runs' headline numbers "
                             "side by side")
    args = parser.parse_args()

    if args.compare:
        CompareRuns(args.compare)
        return

    config = {
        "solver": args.solver,
        "shortcut": not args.no_shortcut,
        "toppra_limits": args.toppra_limits,
        "payload_mass": args.payload_mass,
        "torque_limit_scale": args.torque_limit_scale,
    }
    if args.output is None:
        name = args.label if args.label else "trajopt_metric_comparison"
        args.output = os.path.join(common.RepoDir(), "results", name + ".csv")

    if args.analyze_only:
        Summarize(ReadCsv(args.analyze_only))
        return

    pairs = config_pairs
    trials = args.trials
    if args.smoke:
        pairs = config_pairs[:1]
        trials = 1

    jobs = [(start_name, goal_name, trial) for start_name, goal_name in pairs
            for trial in range(trials)]
    num_settings = len(solver_settings[config["solver"]])
    print(f"configuration: {config}", flush=True)
    print(f"{len(jobs)} RRT plans x {len(metrics)} metrics x {num_settings} solver "
          f"settings = {len(jobs) * len(metrics) * num_settings} trajectory "
          f"optimizations, on {args.jobs} workers", flush=True)

    start_time = time.time()
    rows = []
    if args.jobs == 1:
        InitializeWorker(config)
        for job in jobs:
            rows.extend(PlanAndOptimize(job))
    else:
        context = multiprocessing.get_context("spawn")
        with context.Pool(args.jobs, initializer=InitializeWorker, initargs=(config,)) as pool:
            for result in pool.imap_unordered(PlanAndOptimize, jobs):
                rows.extend(result)
    print(f"\nDone in {time.time() - start_time:.1f}s", flush=True)

    trajectory_path = os.path.splitext(args.output)[0] + "_trajectories.npz"
    os.makedirs(os.path.dirname(trajectory_path), exist_ok=True)
    num_saved = SaveTrajectories(rows, trajectory_path)
    WriteCsv(rows, args.output)
    print(f"Wrote {len(rows)} rows to {args.output}")
    print(f"Wrote {num_saved} trajectories to {trajectory_path}")
    Summarize(rows)


if __name__ == "__main__":
    main()
