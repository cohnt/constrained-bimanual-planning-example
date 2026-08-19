import sys
sys.path.append("./python")

from iiwa_ik import IiwaBimanualReachableConstraint, IiwaBimanualJointLimitConstraint, IiwaBimanualCollisionFreeConstraint, FullFeasibilityConstraint
from pydrake.all import Constraint
import numpy as np

print(isinstance(IiwaBimanualReachableConstraint(True, True, True, 0.6), Constraint))
print(isinstance(IiwaBimanualJointLimitConstraint(-np.zeros(7), np.zeros(7), True, True, True, 0.6), Constraint))

from iiwa_ik import MakeParameterization
from pydrake.all import IrisParameterizationFunction

print(isinstance(MakeParameterization(True, True, True, 0.6), IrisParameterizationFunction))
print(MakeParameterization(True, True, True, 0.6).get_parameterization_is_threadsafe())

parameterization = MakeParameterization(True, True, True, 0.6)
parameterization.get_parameterization_double()(np.zeros(8))
# --------------------------------------------------
# Costs.

import os

from iiwa_ik import IiwaBimanualPathCost, IiwaBimanualKineticEnergyPathCost
from pydrake.all import (
    Cost,
    ExtractGradient,
    ExtractValue,
    InitializeAutoDiff,
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
    RobotDiagramBuilder,
)

def Check(label, passed):
    """Prints True/False like the checks above, but labelled -- there are
    enough checks below that an unlabelled False is hard to track down."""
    print(bool(passed), "\t", label)

repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

shoulder_up, elbow_up, wrist_up = True, True, False
grasp_distance = 0.6
num_positions = 8
num_control_points = 5

# The kinetic energy cost needs a plant to get the mass matrix from.
builder = RobotDiagramBuilder(time_step=0.0)
plant = builder.plant()
parser = Parser(plant)
parser.package_map().AddPackageXml(os.path.join(repo_dir, "package.xml"))
ProcessModelDirectives(LoadModelDirectives(os.path.join(repo_dir, "models/old_shelves.dmd.yaml")), parser)
plant.Finalize()
plant_context = plant.CreateDefaultContext()

path_cost = IiwaBimanualPathCost(num_positions, num_control_points, shoulder_up, elbow_up, wrist_up, grasp_distance, True)
path_length_cost = IiwaBimanualPathCost(num_positions, num_control_points, shoulder_up, elbow_up, wrist_up, grasp_distance, False)
kinetic_energy_cost = IiwaBimanualKineticEnergyPathCost(num_positions, num_control_points, shoulder_up, elbow_up, wrist_up, grasp_distance, plant)

Check("IiwaBimanualPathCost is a Cost", isinstance(path_cost, Cost))
Check("IiwaBimanualKineticEnergyPathCost is a Cost", isinstance(kinetic_energy_cost, Cost))

# The parameterization is only defined on part of the domain, so sample control
# points near a configuration that is known to be reachable.
q_tilde_middle = np.array([-0.5997312520566763, 1.489780849654964, -1.4739679827359913, 1.2905366081785483, -0.04421061906813227, -0.8793712572715165, -1.1603461715511334, 1.45])
np.random.seed(0)
control_points = q_tilde_middle.reshape(-1, 1) + 0.1 * np.random.randn(num_positions, num_control_points)
x = control_points.flatten()

parameterization_double = MakeParameterization(shoulder_up, elbow_up, wrist_up, grasp_distance).get_parameterization_double()

def ReferenceCost(control_points, mass_matrix_at):
    """NumPy reimplementation of the path costs. mass_matrix_at maps a full
    (14-dimensional) configuration to the metric tensor to use."""
    total = 0.0
    for i in range(1, control_points.shape[1]):
        q_full_0 = parameterization_double(control_points[:, i-1])
        q_full_1 = parameterization_double(control_points[:, i])
        q_full_mid = parameterization_double(0.5 * (control_points[:, i-1] + control_points[:, i]))
        delta = q_full_1 - q_full_0
        total += delta @ mass_matrix_at(q_full_mid) @ delta
    return total

def MassMatrix(q_full):
    plant.SetPositions(plant_context, q_full)
    return plant.CalcMassMatrix(plant_context)

# The kinetic energy cost matches an independent implementation.
Check("kinetic energy cost value", np.abs(kinetic_energy_cost.Eval(x)[0] - ReferenceCost(control_points, MassMatrix)) < 1e-9)

# With the metric tensor replaced by the identity, the same implementation
# reduces to the C-space path energy cost. (This checks that the two costs
# agree on how the flattened control points are laid out and paired up.)
Check("path cost value (identity metric)", np.abs(path_cost.Eval(x)[0] - ReferenceCost(control_points, lambda q_full: np.eye(14))) < 1e-9)

# The gradients are correct. This is the part most likely to be wrong, since
# the kinetic energy cost chains a small local AutoDiff computation into the
# full gradient rather than propagating every partial derivative through the
# mass matrix.
def NumericalGradient(cost, x, eps=1e-6):
    gradient = np.zeros(len(x))
    for i in range(len(x)):
        x_plus, x_minus = x.copy(), x.copy()
        x_plus[i] += eps
        x_minus[i] -= eps
        gradient[i] = (cost.Eval(x_plus)[0] - cost.Eval(x_minus)[0]) / (2 * eps)
    return gradient

# The unsquared (path length) form takes a different gradient branch, so it is
# checked too.
def ReferenceLength(control_points):
    total = 0.0
    for i in range(1, control_points.shape[1]):
        q_full_0 = parameterization_double(control_points[:, i-1])
        q_full_1 = parameterization_double(control_points[:, i])
        total += np.linalg.norm(q_full_1 - q_full_0)
    return total

Check("path length cost value", np.abs(path_length_cost.Eval(x)[0] - ReferenceLength(control_points)) < 1e-9)

for name, cost in [("path cost", path_cost), ("path length cost", path_length_cost),
                   ("kinetic energy cost", kinetic_energy_cost)]:
    y_autodiff = cost.Eval(InitializeAutoDiff(x))
    Check(name + " AutoDiff value", np.abs(ExtractValue(y_autodiff)[0, 0] - cost.Eval(x)[0]) < 1e-9)
    analytic_gradient = ExtractGradient(y_autodiff).flatten()
    Check(name + " AutoDiff gradient", np.max(np.abs(analytic_gradient - NumericalGradient(cost, x))) < 1e-6)

# The scale factor is applied, and defaults to 1.0 so that existing callers are
# unaffected.
scale = 3.5
scaled_path_cost = IiwaBimanualPathCost(num_positions, num_control_points, shoulder_up, elbow_up, wrist_up, grasp_distance, True, scale)
scaled_kinetic_energy_cost = IiwaBimanualKineticEnergyPathCost(num_positions, num_control_points, shoulder_up, elbow_up, wrist_up, grasp_distance, plant, scale)
Check("path cost scale", np.abs(scaled_path_cost.Eval(x)[0] - scale * path_cost.Eval(x)[0]) < 1e-9)
Check("kinetic energy cost scale", np.abs(scaled_kinetic_energy_cost.Eval(x)[0] - scale * kinetic_energy_cost.Eval(x)[0]) < 1e-9)
Check("kinetic energy cost scale (gradient)", np.max(np.abs(ExtractGradient(scaled_kinetic_energy_cost.Eval(InitializeAutoDiff(x))) - scale * ExtractGradient(kinetic_energy_cost.Eval(InitializeAutoDiff(x))))) < 1e-9)

# The kinetic energy cost holds mutable contexts, so it must not advertise
# itself as thread safe.
Check("path cost is thread safe", path_cost.is_thread_safe())
Check("kinetic energy cost is not thread safe", not kinetic_energy_cost.is_thread_safe())
