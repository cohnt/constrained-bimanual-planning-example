import sys
sys.path.append("./python")

from iiwa_ik import safe_arccos
from pydrake.all import AutoDiffXd

print(safe_arccos(3, -1, 1))
x = AutoDiffXd(3)
print(safe_arccos(x, -1, 1))

from iiwa_ik import IiwaBimanualReachableConstraint, IiwaBimanualJointLimitConstraint
from pydrake.all import Constraint
import numpy as np

print(isinstance(IiwaBimanualReachableConstraint(True, True, True), Constraint))
print(isinstance(IiwaBimanualJointLimitConstraint(-np.zeros(7), np.zeros(7), True, True, True), Constraint))

from iiwa_ik import MakeParameterization
from pydrake.all import IrisParameterizationFunction

print(isinstance(MakeParameterization(True, True, True), IrisParameterizationFunction))
print(MakeParameterization(True, True, True).get_parameterization_is_threadsafe())