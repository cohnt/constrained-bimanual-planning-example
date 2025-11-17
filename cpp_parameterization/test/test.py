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