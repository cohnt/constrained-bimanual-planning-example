import numpy as np
import pydrake.math
from pydrake.all import (
    AutoDiffXd,
    sin,
    cos,
    sqrt,
    atan2,
    RigidTransform_,
    AutoDiffXd
)

# See "Position-based kinematics for 7-DoF serial manipulators with global
# configuration control, joint limit and singularity avoidance" by Faria et. al.
# for details.

iiwa_alpha = np.array([
	-np.pi/2,
	np.pi/2,
	np.pi/2,
	-np.pi/2,
	-np.pi/2,
	np.pi/2,
	0
])
# NOTE: We're using the LBR iiwa 14 R820, so our values are slightly different
# than the report found here: https://zenodo.org/record/4063575
iiwa_d = np.array([
	0.36,
	0,
	0.42,
	0,
	0.4,
	0,
	0.126-0.045 # This adjustment is necessary to match drake. Probably due to a flange or something?
])
iiwa_limits_lower = np.array([
	-2.967060,
	-2.094395,
	-2.967060,
	-2.094395,
	-2.967060,
	-2.094395,
	-3.054326
])
iiwa_limits_upper = np.array([
	2.967060,
	2.094395,
	2.967060,
	2.094395,
	2.967060,
	2.094395,
	3.054326
])


def cross_product_matrix(a):
	# Returns a matrix, such that multiplication by a vector yields the cross product
	# See: https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
	# Taken from https://stackoverflow.com/questions/66707295/numpy-cross-product-matrix-function#comment117919990_66707295
	return np.cross(a, np.identity(a.shape[0]) * -1)

def scalar_clip(val, a, b):
	if type(val) == AutoDiffXd:
		a = AutoDiffXd(a, np.zeros(val.derivatives().shape))
		b = AutoDiffXd(b, np.zeros(val.derivatives().shape))

		# a = AutoDiffXd(a, np.full(val.derivatives().shape, -np.pi/2))
		# b = AutoDiffXd(b, np.full(val.derivatives().shape, np.pi/2))
		# a = AutoDiffXd(a, -val.derivatives())
		# b = AutoDiffXd(b, val.derivatives())

	return pydrake.math.max(
		a, pydrake.math.min(
			b, val
		)
	)

def safe_arccos(val, a, b, penalty=10.0):
	return pydrake.math.arccos(scalar_clip(val, a, b))

# Code yoinked from https://www.geeksforgeeks.org/find-intersection-of-intervals-given-by-two-lists/#
def intersect_intervals(arr1, arr2):
    # i and j pointers for arr1
    # and arr2 respectively
    i = j = 0
     
    n = len(arr1)
    m = len(arr2)

    out = []
 
    # Loop through all intervals unless one
    # of the interval gets exhausted
    while i < n and j < m:
         
        # Left bound for intersecting segment
        l = max(arr1[i][0], arr2[j][0])
         
        # Right bound for intersecting segment
        r = min(arr1[i][1], arr2[j][1])
         
        # If segment is valid print it
        if l <= r:
            out.append([l, r])
 
        # If i-th interval's right bound is
        # smaller increment i else increment j
        if arr1[i][1] < arr2[j][1]:
            i += 1
        else:
            j += 1

    return out
    # This code is contributed by sarthak_eddy


class Analytic_IK_7DoF():
	# Class that performs analytic forward and inverse kinematics for
	# a S-R-S 7-DoF manipulator.
	def __init__(self, alpha = iiwa_alpha, d = iiwa_d, limits_lower = iiwa_limits_lower, limits_upper = iiwa_limits_upper):
		# alpha and d are the DH parameters. We assume all other parameters are zero
		# limits_lower and limits_upper encode the joint limits
		# All arguments should be length seven numpy arrays
		assert len(alpha) == len(d) == len(limits_lower) == len(limits_upper) == 7
		assert d[1] == d[3] == d[5] == 0
		self.alpha = alpha.copy()
		self.d = d.copy()
		self.d_bs, self.d_se, self.d_ew, self.d_wf = d[0], d[2], d[4], d[6]
		self.limits_lower = limits_lower.copy()
		self.limits_upper = limits_upper.copy()

		self.Ts = [
			lambda ti, ai=ai, di=di : np.array([
				[pydrake.math.cos(ti), -pydrake.math.sin(ti)*pydrake.math.cos(ai), pydrake.math.sin(ti)*pydrake.math.sin(ai), 0],
				[pydrake.math.sin(ti), pydrake.math.cos(ti)*pydrake.math.cos(ai), -pydrake.math.cos(ti)*pydrake.math.sin(ai), 0],
				[0, pydrake.math.sin(ai), pydrake.math.cos(ai), di],
				[0, 0, 0, 1]
			])
			for ai, di in zip(self.alpha, self.d)
		]

	def FK(self, thetas):
		# thetas should be the joint angles
		# Returns the transform from the base frame B to the end effector frame E, X_EB
		# Also returns the global configuration parameters and elbow angle psi
		eval_Ts = [eval_T(t) for (eval_T, t) in zip(self.Ts, thetas)]
		full_mat = np.linalg.multi_dot(eval_Ts)
		return full_mat

	def GC(self, thetas):
		# Return the global configuration parameters for a given theta input
		ks = [1, 3, 5] # Note the difference with the paper due to zero-indexing
		return np.array([1 if thetas[k] >= 0 else -1 for k in ks])

	def psi(self, thetas, return_T_vs=False):
		# Returns the elbow parameter for a given configuration
		# psi in the set of self-motions. Given a config, this returns the specific self-motion parameter this falls under
		T_07 = self.FK(thetas)
		p_07 = T_07[:-1,-1]
		R_07 = T_07[:-1,:-1]
		GC2, GC4, GC6 = self.GC(thetas)

		p_02 = np.array([0, 0, self.d_bs])
		p_24 = np.array([0, self.d_se, 0])
		p_46 = np.array([0, 0, self.d_ew])
		p_67 = np.array([0, 0, self.d_wf])

		p_26 = p_07 - p_02 - (R_07 @ p_67)
		arccos_in = (np.dot(p_26, p_26) - self.d_se**2 - self.d_ew**2) / (2 * self.d_se * self.d_ew)
		theta_4v = GC4 * pydrake.math.arccos(scalar_clip(arccos_in, -1, 1))

		R_01 = self.Ts[0](thetas[0])[0:3,0:3]
		cross = np.cross(p_26, R_01[:,-1])
		cond = np.dot(cross, cross) > 0
		theta_1v = pydrake.math.arctan2(p_26[1], p_26[0]) if cond else 0

		arccos_in = (self.d_se**2 + np.dot(p_26, p_26) - self.d_ew**2) / (2 * self.d_se * np.linalg.norm(p_26))
		phi = pydrake.math.arccos(scalar_clip(arccos_in, -1, 1))
		theta_2v = pydrake.math.arctan2(np.linalg.norm(p_26[:2]), p_26[2]) + (GC4 * phi)

		theta_3v = 0
		theta_vs = [theta_1v, theta_2v, theta_3v, theta_4v]
		T_vs = [T(theta_v) for T, theta_v in zip(self.Ts[:len(theta_vs)], theta_vs)]
		T_02_v = np.linalg.multi_dot(T_vs[0:2])
		T_04_v = np.linalg.multi_dot(T_vs[0:4])
		p_02_v = T_02_v[:-1,-1]
		p_04_v = T_04_v[:-1,-1]

		Ts = [T(theta) for T, theta in zip(self.Ts, thetas)]
		T_04 = np.linalg.multi_dot(Ts[0:4])
		p_04 = T_04[:-1,-1]
		T_06 = np.linalg.multi_dot(Ts[0:6])
		p_06 = T_06[:-1,-1]
		p_06_v = p_06

		v_se_v = (p_04_v - p_02_v) / np.linalg.norm(p_04_v - p_02_v)
		v_sw_v = (p_06_v - p_02_v) / np.linalg.norm(p_06_v - p_02_v)
		v_sew_v = np.cross(v_se_v, v_sw_v)
		v_sew_v_hat = v_sew_v / np.linalg.norm(v_sew_v)

		v_se = (p_04 - p_02) / np.linalg.norm(p_04 - p_02)
		v_sw = (p_06 - p_02) / np.linalg.norm(p_06 - p_02)
		v_sew = np.cross(v_se, v_sw)
		v_sew_hat = v_sew / np.linalg.norm(v_sew)

		sg_psi = np.sign(np.dot(np.cross(v_sew_v_hat, v_sew_hat), p_26))
		psi = sg_psi * pydrake.math.arccos(np.dot(v_sew_v_hat, v_sew_hat))

		if return_T_vs:
			return psi, T_vs
		else:
			return psi

	def IK(self, rigid_transform, GC, psi, return_unclipped_vals=False, return_singularity_vals=False, return_sw_mats=False, clip_stepback= 1e-6):
		# Set check_clip to 1, 2, 3, or 4 to look at the input to the arccos function
		# Given rigid_transform, and a psi (and a GC input which is like a 3 element +-1 array), compute thetas
		clip = 1 - clip_stepback

		assert not (return_unclipped_vals and return_singularity_vals)
		ad = isinstance(rigid_transform, RigidTransform_[AutoDiffXd])

		rigid_transform = rigid_transform.GetAsMatrix4()

		if ad:
			thetas = np.array([AutoDiffXd(0) for _ in range(7)])
			unclipped_vals = np.array([AutoDiffXd(0) for _ in range(4)])
			singularity_vals = np.array([AutoDiffXd(0) for _ in range(2)])
		else:
			thetas = np.zeros(7)
			unclipped_vals = np.zeros(4)
			singularity_vals = np.zeros(2)

		GC2, GC4, GC6 = GC

		p_02 = np.array([0, 0, self.d_bs])
		p_24 = np.array([0, self.d_se, 0])
		p_46 = np.array([0, 0, self.d_ew])
		p_67 = np.array([0, 0, self.d_wf])

		p_07 = rigid_transform[:-1,-1]
		R_07 = rigid_transform[:-1,:-1]
		p_26 = p_07 - p_02 - (R_07 @ p_67) # EQ (3)
		p_26_hat = p_26 / np.linalg.norm(p_26)

		theta_1v = pydrake.math.arctan2(p_26[1], p_26[0]) # EQ (5)

		# EQ (7)
		arccos_in = (self.d_se**2 + np.dot(p_26, p_26) - self.d_ew**2) / (2 * self.d_se * np.linalg.norm(p_26))
		unclipped_vals[0] = arccos_in
		phi = safe_arccos(arccos_in, -clip, clip)
		theta_2v = pydrake.math.arctan2(np.linalg.norm(p_26[:2]), p_26[2]) + (GC4 * phi)

		theta_3v = 0
		
		# EQ (4)
		arccos_in = (np.dot(p_26, p_26) - self.d_se**2 - self.d_ew**2) / (2 * self.d_se * self.d_ew)
		unclipped_vals[1] = arccos_in
		theta_4v = GC4 * safe_arccos(arccos_in, -clip, clip)
		thetas[3] = theta_4v

		theta_vs = [theta_1v, theta_2v, theta_3v, theta_4v]
		T_vs = [T(theta_v) for T, theta_v in zip(self.Ts[:len(theta_vs)], theta_vs)]
		T_03_v = np.linalg.multi_dot(T_vs[0:3])
		R_03_v = T_03_v[:-1,:-1]

		# EQ (15)
		cprod_p_26 = cross_product_matrix(p_26_hat)
		A_s = cprod_p_26 @ R_03_v
		B_s = -1 * cprod_p_26 @ cprod_p_26 @ R_03_v
		C_s = np.outer(p_26_hat, p_26_hat) @ R_03_v

		# EQ (17)-(19)
		thetas[0] = pydrake.math.arctan2(
			GC2 * (A_s[1,1] * pydrake.math.sin(psi) + B_s[1,1] * pydrake.math.cos(psi) + C_s[1,1]),
			GC2 * (A_s[0,1] * pydrake.math.sin(psi) + B_s[0,1] * pydrake.math.cos(psi) + C_s[0,1])
		)
		arccos_in = A_s[2,1] * pydrake.math.sin(psi) + B_s[2,1] * pydrake.math.cos(psi) + C_s[2,1]
		unclipped_vals[2] = arccos_in
		thetas[1] = GC2 * safe_arccos(arccos_in, -clip, clip)
		thetas[2] = pydrake.math.arctan2(
			GC2 * (-A_s[2,2] * pydrake.math.sin(psi) - B_s[2,2] * pydrake.math.cos(psi) - C_s[2,2]),
			GC2 * (-A_s[2,0] * pydrake.math.sin(psi) - B_s[2,0] * pydrake.math.cos(psi) - C_s[2,0])
		)

		# EQ (20)
		T_34 = T_vs[3]
		R_34 = T_34[:-1,:-1]
		A_w = R_34.T @ A_s.T @ R_07
		B_w = R_34.T @ B_s.T @ R_07
		C_w = R_34.T @ C_s.T @ R_07

		# EQ (22)-(24)
		thetas[4] = pydrake.math.arctan2(
			GC6 * (A_w[1,2] * pydrake.math.sin(psi) + B_w[1,2] * pydrake.math.cos(psi) + C_w[1,2]),
			GC6 * (A_w[0,2] * pydrake.math.sin(psi) + B_w[0,2] * pydrake.math.cos(psi) + C_w[0,2])
		)
		arccos_in = A_w[2,2] * pydrake.math.sin(psi) + B_w[2,2] * pydrake.math.cos(psi) + C_w[2,2]
		unclipped_vals[3] = arccos_in
		thetas[5] = GC6 * safe_arccos(arccos_in, -clip, clip)
		thetas[6] = pydrake.math.arctan2(
			GC6 * (A_w[2,1] * pydrake.math.sin(psi) + B_w[2,1] * pydrake.math.cos(psi) + C_w[2,1]),
			GC6 * (-A_w[2,0] * pydrake.math.sin(psi) - B_w[2,0] * pydrake.math.cos(psi) - C_w[2,0])
		)

		if return_unclipped_vals:
			return unclipped_vals
		if return_singularity_vals:
			return thetas[[3, 5]]
		else:
			if return_sw_mats:
				return np.asarray(thetas), A_s, B_s, C_s, A_w, B_w, C_w
			return thetas
		
	def gripper_ik(self, rigid_transform, GC, psi, return_unclipped_vals=False, return_singularity_vals=False, return_sw_mats=False, clip_stepback= 1e-3, finray = False):
		ad = isinstance(rigid_transform, RigidTransform_[AutoDiffXd])
		type_used = AutoDiffXd if ad else float
		T = RigidTransform_[type_used](np.array([
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 1, 0, 0.135],
                [0, 0, 0, 1]
            ]))
		if finray:
			T = RigidTransform_[type_used](np.array([
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 1, 0, 0.185],
                [0, 0, 0, 1]
            ]))
		T_gripper = rigid_transform.multiply(T.inverse())
		# T_gripper = T.inverse().multiply(pose)
		return self.IK(rigid_transform=T_gripper, psi=psi, GC=GC, clip_stepback=clip_stepback, return_unclipped_vals=return_unclipped_vals, return_singularity_vals=return_singularity_vals)


	def psi_interval(self, rigid_transform, GC, sing_detection_barrier=1e-12, sing_delta=1e-5):
		# Computes the interval of values psi can attain while avoiding joint limits and singularities
		GC2, GC4, GC6 = GC
		theta_mins = np.zeros(7)
		theta_maxs = np.zeros(7)
		pivot_joints = [0, 2, 4, 6]
		hinge_joints = [1, 5]

		# The elbow does not matter for this
		theta_mins[3] = -np.inf
		theta_maxs[3] = np.inf

		thetas, A_s, B_s, C_s, A_w, B_w, C_w = self.IK(rigid_transform, GC, 0, return_sw_mats=True)
		a_n = [A_s[1,1], None, -A_s[2,2], None, A_w[1,2], None, A_w[2,1]]
		b_n = [B_s[1,1], None, -B_s[2,2], None, B_w[1,2], None, B_w[2,1]]
		c_n = [C_s[1,1], None, -C_s[2,2], None, C_w[1,2], None, C_w[2,1]]
		a_d = [A_s[0,1], None, -A_s[2,0], None, A_w[0,2], None, -A_w[2,0]]
		b_d = [B_s[0,1], None, -B_s[2,0], None, B_w[0,2], None, -B_w[2,0]]
		c_d = [C_s[0,1], None, -C_s[2,0], None, C_w[0,2], None, -C_w[2,0]]
		a = [None, A_s[2,1], None, None, None, A_w[2,2], None]
		b = [None, B_s[2,1], None, None, None, B_w[2,2], None]
		c = [None, C_s[2,1], None, None, None, C_w[2,2], None]
		GCk = [GC2, GC2, GC2, None, GC6, GC6, GC6]

		psi_intervals = [[[-np.pi, np.pi]] for _ in range(7)]
		for i in pivot_joints:
			# print("i", i)
			at = GCk[i] * (c_n[i]*b_d[i] - b_n[i]*c_d[i])
			bt = GCk[i] * (a_n[i]*c_d[i] - c_n[i]*a_d[i])
			ct = GCk[i] * (a_n[i]*b_d[i] - b_n[i]*a_d[i])

			# Check for singularities
			sing_intervals = []
			sing_disc = at**2 + bt**2 - ct**2
			if np.abs(sing_disc) <= sing_detection_barrier:
				psi_sing = 2 * np.arctan(at / (bt - ct))
				# print("Joint %d singularity at psi=%f" % (i, psi_sing))
				start = np.max([-np.pi, psi_sing - sing_delta])
				end = np.min([np.pi, psi_sing + sing_delta])
				if start > -np.pi:
					sing_intervals.append([-np.pi, start])
				if end < np.pi:
					sing_intervals.append([end, np.pi])
			else:
				sing_intervals.append([-np.pi, np.pi])

			# Check joint limits
			limit_intervals = []
			ap = lambda theta : GCk[i] * ((c_d[i] - b_d[i]) * np.tan(theta) + (b_n[i] - c_n[i]))
			bp = lambda theta : 2 * GCk[i] * (a_d[i] * np.tan(theta) - a_n[i])
			cp = lambda theta : GCk[i] * ((b_d[i] + c_d[i]) * np.tan(theta) - (b_n[i] + c_n[i]))
			psi_disc = lambda theta : bp(theta)**2 - 4 * ap(theta) * cp(theta)
			psi_of_theta = lambda theta, sign : 2 * np.arctan((-bp(theta) + sign * np.sqrt(psi_disc(theta))) / (2 * ap(theta))) # sign should be 1 or -1
			if psi_disc(self.limits_lower[i]) < 0 and psi_disc(self.limits_upper[i]) < 0:
				# In this case, either all of [-pi, pi] is good or it's all bad
				# It's sufficient to check any psi value, so we use psi=0, since it's already
				# computed above.
				if thetas[i] >= self.limits_lower[i] and thetas[i] <= self.limits_upper[i]:
					limit_intervals.append([-np.pi, np.pi])
			else:
				# There are critical points, so we have to check them
				crit_thetas = []
				crit_points = []
				if psi_disc(self.limits_lower[i]) == 0:
					crit_thetas.append(self.limits_lower[i])
					crit_points.append(psi_of_theta(self.limits_lower[i], 1))
				elif psi_disc(self.limits_lower[i]) > 0:
					crit_thetas.append(self.limits_lower[i])
					crit_thetas.append(self.limits_lower[i])
					crit_points.append(psi_of_theta(self.limits_lower[i], 1))
					crit_points.append(psi_of_theta(self.limits_lower[i], -1))
				if psi_disc(self.limits_upper[i]) == 0:
					crit_thetas.append(self.limits_upper[i])
					crit_points.append(psi_of_theta(self.limits_upper[i], 1))
				elif psi_disc(self.limits_upper[i]) > 0:
					crit_thetas.append(self.limits_upper[i])
					crit_thetas.append(self.limits_upper[i])
					crit_points.append(psi_of_theta(self.limits_upper[i], 1))
					crit_points.append(psi_of_theta(self.limits_upper[i], -1))

				# TODO: Figure out why the Weierstrass substitution psi_of_theta returns suprious values
				comp_thetas = [pydrake.math.arctan2(GCk[i] * (a_n[i] * pydrake.math.sin(psi) + b_n[i] * pydrake.math.cos(psi) + c_n[i]),
					                      GCk[i] * (a_d[i] * pydrake.math.sin(psi) + b_d[i] * pydrake.math.cos(psi) + c_d[i])) for psi in crit_points]

				# print("")
				# for psi in crit_points:
				# 	print(self.IK(rigid_transform, GC, psi)[i])
				# print("")

				sorted_idx = np.argsort(crit_points)
				crit_thetas = np.asarray(crit_thetas)[sorted_idx]
				comp_thetas = np.asarray(comp_thetas)[sorted_idx]
				crit_points = np.asarray(crit_points)[sorted_idx]

				keep_idx = np.abs(np.asarray(crit_thetas) - np.asarray(comp_thetas)) < 1e-12
				# print(keep_idx)
				crit_thetas = crit_thetas[keep_idx]
				comp_thetas = comp_thetas[keep_idx]
				crit_points = crit_points[keep_idx]
				if len(crit_thetas) == 0:
					limit_intervals.append([-np.pi, np.pi])
				else:
					dtheta_dpsi_sign = lambda psi : np.sign(at * pydrake.math.sin(psi) + bt * pydrake.math.cos(psi) + ct)
					deriv_signs = [dtheta_dpsi_sign(psi) for psi in crit_points]
					# print(crit_points)
					# print(crit_thetas)
					# print(comp_thetas)
					# print(deriv_signs)
					next_interval_good = GCk[i] * np.sign(comp_thetas) != deriv_signs # TODO: This works, but opposite signs should be correct. Why?
					if not next_interval_good[0]:
						limit_intervals.append([-np.pi, crit_points[0]])
					for j in range(len(crit_points)-1):
						if next_interval_good[j]:
							limit_intervals.append([crit_points[j], crit_points[j+1]])
					if next_interval_good[-1]:
						limit_intervals.append([crit_points[-1], np.pi])

			# Combine intervals
			psi_intervals[i] = intersect_intervals(sing_intervals, limit_intervals)
			# print(sing_intervals)
			# print(limit_intervals)
			# print(psi_intervals[i])
			# print()

		for i in hinge_joints:
			# print("i", i)
			# Don't need to worry about singularities
			sing_intervals = [[-np.pi, np.pi]]

			# Limit intervals
			limit_intervals = []
			disc = lambda theta : a[i]**2 + b[i]**2 - (c[i] - pydrake.math.cos(theta))**2
			# print(self.limits_lower[i], disc(self.limits_lower[i]))
			# print(self.limits_upper[i], disc(self.limits_upper[i]))
			if disc(self.limits_lower[i]) < 0 and disc(self.limits_upper[i]) < 0:
				# In this case, either all of [-pi, pi] is good or it's all bad
				# It's sufficient to check any psi value, so we use psi=0, since it's already
				# computed above.
				if thetas[i] >= self.limits_lower[i] and thetas[i] <= self.limits_upper[i]:
					limit_intervals.append([-np.pi, np.pi])
			else:
				crit_thetas = []
				crit_points = []
				for theta in [self.limits_lower[i], self.limits_upper[i]]:
					if disc(theta) == 0:
						crit_thetas.append(theta)
						crit_points.append(2 * np.arctan(a[i] / (pydrake.math.cos(theta) + b[i] - c[i])))
					elif disc(theta) > 0:
						crit_thetas.append(theta)
						crit_thetas.append(theta)
						crit_points.append(2 * np.arctan((a[i] + np.sqrt(disc(theta))) / (pydrake.math.cos(theta) + b[i] - c[i])))
						crit_points.append(2 * np.arctan((a[i] - np.sqrt(disc(theta))) / (pydrake.math.cos(theta) + b[i] - c[i])))

				# TODO: Figure out why the Weierstrass substitution psi_of_theta returns suprious values
				comp_thetas = [GCk[i] * pydrake.math.arccos(a[i] * pydrake.math.sin(psi) + b[i] * pydrake.math.cos(psi) + c[i]) for psi in crit_points]

				sorted_idx = np.argsort(crit_points)
				crit_thetas = np.asarray(crit_thetas)[sorted_idx]
				comp_thetas = np.asarray(comp_thetas)[sorted_idx]
				crit_points = np.asarray(crit_points)[sorted_idx]

				keep_idx = np.abs(np.asarray(crit_thetas) - np.asarray(comp_thetas)) < 1e-12
				# print(keep_idx)
				crit_thetas = crit_thetas[keep_idx]
				comp_thetas = comp_thetas[keep_idx]
				crit_points = crit_points[keep_idx]
				if len(crit_thetas) == 0:
					limit_intervals.append([-np.pi, np.pi])
				else:
					dtheta_dpsi_sign = lambda psi, theta : np.sign(-GCk[i] * (a[i] * pydrake.math.cos(psi) - b[i] * pydrake.math.sin(psi)) / pydrake.math.sin(theta))
					deriv_signs = [dtheta_dpsi_sign(psi, theta) for psi, theta in zip(crit_points, crit_thetas)]

					next_interval_good = GCk[i] * np.sign(crit_thetas) != deriv_signs # TODO: This works, but opposite signs should be correct. Why?
					if not next_interval_good[0]:
						limit_intervals.append([-np.pi, crit_points[0]])
					for j in range(len(crit_points)-1):
						if next_interval_good[j]:
							limit_intervals.append([crit_points[j], crit_points[j+1]])
					if next_interval_good[-1]:
						limit_intervals.append([crit_points[-1], np.pi])

			# Combine intervals
			psi_intervals[i] = intersect_intervals(sing_intervals, limit_intervals)
			# print(psi_intervals[i])
			# print()

		# for i in range(len(psi_intervals)):
		# 	print(psi_intervals[i])
		# print()

		# Compute overall interval intersection
		combined_intervals = [[-np.pi, np.pi]]
		for i in range(len(psi_intervals)):
			# print(combined_intervals)
			# print(psi_intervals[i])
			# print()
			combined_intervals = intersect_intervals(combined_intervals, psi_intervals[i])
		# print(combined_intervals)

		# Combine adjacent intervals?
		# TODO: Figure out if we should do this

		return combined_intervals