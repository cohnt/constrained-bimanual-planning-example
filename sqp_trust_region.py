import time
import numpy as np
import scipy.sparse as sp
import osqp
from pydrake.solvers import MathematicalProgram, OsqpSolver, SolutionResult
from pydrake.autodiffutils import InitializeAutoDiff, ExtractGradient, ExtractValue

def compute_cost_and_grad(prog, x):
    """
    Evaluates cost f(x) and cost gradient g(x) = grad f(x) using Drake Native C++ AutoDiff.
    """
    costs = prog.GetAllCosts()
    n = len(x)
    f_0 = 0.0
    grad = np.zeros(n)

    for b in costs:
        idx = prog.FindDecisionVariableIndices(b.variables())
        evaluator = b.evaluator()
        x_sub = x[idx]
        x_ad = InitializeAutoDiff(x_sub)
        y_ad = evaluator.Eval(x_ad)
        
        f_0 += float(np.sum(ExtractValue(y_ad)))
        J_sub = ExtractGradient(y_ad)
        if J_sub.ndim == 1:
            grad[idx] += J_sub
        else:
            grad[idx] += np.sum(J_sub, axis=0)

    return f_0, grad

def compute_constraints_and_jacobians(prog, x):
    """
    Evaluates all constraints c(x) and their Jacobians J = dc/dx using Drake Native C++ AutoDiff.
    Returns a list of dicts containing linearized constraint data.
    """
    constraints = prog.GetAllConstraints()
    linearized_constraints = []

    for b in constraints:
        idx = prog.FindDecisionVariableIndices(b.variables())
        evaluator = b.evaluator()
        x_sub = x[idx]
        
        x_ad = InitializeAutoDiff(x_sub)
        y_ad = evaluator.Eval(x_ad)

        val_0 = ExtractValue(y_ad).flatten()
        J_sub = ExtractGradient(y_ad)
        if J_sub.ndim == 1:
            J_sub = J_sub.reshape(1, -1)

        lb = evaluator.lower_bound()
        ub = evaluator.upper_bound()

        linearized_constraints.append({
            "indices": idx,
            "val_0": val_0,
            "J_sub": J_sub,
            "lb": lb,
            "ub": ub
        })

    return linearized_constraints

def eval_feasibility(prog, x):
    """
    Evaluates max constraint violation magnitude at x.
    """
    constraints = prog.GetAllConstraints()
    max_viol = 0.0
    satisfied = True
    for b in constraints:
        idx = prog.FindDecisionVariableIndices(b.variables())
        val = b.evaluator().Eval(x[idx])
        lb, ub = b.evaluator().lower_bound(), b.evaluator().upper_bound()
        if np.any(val < lb - 1e-4) or np.any(val > ub + 1e-4):
            satisfied = False
            m1 = np.max(lb - val) if np.any(val < lb) else 0.0
            m2 = np.max(val - ub) if np.any(val > ub) else 0.0
            max_viol = max(max_viol, m1, m2)
    return satisfied, max_viol

def compute_l1_violation(prog, x):
    constraints = prog.GetAllConstraints()
    l1_viol = 0.0
    for b in constraints:
        idx = prog.FindDecisionVariableIndices(b.variables())
        val = b.evaluator().Eval(x[idx])
        lb, ub = b.evaluator().lower_bound(), b.evaluator().upper_bound()
        v_lb = np.maximum(0.0, lb - val)
        v_ub = np.maximum(0.0, val - ub)
        l1_viol += float(np.sum(v_lb + v_ub))
    return l1_viol

def compute_exact_cost_hessian(prog, n, eps=1e-5):
    """
    Computes exact cost Hessian H = grad^2 f(x) for quadratic trajectory costs.
    """
    costs = prog.GetAllCosts()
    H = np.zeros((n, n))
    
    for b in costs:
        idx = prog.FindDecisionVariableIndices(b.variables())
        evaluator = b.evaluator()
        m_vars = len(idx)
        H_sub = np.zeros((m_vars, m_vars))
        x_sub = np.zeros(m_vars)
        f0 = np.sum(evaluator.Eval(x_sub))
        for i in range(m_vars):
            for j in range(i, m_vars):
                x_ij = x_sub.copy()
                x_ij[i] += eps
                x_ij[j] += eps
                f_ij = np.sum(evaluator.Eval(x_ij))

                x_i = x_sub.copy()
                x_i[i] += eps
                f_i = np.sum(evaluator.Eval(x_i))

                x_j = x_sub.copy()
                x_j[j] += eps
                f_j = np.sum(evaluator.Eval(x_j))

                hij = float((f_ij - f_i - f_j + f0) / (eps * eps))
                H_sub[i, j] = hij
                H_sub[j, i] = hij
        
        for i_local, i_global in enumerate(idx):
            for j_local, j_global in enumerate(idx):
                H[i_global, j_global] += H_sub[i_local, j_local]
                
    eigvals, eigvecs = np.linalg.eigh(H)
    eigvals_clamped = np.maximum(eigvals, 1e-4)
    H_psd = eigvecs @ np.diag(eigvals_clamped) @ eigvecs.T
    return 0.5 * (H_psd + H_psd.T) + 1e-4 * np.eye(n)

def update_damped_bfgs(H, s, y):
    """
    Powell's Damped BFGS update.
    Guarantees H_{k+1} is symmetric positive definite using ONLY 1st-order gradients.
    """
    sy = np.dot(s, y)
    Hs = H @ s
    sHs = np.dot(s, Hs)

    if sHs <= 1e-12:
        return H

    if sy >= 0.2 * sHs:
        theta = 1.0
        r = y
    else:
        theta = (0.8 * sHs) / (sHs - sy)
        r = theta * y + (1.0 - theta) * Hs

    sr = np.dot(s, r)
    if sr <= 1e-12:
        return H

    H_next = H + np.outer(r, r) / sr - np.outer(Hs, Hs) / sHs
    return 0.5 * (H_next + H_next.T)

def update_projected_nullspace_bfgs(H_k, s_k, y_L, J_active, H_cost_baseline, n):
    """
    Projected Null-Space BFGS Update with Positive-Definite Eigenvalue Safeguard.
    Applies Damped BFGS and projects the update delta into the null space of active constraints
    using the projection operator P_null = I - J^T (J J^T)^+ J.
    """
    m_act = J_active.shape[0] if J_active is not None else 0
    H_bfgs = update_damped_bfgs(H_k, s_k, y_L)
    delta_H = H_bfgs - H_k

    if m_act == 0:
        H_res = H_bfgs
    else:
        try:
            JJt = J_active @ J_active.T
            JJt_inv = np.linalg.pinv(JJt, rcond=1e-4)
            P_range = J_active.T @ JJt_inv @ J_active
            P_null = np.eye(n) - P_range
            H_res = H_k + P_null @ delta_H @ P_null.T
        except Exception:
            H_res = H_bfgs

    # Guarantee positive definiteness for OSQP subproblem P matrix
    H_res = 0.5 * (H_res + H_res.T)
    min_eig = float(np.min(np.linalg.eigvalsh(H_res)))
    if min_eig < 1e-4:
        H_res += (1e-4 - min_eig) * np.eye(n)
    return H_res

def compute_soc_step(prog, x_k, p_val, H_k, g_k, mu_vec, delta_k, prob_instance, prev_sol_x, prev_sol_y, N_qp, n, m_total):
    """
    Second-Order Correction (SOC) step.
    Given a rejected trial point x_k + p_val, evaluates the actual nonlinear constraint
    residuals c(x_k + p_val) and solves a new QP with corrected RHS bounds:
        l_i - c(x_k + p) <= J(x_k) p_soc + v_lb - v_ub <= u_i - c(x_k + p)
    The corrected total step is p_val + p_soc, bounded to the same trust region.
    """
    try:
        x_cand = x_k + p_val
        lin_cand = compute_constraints_and_jacobians(prog, x_cand)

        num_slacks = 2 * m_total
        N_qp_soc = n + num_slacks

        # Keep same quadratic cost but zero linear cost (SOC step minimizes constraint violation)
        P_diag = np.concatenate([np.diag(H_k), 1e-4 * np.ones(num_slacks)])
        P_mat = sp.diags(P_diag, format='csc')

        mu_slacks = np.concatenate([mu_vec, mu_vec])
        # Use same g_k gradient — SOC corrects constraint nonlinearity, not cost
        q_soc = np.concatenate([g_k, mu_slacks])

        # Trust region bounds (same delta): p_soc is bounded, total p_val+p_soc <= 2*delta is allowed
        A_bounds = sp.eye(N_qp_soc, format='csc')
        l_bounds = np.concatenate([-delta_k * np.ones(n), np.zeros(num_slacks)])
        u_bounds = np.concatenate([delta_k * np.ones(n), np.full(num_slacks, np.inf)])

        A_lin_list = []
        l_lin_list = []
        u_lin_list = []

        s_idx = 0
        for lc in lin_cand:
            idx = lc["indices"]
            c_cand = lc["val_0"]  # actual c(x_k + p), not linearized
            J_sub = lc["J_sub"]
            # Corrected RHS: shift by actual nonlinear residual at x_k + p
            lb_rel = lc["lb"] - c_cand
            ub_rel = lc["ub"] - c_cand
            m_c = len(c_cand)

            for i_c in range(m_c):
                slk_lb = s_idx
                slk_ub = m_total + s_idx
                s_idx += 1

                row_p = np.zeros(n)
                row_p[idx] = J_sub[i_c]
                row_v = np.zeros(num_slacks)
                row_v[slk_lb] = 1.0
                row_v[slk_ub] = -1.0

                A_lin_list.append(sp.csc_matrix(np.hstack([row_p, row_v])))
                l_lin_list.append(lb_rel[i_c])
                u_lin_list.append(ub_rel[i_c])

        if len(A_lin_list) > 0:
            A_lin_mat = sp.vstack(A_lin_list, format='csc')
            A_mat = sp.vstack([A_bounds, A_lin_mat], format='csc')
            l_all = np.concatenate([l_bounds, np.array(l_lin_list)])
            u_all = np.concatenate([u_bounds, np.array(u_lin_list)])
        else:
            return None, None

        soc_prob = osqp.OSQP()
        soc_prob.setup(P=P_mat, q=q_soc, A=A_mat, l=l_all, u=u_all,
                       warm_start=True, verbose=False, eps_abs=1e-4, eps_rel=1e-4, max_iter=5000)
        if prev_sol_x is not None:
            soc_prob.warm_start(x=prev_sol_x, y=prev_sol_y)

        res_soc = soc_prob.solve()
        if res_soc.info.status not in ('solved', 'solved inaccurate'):
            return None, None

        p_soc = res_soc.x[:n]
        return p_soc, res_soc
    except Exception:
        return None, None


def solve_sqp_trust_region(prog, max_iters=300, delta_0=0.25, delta_min=1e-5, delta_max=2.0, mu=50.0):
    """
    Pure SQP Trust Region solver using Drake C++ OSQP subproblem resolution.
    - Strictly NO line search.
    - Pure trust region step acceptance & expansion/contraction.
    - Dynamic L1 penalty parameter updates to prioritize feasibility.
    - Second Order Correction (SOC) for non-linear constraint curvature.
    - Damped BFGS updates with exact cost Hessian baseline.
    """
    vars_all = prog.decision_variables()
    n = len(vars_all)
    x_k = prog.GetInitialGuess(vars_all).copy()
    delta_k = delta_0

    # Initialize exact cost Hessian and constraint curvature matrix B_cons
    H_cost_exact = compute_exact_cost_hessian(prog, n)
    B_cons = np.zeros((n, n))
    H_k = H_cost_exact.copy()

    # Constraint-individual penalty weights vector mu_vec (length m_total)
    lin_init = compute_constraints_and_jacobians(prog, x_k)
    m_total_init = sum(len(lc["val_0"]) for lc in lin_init)
    mu_vec = mu * np.ones(m_total_init)

    def compute_l1_violation_vector(prog, x):
        lin_c = compute_constraints_and_jacobians(prog, x)
        viols = []
        for lc in lin_c:
            c_k = lc["val_0"]
            lb = lc["lb"]
            ub = lc["ub"]
            v_lb = np.maximum(0.0, lb - c_k)
            v_ub = np.maximum(0.0, c_k - ub)
            viols.append(v_lb + v_ub)
        return np.concatenate(viols) if len(viols) > 0 else np.zeros(0)

    def eval_merit(x, mu_vector):
        f_val, g_val = compute_cost_and_grad(prog, x)
        is_f, max_v = eval_feasibility(prog, x)
        v_vec = compute_l1_violation_vector(prog, x)
        l1_v = float(np.sum(v_vec))
        if len(mu_vector) == len(v_vec):
            merit_val = f_val + float(np.sum(mu_vector * v_vec))
        else:
            merit_val = f_val + mu * l1_v
        return merit_val, f_val, g_val, is_f, max_v, l1_v, v_vec

    merit_k, f_k, g_k, is_feas, max_v, l1_v, v_vec_k = eval_merit(x_k, mu_vec)

    print("=" * 80)
    print(f"PURE SQP TRUST REGION SOLVER (N={n} vars, constraint-individual mu_i, delta_0={delta_k})")
    print(f"Initial Cost: {f_k:.4f} | Initial Feasible: {is_feas} (max_viol={max_v:.2e}, L1_viol={l1_v:.2e})")
    print("=" * 80)

    osqp_solver = OsqpSolver()
    total_solve_time = 0.0
    accepted_steps = 0
    t_start_loop = time.time()

    prob_instance = None
    prev_sol_x = None
    prev_sol_y = None

    for it in range(max_iters):
        lin_constraints = compute_constraints_and_jacobians(prog, x_k)
        f_k, g_k = compute_cost_and_grad(prog, x_k)
        v_vec_k = compute_l1_violation_vector(prog, x_k)
        l1_v = float(np.sum(v_vec_k))

        m_total = sum(len(lc["val_0"]) for lc in lin_constraints)
        if len(mu_vec) != m_total:
            mu_vec = mu * np.ones(m_total)

        num_slacks = 2 * m_total
        N_qp = n + num_slacks

        # Quadratic cost: 0.5 p^T H_k p + 0.5 * 1e-4 * ||v||^2
        P_diag = np.concatenate([np.diag(H_k), 1e-4 * np.ones(num_slacks)])
        P_mat = sp.diags(P_diag, format='csc')

        # When current iterate is feasible, boost mu to fight to maintain feasibility.
        # A high floor ensures the QP strongly penalizes any constraint violation.
        is_currently_feas = float(np.max(v_vec_k)) < 1e-4
        mu_feas_floor = 5e3 if is_currently_feas else 50.0
        mu_vec_qp = np.maximum(mu_vec, mu_feas_floor)

        # Constraint-individual linear penalty weights for slacks: [mu_vec_qp; mu_vec_qp]
        mu_slacks = np.concatenate([mu_vec_qp, mu_vec_qp])
        q_vec = np.concatenate([g_k, mu_slacks])

        # Construct A matrix and bounds
        A_rows = []
        l_vec = []
        u_vec = []

        # 1. Bounding box on p: -delta_k <= p <= delta_k
        # 2. Slacks: 0 <= v <= inf
        A_bounds = sp.eye(N_qp, format='csc')
        l_bounds = np.concatenate([-delta_k * np.ones(n), np.zeros(num_slacks)])
        u_bounds = np.concatenate([delta_k * np.ones(n), np.full(num_slacks, np.inf)])

        A_rows.append(A_bounds)
        l_vec.append(l_bounds)
        u_vec.append(u_bounds)

        # 3. Linearized constraints (fixed sparsity structure for fast in-place update):
        # For each constraint row i:
        # l_b <= J_i p + v_lb_i - v_ub_i <= u_b
        A_lin_list = []
        l_lin_list = []
        u_lin_list = []

        s_idx = 0
        for lc in lin_constraints:
            idx = lc["indices"]
            c_k = lc["val_0"]
            J_sub = lc["J_sub"]
            lb_rel = lc["lb"] - c_k
            ub_rel = lc["ub"] - c_k
            m_c = len(c_k)

            for i_c in range(m_c):
                l_b = lb_rel[i_c]
                u_b = ub_rel[i_c]
                slk_lb = s_idx
                slk_ub = m_total + s_idx
                s_idx += 1

                row_p = np.zeros(n)
                row_p[idx] = J_sub[i_c]
                row_v = np.zeros(num_slacks)
                row_v[slk_lb] = 1.0
                row_v[slk_ub] = -1.0

                A_lin_list.append(sp.csc_matrix(np.hstack([row_p, row_v])))
                l_lin_list.append(l_b)
                u_lin_list.append(u_b)

        if len(A_lin_list) > 0:
            A_lin_mat = sp.vstack(A_lin_list, format='csc')
            A_mat = sp.vstack([A_bounds, A_lin_mat], format='csc')
            l_all = np.concatenate(l_vec + [np.array(l_lin_list)])
            u_all = np.concatenate(u_vec + [np.array(u_lin_list)])
        else:
            A_mat = A_bounds
            l_all = l_bounds
            u_all = u_bounds

        t0 = time.time()
        prob_instance = osqp.OSQP()
        prob_instance.setup(P=P_mat, q=q_vec, A=A_mat, l=l_all, u=u_all, warm_start=True, verbose=False, eps_abs=1e-4, eps_rel=1e-4, max_iter=10000)
        if prev_sol_x is not None:
            prob_instance.warm_start(x=prev_sol_x, y=prev_sol_y)

        res = prob_instance.solve()
        t_sub = time.time() - t0
        total_solve_time += t_sub

        if res.info.status == 'solved':
            prev_sol_x = res.x.copy()
            prev_sol_y = res.y.copy()

        if res.info.status != 'solved':
            print(f"TR Iter {it+1:03d} | OSQP status: {res.info.status} -> Shrinking delta")
            delta_k *= 0.5
            if delta_k < delta_min:
                break
            continue

        y_sol = res.x
        p_val = y_sol[:n]
        v_slacks = y_sol[n:]
        lin_viol_pred_vec = v_slacks[:m_total] + v_slacks[m_total:]

        quad_obj_pred = float(g_k @ p_val + 0.5 * p_val @ H_k @ p_val)
        pred_red = float(np.sum(mu_vec * (v_vec_k - lin_viol_pred_vec))) - quad_obj_pred

        # Evaluate candidate step (PURE TRUST REGION - NO LINE SEARCH)
        x_cand = x_k + p_val
        merit_cand, f_cand, g_cand, cand_feas, cand_max_v, cand_l1_v, cand_v_vec = eval_merit(x_cand, mu_vec)
        act_red = merit_k - merit_cand

        # Dynamically adapt individual penalty weights for violated constraints
        for i_c in range(m_total):
            if cand_v_vec[i_c] > 1e-4 and lin_viol_pred_vec[i_c] < 1e-4:
                mu_vec[i_c] = min(mu_vec[i_c] * 2.0, 1e6)
        act_red = merit_k - merit_cand

        # Hard feasibility rejection: when coming from a feasible iterate,
        # never accept a step that causes a constraint violation.
        if is_currently_feas and cand_max_v > 1e-4:
            act_red = -1.0  # force rejection

        rho = act_red / pred_red if abs(pred_red) > 1e-8 else (1.0 if act_red >= 0 else -1.0)
        step_norm = float(np.linalg.norm(p_val))

        t_cum = time.time() - t_start_loop
        print(f"TR Iter {it+1:03d} [Cum={t_cum:.2f}s, t_qp={t_sub*1000:.1f}ms] | Delta={delta_k:.5f} | Cost={f_cand:.4f} | MaxViol={cand_max_v:.2e} | ActRed={act_red:.4f} | PredRed={pred_red:.4f} | Rho={rho:.3f} | Feas={cand_feas}")

        eta_accept = 1e-4
        if rho >= eta_accept and act_red > 0:
            accepted_steps += 1
            s_k = p_val
            
            # Extract dual multipliers lambda for non-linear constraints from OSQP solution
            lambda_c = res.y[N_qp:] if res.y is not None and len(res.y) >= N_qp + len(l_lin_list) else None
            lin_cand = compute_constraints_and_jacobians(prog, x_cand)

            if lambda_c is not None and len(lambda_c) == len(l_lin_list) and len(l_lin_list) > 0:
                J_k_full = np.zeros((len(l_lin_list), n))
                J_cand_full = np.zeros((len(l_lin_list), n))

                r_idx = 0
                for lc in lin_constraints:
                    idx = lc["indices"]
                    m_c = len(lc["val_0"])
                    J_k_full[r_idx:r_idx+m_c, idx] = lc["J_sub"]
                    r_idx += m_c

                r_idx = 0
                for lc in lin_cand:
                    idx = lc["indices"]
                    m_c = len(lc["val_0"])
                    J_cand_full[r_idx:r_idx+m_c, idx] = lc["J_sub"]
                    r_idx += m_c

                # Difference of constraint Jacobian gradients: y_cons = (J_cand - J_k)^T * lambda
                y_cons = (J_cand_full - J_k_full).T @ lambda_c
                B_cons = update_damped_bfgs(B_cons, s_k, y_cons)
            else:
                y_cost = g_cand - g_k
                B_cons = update_damped_bfgs(B_cons, s_k, y_cost)

            H_k = H_cost_exact + B_cons
            min_eig = float(np.min(np.linalg.eigvalsh(H_k)))
            if min_eig < 1e-4:
                H_k += (1e-4 - min_eig) * np.eye(n)

            x_k = x_cand.copy()
            f_k = f_cand
            g_k = g_cand.copy()
            merit_k = merit_cand
            max_v = cand_max_v
            is_feas = cand_feas

            if rho >= 0.75 and step_norm >= 0.8 * delta_k:
                delta_k = min(delta_k * 1.5, delta_max)
        else:
            # Second-Order Correction (SOC): try to recover when step was rejected due
            # to constraint nonlinearity (linearization under-predicted actual violation)
            soc_attempted = False
            soc_accepted = False
            if pred_red > 0 and cand_max_v > 1e-4:
                p_soc, res_soc = compute_soc_step(
                    prog, x_k, p_val, H_k, g_k, mu_vec, delta_k,
                    prob_instance, prev_sol_x, prev_sol_y, N_qp, n, m_total
                )
                if p_soc is not None:
                    soc_attempted = True
                    x_soc = x_k + p_val + p_soc
                    merit_soc, f_soc, g_soc, feas_soc, maxv_soc, l1_soc, v_vec_soc = eval_merit(x_soc, mu_vec)
                    act_red_soc = merit_k - merit_soc
                    # SOC acceptance: only requires merit improvement (no rho threshold)
                    if act_red_soc > 0:
                        soc_accepted = True
                        accepted_steps += 1
                        s_k = p_val + p_soc

                        lambda_c_soc = res_soc.y[N_qp:] if res_soc.y is not None and len(res_soc.y) >= N_qp + m_total else None
                        lin_soc = compute_constraints_and_jacobians(prog, x_soc)
                        if lambda_c_soc is not None and len(lambda_c_soc) == m_total:
                            J_k_full = np.zeros((m_total, n))
                            J_soc_full = np.zeros((m_total, n))
                            r_idx = 0
                            for lc in lin_constraints:
                                idx2 = lc["indices"]
                                mc2 = len(lc["val_0"])
                                J_k_full[r_idx:r_idx+mc2, idx2] = lc["J_sub"]
                                r_idx += mc2
                            r_idx = 0
                            for lc in lin_soc:
                                idx2 = lc["indices"]
                                mc2 = len(lc["val_0"])
                                J_soc_full[r_idx:r_idx+mc2, idx2] = lc["J_sub"]
                                r_idx += mc2
                            y_cons = (J_soc_full - J_k_full).T @ lambda_c_soc
                            B_cons = update_damped_bfgs(B_cons, s_k, y_cons)
                        H_k = H_cost_exact + B_cons
                        min_eig = float(np.min(np.linalg.eigvalsh(H_k)))
                        if min_eig < 1e-4:
                            H_k += (1e-4 - min_eig) * np.eye(n)

                        x_k = x_soc.copy()
                        f_k = f_soc
                        g_k = g_soc.copy()
                        merit_k = merit_soc
                        max_v = maxv_soc
                        is_feas = feas_soc
                        print(f"  -> SOC step accepted! Cost={f_k:.4f} MaxViol={max_v:.2e}")

            if not soc_accepted:
                delta_k *= 0.5
                if soc_attempted:
                    print(f"  -> Step rejected (SOC also failed). Shrinking radius to Delta={delta_k:.5f}")
                else:
                    print(f"  -> Step rejected. Shrinking radius to Delta={delta_k:.5f}")

        if delta_k < delta_min:
            print(f"Trust region radius shrank below minimum threshold ({delta_min:.1e}). Stopping.")
            break
        if is_feas and max_v < 1e-4 and abs(act_red) < 1e-5:
            print(f"High precision optimality reached at iteration {it+1}!")
            break

    print("=" * 80)
    print(f"PURE SQP TRUST REGION COMPLETE: Total Iterations: {it+1} | Accepted Steps: {accepted_steps}")
    print(f"Final Cost: {f_k:.4f} | Max Violation: {max_v:.2e} | All constraints satisfied? {is_feas}")
    print("=" * 80)

    prog.SetInitialGuess(vars_all, x_k)
    return x_k, f_k, total_solve_time
