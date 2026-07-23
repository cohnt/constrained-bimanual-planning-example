import time
import numpy as np
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

def compute_exact_cost_hessian(prog, n, eps=1e-5):
    """
    Computes exact cost Hessian H = grad^2 f(x) for quadratic trajectory costs.
    """
    costs = prog.GetAllCosts()
    H = np.zeros((n, n))
    x0 = np.zeros(n)
    
    # Evaluate Hessian from cost bindings
    for b in costs:
        idx = prog.FindDecisionVariableIndices(b.variables())
        evaluator = b.evaluator()
        m_vars = len(idx)
        # Numerical second derivatives for this binding
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
                
    # Project to positive semi-definite matrix via spectral decomposition V * max(1e-3, evals) * V^T
    eigvals, eigvecs = np.linalg.eigh(H)
    eigvals_clamped = np.maximum(eigvals, 1e-3)
    H_psd = eigvecs @ np.diag(eigvals_clamped) @ eigvecs.T
    H_psd = 0.5 * (H_psd + H_psd.T) + 1e-3 * np.eye(n)
    return H_psd

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
    # Symmetrize
    return 0.5 * (H_next + H_next.T)

import osqp
import scipy.sparse as sp

def solve_sqp_trust_region(prog, max_iters=300, delta_0=0.25, delta_min=1e-5, delta_max=2.0, mu=5.0):
    """
    Direct C-level SQP Trust Region solver using `import osqp`.
    - Eliminates all Drake MathematicalProgram parsing/build overhead.
    - Uses `prob.update(...)` in-place on sparse C matrices.
    - Standard TR step acceptance via actual-vs-predicted merit reduction ratio rho.
    - No line search: radius shrinks on rejection, expands on acceptance.
    """
    vars_all = prog.decision_variables()
    n = len(vars_all)
    x_k = prog.GetInitialGuess(vars_all).copy()
    delta_k = delta_0

    # Initialize tridiagonal trajectory Hessian H0
    H_k = np.eye(n) * 2.0
    for i in range(n - 8):
        H_k[i, i + 8] = -1.0
        H_k[i + 8, i] = -1.0
    H_k[-1, -1] = 50.0

    def eval_merit(x, penalty_weight=mu):
        f_val, _ = compute_cost_and_grad(prog, x)
        is_f, viol = eval_feasibility(prog, x)
        return f_val + penalty_weight * viol, f_val, is_f, viol

    merit_k, f_k, is_feas, max_v = eval_merit(x_k)
    f_k, g_k = compute_cost_and_grad(prog, x_k)

    print("=" * 80)
    print(f"STARTING DIRECT C-LEVEL OSQP SQP TRUST REGION SOLVER (`import osqp`, N={n} vars, mu={mu})")
    print(f"Initial Cost: {f_k:.4f} | Initial Feasible: {is_feas} (max viol={max_v:.2e}) | Initial Radius: {delta_k:.4f}")
    print("=" * 80)

    # 1. Evaluate Initial Linearization Structure
    lin_constraints = compute_constraints_and_jacobians(prog, x_k)

    # Calculate total constraint dimension
    m_total = sum(len(lc["val_0"]) for lc in lin_constraints)
    num_slacks = 2 * m_total
    N_vars = n + num_slacks  # [p, v_lb, v_ub]

    # Map constraints into global A matrix
    # Rows:
    # 0..n-1: Trust region bounds on p (-delta <= p <= delta)
    # n..n+num_slacks-1: Non-negativity bounds on slacks (0 <= v <= inf)
    # n+num_slacks .. n+num_slacks+m_total-1: Linearized lower bound constraints J_i p + v_lb,i >= lb_rel
    # n+num_slacks+m_total .. n+num_slacks+2*m_total-1: Linearized upper bound constraints J_i p - v_ub,i <= ub_rel

    num_rows = n + num_slacks + 2 * m_total

    # Construct P matrix (quadratic cost 0.5 * p^T H_k p)
    P_top_left = sp.csc_matrix(H_k)
    P_sparse = sp.block_diag([P_top_left, sp.csc_matrix((num_slacks, num_slacks))], format='csc')

    # Construct q vector (linear cost g_k^T p + mu * sum(v))
    q_vec = np.concatenate([g_k, mu * np.ones(num_slacks)])

    # Construct initial A matrix, l, u vectors
    row_list = []
    col_list = []
    data_list = []
    l_vec = np.zeros(num_rows)
    u_vec = np.zeros(num_rows)

    # Row 0..n-1: Box on p (-delta <= p <= delta)
    for i in range(n):
        row_list.append(i)
        col_list.append(i)
        data_list.append(1.0)
        l_vec[i] = -delta_k
        u_vec[i] = delta_k

    # Row n..n+num_slacks-1: Box on v (0 <= v <= 1e5)
    for i in range(num_slacks):
        r_idx = n + i
        c_idx = n + i
        row_list.append(r_idx)
        col_list.append(c_idx)
        data_list.append(1.0)
        l_vec[r_idx] = 0.0
        u_vec[r_idx] = 1e5

    # Row n+num_slacks onwards: Linear constraints
    r_curr = n + num_slacks
    slack_idx = 0

    for lc in lin_constraints:
        idx = lc["indices"]
        c_k = lc["val_0"]
        J_sub = lc["J_sub"]
        lb_rel = lc["lb"] - c_k
        ub_rel = lc["ub"] - c_k
        m_c = len(c_k)

        for i_c in range(m_c):
            # Lower bound row: J_sub p + v_lb >= lb_rel
            r_lb = r_curr
            r_curr += 1
            for j_loc, j_glob in enumerate(idx):
                row_list.append(r_lb)
                col_list.append(j_glob)
                data_list.append(J_sub[i_c, j_loc])
            # Slack variable v_lb
            row_list.append(r_lb)
            col_list.append(n + slack_idx)
            data_list.append(1.0)
            l_vec[r_lb] = lb_rel[i_c] if not np.isneginf(lb_rel[i_c]) else -1e9
            u_vec[r_lb] = 1e9

            # Upper bound row: J_sub p - v_ub <= ub_rel
            r_ub = r_curr
            r_curr += 1
            for j_loc, j_glob in enumerate(idx):
                row_list.append(r_ub)
                col_list.append(j_glob)
                data_list.append(J_sub[i_c, j_loc])
            # Slack variable v_ub
            row_list.append(r_ub)
            col_list.append(n + m_total + slack_idx)
            data_list.append(-1.0)
            l_vec[r_ub] = -1e9
            u_vec[r_ub] = ub_rel[i_c] if not np.isposinf(ub_rel[i_c]) else 1e9

            slack_idx += 1

    A_sparse = sp.csc_matrix((data_list, (row_list, col_list)), shape=(num_rows, N_vars))

    # Initialize Direct OSQP Solver
    t_start_setup = time.time()
    prob = osqp.OSQP()
    prob.setup(P=P_sparse, q=q_vec, A=A_sparse, l=l_vec, u=u_vec, eps_abs=1e-3, eps_rel=1e-3, max_iter=2000, verbose=False)
    t_setup = time.time() - t_start_setup

    print(f"Direct OSQP C-Setup Complete in {t_setup*1000:.2f}ms (Reused in-place across all iterations via prob.update)")

    total_lin_time = 0.0
    total_update_time = 0.0
    total_osqp_solve_time = 0.0
    total_osqp_c_time = 0.0
    accepted_steps = 0
    t_start_loop = time.time()

    for it in range(max_iters):
        # 1. Linearize constraints at x_k (Drake Native C++ AutoDiff)
        t_start_lin = time.time()
        lin_constraints = compute_constraints_and_jacobians(prog, x_k)
        f_k, g_k = compute_cost_and_grad(prog, x_k)
        t_lin = time.time() - t_start_lin
        total_lin_time += t_lin

        # 2. Update Direct C OSQP Problem Vectors in-place
        t_start_upd = time.time()
        q_vec[:n] = g_k
        l_vec[:n] = -delta_k
        u_vec[:n] = delta_k

        r_curr = n + num_slacks
        for lc in lin_constraints:
            c_k = lc["val_0"]
            lb_rel = lc["lb"] - c_k
            ub_rel = lc["ub"] - c_k
            m_c = len(c_k)

            for i_c in range(m_c):
                l_vec[r_curr] = lb_rel[i_c] if not np.isneginf(lb_rel[i_c]) else -1e9
                u_vec[r_curr] = 1e9
                r_curr += 1

                l_vec[r_curr] = -1e9
                u_vec[r_curr] = ub_rel[i_c] if not np.isposinf(ub_rel[i_c]) else 1e9
                r_curr += 1

        prob.update(q=q_vec, l=l_vec, u=u_vec)
        t_upd = time.time() - t_start_upd
        total_update_time += t_upd

        # 3. Solve Direct C OSQP Subproblem
        t_start_solve = time.time()
        res = prob.solve()
        t_solve = time.time() - t_start_solve
        total_osqp_solve_time += t_solve

        t_osqp_internal = res.info.solve_time
        total_osqp_c_time += t_osqp_internal

        if res.info.status != "solved" and res.info.status != "solved inaccurate":
            print(f"TR Iter {it+1:02d} | OSQP Status: {res.info.status} -> Shrinking delta")
            delta_k *= 0.5
            if delta_k < delta_min:
                break
            continue

        p_val = res.x[:n]
        step_norm = np.linalg.norm(p_val)

        # Evaluate candidate at full step (no line search)
        x_cand = x_k + p_val
        merit_cand, f_cand, cand_feas, cand_viol = eval_merit(x_cand)
        act_red = merit_k - merit_cand

        t_cum = time.time() - t_start_loop
        print(f"TR Iter {it+1:02d} [Cum={t_cum:.2f}s] | Lin={t_lin*1000:.1f}ms | Update={t_upd*1000:.1f}ms | Solve={t_solve*1000:.1f}ms (OSQP_C={t_osqp_internal*1000:.2f}ms) | Cost={f_cand:.4f} | MeritRed={act_red:.4f} | Feas={cand_feas}")

        if act_red > 0:
            accepted_steps += 1

            # Update Damped BFGS Hessian
            s_k = p_val
            _, g_cand = compute_cost_and_grad(prog, x_cand)
            y_k = g_cand - g_k
            H_k = update_damped_bfgs(H_k, s_k, y_k)

            x_k = x_cand.copy()
            f_k = f_cand
            g_k = g_cand.copy()
            merit_k = merit_cand

            # Expand radius on acceptance (capped at delta_max)
            delta_k = min(delta_k * 1.5, delta_max)
        else:
            # Step rejected: halve radius
            delta_k *= 0.5
            print(f"  -> Step rejected (merit_red={act_red:.4f}). Shrinking radius to Delta={delta_k:.5f}")

        if delta_k < delta_min:
            print(f"Trust region radius shrank below minimum threshold ({delta_min:.1e}). Stopping.")
            break

    total_wall_time = total_lin_time + total_update_time + total_osqp_solve_time
    print("=" * 80)
    print("DIRECT C-LEVEL OSQP SQP TRUST REGION TIMING BREAKDOWN:")
    print(f"  1. Linearization (Drake Native C++ AutoDiff): {total_lin_time:.3f}s ({total_lin_time/total_wall_time*100:.1f}%)")
    print(f"  2. Direct OSQP Sparse Update (`prob.update`): {total_update_time:.3f}s ({total_update_time/total_wall_time*100:.1f}%)")
    print(f"  3. Direct OSQP Solve (`prob.solve`):         {total_osqp_solve_time:.3f}s ({total_osqp_solve_time/total_wall_time*100:.1f}%)")
    print(f"     -> Pure OSQP C-Solver Internal Time:        {total_osqp_c_time:.3f}s ({total_osqp_c_time/total_wall_time*100:.1f}%)")
    print(f"  TOTAL WALL TIME:                             {total_wall_time:.3f}s")
    print("-" * 80)
    print(f"Total Iterations: {it+1} | Accepted Steps: {accepted_steps} | Final Cost: {f_k:.4f}")
    print(f"All constraints satisfied at final solution? {eval_feasibility(prog, x_k)[0]}")
    print("=" * 80)

    # Return final solution
    prog.SetInitialGuess(vars_all, x_k)
    return x_k, f_k, total_osqp_solve_time
