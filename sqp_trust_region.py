import time
import numpy as np
from pydrake.solvers import MathematicalProgram, OsqpSolver, SolutionResult

def compute_cost_and_grad(prog, x, eps=1e-7):
    """
    Evaluates cost f(x) and cost gradient g(x) = grad f(x) using finite differences.
    """
    costs = prog.GetAllCosts()
    def eval_f(val_x):
        tot = 0.0
        for b in costs:
            idx = prog.FindDecisionVariableIndices(b.variables())
            tot += np.sum(b.evaluator().Eval(val_x[idx]))
        return tot

    f_0 = eval_f(x)
    n = len(x)
    grad = np.zeros(n)
    for i in range(n):
        x_plus = x.copy()
        x_plus[i] += eps
        x_minus = x.copy()
        x_minus[i] -= eps
        grad[i] = (eval_f(x_plus) - eval_f(x_minus)) / (2.0 * eps)

    return f_0, grad

def compute_constraints_and_jacobians(prog, x, eps=1e-7):
    """
    Evaluates all constraints c(x) and their Jacobians J = dc/dx using finite differences.
    Returns a list of dicts containing linearized constraint data.
    """
    constraints = prog.GetAllConstraints()
    linearized_constraints = []

    for b in constraints:
        idx = prog.FindDecisionVariableIndices(b.variables())
        evaluator = b.evaluator()
        x_sub = x[idx]
        val_0 = evaluator.Eval(x_sub)
        m = len(val_0)
        n_sub = len(idx)

        J_sub = np.zeros((m, n_sub))
        for j in range(n_sub):
            x_plus = x_sub.copy()
            x_plus[j] += eps
            x_minus = x_sub.copy()
            x_minus[j] -= eps
            J_sub[:, j] = (evaluator.Eval(x_plus) - evaluator.Eval(x_minus)) / (2.0 * eps)

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

def solve_sqp_trust_region(prog, max_iters=100, delta_0=0.10, delta_min=1e-5, delta_max=2.0, mu=100.0):
    """
    L1-Penalty SQP Trust Region solver using Powell's Damped BFGS (1st-order gradients only).
    """
    vars_all = prog.decision_variables()
    n = len(vars_all)
    x_k = prog.GetInitialGuess(vars_all).copy()
    delta_k = delta_0

    # Initialize scaled diagonal Hessian H0
    H_k = np.eye(n) * 1.0

    def eval_merit(x, penalty_weight=mu):
        f_val, _ = compute_cost_and_grad(prog, x)
        is_f, viol = eval_feasibility(prog, x)
        return f_val + penalty_weight * viol, f_val, is_f, viol

    merit_k, f_k, is_feas, max_v = eval_merit(x_k)
    f_k, g_k = compute_cost_and_grad(prog, x_k)

    print("=" * 80)
    print(f"STARTING FAST L1-PENALTY SQP TRUST REGION SOLVER (OSQP backend, N={n} vars, mu={mu})")
    print(f"Initial Cost: {f_k:.4f} | Initial Feasible: {is_feas} (max viol={max_v:.2e}) | Initial Merit: {merit_k:.4f} | Initial Radius: {delta_k:.4f}")
    print("=" * 80)

    # Fast OSQP solver options
    from pydrake.solvers import SolverOptions, CommonSolverOption
    osqp_solver = OsqpSolver()
    osqp_options = SolverOptions()
    osqp_options.SetOption(OsqpSolver().solver_id(), "eps_abs", 1e-4)
    osqp_options.SetOption(OsqpSolver().solver_id(), "eps_rel", 1e-4)
    osqp_options.SetOption(OsqpSolver().solver_id(), "max_iter", 4000)

    total_lin_time = 0.0
    total_build_time = 0.0
    total_solve_time = 0.0
    total_osqp_internal_time = 0.0
    accepted_steps = 0

    for it in range(max_iters):
        # 1. Linearize constraints at x_k (Python finite differences & evaluation)
        t_start_lin = time.time()
        lin_constraints = compute_constraints_and_jacobians(prog, x_k)
        t_lin = time.time() - t_start_lin
        total_lin_time += t_lin

        # 2. Formulate L1 Penalty QP Subproblem in Drake (Python AST & binding construction)
        t_start_build = time.time()
        qp_prog = MathematicalProgram()
        p = qp_prog.NewContinuousVariables(n, "p")

        # Count total scalar constraints for slacks
        num_slacks = 0
        for lc in lin_constraints:
            m = len(lc["val_0"])
            num_slacks += 2 * m  # lower and upper slacks

        v = qp_prog.NewContinuousVariables(num_slacks, "v")
        qp_prog.AddBoundingBoxConstraint(0.0, 1e5, v)

        # Objective: 0.5 * p^T * H_k * p + g_k^T * p + mu * sum(v)
        qp_prog.AddQuadraticCost(H_k, g_k, p)
        qp_prog.AddLinearCost(mu * np.ones(num_slacks), v)

        # Trust Region Bounding Box: -delta_k <= p <= delta_k
        qp_prog.AddBoundingBoxConstraint(-delta_k, delta_k, p)

        # Linearized Constraints with slacks
        slack_idx = 0
        for lc in lin_constraints:
            idx = lc["indices"]
            c_k = lc["val_0"]
            J_sub = lc["J_sub"]
            lb_rel = lc["lb"] - c_k
            ub_rel = lc["ub"] - c_k
            m = len(c_k)

            v_lb = v[slack_idx : slack_idx + m]
            v_ub = v[slack_idx + m : slack_idx + 2 * m]
            slack_idx += 2 * m

            # J_sub * p + v_lb >= lb_rel
            # J_sub * p - v_ub <= ub_rel
            for i_c in range(m):
                if not np.isneginf(lb_rel[i_c]):
                    vars_lb = np.concatenate([p[idx], [v_lb[i_c]]])
                    coeff_lb = np.concatenate([J_sub[i_c, :], [1.0]])
                    qp_prog.AddLinearConstraint(coeff_lb, lb_rel[i_c], 1e9, vars_lb)
                if not np.isposinf(ub_rel[i_c]):
                    vars_ub = np.concatenate([p[idx], [v_ub[i_c]]])
                    coeff_ub = np.concatenate([J_sub[i_c, :], [-1.0]])
                    qp_prog.AddLinearConstraint(coeff_ub, -1e9, ub_rel[i_c], vars_ub)
        t_build = time.time() - t_start_build
        total_build_time += t_build

        # 3. Solve QP Subproblem using OSQP
        t_start_solve = time.time()
        qp_result = osqp_solver.Solve(qp_prog, None, osqp_options)
        t_solve = time.time() - t_start_solve
        total_solve_time += t_solve

        # Extract internal OSQP solver details if available
        t_osqp_internal = 0.0
        try:
            details = qp_result.get_solver_details()
            if hasattr(details, "solve_time"):
                t_osqp_internal = float(details.solve_time)
            elif hasattr(details, "run_time"):
                t_osqp_internal = float(details.run_time)
        except Exception:
            pass
        total_osqp_internal_time += t_osqp_internal

        if not qp_result.is_success():
            print(f"TR Iter {it+1:02d} | OSQP Failed ({qp_result.get_solution_result()}) -> Shrinking delta")
            delta_k *= 0.5
            if delta_k < delta_min:
                break
            continue

        p_val = qp_result.GetSolution(p)
        step_norm = np.linalg.norm(p_val)

        # Candidate point
        x_cand = x_k + p_val
        merit_cand, f_cand, cand_feas, cand_viol = eval_merit(x_cand)

        # Actual merit reduction
        act_red = merit_k - merit_cand

        print(f"TR Iter {it+1:02d} | Lin={t_lin*1000:.1f}ms | Build={t_build*1000:.1f}ms | Solve={t_solve*1000:.1f}ms (OSQP={t_osqp_internal*1000:.1f}ms) | Cost={f_cand:.4f} | Feas={cand_feas}")

        # Step acceptance criteria
        if act_red > 1e-4 or (cand_feas and f_cand < f_k):
            accepted_steps += 1

            # Update Damped BFGS Hessian (1st-order gradients only)
            s_k = p_val
            _, g_cand = compute_cost_and_grad(prog, x_cand)
            y_k = g_cand - g_k
            H_k = update_damped_bfgs(H_k, s_k, y_k)

            x_k = x_cand.copy()
            f_k = f_cand
            g_k = g_cand.copy()
            merit_k = merit_cand

            # Expand radius if step hits boundary
            if step_norm >= 0.7 * delta_k:
                delta_k = min(delta_k * 1.5, delta_max)
        else:
            # Reject step and shrink trust region
            delta_k *= 0.5
            print(f"  -> Step rejected (act_red={act_red:.4f}). Shrinking radius to Delta = {delta_k:.5f}")

        if delta_k < delta_min:
            print(f"Trust region radius shrank below minimum threshold ({delta_min:.1e}). Stopping.")
            break

    total_wall_time = total_lin_time + total_build_time + total_solve_time
    print("=" * 80)
    print("SQP TRUST REGION TIMING BREAKDOWN:")
    print(f"  1. Linearization (Python Finite Diff & Eval): {total_lin_time:.3f}s ({total_lin_time/total_wall_time*100:.1f}%)")
    print(f"  2. QP Subproblem Build (Drake Python AST):    {total_build_time:.3f}s ({total_build_time/total_wall_time*100:.1f}%)")
    print(f"  3. QP Subproblem Solve (Drake C++ Interface):  {total_solve_time:.3f}s ({total_solve_time/total_wall_time*100:.1f}%)")
    if total_osqp_internal_time > 0:
        print(f"     -> Pure OSQP C-Solver Internal Time:        {total_osqp_internal_time:.3f}s ({total_osqp_internal_time/total_wall_time*100:.1f}%)")
    print(f"  TOTAL WALL TIME:                             {total_wall_time:.3f}s")
    print("-" * 80)
    print(f"Total Iterations: {it+1} | Accepted Steps: {accepted_steps} | Final Cost: {f_k:.4f}")
    print(f"All constraints satisfied at final solution? {eval_feasibility(prog, x_k)[0]}")
    print("=" * 80)

    # Return final solution
    prog.SetInitialGuess(vars_all, x_k)
    return x_k, f_k, total_solve_time
