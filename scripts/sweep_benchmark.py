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

from main_cpp import run_pipeline

def run_sweeps():
    print("=" * 80)
    print("STARTING PARAMETER SWEEPS ACROSS SOLVERS & TRUST REGION HYPERPARAMETERS")
    print("=" * 80)

    results = []

    # 1. SNOPT Parameter Sweep
    print("\n--- SWEEP 1: Standard SNOPT Optimality Tolerance Sweep ---")
    for opt_tol in [1e-1, 1e-2, 1e-3, 1e-4, 1e-6]:
        try:
            print(f"\n[SNOPT] Testing Major optimality tolerance = {opt_tol}...")
            t0 = time.time()
            succ, feas, t_solve = run_pipeline(
                strip_special_optimizer_settings=False,
                use_solver="snopt",
                use_trust_region=False,
                opt_tolerance=opt_tol
            )
            results.append({
                "category": "SNOPT Standard",
                "params": f"opt_tol={opt_tol}",
                "success": succ,
                "feasible": feas,
                "solve_time": t_solve
            })
        except Exception as e:
            print(f"Error running SNOPT opt_tol={opt_tol}: {e}")

    # 2. IPOPT Parameter Sweep
    print("\n--- SWEEP 2: IPOPT Settings Sweep ---")
    for mode in ["special_penalty", "stripped_default"]:
        try:
            strip_opt = (mode == "stripped_default")
            print(f"\n[IPOPT] Testing mode = {mode}...")
            succ, feas, t_solve = run_pipeline(
                strip_special_optimizer_settings=strip_opt,
                use_solver="ipopt",
                use_trust_region=False
            )
            results.append({
                "category": "IPOPT Standard",
                "params": f"mode={mode}",
                "success": succ,
                "feasible": feas,
                "solve_time": t_solve
            })
        except Exception as e:
            print(f"Error running IPOPT mode={mode}: {e}")

    # 3. SNOPT Trust Region Hyperparameter Sweep
    print("\n--- SWEEP 3: SNOPT Trust Region (TR) Hyperparameter Sweep ---")
    delta_vals = [0.1, 0.25, 0.5]
    opt_tols = [1e-1, 1e-2, 1e-3]

    for delta_0 in delta_vals:
        for opt_tol in opt_tols:
            try:
                print(f"\n[SNOPT TR] Testing delta_0={delta_0}, opt_tol={opt_tol}...")
                succ, feas, t_solve = run_pipeline(
                    strip_special_optimizer_settings=False,
                    use_solver="snopt",
                    use_trust_region=True,
                    opt_tolerance=opt_tol,
                    delta_0=delta_0
                )
                results.append({
                    "category": "SNOPT Trust Region",
                    "params": f"delta_0={delta_0}, opt_tol={opt_tol}",
                    "success": succ,
                    "feasible": feas,
                    "solve_time": t_solve
                })
            except Exception as e:
                print(f"Error running TR delta_0={delta_0}, opt_tol={opt_tol}: {e}")

    print("\n" + "=" * 80)
    print("PARAM SWEEP RESULTS SUMMARY")
    print("=" * 80)
    print(f"{'Category':<22} | {'Parameters':<30} | {'Success':<8} | {'Feasible':<8} | {'Time (s)':<10}")
    print("-" * 88)
    for r in results:
        print(f"{r['category']:<22} | {r['params']:<30} | {str(r['success']):<8} | {str(r['feasible']):<8} | {r['solve_time']:<10.2f}")

if __name__ == "__main__":
    run_sweeps()
