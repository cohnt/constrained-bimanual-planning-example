import subprocess
import sys
import time

def run_cmd(cmd_list):
    print(f"\nRunning command: {' '.join(cmd_list)}")
    t0 = time.time()
    res = subprocess.run(cmd_list, capture_output=True, text=True)
    t_elapsed = time.time() - t0
    stdout = res.stdout
    
    # Parse status from output
    succ = "Solve success: True" in stdout
    feas = "All constraints satisfied at solution? True" in stdout
    
    # Extract solve time and cost from output
    solve_time = t_elapsed
    cost = 0.0
    for line in stdout.splitlines():
        if "Solve completed in" in line:
            try:
                solve_time = float(line.split("Solve completed in")[1].split("s")[0].strip())
            except:
                pass
        elif "Final Solution Cost:" in line:
            try:
                cost = float(line.split("Final Solution Cost:")[1].strip())
            except:
                pass
        elif "Cost=" in line and "diff=" in line:
            # Fallback for TR loop if printed in line
            try:
                cost = float(line.split("Cost=")[1].split("(")[0].strip())
            except:
                pass

    return succ, feas, solve_time, cost

def main():
    configs = [
        # SNOPT Standard
        ("SNOPT Standard", "Major opt_tol = 1e-1 (Original Baseline)", ["python3", "main_cpp.py", "--solver", "snopt", "--opt-tolerance", "1e-1"]),
        ("SNOPT Standard", "Major opt_tol = 1e-2", ["python3", "main_cpp.py", "--solver", "snopt", "--opt-tolerance", "1e-2"]),
        ("SNOPT Standard", "Major opt_tol = 1e-3", ["python3", "main_cpp.py", "--solver", "snopt", "--opt-tolerance", "1e-3"]),
        ("SNOPT Standard", "Major opt_tol = 1e-6 (Default Strict)", ["python3", "main_cpp.py", "--solver", "snopt", "--strip-options"]),

        # IPOPT Standard
        ("IPOPT Standard", "Special Penalty Line Search (Baseline)", ["python3", "main_cpp.py", "--solver", "ipopt"]),
        ("IPOPT Standard", "Default Strict (No Special Options)", ["python3", "main_cpp.py", "--solver", "ipopt", "--strip-options"]),

        # SNOPT Trust Region
        ("SNOPT Trust Region", "TR (delta_0=0.25, opt_tol=1e-3)", ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", "1e-3"]),
        ("SNOPT Trust Region", "TR (delta_0=0.25, opt_tol=1e-2)", ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", "1e-2"]),
        ("SNOPT Trust Region", "TR (delta_0=0.25, opt_tol=1e-1)", ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", "1e-1"]),
    ]

    results = []
    for cat, label, cmd in configs:
        succ, feas, t_solve, cost = run_cmd(cmd)
        results.append({
            "category": cat,
            "params": label,
            "success": succ,
            "feasible": feas,
            "solve_time": t_solve,
            "cost": cost
        })
        print(f"--> Result: Succ={succ}, Feas={feas}, Time={t_solve:.2f}s, Cost={cost:.4f}")

    header = f"{'Category':<22} | {'Configuration / Hyperparameters':<38} | {'Success':<8} | {'Feasible':<8} | {'Time (s)':<10} | {'Optimal Cost':<12}"
    divider = "-" * 114
    
    print("\n" + "=" * 114)
    print("BEST HYPERPARAMETERS PERFORMANCE COMPARISON SUMMARY")
    print("=" * 114)
    print(header)
    print(divider)
    for r in results:
        print(f"{r['category']:<22} | {r['params']:<38} | {str(r['success']):<8} | {str(r['feasible']):<8} | {r['solve_time']:<10.2f} | {r['cost']:<12.4f}")

    with open("sweep_results_summary.txt", "w") as f:
        f.write("=" * 114 + "\n")
        f.write("BEST HYPERPARAMETERS PERFORMANCE COMPARISON SUMMARY\n")
        f.write("=" * 114 + "\n")
        f.write(header + "\n")
        f.write(divider + "\n")
        for r in results:
            f.write(f"{r['category']:<22} | {r['params']:<38} | {str(r['success']):<8} | {str(r['feasible']):<8} | {r['solve_time']:<10.2f} | {r['cost']:<12.4f}\n")

if __name__ == "__main__":
    main()
