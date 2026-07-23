import subprocess
import sys

def test_config(delta_0, opt_tol, max_iters=5):
    cmd = ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", str(opt_tol), "--delta-0", str(delta_0)]
    print(f"Testing delta_0={delta_0}, opt_tol={opt_tol}...")
    res = subprocess.run(cmd, capture_output=True, text=True)
    stdout = res.stdout

    solve_time = None
    cost = None
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

    return solve_time, cost

def main():
    print("=" * 80)
    print("TESTING FAST TRUST REGION PARAMETERS FOR OPTIMAL COST & SUB-2s TIME")
    print("=" * 80)

    radii = [0.25, 0.5, 1.0, 1.5, 2.0]
    tols = [1e-2, 1e-3, 5e-4, 1e-4]

    results = []
    for r in radii:
        for t in tols:
            s_time, c_val = test_config(r, t)
            results.append({"delta_0": r, "opt_tol": t, "time": s_time, "cost": c_val})
            print(f"delta_0={r:<4} | opt_tol={t:<6} | Time={s_time:.2f}s | Cost={c_val:.4f}")

    print("\n" + "=" * 80)
    print("SUMMARY RESULTS TABLE")
    print("=" * 80)
    print(f"{'delta_0':<10} | {'opt_tol':<10} | {'Time (s)':<10} | {'Optimal Cost':<12}")
    print("-" * 50)
    for res in results:
        t_str = f"{res['time']:.2f}" if res['time'] is not None else "N/A"
        c_str = f"{res['cost']:.4f}" if res['cost'] is not None else "N/A"
        print(f"{res['delta_0']:<10} | {res['opt_tol']:<10} | {t_str:<10} | {c_str:<12}")

if __name__ == "__main__":
    main()
