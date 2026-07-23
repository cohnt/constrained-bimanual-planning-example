import subprocess
import sys

def test_config(delta_0, opt_tol):
    cmd = ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", str(opt_tol), "--delta-0", str(delta_0)]
    print(f"\nTesting delta_0={delta_0}, opt_tol={opt_tol}...")
    res = subprocess.run(cmd, capture_output=True, text=True)
    stdout = res.stdout

    solve_time = None
    cost = None
    iters_run = 0
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
        elif "TR Iter" in line:
            iters_run += 1
            print(line)

    return solve_time, cost, iters_run

def main():
    print("=" * 80)
    print("TESTING LOOSER SNOPT TOLERANCES WITH SMALL/MODERATE BOXES")
    print("=" * 80)

    tols = [1e-1, 5e-2, 1e-2, 5e-3]
    radii = [0.01, 0.02, 0.05, 0.10]

    results = []
    for r in radii:
        for t in tols:
            s_time, c_val, iters = test_config(r, t)
            results.append({"delta_0": r, "opt_tol": t, "time": s_time, "cost": c_val, "iters": iters})
            print(f"delta_0={r:<5} | opt_tol={t:<6} | Time={s_time:.2f}s | Cost={c_val:.4f} | Iters={iters}")

    print("\n" + "=" * 80)
    print("SUMMARY RESULTS TABLE")
    print("=" * 80)
    print(f"{'delta_0':<8} | {'opt_tol':<8} | {'Iters':<6} | {'Time (s)':<10} | {'Optimal Cost':<12}")
    print("-" * 55)
    for res in results:
        t_str = f"{res['time']:.2f}" if res['time'] is not None else "N/A"
        c_str = f"{res['cost']:.4f}" if res['cost'] is not None else "N/A"
        print(f"{res['delta_0']:<8} | {res['opt_tol']:<8} | {res['iters']:<6} | {t_str:<10} | {c_str:<12}")

if __name__ == "__main__":
    main()
