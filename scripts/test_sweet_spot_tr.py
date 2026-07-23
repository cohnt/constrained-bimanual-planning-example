import subprocess
import sys

def test_config(delta_0, opt_tol, max_iters):
    cmd = ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", str(opt_tol), "--delta-0", str(delta_0), "--max-iters", str(max_iters)]
    print(f"\nTesting delta_0={delta_0}, opt_tol={opt_tol}, max_iters={max_iters}...")
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
    print("TESTING TR SWEET SPOT FOR SUB-5s SOLVE TIME & ~1.35 OPTIMAL COST")
    print("=" * 80)

    test_cases = [
        (0.05, 1e-3, 5),
        (0.05, 1e-3, 6),
        (0.05, 2e-3, 6),
        (0.10, 1e-3, 4),
        (0.10, 1e-3, 5),
        (0.10, 2e-3, 5),
    ]

    results = []
    for r, t, m in test_cases:
        s_time, c_val, iters = test_config(r, t, m)
        results.append({"delta_0": r, "opt_tol": t, "max_iters": m, "time": s_time, "cost": c_val, "iters": iters})

    print("\n" + "=" * 80)
    print("SUMMARY RESULTS TABLE")
    print("=" * 80)
    print(f"{'delta_0':<8} | {'opt_tol':<8} | {'Max Iters':<10} | {'Time (s)':<10} | {'Optimal Cost':<12}")
    print("-" * 60)
    for res in results:
        t_str = f"{res['time']:.2f}" if res['time'] is not None else "N/A"
        c_str = f"{res['cost']:.4f}" if res['cost'] is not None else "N/A"
        print(f"{res['delta_0']:<8} | {res['opt_tol']:<8} | {res['max_iters']:<10} | {t_str:<10} | {c_str:<12}")

if __name__ == "__main__":
    main()
