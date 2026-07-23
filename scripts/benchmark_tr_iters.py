import subprocess
import sys

def run_tr(max_iters, delta_0=0.25, opt_tol=1e-3):
    cmd = ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", str(opt_tol), "--delta-0", str(delta_0), "--max-iters", str(max_iters)]
    print(f"\nRunning TR (max_iters={max_iters}, delta_0={delta_0}, opt_tol={opt_tol})...")
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
    print("BENCHMARKING TRUST REGION ITERATIONS VS COST & SOLVE TIME")
    print("=" * 80)

    results = []
    for iters in [1, 2, 3, 4, 5]:
        t, c = run_tr(max_iters=iters, delta_0=0.25, opt_tol=1e-3)
        results.append({"max_iters": iters, "time": t, "cost": c})
        print(f"Max Iters = {iters} | Solve Time = {t:.2f}s | Final Path Cost = {c:.4f}")

    print("\n" + "=" * 80)
    print("ITERATION SWEEP SUMMARY TABLE")
    print("=" * 80)
    print(f"{'Max Iters':<12} | {'Solve Time (s)':<16} | {'Optimal Path Cost':<18}")
    print("-" * 50)
    for r in results:
        print(f"{r['max_iters']:<12} | {r['time']:<16.2f} | {r['cost']:<18.4f}")

if __name__ == "__main__":
    main()
