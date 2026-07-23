import subprocess
import sys

def test_small_box(delta_0, opt_tol=1e-3, max_iters=20):
    cmd = ["python3", "main_cpp.py", "--solver", "snopt", "--trust-region", "--opt-tolerance", str(opt_tol), "--delta-0", str(delta_0), "--max-iters", str(max_iters)]
    print(f"\nTesting small box delta_0={delta_0}, opt_tol={opt_tol}, max_iters={max_iters}...")
    res = subprocess.run(cmd, capture_output=True, text=True)
    stdout = res.stdout

    solve_time = None
    cost = None
    iters_run = None
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
            print(line)

    return solve_time, cost

def main():
    print("=" * 80)
    print("TESTING SMALL TRUST REGION BOXES (delta_0 = 0.01, 0.02, 0.05)")
    print("=" * 80)

    for delta_0 in [0.01, 0.02, 0.05]:
        for opt_tol in [1e-3, 1e-4]:
            t, c = test_small_box(delta_0=delta_0, opt_tol=opt_tol, max_iters=20)
            print(f"--> Small Box delta_0={delta_0}, opt_tol={opt_tol} | Time={t:.2f}s | Final Cost={c:.4f}")

if __name__ == "__main__":
    main()
