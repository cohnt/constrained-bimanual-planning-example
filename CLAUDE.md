# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A tutorial/demonstration repository (not a library) for the ICRA 2024 paper "Constrained bimanual
planning with analytic inverse kinematics." Two KUKA IIWA-14 arms hold an object, so the relative
transform between the two end-effectors is fixed. Instead of enforcing that as an equality
constraint, an analytic IK function *parameterizes* the constraint manifold, giving a minimal
coordinate system in which the constraint holds by construction. Everything (region generation,
planning, optimization) then happens in the low-dimensional parameterized space, and only at the
very end is the result mapped back to the full configuration space.

The deliverable is the pair of notebooks; `src/` and `cpp_parameterization/` exist to support them.

## The central abstraction: the parameterization

`q_tilde` (8-D) → `q_full` (14-D):

- `q_tilde[:7]` — the **leader** arm's joint angles, passed through unchanged to `q_full[:7]`.
- `q_tilde[7]` — `psi`, the **self-motion / nullspace angle** of the follower arm (7-DoF redundancy).
- `q_full[7:]` — the **follower** arm's joint angles, computed by analytic IK from the leader's
  forward kinematics + the fixed grasp offset (`grasp_distance`) + `psi`.
- `GC2`, `GC4`, `GC6` (Python: `±1`; C++: `shoulder_up`/`elbow_up`/`wrist_up` booleans) select the
  IK branch. They must stay fixed for a whole planning problem — changing them jumps to a different
  sheet of the manifold.

Two properties of this function are load-bearing and must be preserved by any change:

1. **Scalar-type genericity.** The mapping must work with both `float` and Drake's `AutoDiffXd`
   (and numpy arrays of each), so Drake can push gradients through it via the chain rule. In Python
   this means using `pydrake.math` functions and explicit `dtype=type(q_tilde[0])` / `RigidTransform_[T]`
   templating rather than plain numpy math. In C++ this is the `DoEvalGeneric<T>` template pattern.
2. **Finite gradients at the domain boundary.** Inputs to `arccos` are clipped to `[-0.9999, 0.9999]`
   (`scalar_clip` / `safe_arccos` in `src/iiwa_analytic_ik.py`). Whether the clip was active is
   exactly what the *reachability* constraint tests: `analytic_ik.IK(..., return_unclipped_vals=True)`
   returns the 4 pre-clip values, constrained to `[-1+eps, 1-eps]`.

Because IK is only defined on part of the domain, the parameterized space needs two extra
constraints that the full space would not: **reachability** (above) and **follower joint limits**
(the leader's limits are just box bounds on `q_tilde[:7]`). Both are supplied to IRIS via
`sampled_iris_options.prog_with_additional_constraints` — a `MathematicalProgram` whose only
decision variables are the 8 `q_tilde` variables.

## Python vs. C++ parameterization

There are two parallel implementations of the same math. They are kept deliberately in sync;
**a change to one usually needs the matching change to the other.**

| | Python | C++ |
|---|---|---|
| IK / parameterization | `src/iiwa_analytic_ik.py` | `cpp_parameterization/cpp/iiwa_ik/iiwa_analytic_ik.h`, `parameterization.cc` |
| Constraints | built inline in the notebook via `iris_prog.AddConstraint(callable, ...)` | `constraints.cc` (`IiwaBimanualReachableConstraint`, `IiwaBimanualJointLimitConstraint`, `IiwaBimanualCollisionFreeConstraint`, `FullFeasibilityConstraint`) |
| Costs | inline | `costs.cc` (`IiwaBimanualPathCost`, `IiwaBimanualKineticEnergyPathCost`) |
| Notebook | `notebooks/main.ipynb` | `notebooks/main_cpp.ipynb` |

The two notebooks are otherwise near-identical; diff them to see exactly what swapping in C++
changes. The C++ path is ~orders of magnitude faster (region generation ~1.2s) and is the only way
IRIS-ZO parallelizes, since a Python parameterization callback cannot be called concurrently from
C++ — `MakeParameterization(...).get_parameterization_is_threadsafe()` is `True` only for C++.

`IiwaBimanualKineticEnergyPathCost` is the one piece with no Python counterpart, because it needs
a mass matrix. It measures each control-point displacement under the mass matrix rather than the
Euclidean norm — `sum_i df_i^T M(f(q~_mid,i)) df_i`, the kinetic-energy Riemannian metric of
arXiv:2602.00992, with `M` evaluated at the *lifted midpoint* of each control point pair. Three
things about it differ from every other class here and are easy to get wrong:

- It owns a `MultibodyPlant<AutoDiffXd>` copy plus contexts for both scalar types, so it is **not**
  thread safe and does **not** support symbolic evaluation (the symbolic `DoEval` throws).
- Its `DoEval(AutoDiffXd)` does not just call a `DoEvalGeneric<T>`. Each segment touches only two
  control points, so it seeds a *local* AutoDiff vector of `2 * num_positions` partials, evaluates
  the segment, and chains the result into the output gradient. Propagating all
  `num_positions * num_control_points` partials through the mass matrix is prohibitively slow.
- Drake's `AutoDiffXd::derivatives()` returns an *expression* (`drake::ad::DerivativesConstXpr`),
  not a reference. Binding it to `const auto&` gives a dangling temporary and silently corrupt
  gradients; it must be evaluated into an `Eigen::VectorXd`.

Both costs take an optional trailing `scale`, defaulted to 1.0 so existing callers are unaffected.

## Scripts

`scripts/compare_trajopt_metrics.py` is a headless experiment (not part of the tutorial) comparing
three trajectory-optimization objectives — parameterized path energy, C-space path energy, and
kinetic energy — by the duration of the resulting trajectory after TOPPRA retiming. It reproduces
the notebook's environment, RRT, shortcut, trajopt, lift and retime steps in one file, runs 6
configuration pairs x 10 RRT plans x 3 metrics x 2 SNOPT settings across a process pool, and writes
`results/trajopt_metric_comparison.csv`. Each metric is normalized to 1.0 at the initial guess so
the solver's relative tolerances mean the same thing for all three. `--smoke` runs a single plan;
`--analyze-only <csv>` re-prints the summary. Requires the C++ module.

Independent variations, each writing its own CSV via `--label`: `--toppra-limits no-acceleration`,
`--no-shortcut`, `--solver ipopt`, `--payload-mass 28` (28kg = the two IIWA-14s' combined rated
payload; welded half-and-half to the grippers at the object centre, which is *measured* from the
kinematics rather than hard-coded, and carries no collision geometry so paths stay comparable), and
`--torque-limit-scale` (derates every joint's torque limit; torque only begins to bind below ~45% of
nominal). Alongside duration the summary reports the energy of the executed trajectory — mechanical
work, positive work, the thermal proxy `int tau^2`, integrated KE and peak KE.

**Results live in `docs/trajopt_metric_findings.md`** — read that before touching this experiment
or drawing conclusions from it. It carries the full tables, the hardware comparison against the
Franka Panda (the intended eventual platform), the caveats, and an explicit statement of what the
experiment does *not* establish. The seven raw per-solve CSVs backing it are committed in `results/`
as force-added exceptions to that directory's ignore rule, so its numbers can be checked with
`--analyze-only` without a re-run. What follows is the condensed version.

**What the experiment found (seven runs, n=58 paired solves each).** The kinetic-energy objective beats
the C-space one on post-TOPPRA duration by a median of only 0.8%-3.6%, and that band does not move
under any variation tried: derating torque to 50% or 30%, removing acceleration limits, skipping the
shortcut, switching to IPOPT, or adding the 28 kg payload. The reason is structural — velocity is
saturated on 100% of solves in every configuration, and the velocity and acceleration limits are
purely kinematic, so they carry no inertia term for an inertia-aware objective to exploit. The 30%
torque run is the decisive one: it takes torque from never binding to binding on 100% of solves with
nothing else changed, and duration moves from -3.3% to -3.2%, i.e. not at all.

Where inertia awareness does pay is *energy*, and it scales with how much `M(q)` varies. Under the
28 kg payload the kinetic objective cuts the thermal proxy by 13.7% and executed kinetic energy by
16.4% (versus 3.1% and *+1.5%* unloaded) — and duration still does not respond. That gain is *not*
from torque binding (it binds in the 30% run too, where the energy benefit vanishes); it is from the
payload raising the mass matrix's eigenvalue spread from 6.8x to 19.7x. Summary: **inertia awareness
buys energy, not time, and what it buys scales with metric anisotropy rather than torque tightness.** The kinetic objective also corner-cuts about half as often as the others, so it is the
best-behaved of the three against the hardware caveat below.

Note when interpreting peak limit-utilization numbers: the *median* utilization is the trustworthy
statistic. Peak acceleration utilization routinely reads 1.2-1.6 because the lifted trajectory's
second derivative comes from a five-point stencil whose outliers are artifacts, not real violations.

**Hardware context (computed from the Drake URDFs, 4000 sampled configurations).** The IIWA-14 is
torque-rich: its motors deliver 2.2-6.7x the acceleration its `drake:acceleration` limits permit.
The Franka Panda — the intended eventual platform — is *not* more torque-limited despite having 3.7x
less shoulder torque, because its links are ~3x lighter; its minimum headroom is 2.69 versus the
IIWA's 2.16. What genuinely differs is metric anisotropy: the Panda's mass matrix has a 16.6x median
eigenvalue spread against the IIWA's 6.8x. Loaded to their respective rated payloads (14 kg and
3 kg — note two Pandas carry 6 kg total, not 28) the two arms reach near-identical torque headroom
(0.78 vs 0.79), so the payload run transfers to the Panda; the Panda is the more anisotropic of the
two (26.1x vs 19.7x), predicting larger energy gains and, if anything, a weaker duration result.

**Score solves by feasibility, not by `result.is_success()`.** SNOPT exit codes 3 ("accuracy not
achieved") and 41 ("point cannot be improved") return usable points — empirically 34/34 of the
code-41 solves and 29/31 of the code-3 solves are feasible. Filtering on the success flag discarded
a third of the data and changed the conclusion. `Usable()` in the script instead checks the densely
sampled trajectory against tolerances (`1e-3` on the analytic violations, since converged solves
routinely sit a few `1e-5` rad outside a joint limit) plus a collision-free fraction. The genuine
failures (code 31) concentrate on initial guesses that hug the reachability boundary, where the
clipped `arccos` flattens the constraint gradient; they hit all three objectives on the same RRT
seed, so they are a property of the guess, not of the metric.

## Commands

Python-only path:

```bash
pip install drake tqdm matplotlib networkx ipywidgets jupyter scipy pyyaml pydot
jupyter notebook   # then open notebooks/main.ipynb
```

C++ path (requires a binary or source Drake install):

```bash
export DRAKE_INSTALL_DIR=/path/to/drake/installation
cmake -S cpp_parameterization -B cpp_parameterization/build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR
cmake --build cpp_parameterization/build --target _iiwa_ik -j$(nproc)

# MUST come first on the path, or you get an ABI mismatch with the pip-installed drake.
export PYTHONPATH=$DRAKE_INSTALL_DIR/lib/python3.10/site-packages:$PYTHONPATH

python3 cpp_parameterization/test/test.py   # the only test: ABI smoke check + numerical checks
```

`test/test.py` is the only test, and its convention is one bare `print(...)` of a boolean per check;
every line should read `True`. The first four checks are an ABI smoke test — they confirm the
pybind11 classes are recognized as Drake `Constraint`/`IrisParameterizationFunction` subclasses and
that the parameterization evaluates. An ABI mismatch typically shows up as those `isinstance` checks
printing `False`, or as a segfault. The checks after them are numerical (cost values against a NumPy
reimplementation, AutoDiff gradients against central finite differences, the `scale` factor) and are
labelled, since an unlabelled `False` in a long list is hard to track down. Add new checks in that
same style rather than reaching for a test framework.

For an optimized build (`-O3 -flto -ffast-math ...`) see `cpp_parameterization/README.md`. Do **not**
add `-march=native` — it causes Eigen alignment/pybind failures.

The build writes `_iiwa_ik.cpython-*.so` directly into `cpp_parameterization/python/iiwa_ik/`, which
is what the notebook adds to `sys.path`. `.so` files are gitignored, so a fresh clone must build
before `main_cpp.ipynb` will run.

Docker (`cohnt/constrained-bimanual-planning-example:latest`) ships Drake + a prebuilt C++ module;
see `docker/README.md`. Jupyter on 8888, Meshcat on 7000.

## Notebook workflow

Both notebooks follow the same arc; when editing, keep the ordering intact since later cells depend
on earlier state:

1. **Parameters** — `directives_file`, `grasp_distance`, IK branch flags, and the hard-coded key
   configurations `q_tilde_bottom` / `q_tilde_middle` / `q_tilde_top` plus the `seeds` list.
2. **Environment** — `RobotDiagramBuilder` → `plant` (models loaded via `package.xml` +
   `models/*.dmd.yaml` directives) → `SceneGraphCollisionChecker`. Two `MeshcatVisualizer`s are
   attached (illustration and proximity roles). The diagram's actuation input and state output are
   exported so it can later be wrapped in a simulation diagram.
3. **Regions** — `IrisNp2` (default) or `IrisZo`, given `IrisParameterizationFunction(parameterization, 8)`,
   the extra-constraints program, and a `domain` box (IIWA joint limits for the 7 leader joints,
   `[0, 2*pi]` for `psi`). Regions are grown in the 8-D space.
4. **Planning** — three interchangeable planners, all producing an 8-D trajectory:
   `GcsTrajectoryOptimization` over the regions; `src/rrt.py` BiRRT + `src/shortcut.py`;
   `KinematicTrajectoryOptimization` seeded from the RRT path.
5. **Lift and retime** — wrap the 8-D trajectory in a `FunctionHandleTrajectory` of dimension 14 via
   the parameterization, then `Toppra` for velocity/acceleration limits, then
   `PathParameterizedTrajectory`. Derivatives of the lifted trajectory are computed by finite
   differencing (a five-point stencil), since `FunctionHandleTrajectory` has no analytic derivative.
6. **Visualize** — Meshcat playback, plus a simulation diagram with `TrajectorySource` +
   `InverseDynamicsController` exported to `trajectory.html`.

Toppra grid-point selection is finicky (see the linked Drake issue in the notebook); a retimed
duration of `inf` means the grid points need adjusting.

## Conventions and gotchas

- `src/*.py` is tab-indented; the notebooks and C++ use spaces. Match the file you are editing.
- Notebooks import `src` via `sys.path.append("..")`, so they only run from within `notebooks/`.
  `src/common.py:RepoDir()` is the canonical way to build paths to `models/` and `package.xml`.
- The notebooks are heavily commented as teaching material. Explanatory markdown and inline comments
  are part of the product — preserve and extend them rather than tightening code at their expense.
- Many cells intentionally ship commented-out alternatives (IrisZo vs IrisNp2, the two
  `KinematicTrajectoryOptimization` initial-guess strategies, growing regions along an RRT path,
  which planner's trajectory to retime). Do not delete these as dead code.
- `notebooks/extracted_cells.py` is a scratch dump of notebook cells, not a source file; `snopt.log`
  is solver output. Neither is tracked (the root-level `snopt.log` is large and untracked despite
  the gitignore only covering `notebooks/snopt.log`). `results/` is likewise gitignored.
- The C++ API is kept backwards compatible: new constructor arguments go at the end with defaults,
  so the notebooks keep working without edits.
- **Deferred, must be fixed before hardware:** `KinematicTrajectoryOptimization` enforces the
  feasibility constraints only at 100 sample points, so the optimized trajectory clips obstacles
  slightly between them — about 30% of solves touch collision somewhere, for under 1% of the path
  (worst observed 0.8%, i.e. 4 densely-sampled configurations out of 500). This is harmless in
  simulation and does not bias the metric comparison (all three objectives clip at similar rates,
  and the kinetic-energy one is the cleanest), so it is deliberately not corrected. Denser
  constraint sampling, or a path-integral collision constraint, is the fix. The same applies to
  follower joint limits, which drift a few `1e-5` rad past their bounds and can simply be projected.
  `scripts/compare_trajopt_metrics.py` prints both in its summary so they stay visible.
