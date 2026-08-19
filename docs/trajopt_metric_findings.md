# Comparing trajectory optimization objectives by post-TOPPRA duration

A summary of what `scripts/compare_trajopt_metrics.py` measured.

## Orientation, if you are picking this up cold

Read `CLAUDE.md` first for the repository's central abstraction (the 8-D -> 14-D analytic-IK
parameterization); this report assumes it.

Every number below can be re-derived without re-running the experiment. The raw per-solve CSVs are
committed in `results/` -- one per run, named `e_<run>.csv` -- and the summary that produced these
tables is printed by:

```bash
export PYTHONPATH=$DRAKE_INSTALL_DIR/lib/python3.12/site-packages:$PWD/cpp_parameterization/python:$PYTHONPATH
python3 scripts/compare_trajopt_metrics.py --analyze-only results/e_baseline.csv
```

`results/` is otherwise gitignored; those seven files are force-added exceptions so this report can
be checked. The hardware tables in the "Hardware context" section are *not* from those CSVs -- they
are computed directly from the Drake URDFs, and the section says how.

Re-running an experiment (hours, needs the C++ module built per `cpp_parameterization/README.md`):

```bash
python3 scripts/compare_trajopt_metrics.py --jobs $(nproc) --label my_run   # add variation flags
```

## The question

`notebooks/main_cpp.ipynb` optimizes under a **C-space path energy** objective. The Riemannian
planning literature ([Kyaw and Kelly](https://arxiv.org/abs/2602.00992)) instead uses the
mass-inertia matrix as the metric tensor, so that geodesics minimize the kinetic energy needed to
traverse a path in a given time. Does that produce meaningfully shorter trajectories once TOPPRA
retimes them against the robot's velocity, acceleration and torque limits?

All three objectives are the same discrete sum over consecutive B-spline control points, differing
only in the metric tensor, so the comparison is apples to apples:

| | objective | metric tensor |
|---|---|---|
| parameterized | `sum ‖q~_i - q~_{i-1}‖^2` | `I` |
| cspace | `sum ‖f(q~_i) - f(q~_{i-1})‖^2` | `J^T J` |
| kinetic | `sum df_i^T M(f(q~_mid,i)) df_i` | `J^T M J` |

Each run is 6 ordered configuration pairs x 10 BiRRT plans x 3 objectives, paired on the RRT seed so
every objective optimizes the identical initial guess. Solves are scored by dense feasibility of the
returned trajectory, not by the solver's success flag.

## Headline: the duration effect is real, small, and immovable

Kinetic versus cspace, median change in post-TOPPRA duration (n = 58 paired solves per run):

| run | median | wins | p |
|---|---|---|---|
| baseline | -3.3% | 69% | 2e-6 |
| torque limits at 50% | -3.3% | 69% | 2e-6 |
| no acceleration limits | -3.6% | 69% | 5e-6 |
| **torque limits at 30%** | **-3.2%** | **69%** | **3e-6** |
| no shortcut (raw RRT guess) | -2.6% | 70% | 9e-5 |
| IPOPT instead of SNOPT | -1.5% | 57% | 3e-3 |
| 28 kg payload | -0.8% | 59% | 5e-3 |

Seven configurations spanning two solvers, two initial-guess strategies, three torque regimes, a
removed acceleration limit and a rated-payload load. The effect never leaves the -0.8% to -3.6%
band. The parameterized objective, by contrast, is 60-70% *worse* than either of the others in every
run, so the experiment is clearly sensitive enough to detect a real difference when one exists.

## Why: the duration floor is kinematic

**Velocity is saturated on 100% of solves in every run.** The velocity and acceleration limits are
purely kinematic -- they carry no inertia term -- so an inertia-aware objective has no lever on them.

The `torque limits at 30%` run isolates this. Derating torque to 30% takes it from never binding to
binding on **100%** of solves, with nothing else changed: same robot, same inertia, same metric
anisotropy. The duration advantage moved from -3.3% to -3.2%, which is nothing.

Making torque the binding constraint does not help an inertia-aware objective. That was the
experiment's main open hypothesis, and it is answered in the negative.

## Where inertia awareness does pay: energy, and only via anisotropy

Kinetic versus cspace on the executed trajectory:

| run | thermal `int tau^2` | executed `int KE` | peak KE |
|---|---|---|---|
| baseline | -3.1% | +1.5% | +11% |
| torque limits at 30% | +1.0% | +2.7% | +4.8% |
| **28 kg payload** | **-13.7%** | **-16.4%** | **-8.1%** |

The payload run is the only one with a large energy benefit, and the 30% torque run explains why it
is not about torque: torque binds on 100% of solves there too, and the energy benefit disappears.
What the payload changes is how much `M(q)` **varies** -- the mass matrix's median eigenvalue spread
goes from 6.8x to 19.7x. That variation is the entire source of the objective's leverage, which
stands to reason: if `M` were constant the kinetic metric would be a fixed change of variables on
the C-space metric and would select the same path.

**Summary: inertia awareness buys energy, not time, and the amount it buys scales with metric
anisotropy rather than with how tight the torque limits are.**

## Hardware context, and what transfers to the Panda

Computed from the Drake URDFs over 4000 sampled configurations. `headroom` is the acceleration a
joint's motor can actually produce divided by the URDF's `drake:acceleration` limit; below 1.0
torque binds first.

| | min headroom | M eigenvalue spread | inertia swing |
|---|---|---|---|
| IIWA-14 unloaded | 2.16 | 6.8x | 2.0x |
| IIWA-14 + 14 kg (rated) | 0.78 | 19.7x | 3.7x |
| Panda unloaded | 2.69 | 16.6x | 3.1x |
| Panda + 3 kg (rated) | 0.79 | 26.1x | 4.0x |

Three things follow.

1. **The Panda is not more torque-limited than the IIWA-14.** Its shoulder torque is 3.7x lower
   (87 vs 320 Nm), but its links are about 3x lighter, so the ratios cancel and its headroom is
   slightly *better*. Both arms are specified so that torque never binds before acceleration.
   Derating IIWA torque is therefore not a Panda surrogate; it models an arm neither of these is.
2. **The Panda's rated payload is 3 kg**, so a bimanual Panda manipuland is about 6 kg, not the
   28 kg used here (which is the two IIWA-14s' combined rating). Do not carry 28 kg over.
3. **The payload run is the result that transfers.** At their respective rated payloads the two arms
   reach near-identical torque headroom (0.78 vs 0.79), and the Panda is the more anisotropic of the
   two. Expect *larger* energy gains on a loaded Panda than the -13.7% / -16.4% measured here, and a
   duration gain no better than, and probably weaker than, the few percent seen throughout.

## What this does *not* establish

**Whether a metric-aware sampling-based planner would do better.** Every run here used the same
BiRRT, which samples and connects under the Euclidean metric in the 8-D parameterized space, and the
same shortcutter. Only the *optimizer's* objective varied. Trajectory optimization is local: it
polishes whatever basin the initial guess lands in and does not change homotopy class. So this
experiment measures the value of an inertia-aware objective **given a metric-agnostic initial
guess**, which is a strictly weaker question than whether inertia-aware planning helps.

There is direct evidence the initial guess matters a great deal in absolute terms: with
`--no-shortcut` the optimizer improves on its guess by -87%, versus -70% when seeded from the
shortcut path. The relative ranking of the three objectives was stable across that change, but both
conditions drew their guesses from the same Euclidean-metric RRT, so that stability says nothing
about a guess distribution generated under a different metric.

A kinetic-energy-aware RRT -- the actual proposal in
[Kyaw and Kelly](https://arxiv.org/abs/2602.00992), and the original motivation for this work --
would explore a different distribution of paths, plausibly reaching basins the current pipeline
never samples. **Nothing measured here rules that out, and nothing here supports it either.** The
finding that the duration floor is kinematic does constrain how much *any* method could gain on
duration for this arm, but it says nothing about path quality in the basins a metric-aware planner
might find. Testing it requires replacing the planner, not the objective.

## Caveats

- **Corner-cutting (must be fixed before hardware).** `KinematicTrajectoryOptimization` enforces
  feasibility at only 100 sample points, so optimized trajectories clip obstacles slightly in
  between -- 19-43% of solves, for under 1% of the path. Harmless in simulation and it does not bias
  the comparison, but the fix is denser constraint sampling or a path-integral collision constraint.
  Notably the kinetic objective corner-cuts about half as often as the others.
- **Read median limit utilization, not peak.** Peak acceleration utilization routinely reads 1.2-1.6
  because the lifted trajectory's second derivative comes from a five-point stencil whose outliers
  are artifacts, not real violations.
- **Two RRT seeds out of 60 fail** under all three objectives (`bottom->middle` trial 0 and
  `top->middle` trial 8). Both have the tightest reachability margins in the set (0.013 and 0.032
  versus about 0.15 for healthy seeds), where the clipped `arccos` flattens the constraint gradient.
  This is a property of the guess, not of any objective.
- Follower joint limits drift a few `1e-5` rad past their bounds on converged solves and can simply
  be projected.
