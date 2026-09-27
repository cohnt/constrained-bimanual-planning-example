# Contents

The C++ implementation mirrors the Python one in `src/`, and the two are kept deliberately in sync.

- `cpp/iiwa_ik/iiwa_analytic_ik.h` -- the analytic IK and the parameterization itself, templated on the scalar type.
- `cpp/iiwa_ik/parameterization.cc` -- wraps the above in an `IrisParameterizationFunction`.
- `cpp/iiwa_ik/constraints.cc` -- the reachability, subordinate joint limit, collision-free, and all-in-one feasibility constraints.
- `cpp/iiwa_ik/costs.cc` -- path costs.
  - `IiwaBimanualPathCost` accumulates the squared (`square=true`) or unsquared distance between consecutive control points, measured in the *full* configuration space.
  - `IiwaBimanualKineticEnergyPathCost` accumulates the same displacement, but measured under the mass matrix, i.e. `sum_i df_i^T M(f(q~_mid,i)) df_i`. This is the kinetic energy Riemannian metric of [Kyaw and Kelly](https://arxiv.org/abs/2602.00992), with the metric evaluated at the midpoint of each pair of control points (their midpoint approximation). It needs a `MultibodyPlant` to get the mass matrix from, and that plant must outlive the cost. Unlike the other costs it is *not* thread safe, since it holds contexts that it writes into, and it does not support symbolic evaluation.
  - Both costs take an optional trailing `scale`, which multiplies the accumulated total. It does not change the minimizer, but it lets a caller normalize costs of differing physical units onto a common numerical scale.

Note that the gradient of the kinetic energy cost is not computed by simply propagating AutoDiff through the whole cost.
Each segment depends on only two control points, so each segment is evaluated against a *local* AutoDiff vector of `2 * num_positions` partial derivatives, and the result is chained into the full gradient.
That keeps the expensive mass matrix computation from propagating `num_positions * num_control_points` partial derivatives.

# Building and Testing

To build this C++ project, you will have to download and use a compiled Drake binary or [compile Drake from source](https://drake.mit.edu/from_source.html).
But it's worth it for the major speedups!

**This branch targets Drake's `nanobind` bindings**, which Drake currently only
publishes as nightly builds (the project-wide default is expected to change
around 2026-12-01). Grab a nightly tarball with an `a1` suffix:
```
wget https://drake-packages.csail.mit.edu/drake/nightly/drake-0.0.20260927a1-noble.tar.gz
```
The plain `drake-0.0.YYYYMMDD-noble.tar.gz` tarballs, and the
[tagged releases](https://github.com/RobotLocomotion/drake/releases), are still
`pybind11` builds and **will not work with this branch**: our extension module
and `pydrake` must be built with the same binding framework, or
`drake::solvers::Constraint` is invisible to our bindings.

It's hard for me to test this on other setups, so if you get any sort of compilation errors, please let me know!

*You can also use the [Docker image](https://hub.docker.com/repository/docker/cohnt/constrained-bimanual-planning-example/general) instead of installing and building everything locally.*
See the [docker/README.md](../docker/README.md) for instructions.

Otherwise, the instructions below show how to build and test locally.

## Prerequisites

Drake's `nanobind` builds ship `libdrake_nanobind.so` but not nanobind's headers
or its CMake package, so you need to install [`nanobind`](https://nanobind.readthedocs.io/)
yourself:
```
pip install nanobind==3.1.0
```
The version matters. Two nanobind extensions only share a type registry when
their nanobind ABI, their `NB_DOMAIN`, and their limited-API setting all agree,
so `nanobind` must match the version Drake was built against (currently 3.1.0).
`cpp/CMakeLists.txt` takes care of `NB_DOMAIN=pydrake` and the stable ABI for
you. If they don't match, you'll see
`base type "drake::solvers::Constraint" not known to nanobind` at import time.

You also need CMake 3.26 or newer (for the `Development.SABIModule` component)
and Eigen's development headers (`libeigen3-dev` on Debian/Ubuntu).

It's essential that your `PYTHONPATH` points to the Python bindings associated with the Drake installation you're compiling your C++ against.
(Otherwise, you'll get an ABI mismatch, and none of the code will work.)
The easiest way to avoid this is to simply prepend your local Drake installation to the `PYTHONPATH`, ensuring Python sees them first, with
```
export PYTHONPATH=/path/to/drake/installation/lib/python3.12/site-packages:$PYTHONPATH
```
(Make sure to replace `3.12` with the Python version you are using.)

There are some additional python packages you'll need to install with pip if you don't have them already:
```
pip install nanobind==3.1.0 numpy tqdm matplotlib networkx ipywidgets jupyter scipy pyyaml pydot;
```

## Basic Build
These commands build the project with default settings.

```bash
export DRAKE_INSTALL_DIR=/path/to/drake/installation;
cmake -S . -B build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR;
cmake --build build --target _iiwa_ik -j$(nproc);
python3 test/test.py;
```

## Optimized Build

These commands enable compiler optimizations for maximum speed, while remaining safe with AddressSanitizer and Eigen alignment.

```bash
export DRAKE_INSTALL_DIR=/path/to/drake/installation;
cmake -S . -B build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR \
  -DCMAKE_C_FLAGS="-g -O3 -flto -fstack-protector-strong -D_FORTIFY_SOURCE=2 \
    -ffast-math -fno-math-errno -funroll-loops -finline-small-functions \
    -fprefetch-loop-arrays -fstrict-aliasing" \
  -DCMAKE_CXX_FLAGS="-g -O3 -flto -fstack-protector-strong -D_FORTIFY_SOURCE=2 \
    -ffast-math -fno-math-errno -funroll-loops -finline-small-functions \
    -fprefetch-loop-arrays -fstrict-aliasing -DEIGEN_NO_DEBUG -DEIGEN_VECTORIZE" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo;
cmake --build build --target _iiwa_ik -j$(nproc);
python3 test/test.py;
```

**Warning:** `-ffast-math` is potentially dangerous, but I haven't experienced any issues with it.

**Warning:** Do not use `-march=native here` -- it can cause errors related to Eigen allocation and the Python bindings.

## What the test checks

`test/test.py` prints one `True` per check, and every line should read `True`.
The first few checks are an ABI smoke test: they confirm that the nanobind classes are recognized as Drake `Constraint` and `IrisParameterizationFunction` subclasses, and that the parameterization evaluates.
If those print `False`, or if you get a segfault, your `PYTHONPATH` is almost certainly pointing at a different Drake installation than the one you compiled against.

The remaining checks are numerical, and are labelled so a failure can be tracked down.
They compare the costs against an independent NumPy reimplementation, compare their AutoDiff gradients against central finite differences, and check that the scale factor is applied and defaults to 1.

# A Complete Build-and-Run Recipe

This assumes you have just cloned the repository, you have an appropriate Drake installation, and are starting at the root directory.
```
# Create a virtual environment
python3 -m venv venv;
source venv/bin/activate;

# Indicate the Drake installation path
export DRAKE_INSTALL_DIR=/path/to/drake/installation;

# Install remaining Python dependencies
pip install nanobind==3.1.0 numpy tqdm matplotlib networkx ipywidgets jupyter scipy pyyaml pydot;

# Build the C++ project. (You can switch in the optimized build steps.)
cd cpp_parameterization;
cmake -S . -B build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR;
cmake --build build --target _iiwa_ik -j$(nproc);

# Point your PYTHONPATH to the correct build.
export PYTHONPATH=$DRAKE_INSTALL_DIR/lib/python3.12/site-packages:$PYTHONPATH;

# Quick test for ABI compatibility
python3 test/test.py;
cd ..

# Launch the jupyter notebook.
jupyter notebook;
```
From here, you just open `notebooks/main_cpp.ipynb` and all code should run.