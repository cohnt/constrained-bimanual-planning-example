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