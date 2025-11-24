# Building and Testing

To build this C++ project, you will have to download and use the [compiled Drake binaries](https://github.com/RobotLocomotion/drake/releases) or [compile Drake from source](https://drake.mit.edu/from_source.html).
But it's worth it for the major speedups!

It's hard for me to test this on other setups, so if you get any sort of compilation errors, please let me know!

*You can also use the Docker image instead of installing and building everything locally.*
See the [docker/README.md](../docker/README.md) for instructions.

Otherwise, the instructions below show how to build and test locally.

## Prerequisites

You may need to install [`pybind11-dev`](https://pybind11.readthedocs.io/en/stable/basics.html).

It's essential that your `PYTHONPATH` points to the Python bindings associated with the Drake installation you're compiling your C++ against.
(Otherwise, you'll get an ABI mismatch, and none of the code will work.)
The easiest way to avoid this is to simply prepend your local Drake installation to the `PYTHONPATH`, ensuring Python sees them first, with
```
export PYTHONPATH=/path/to/drake/installation/lib/python3.10/site-packages:$PYTHONPATH
```
(Make sure to replace `3.10` with the Python version you are using.)

There are some additional python packages you'll need to install with pip if you don't have them already:
```
pip install numpy tqdm matplotlib networkx ipywidgets jupyter scipy pyyaml pydot;
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

**Warning:** Do not use `-march=native here` -- it can cause errors related to Eigen allocation and pybind.

# A Complete Build-and-Run Recipe

This assumes you have just cloned the repository, you have an appropriate Drake installation, and are starting at the root directory.
```
# Create a virtual environment
python3 -m venv venv;
source venv/bin/activate;

# Indicate the Drake installation path
export DRAKE_INSTALL_DIR=/path/to/drake/installation;

# Install remaining Python dependencies
pip install numpy tqdm matplotlib networkx ipywidgets jupyter scipy pyyaml pydot;

# Build the C++ project. (You can switch in the optimized build steps.)
cd cpp_parameterization;
cmake -S . -B build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR;
cmake --build build --target _iiwa_ik -j$(nproc);

# Point your PYTHONPATH to the correct build.
export PYTHONPATH=$DRAKE_INSTALL_DIR/lib/python3.10/site-packages:$PYTHONPATH;

# Quick test for ABI compatibility
python3 test/test.py;
cd ..

# Launch the jupyter notebook.
jupyter notebook;
```
From here, you just open `notebooks/main_cpp.ipynb` and all code should run.