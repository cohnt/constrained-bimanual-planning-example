# Building and Testing

You have to [compile Drake from source](https://drake.mit.edu/from_source.html) on your machine to build this C++ project.
But it's worth it for the major speedups!

It's hard for me to test this on other setups, so if you get any sort of compilation errors, please let me know!

## Prerequisites

Someone TODO based on any reported issues.
You'll definitely need to install [`pybind11-dev`](https://pybind11.readthedocs.io/en/stable/basics.html).

## Basic Build
These commands build the project with default settings.

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/drake/installation;
cmake --build build --target _iiwa_ik -j$(nproc);
python3 test/test.py;
```

## Optimized Build

These commands enable compiler optimizations for maximum speed, while remaining safe with AddressSanitizer and Eigen alignment.

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/drake/installation \
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

**Warning:** Do not use `-march=native here` -- it can cause errors related to Eigen allocation and pybind.