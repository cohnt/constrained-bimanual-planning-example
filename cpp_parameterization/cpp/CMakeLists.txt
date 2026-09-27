# 3.26 is required for Python's Development.SABIModule component, which is
# what lets nanobind_add_module(STABLE_ABI) actually take effect.
cmake_minimum_required(VERSION 3.26)
project(iiwa_ik)

# User must supply the Drake installation via CMAKE_PREFIX_PATH.
# Example:
#   cmake .. -DCMAKE_PREFIX_PATH=/path/to/drake/install
find_package(drake REQUIRED)
message(STATUS "drake found at: ${drake_DIR}")

# Unlike its pybind11 builds, Drake's nanobind builds do not ship nanobind's
# headers or a nanobind CMake package, so we take nanobind from pip instead.
# The nanobind version MUST match the one Drake was built against (currently
# 3.1.0), because nanobind extensions only share a type registry when their
# nanobind ABI, NB_DOMAIN, and limited-API setting all agree. If they don't,
# drake::solvers::Constraint won't be found as a base class below.
find_package(Python REQUIRED
    COMPONENTS Interpreter Development.Module Development.SABIModule)

# Guard against the classic failure mode: building against one Drake's headers
# while `import pydrake` picks up a different one. Catch it at configure time
# rather than as a baffling import error later.
if(DEFINED drake_PYTHON_VERSION
    AND NOT Python_VERSION MATCHES "^${drake_PYTHON_VERSION}\\.")
  message(FATAL_ERROR
    "Python version mismatch: this Drake installation expects Python "
    "${drake_PYTHON_VERSION}, but CMake found Python ${Python_VERSION} at "
    "${Python_EXECUTABLE}. Point -DPython_EXECUTABLE at the interpreter whose "
    "PYTHONPATH includes ${drake_PYTHON_DIR}.")
endif()

execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_VARIABLE nanobind_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY
)
find_package(nanobind CONFIG REQUIRED)
message(STATUS "nanobind found at: ${nanobind_DIR}")

# Build module named _iiwa_ik (not iiwa_ik) so Python package exports it cleanly
#
# STABLE_ABI  -- compile against Python's limited API, as Drake does.
# NB_DOMAIN   -- must match Drake's own NB_DOMAIN so that we share pydrake's
#                nanobind type registry.
# NOMINSIZE   -- nanobind otherwise forces -Os, which would silently undo the
#                -O3 of the optimized build described in the README.
nanobind_add_module(_iiwa_ik
    STABLE_ABI
    NB_DOMAIN pydrake
    NOMINSIZE
    iiwa_ik/bindings.cc
    iiwa_ik/parameterization.cc
    iiwa_ik/constraints.cc
    iiwa_ik/costs.cc
)

target_link_libraries(_iiwa_ik PRIVATE drake::drake)

# Output the shared library directly into the Python package directory:
#   <repo-root>/python/iiwa_ik/
set(PY_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../python/iiwa_ik)

set_target_properties(_iiwa_ik
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PY_PACKAGE_DIR}
)
