cmake_minimum_required(VERSION 3.18)
project(iiwa_ik)

# User must supply the Drake installation via CMAKE_PREFIX_PATH.
# Example:
#   cmake .. -DCMAKE_PREFIX_PATH=/path/to/drake/install
find_package(drake REQUIRED)
find_package(pybind11 REQUIRED)

# Build module named _iiwa_ik (not iiwa_ik) so Python package exports it cleanly
pybind11_add_module(_iiwa_ik
    iiwa_ik/bindings.cc
)

target_link_libraries(_iiwa_ik PRIVATE drake::drake)

# Output the shared library directly into the Python package directory:
#   <repo-root>/python/iiwa_ik/
set(PY_PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../python/iiwa_ik)

set_target_properties(_iiwa_ik
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PY_PACKAGE_DIR}
)
