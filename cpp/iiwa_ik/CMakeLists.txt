cmake_minimum_required(VERSION 3.18)
project(iiwa_ik)

find_package(pybind11 REQUIRED)
find_package(drake REQUIRED)

pybind11_add_module(iiwa_ik bindings.cc)
target_link_libraries(iiwa_ik PRIVATE drake::drake)