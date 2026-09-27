# Nanobind build of the constrained bimanual planning example.
#
# Drake is migrating its Python bindings from pybind11 to nanobind; the
# project-wide default is expected to change around 2026-12-01. Until then, the
# only nanobind artifacts Drake publishes are nightly builds, distinguished by
# an "a1" suffix on the version number.
FROM ubuntu:24.04

# =========
# System deps
# =========
# Just enough to fetch and unpack Drake; Drake's own installer below pulls in
# the rest (compiler, CMake, Eigen, and the runtime libraries pydrake needs).
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    wget \
    ca-certificates \
    git \
    python3 \
    python3-dev \
    python3-venv \
    libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# =========
# Install Drake binary (nanobind nightly)
# =========
# TODO(nanobind): once Drake's project-wide default is nanobind (expected around
# 2026-12-01), switch this back to a stable release from
# https://github.com/RobotLocomotion/drake/releases -- i.e. set DRAKE_VERSION to
# e.g. 1.60.0, drop the "a1" suffix, and use the GitHub releases URL again.
ENV DRAKE_VERSION=0.0.20260927a1
RUN mkdir -p /opt/drake && \
    cd /opt/drake && \
    wget https://drake-packages.csail.mit.edu/drake/nightly/drake-${DRAKE_VERSION}-noble.tar.gz && \
    tar -xzf drake-${DRAKE_VERSION}-noble.tar.gz && \
    rm drake-${DRAKE_VERSION}-noble.tar.gz

ENV DRAKE_INSTALL_DIR=/opt/drake/drake

# Let Drake install its own prerequisites rather than maintaining a duplicate
# apt list here that silently rots. The noble binary flavor needs a handful of
# libraries (libglib2.0-0t64, the GL/X stack, libgfortran5, ...) that are easy
# to miss by hand; getting one wrong shows up only as an ImportError from
# pydrake at runtime. Safe to run as root inside Docker.
RUN DEBIAN_FRONTEND=noninteractive \
    ${DRAKE_INSTALL_DIR}/share/drake/setup/install_prereqs -y && \
    rm -rf /var/lib/apt/lists/*

# expose drake runtime bits
ENV LD_LIBRARY_PATH="${DRAKE_INSTALL_DIR}/lib:${LD_LIBRARY_PATH:-}"
ENV PATH="${DRAKE_INSTALL_DIR}/bin:${PATH}"
ENV PYTHONPATH="${DRAKE_INSTALL_DIR}/lib/python3.12/site-packages:${PYTHONPATH:-}"

# =========
# Python deps
# =========
# Ubuntu 24.04 marks the system Python as externally managed (PEP 668), so
# install into a virtual environment rather than fighting pip about it.
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"
RUN pip install --upgrade pip
# nanobind must match the version Drake was built against. Extensions only
# share a type registry when their nanobind ABI, NB_DOMAIN and limited-API
# setting all agree; if they don't, drake::solvers::Constraint is invisible to
# our bindings and the build-time test below fails.
RUN pip install nanobind==3.1.0
RUN pip install \
    numpy \
    tqdm \
    matplotlib \
    networkx \
    ipywidgets \
    jupyter \
    scipy \
    pyyaml \
    pydot

# =========
# Clone your code
# =========
# TODO(nanobind): set the default back to `trust-region-experiment` when merging
# this branch.
ARG PROJECT_REF=trust-region-experiment-nanobind
RUN git clone -b ${PROJECT_REF} https://github.com/cohnt/constrained-bimanual-planning-example.git /opt/proj

WORKDIR /opt/proj

# =========
# Basic Build
# =========
WORKDIR /opt/proj/cpp_parameterization

# RUN cmake -S . -B build -DCMAKE_PREFIX_PATH=${DRAKE_INSTALL_DIR}
RUN cmake -S . -B build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR \
  -DPython_EXECUTABLE=/opt/venv/bin/python3 \
  -DCMAKE_C_FLAGS="-g -O3 -flto -fstack-protector-strong -D_FORTIFY_SOURCE=2 \
    -ffast-math -fno-math-errno -funroll-loops -finline-small-functions \
    -fprefetch-loop-arrays -fstrict-aliasing" \
  -DCMAKE_CXX_FLAGS="-g -O3 -flto -fstack-protector-strong -D_FORTIFY_SOURCE=2 \
    -ffast-math -fno-math-errno -funroll-loops -finline-small-functions \
    -fprefetch-loop-arrays -fstrict-aliasing -DEIGEN_NO_DEBUG -DEIGEN_VECTORIZE" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo;

RUN cmake --build build --target _iiwa_ik -j"$(nproc)"

# =========
# quick ABI compatibility test
# =========
RUN python3 test/test.py

WORKDIR /opt/proj

CMD ["/bin/bash"]
