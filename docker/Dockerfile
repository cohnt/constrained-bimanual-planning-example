FROM ubuntu:22.04

# =========
# System deps
# =========
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    wget \
    git \
    build-essential \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    pybind11-dev \
    libfmt-dev \
    libspdlog-dev \
    libblas3 \
    liblapack3 \
    && rm -rf /var/lib/apt/lists/*

# =========
# Install Drake binary (v1.47.0)
# =========
ENV DRAKE_VERSION=1.47.0
RUN mkdir -p /opt/drake && \
    cd /opt/drake && \
    wget https://github.com/RobotLocomotion/drake/releases/download/v${DRAKE_VERSION}/drake-${DRAKE_VERSION}-jammy.tar.gz && \
    tar -xzf drake-${DRAKE_VERSION}-jammy.tar.gz && \
    rm drake-${DRAKE_VERSION}-jammy.tar.gz

ENV DRAKE_INSTALL_DIR=/opt/drake/drake

# expose drake runtime bits
ENV LD_LIBRARY_PATH="${DRAKE_INSTALL_DIR}/lib:${LD_LIBRARY_PATH:-}"
ENV PATH="${DRAKE_INSTALL_DIR}/bin:${PATH}"
ENV PYTHONPATH="${DRAKE_INSTALL_DIR}/lib/python3.10/site-packages:${PYTHONPATH:-}"

# =========
# Python deps
# =========
RUN pip install --upgrade pip
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
RUN git clone https://github.com/cohnt/constrained-bimanual-planning-example.git /opt/proj

WORKDIR /opt/proj

# =========
# Basic Build
# =========
WORKDIR /opt/proj/cpp_parameterization

# RUN cmake -S . -B build -DCMAKE_PREFIX_PATH=${DRAKE_INSTALL_DIR}
RUN cmake -S . -B build -DCMAKE_PREFIX_PATH=$DRAKE_INSTALL_DIR \
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
