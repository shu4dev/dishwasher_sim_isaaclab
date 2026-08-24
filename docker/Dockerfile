# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# The dishsim runtime image for the corallab workstation: Isaac Sim 4.5.0 (the newest
# release NVIDIA documents for the host's 535 driver) + Isaac Lab v2.1.1 + the pinned
# planning deps, all installed into Kit's own python 3.10 — deliberately no venv (the old
# Brev venv was the sole cause of the EXP_PATH launcher landmine).
#
# The repo itself is never baked in; docker/compose.yaml bind-mounts it at
# /workspace/dishsim and editable-installs it on container start.
#
# Build (from the repo root; the context is the repo so requirements-planning.txt is
# COPYable, .dockerignore keeps everything else out):
#   docker build -f docker/Dockerfile -t dishsim-isaac:4.5.0 .

FROM nvcr.io/nvidia/isaac-sim:4.5.0

ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    ISAACSIM_PATH=/isaac-sim \
    DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

# git for the Isaac Lab clone; cmake/build-essential only as a fallback for any planning
# wheel that has no py3.10 binary and must compile.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab v2.1.1 — the 2.x line paired with Isaac Sim 4.5. --install none skips every RL
# framework (the RL pipeline lives in the repo's git history only).
RUN git clone --depth 1 --branch v2.1.1 https://github.com/isaac-sim/IsaacLab.git /workspace/isaaclab \
    && ln -s /isaac-sim /workspace/isaaclab/_isaac_sim \
    && cd /workspace/isaaclab \
    && ./isaaclab.sh --install none \
    # isaaclab.sh silently loses the CORE package: its isolated build env pulls a setuptools
    # without pkg_resources, which 2.1.1's setup.py imports. Install it against Kit's own
    # setuptools and prove the import, or fail the build.
    && /isaac-sim/python.sh -m pip install --no-build-isolation -e source/isaaclab \
    && /isaac-sim/python.sh -c "import isaaclab"

# Planning stack into Kit's python (pins from requirements-planning.txt; adjusted there,
# never here, if a py3.10 wheel is missing).
COPY requirements-planning.txt /tmp/requirements-planning.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements-planning.txt \
    && /isaac-sim/python.sh -m pip install --no-cache-dir pytest huggingface_hub requests pyyaml filelock tqdm fsspec \
    && rm /tmp/requirements-planning.txt

WORKDIR /workspace/dishsim
