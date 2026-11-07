# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
#
# The dishsim runtime image for the Brev box dishsim-l4 (L4, driver 595): Isaac Sim 6.2.0 + Isaac Lab
# v3.0.0 + the pinned planning deps, all installed into Kit's own python 3.12 — deliberately no venv
# (the old Brev venv was the sole cause of the EXP_PATH launcher landmine). corallab (driver 535)
# stays on the Isaac Sim 4.5 image this file built before 2026-11-01 (git tag isaac45-final).
#
# The repo itself is never baked in; the container bind-mounts it at /workspace/dishsim and
# editable-installs it on start (code/util/docker/compose.yaml's entrypoint).
#
# Build (from the repo root; the context is the repo so requirements-planning.txt is
# COPYable, .dockerignore keeps everything else out):
#   docker build -f code/util/docker/Dockerfile -t dishsim-isaac:6.2.0 .

FROM nvcr.io/nvidia/isaac-sim:6.2.0

# The 6.2 image ends as uid 1234; the dishsim container runs as root (root-owned outputs, CLAUDE.md).
USER root

ENV ACCEPT_EULA=Y \
    PRIVACY_CONSENT=Y \
    ISAACSIM_PATH=/isaac-sim \
    DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

# git for the Isaac Lab clone; cmake/build-essential only as a fallback for any planning
# wheel that has no cp312 binary and must compile.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab v3.0.0, pinned to the tag (main has dropped isaaclab.sh and changed PhysicsManager).
# Editable installs into Kit's python with --no-deps: 3.0 keeps its third-party pins in the root
# pyproject (torch, newton, ovstage, ...) and Kit already ships numpy. Besides isaaclab +
# isaaclab_physx, Isaac Lab 3.0's core imports three more workspace packages at module scope:
# isaaclab.app.sim_launcher the isaaclab_newton and isaaclab_ov config modules, and
# isaaclab.scene.interactive_scene isaaclab_contrib's tacsl sensor cfg (no Newton/OVRTX runtimes).
# lazy_loader, rich and prettytable are the light modules the core import path needs. Kit 6.2's
# python ships no setuptools, which --no-build-isolation needs (pins from Isaac Lab's build-system).
RUN git clone --depth 1 --branch v3.0.0 https://github.com/isaac-sim/IsaacLab.git /workspace/isaaclab \
    && ln -s /isaac-sim /workspace/isaaclab/_isaac_sim \
    && cd /workspace/isaaclab \
    && /isaac-sim/python.sh -m pip install --no-cache-dir "setuptools>=70.0,<82.0.0" wheel \
    && /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation --no-deps \
        -e source/isaaclab -e source/isaaclab_physx -e source/isaaclab_newton -e source/isaaclab_ov \
        -e source/isaaclab_contrib \
    && /isaac-sim/python.sh -m pip install --no-cache-dir "lazy_loader>=0.4" "prettytable>=3.3.0" rich \
    && /isaac-sim/python.sh -c "import isaaclab, isaaclab_physx, isaaclab_newton, isaaclab_ov, isaaclab_contrib, isaaclab.utils"

# Isaac Lab 3.0 imports torch and warp before Kit starts (isaaclab.app included), but Isaac Sim 6.2 keeps
# them in extensions that load with Kit (torch only in the deprecated omni.isaac.ml_archive, which 3.0's
# apps no longer load; warp 1.17.0 in omni.warp.core); and every PhysX Articulation imports newton
# (isaaclab.actuators.newton), which Kit does not load either. Install Isaac Lab's own pins from its root
# pyproject: torch 2.11.0 on cu130 (as its installer does), warp-lang 1.17.0 (the version Kit bundles)
# and newton 1.6.1 (base package only: its one requirement is warp-lang).
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cu130 torch==2.11.0 \
    && /isaac-sim/python.sh -m pip install --no-cache-dir warp-lang==1.17.0 newton==1.6.1 \
    && /isaac-sim/python.sh -c "from isaaclab.app import add_launcher_args, launch_simulation; import newton.actuators, newton.selection"

# Planning stack into Kit's python (pins from requirements-planning.txt; adjusted there,
# never here, if a cp312 wheel is missing). imageio-ffmpeg is imageio's ffmpeg backend for
# dishsim.media.VideoWriter (libx264).
COPY requirements-planning.txt /tmp/requirements-planning.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements-planning.txt \
    && /isaac-sim/python.sh -m pip install --no-cache-dir pytest huggingface_hub requests pyyaml filelock tqdm fsspec \
        imageio-ffmpeg==0.6.0 \
    && rm /tmp/requirements-planning.txt

# CP-SAT for the bench's fit (code/src/bench/build.py, plans/2026-11-02-fit-score-prove.md). --no-deps keeps Kit's own
# numpy (2.3.1) and protobuf: ortools.sat.python.cp_model imports numpy and pandas only (pandas 3.1.0 added when Kit's
# python lacks it; python-dateutil comes with matplotlib). The live container got the same lines in the 2026-11-02 window.
RUN /isaac-sim/python.sh -m pip install --no-cache-dir --no-deps ortools==9.15.6755 absl-py==2.5.1 immutabledict==4.3.1 \
    && (/isaac-sim/python.sh -c "import pandas" 2>/dev/null || /isaac-sim/python.sh -m pip install --no-cache-dir --no-deps pandas==3.1.0) \
    && /isaac-sim/python.sh -c "from ortools.sat.python import cp_model; m = cp_model.CpModel(); m.NewBoolVar('x'); assert cp_model.CpSolver().Solve(m) == cp_model.OPTIMAL"

WORKDIR /workspace/dishsim
