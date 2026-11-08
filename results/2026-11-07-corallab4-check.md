# Check note: can this project run on corallab4? (2026-11-07, read-only survey over ssh)

The user's decision: from 2026-11-07 the project runs on corallab4, not on the Brev box `dishsim-l4`. This note records what the machine has, what the stack needs (Isaac Sim 6.2.0 in the docker image `dishsim-isaac:6.2.0`, Isaac Lab 3.0.0, python 3.12), the risks, and the proposed path. Nothing was installed or changed on the machine.

## The machine (survey 16:45 local, `ssh corallab4`, user shu4)

| Item | corallab4 | Isaac Sim 6.2.0 requirements page (x86_64) | Verdict |
|---|---|---|---|
| OS, kernel, glibc | Ubuntu 24.04.5, kernel 7.0.0-42-generic, glibc 2.39 | Ubuntu 22.04 / 24.04 (22.04 deprecated) | ok |
| GPU | GeForce RTX 3090, 24 GB, compute 8.6 (Ampere), RT cores | minimum card listed RTX 4080 16 GB; only "GPUs without RT Cores (A100, H100)" excluded | ok on paper, older than the listed minimum (see risk 3) |
| Driver | 595.99.02, `nvidia-driver-595-open`, modules nvidia, nvidia_uvm, nvidia_drm loaded, CUDA 13.2 | tested on 595.58.03 | ok (newer) |
| CPU | i9-11900KF, 8 cores / 16 threads, governor powersave | minimum 4 cores, good 8, ideal 16 | ok ("good") |
| RAM | 31 GiB, 8 GB swap, 25 GiB free with the desktop session up | minimum 32 GB, good 64 GB | borderline (risk 2); the box had 30 GiB |
| Disk | 937 GB NVMe (Micron 2300), 829 GB free | ideal 1 TB NVMe | ok |
| Containers | no docker, podman, apptainer; `nvidia-ctk` absent; AppArmor restricts unprivileged user namespaces (=1) | the project's stack is a docker image | risk 1 |
| Session tools | no tmux or screen; nohup and setsid present; no curl; git 2.x, rsync, python3 3.12.3 | setsid nohup is the project's convention already | ok |
| Privileges | shu4 is in `sudo` (password required) and `lxd`; no sudo without a password | docker install needs sudo once | ok, the user runs it |
| Python | miniforge3 with one env `cs593` (py3.10, torch 2.6.0+cu124); no isaacsim, isaaclab or IsaacLab checkout anywhere | the image carries everything | nothing to reuse |
| Display | gdm active, a desktop session on tty2, `sunshine` streaming on the GPU (264 MiB); DISPLAY unset over ssh | everything here runs `--headless` | ok, shared GPU (risk 5) |
| Network | github.com, huggingface.co, pypi.org, pypi.nvidia.com/isaacsim, catalog.ngc.nvidia.com, download.docker.com, nvidia.github.io reachable; nvcr.io/v2 answers 401 (the normal anonymous challenge) | image base `nvcr.io/nvidia/isaac-sim:6.2.0` is public | ok |
| Repo and data | no checkout; `~/Workspace/MetaFine` only; `/media/shu4` empty | data/assets is 13 MB on the Mac; results, media and logs live on the box | risk 4 |
| apt candidates | `docker.io` 29.1.3 from Ubuntu; `docker-ce` not configured; NVIDIA's container toolkit repo reachable | | |

Requirements source: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html (page dated 2026-10-14).

## Risks, ranked, each with its solution

1. **No Docker, so nothing of the stack runs today.** The launchers `run_kit.sh` and `run_py.sh` docker-exec into `dishsim-isaac`; the image bakes Isaac Lab 3.0, its torch 2.11 cu130, warp 1.17, newton 1.6.1 and the planning pins. Solution: install `docker.io` and `nvidia-container-toolkit` (sudo, about 10 minutes, the script below), add shu4 to the docker group, then build the image from `code/util/docker/Dockerfile` as on the box (the box's build took 1,470 s on 4 cores; expect 20 to 40 minutes here, 60 to 70 GB of disk), and point `compose.yaml`'s nine host paths (`/media/corallab-s1/2tbhdd/brianshu/...`, the old corallab drive) at a folder on the NVMe, for example `/home/shu4/dishsim-cache`. Everything else in CLAUDE.md's workflow stays the same, the cpu pinning included (16 threads: siblings are j and j+8, not j+4). Rootless docker or podman would avoid sudo but Ubuntu 24.04's AppArmor rule blocks unprivileged user namespaces unless sudo changes it, so they save nothing. A native pip install (isaacsim 6.2.0 plus Isaac Lab 3.0 in a py3.12 conda env) needs no sudo but rebuilds the Dockerfile's six pins and three import-order fixes by hand, runs under the host's 1,024 open-file limit, and gives a second stack whose settle fixed points differ from the image's; rejected unless sudo is refused.
2. **RAM 31 GiB against a 32 GB minimum.** The box ran three or four Kit workers on 30 GiB. Keep three workers at most, and log out of the desktop session before long rounds (it holds about 5 GB with sunshine). Watch for the `malloc(): corrupted top size` deaths CLAUDE.md describes; they get more likely under memory pressure.
3. **The RTX 3090 sits below the page's minimum card.** It has RT cores and 24 GB, and nothing on the page excludes Ampere, so RTX rendering should work; the first `--enable_cameras` boot compiles shaders (about 4 minutes on the box). Solution: day one runs `kit_smoke.py --headless --enable_cameras` and judges the still. Separately, PhysX settle fixed points are per stack and this is a new CPU (Intel i9 after the EPYC): treat the two kept samples (s0, s10 under data/results/bench/samples) as unverified until `gate_initial` and `certify` pass here, and expect to regenerate samples.
4. **The data lives on the box.** The Mac holds data/assets (13 MB), 3.7 GB of results and 1.4 GB of media; the complete `data/results`, `data/media` and `data/logs` are only on `dishsim-l4`. Solution: one last box start to rsync `data/` to corallab4 (box to corallab4 directly if ssh between them works, otherwise through the Mac), then stop the box; delete it only after the copy is checked, with the user's OK. Restore the assets by rsync from the Mac (no `hf` CLI on corallab4; `pip install huggingface_hub` is a one-liner when the hub is needed again).
5. **The GPU is shared with a desktop and a game stream.** Headless Kit does not need the display, but a stream or a game during a round contends for VRAM and time. Solution: run rounds when the desktop is idle; `nvidia-smi` before launching, as CLAUDE.md asks for corallab4.
6. **Small ones.** No tmux: keep the `setsid nohup ... taskset -c j,j+8` convention. Git over https works for pull (public repo); the machine has no GitHub credential, so commits keep leaving through the Mac, or by rsync as since 2026-11-05. The CPU governor is powersave (a `cpupower` change needs sudo; optional). The refactor branch `worktree-agent-afdb8fb5e23a54661` was waiting for a box test; that test now runs here, after the smoke.

## Proposed order (one window, about 2 hours of wall time, mostly the image build)

1. The user runs the install script (sudo): Docker, the NVIDIA container toolkit, the docker group, a new login.
2. Clone the repo at `~/isaac-launchable/dishwasher_sim_isaaclab` (branch `plan`), create `/home/shu4/dishsim-cache/{kit_cache,ov_data/{cache_ov,glcache,computecache,logs,data},pip_cache}`, edit `compose.yaml`'s host paths, `docker compose up -d` after `docker build -f code/util/docker/Dockerfile -t dishsim-isaac:6.2.0 .`.
3. rsync `data/assets` from the Mac; run `code/util/run_py.sh -m pytest code/tests/ code/frigidaire/tests/ -q` and `code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras`; judge by log content.
4. Copy `data/` from the box (one start), then the refactor branch's test here, then the sample re-certification.

Install script for step 1 (the user runs it; it asks for the sudo password):

```bash
sudo apt-get update && sudo apt-get install -y docker.io curl
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
sudo usermod -aG docker "$USER"   # then log out and in; test: docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

The one number that decides: 31 GiB of RAM holds three Kit workers, as the box's 30 GiB did; nothing else on the list is below what the box had, and the box ran every stage.
