# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit side: dump the settled scene's geometry into the collision-world cache.

Builds the exact check_scene scene (welded object, locked statics), settles it, then writes
per-rigid-body meshes + ``scene_state.json`` to ``assets/cache/``. Runs with
``use_fabric=False`` so live USD prim transforms are trustworthy (the Fabric staleness trap).

Run with:
    scripts/run_kit.sh scripts/setup/extract_geometry.py --headless
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import os

from isaaclab.app import AppLauncher

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py

parser = argparse.ArgumentParser(description="Dump collision-world cache from the v0 scene.")
parser.add_argument("--settle_steps", type=int, default=200)
parser.add_argument("--placement", type=str, default=None,
                    help="Named base placement (see config.BASE_PLACEMENTS); default: the machine's.")
parser.add_argument("--machine", type=str, default=None,
                    help="Machine name (see config.MACHINES); default: the v1 baseline.")
parser.add_argument("--object", type=str, default="mug", help="Carried object class (see config.OBJECTS).")
parser.add_argument("--scenario", type=str, default="both_out",
                    help="Rack-state scenario (see config.SCENARIOS).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import sys

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext
from isaaclab_physx.physics import PhysxCfg

sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

# scenario BEFORE scene/robots imports — they bind rack targets + the derived USD at import
if args_cli.machine:
    config.apply_machine(args_cli.machine)  # first: it resets scenario + base placement
config.set_active_object(args_cli.object)
config.apply_scenario(args_cli.scenario)
if args_cli.placement:
    config.apply_base_placement(args_cli.placement)  # after machine/scenario — they reset it

from dishsim import geometry as dgeom  # noqa: E402
from dishsim import scene as dscene  # noqa: E402


def main() -> None:
    sim = SimulationContext(
        sim_utils.SimulationCfg(dt=config.SIM_DT, device=args_cli.device, physics=PhysxCfg(), use_fabric=False)
    )
    scene = InteractiveScene(dscene.make_scene_cfg(with_object=True))
    dscene.author_weld(scene.stage)
    sim.reset()
    dscene.write_default_states(scene)
    dt = sim.get_physics_dt()
    for _ in range(args_cli.settle_steps):
        dscene.hold_targets(scene)
        scene.write_data_to_sim()
        sim.step()
        scene.update(dt)
    dscene.assert_frames(scene)
    # the settled racks must sit at the scenario targets — also catches a stale derived USD
    # whose baked drive targets fight the configured scenario
    sr = dscene.statics_report(scene)
    rack_err = max(sr["rack_lower_err_m"], sr["rack_upper_err_m"])
    assert rack_err < 5e-3, f"racks off scenario targets by {rack_err * 1e3:.1f} mm: {sr}"
    # never bake a bad grasp into the cache: the settled state must hold the calibrated
    # pad-force band with everything else silent. Weld-carried families (edge_pinch on thin
    # discs, rim_edge on thin walls) close to first-contact only — the pinch band does not
    # apply; the weld error is the load-bearing gate for them (checked in dump_cache).
    if config.active_object_spec().grasp.family not in ("edge_pinch", "rim_edge", "handle_pinch"):
        ok, detail = dscene.grip_gate(scene)
        assert ok, f"grip gate failed before cache dump: {detail}"

    manifest_path = dgeom.dump_cache(scene, sim, cache_dir=config.scenario_cache_dir())
    print(f"[INFO] scenario {config.SCENARIO_NAME}: rack_lower {sr['rack_lower_m']:+.3f} m, "
          f"rack_upper {sr['rack_upper_m']:+.3f} m")
    print(f"[RESULT] PASS ({manifest_path})")


if __name__ == "__main__":
    main()
    simulation_app.close()
