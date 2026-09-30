#!/usr/bin/env python3
"""Diagnostic: which UR5e/2F-85 prims carry colliders, are any disabled or filtered, and what the collision groups are.

    scripts/run_kit.sh frigidaire/scripts/setup/frigidaire_grip_colliders.py --headless

Writes results/robot/grip_probe/colliders.json. Read-only on the asset (the spawned stage is inspected only).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    app = AppLauncher(args).app
    out = {"result": "FAIL"}
    try:
        sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation
        from pxr import Usd, UsdPhysics, PhysxSchema
        from dishsim_frigidaire.robot import ur5e
        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cpu"))
        Articulation(ur5e.robot_cfg())
        stage = sim.stage
        cols, groups, filtered = [], [], []
        for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"), Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                api = UsdPhysics.CollisionAPI(prim)
                en = api.GetCollisionEnabledAttr().Get()
                approx = prim.GetAttribute("physics:approximation").Get() if prim.HasAttribute("physics:approximation") else None
                px = PhysxSchema.PhysxCollisionAPI(prim) if prim.HasAPI(PhysxSchema.PhysxCollisionAPI) else None
                cols.append({"path": str(prim.GetPath()), "enabled": en, "approx": str(approx) if approx else None,
                             "type": prim.GetTypeName(),
                             "contact_offset": None if px is None else px.GetContactOffsetAttr().Get(),
                             "rest_offset": None if px is None else px.GetRestOffsetAttr().Get()})
            if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                filtered.append({"path": str(prim.GetPath()),
                                 "targets": [str(t) for t in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()]})
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.CollisionGroup):
                g = UsdPhysics.CollisionGroup(prim)
                groups.append({"path": str(prim.GetPath()),
                               "includes": [str(t) for t in g.GetCollidersCollectionAPI().GetIncludesRel().GetTargets()],
                               "filtered_groups": [str(t) for t in g.GetFilteredGroupsRel().GetTargets()]})
        finger = [c for c in cols if "finger" in c["path"] or "knuckle" in c["path"] or "pad" in c["path"]]
        # what PhysX actually created: colliders per rigid body, after the simulation starts
        sim.reset()
        sim.step()
        from omni.physx import get_physx_property_query_interface
        from omni.physx.bindings._physx import PhysxPropertyQueryMode
        created = {}
        for body in ("left_inner_finger", "right_inner_finger", "left_outer_finger", "wrist_3_link"):
            path = next((str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"), Usd.TraverseInstanceProxies())
                         if p.GetName() == body and p.HasAPI(UsdPhysics.RigidBodyAPI)), None)
            rows = []
            if path:
                from pxr import PhysicsSchemaTools
                done = {"ok": False}
                def rb_fn(r, rows=rows):
                    rows.append({"rigid_body_mass": getattr(r, "mass", None)})
                def col_fn(c, rows=rows):
                    rows.append({"collider": PhysicsSchemaTools.intToSdfPath(c.path_id) if hasattr(c, "path_id") else str(c),
                                 "volume": getattr(c, "volume", None)})
                def fin_fn(done=done):
                    done["ok"] = True
                path_id = PhysicsSchemaTools.sdfPathToInt(path)
                get_physx_property_query_interface().query_prim(stage_id=__import__("omni.usd").usd.get_context().get_stage_id(),
                    prim_id=path_id, query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
                    rigid_body_fn=rb_fn, collider_fn=col_fn, finished_fn=fin_fn)
            created[body] = {"path": path, "rows": [{k: str(v) for k, v in r.items()} for r in rows]}
        out["physx_created"] = created
        def dump(path):
            p = stage.GetPrimAtPath(path)
            chain, q_ = [], p
            while q_ and q_.GetPath() != Sdf.Path("/World"):
                chain.append({"path": str(q_.GetPath()), "type": q_.GetTypeName(), "instance": q_.IsInstance(),
                              "instance_proxy": q_.IsInstanceProxy(), "schemas": list(q_.GetAppliedSchemas()),
                              "purpose": str(UsdGeom.Imageable(q_).GetPurposeAttr().Get()) if q_.IsA(UsdGeom.Imageable) else None})
                q_ = q_.GetParent()
            attrs = {a.GetName(): str(a.Get()) for a in p.GetAttributes()
                     if any(k in a.GetName() for k in ("physics", "physx", "collision", "purpose", "visibility"))}
            return {"attrs": attrs, "chain": chain}
        from pxr import Sdf, UsdGeom
        out["dump_fingertip"] = dump("/World/Robot/Gripper/Robotiq_2F_85/left_inner_finger/visuals/Defeatured_2F_85_PAD_OPEN_fingertipsstep_01/Defeatured_2F_85_PAD_OPEN_fingertipsstep")
        out["dump_wrist"] = dump("/World/Robot/wrist_3_link/collisions/wrist3/mesh")
        out.update(result="PASS", n_colliders=len(cols), finger_colliders=finger, disabled=[c for c in cols if c["enabled"] is False],
                   filtered_pairs=filtered, collision_groups=groups, all_paths=[c["path"] for c in cols])
    except Exception:
        out["error"] = traceback.format_exc()
        print(out["error"], flush=True)
    p = ROOT / "results/robot/grip_probe/colliders.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, indent=1) + "\n")
    print(f"[RESULT] {out['result']} colliders: {out.get('n_colliders')} total, finger {len(out.get('finger_colliders', []))}, "
          f"disabled {len(out.get('disabled', []))}, filtered {len(out.get('filtered_pairs', []))}, groups {len(out.get('collision_groups', []))}",
          flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


if __name__ == "__main__":
    main()
