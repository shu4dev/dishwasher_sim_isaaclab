# ArtVIP dishwasher variant survey (Phase 1)

Source: HuggingFace dataset `X-Humanoid/ArtVIP`, `Articulated_objects/major_appliances/dishwasher/`
(Apache-2.0), downloaded to `assets/artvip/` (79 MB, all 7 variants). Surveyed with a pxr/UsdPhysics
traversal of each composed stage (all stages: meters, Z-up, `defaultPrim=/root`).

| Variant | Door joint | Racks | Articulation root | Collision | Notes |
|---|---|---|---|---|---|
| `dishwasher_1` | revolute, **axis Z** (vertical hinge), 0–90°, drive k=20 d=10 tgt=0 | 2 prismatic, −0.2–0 m, k=10 sprung shut | `/root/E_body_1` | 12× SDF | vertical hinge = fridge-style door, atypical |
| **`dishwasher_2`** ✅ | revolute, axis X, 0–90°, drive k=20 d=10 **tgt=90 (pulls open)** | 2 prismatic (`_up`/`_down`), −0.2–0 m, k=10 d=10 tgt=0 | `/root/E_body_5` | 13× SDF | **chosen**: clean unique joint names, text `.usda`, dedicated `handle` mesh under door body `E_door_4` |
| `dishwasher_3` | revolute, axis X, 0–90°, k≈20 d=2 tgt=0 (pulls shut) | 2 prismatic, −0.2–0 m | `/root/E_body_6` | 17× SDF | fallback candidate |
| `dishwasher_4` | revolute, axis X, 0–90°, k=10 d=10 tgt=90 | 2 prismatic, tgt=−0.2 (pulled OUT) | `/root/E_body_1` | 15× SDF | joint prims have duplicate generic names (`RevoluteJoint`, `PrismaticJoint`×2) — name-collision risk |
| `l_dishwasher_1..3` | — (no revolute door) | 2 prismatic drawers, −0.2/−0.3–0 m, k=50 | nested body | 8–16× SDF | drawer-style machines; excluded (task needs a hinged door) |

Decisions:

- **Main asset: `dishwasher_2`** (`model_dishwasher_2.usda`). Door joint
  `RevoluteJoint_dishwasher_2_middle` (axis X, limits 0–90°), rack joints
  `PrismaticJoint_dishwasher_2_up` / `_down`, bodies `E_body_5` (base), `E_door_4` (door),
  `E_shelf_03` / `E_shelf_1_04` (racks). The USD-authored door drive (k=20 toward 90°) is an
  *active* drive and is overridden to stiffness 0 in the RL config (Phase 3) — no USD edits.
- Every mesh in every variant uses **SDF mesh collision**. Kept as authored for v0 (good concave
  fidelity for racks); per-env cost is measured at low env counts before scaling (fallback: local
  re-authored copy in `assets/artvip_derived/` with convex decomposition).
- The per-variant `resource/dishwasher_control.py` behavior scripts (magnetic door snap etc.) do
  not run headless and are ignored.

## YCB mug derivation

`Props/YCB/Axis_Aligned_Physics/` in the 6.0 asset library only contains 4 objects (cracker box,
sugar box, soup can, mustard bottle) — no mug/bowl. `scripts/01_make_mug_physics_usd.py` derives
`assets/props/025_mug_physics.usd` from the plain `Axis_Aligned/025_mug.usd`: `RigidBodyAPI` +
`MassAPI` (0.118 kg) on the root prim, convex-decomposition collision on the mesh, textures
co-located. Verified: schemas present on reopen.
