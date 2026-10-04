# Frigidaire FDPC4221AS dishwasher asset

> **HISTORY: dated development log of the Frigidaire twin (v1 to v8, up to the 2026-09-23
> outside-heights rebuild); paths and scripts named here may no longer exist.** The current
> geometry is [geometry.md](../geometry.md) (tape re-measured 2026-09-28). Retired or moved on
> 2026-09-29: the v3-era experiment scripts, tests and docs (in git history at HEAD 4455813); the
> Frigidaire result folders for initial states, random poses and the planner, the HOTEC runs v2
> to v5 and most exposure results (in the hold folder
> `/media/corallab-s1/2tbhdd/brianshu/dishsim/_trash_20260929/results/` until 2026-10-29); and the
> unpacked v1 and v2 media galleries (full copies remain in
> `build/frigidaire_collection/history/{v1,v2}/gallery`).

This independently authored USD reconstructs the 24-inch Frigidaire FDPC4221AS from
the photographs and specification in `frigidaire/`. Its detailed coated-wire racks,
open basket lattice, dish-support surfaces, door hinge and rack slides are intended
for manipulation and dish placement in Isaac Sim 4.5.0 / Isaac Lab 2.1.1. Geometry
that was not measured directly is documented as an estimate, rather than manufacturer CAD.

The revised appliance and component USDs are generated beneath
`assets/models/frigidaire_fdpc4221as_v2/`. The previous validated release remains in
`assets/models/frigidaire_fdpc4221as/`. These directories are independent of the Bosch
benchmark assets and caches. Keep the generated directory together when copying
the appliance to another machine: the assembly uses relative component references.
The current source includes the [52-tine upper-rack revision](#upper-rack-tine-polish)
and [72-tine lower-rack revision](#lower-rack-tine-polish). These source revisions
and their staged inspection views have not replaced the installed v2 USD bundle;
the installed release and its preserved simulation evidence still describe the
earlier 32-tine upper rack, 56-tine lower rack, and previous basket pose.

## Build and inspect

From the project root, using the existing Isaac container:

```bash
scripts/run_py.sh scripts/setup/build_frigidaire.py
scripts/run_py.sh scripts/setup/load_frigidaire.py \
    --ban-file assets/models/frigidaire_fdpc4221as_v2/full_load_banned_candidates.json
scripts/run_kit.sh scripts/evaluation/frigidaire_asset_evidence.py --headless --physics-only --device cpu
scripts/run_kit.sh scripts/evaluation/frigidaire_asset_evidence.py --headless --enable_cameras --render-only --device cpu
```

The small demonstration exercises ordered motion or force-driven interaction:

```bash
scripts/run_kit.sh scripts/experiment/frigidaire_demo.py --headless --mode scripted --device cpu
scripts/run_kit.sh scripts/experiment/frigidaire_demo.py --headless --mode passive --device cpu
```

Open `build/frigidaire_collection/history/v2/gallery/index.html` for the revised labeled
full-resolution image gallery. Each component has front, right, front-left and overhead views. Additional
images expose rack bends, the basket underside, the exploded appliance, closed/open
states, independently extended racks, and a representative full-size dish load.
The basket photograph convention differs from the appliance: its front is the long
handle side viewed from +X, and its right view is from +Y. Appliance and rack front
views look from −Y. Camera poses and these conventions are recorded in `renders.json`.
All images come from the generated USD geometry rendered by Isaac Sim's RTX camera;
the neutral studio floor keeps wire openings visible. The pictures are not generated
illustrations or reference-photo composites.

The script writes `physics.json`, `renders.json`, `contacts.json`, and a combined
`evidence.json`. The combined verdict is `PASS` only when all three reports pass and their asset
and executable-source hashes match the current files. A missing report is `NOT_RUN`;
a report for older geometry or validation code is `STALE`. Check the `[RESULT]` line in the run log as well as the JSON:
the pinned Isaac wrapper may return exit code zero after a Python error.

Each simulation or rendering run is a separate, bounded project job. Run only one
Kit job at a time on the shared machine, then close it and release the project's
compute resources. The evidence scripts close their own simulation cleanly; they
never enumerate or terminate other users' processes. Installed photographs,
contact sheets, galleries, and image archives remain under `media/`. The current
rack source previews use the writable `build/` staging directories shown below
because the linked media directory is unavailable for writes in this session.
The supplied references are read in place.

## Rack reconstruction revision

The additional `frigidaire/overall/` and `frigidaire/loaded/` photographs anchor the
front and rear tine banks, plate orientation, basket location, and upper-rack floor
channels. The user's best dimension estimates set the outer wire-rim targets;
wheel and handle projections are reported separately.

| Rack, current source | Front-to-back outer rim | Side-to-side outer rim | Tine layout and center spacing |
|---|---:|---:|---|
| Lower | 581.66 mm / 22.9 in | 548.64 mm / 21.6 in | 72 tines: 12 columns × 6 rows; 31.694545… mm left/right pitch, 73.332 mm front/back pitch |
| Upper | 548.64 mm / 21.6 in | 508.00 mm / 20.0 in | 52 tines: 4 columns × 13 positions; 95 / 75 / 95 mm left/right gaps, 33 mm front/back pitch |

The current lower rack has six complete transverse support rows for
sideways-facing plates, with twelve tine positions per row. The removable basket
keeps its rearward placement and moves 27.5 mm right to accommodate the revised
grid. The earlier installed lower rack instead had four rows with 16 / 16 / 12 / 12
tines. The upper rack has five contoured floor channels, lower
side walls, two front rails, and nine longitudinal floor wires that turn upward at
the front. Visible wires and collision capsules share the same centerlines.
The earlier artificial rear bowl opening is removed; the standalone bowl fixture
now uses the lower-front loading area.

These estimates do not turn the reconstruction into measured manufacturer CAD.
The implementation reconciles unmeasured interior surfaces, hinge placement, and
wheel tracks with the deeper rack targets while retaining the specified exterior
envelope and open-door depth. `parameters.json` and `geometry_validation.json`
record the generated dimensions, estimates, and validation results.

## Upper-rack tine polish

The current source revision, `upper_tines_4x13_v1`, replaces two banks of sixteen
tines with **four columns across the width and thirteen positions front to back**.
Coordinates use the rack's local frame: X runs left to right, −Y faces the front,
and +Y faces the rear. All margins below run from **tine base centers to the outer
wire-rim edge**, and all tine gaps are center to center.

| Measurement | Current source value |
|---|---:|
| Tine count | 52 = 4 × 13 |
| Column X coordinates, left to right | −132.5 / −37.5 / +37.5 / +132.5 mm |
| Column gaps, left to right | 95 / 75 / 95 mm |
| Front-to-back row coordinates | Y = −173.18 + 33 × i mm, i = 0…12 |
| Front-to-back pitch | 33 mm |
| Left and right margins | 121.5 mm each |
| Rear margin | 51.5 mm |
| Front margin | 101.14 mm |
| Tine diameter / vertical base-to-tip rise / rearward lean | 3.6 / 91 / 8 mm |
| Rear tip-center margin | 43.5 mm |

The 101.14 mm front margin follows from retaining the existing 548.64 mm rack
depth, 33 mm pitch, and 51.5 mm rear margin. Four 490 mm longitudinal base rails
support the columns; each follows the actual rounded floor profile at its X
coordinate with its centerline 3 mm above the floor-wire centerline. The tines
retain rounded bends and tips. Visible wires and contact capsules share paths.
This upper-rack edit retains the outer rim, floor channels, front wires, wheels,
mounting pose, and other appliance components. The separate lower-rack edit is
described below.

Generate host-side inspection images without Isaac Sim or USD:

```bash
python3 scripts/evaluation/frigidaire_upper_rack_preview.py \
    --out-dir build/frigidaire_upper_rack_polish/preview
```

This writes `upper_rack_overhead.png` and `.svg` with all 52 base centers and
dimensions, `upper_rack_front.png`, `upper_rack_oblique.png`, and
`measurements.json`. The overhead image projects generated wire paths and authored
wheel solids; the other views project the actual generated meshes and solids.
They are source-geometry inspection views, not Isaac renders or physics evidence.
The JSON records the geometry-source hash and every tine's base and tip coordinates.
The script requires only NumPy and Matplotlib; without `--out-dir` it uses
`media/frigidaire_upper_rack_polish/`.

To update an existing asset bundle once the Isaac Python runtime and output
directory are available:

```bash
scripts/run_py.sh scripts/setup/build_frigidaire.py --component UpperRack
```

The component option updates the upper-rack USD and its geometry metadata;
omitting it retains the full-build behavior. Stage and inspect a copy of the bundle
before replacing the installed component. Existing full-load manifests and
validation reports remain historical evidence and must be regenerated and checked
against the new geometry before claiming capacity. Upper saucer candidates now
derive their twelve longitudinal gaps from the new tine positions. The added
outer columns also require renewed cup-fixture and load collision checks.
The geometry audit found that the legacy cup fixture seed at rack-local
`[-0.109, -0.174, 0.042]` m intersects the new outer tine bank and its base rail.
The seed pose is preserved for this tine-only edit; choose and validate a new
placement before running cup-fixture physics. The old cup and load certifications
do not apply to this geometry.
USD installation, Isaac physics/render checks, and full-load repacking have not
been run for this revision in the restricted host session.

## Lower-rack tine polish

The source revision `lower_tines_6x12_v1` defines
**six rows front to back, each containing twelve tines left to right**.
All seventy-two positions are present, including the rightmost positions beside the
basket. Coordinates use the lower rack's local frame, and margins run from tine
base centers to the outer wire-rim edges. The original 548.64 × 581.66 mm rim
footprint remains unchanged.

| Measurement | Current source value |
|---|---:|
| Tine count | 72 = 12 columns × 6 rows |
| Column X coordinates | −194.32 + (348.64 / 11) × i mm, i = 0…11 |
| Left-to-right pitch | 31.694545… mm |
| Row Y coordinates, front to back | −185.830 / −112.498 / −39.166 / +34.166 / +107.498 / +180.830 mm |
| Front-to-back pitch | 73.332 mm |
| Left / right margins | 80 / 120 mm |
| Front / rear margins | 105 / 110 mm |
| Tine diameter / vertical base-to-tip rise / rightward lean | 3.9 / 105 / 8 mm |
| Base-rail center height | Z = 6 mm |
| Rightmost tip-center margin | 112 mm |

The transverse base rails connect the twelve bases in each row. The existing
tine profile retains rounded bends and tips; mesh sweeps and contact capsules
share the same centerlines. Floor wires, perimeter rails, wheels, grip, and lower
rack mounting position retain their earlier geometry.

The unchanged basket shape retains its **27.5 mm rightward shift**. Its assembly origin is
`[0.2135, 0.128, 0.226]` m; relative to the lower rack it is
`[0.2135, 0.120, 0.011]` m. At that pose its authored capsule envelope is
X = 169.5…257.5 mm, Y = −36…276 mm, and approximately Z = 12.819…214 mm in the
lower-rack frame. These envelope coordinates are geometry measurements, not
simulation clearance or load-retention certification.

The source clearance audit (`source_clearance.json`, removed)
measures a 5.23 mm conservative X-envelope gap, 9.50 mm minimum basket-to-tine/base-rail
capsule clearance, 3.63 mm to the right wall, and 4.44 mm to the closest remaining
rear-wall wire. These are static source-geometry measurements; they do not establish
settled basket support or Isaac validation. The same audit confirms that both
retained legacy **plate and bowl fixture seed poses collide with the new tines**.
Those seeds require new placements and validation before fixture physics; no fresh
plate/bowl fixture certification is claimed for this revision.

Generate inspection views on the host:

```bash
python3 scripts/evaluation/frigidaire_lower_rack_preview.py \
    --out-dir build/frigidaire_lower_rack_polish/preview
```

The script writes an annotated `lower_rack_overhead.png` and `.svg`, an
actual-mesh `lower_rack_oblique.png`, and `measurements.json`. The overhead image
shows the basket's generated rim/handle outline to keep all seventy-two tine bases
legible; the oblique view includes the complete basket mesh at its assembly pose.
The JSON includes every tine base/tip, the basket's relative pose and capsule
envelope, and the geometry-source hash. NumPy and Matplotlib are sufficient;
without `--out-dir`, previews go to `media/frigidaire_lower_rack_polish/`.

Once the Isaac Python runtime and output directory are available, update a
staged copy of an existing bundle using:

```bash
scripts/run_py.sh scripts/setup/build_frigidaire.py --component LowerRack \
    --out-dir build/frigidaire_lower_rack_polish/bundle
```

The destination must already contain the complete existing bundle. The lower
component build updates the lower-rack USD, relevant geometry/pose metadata, and
the basket's translation in the assembly; it preserves the basket component's
shape and the installed upper-rack component. A separate `--component UpperRack`
build is needed to update that component in a bundle that still has the earlier
32-tine upper rack. Omitting `--component` retains the full-build behavior.
Old load manifests, fixed fixture seeds, and previous basket insertion/removal
evidence require collision checks and new Isaac validation against both rack
revisions and the moved basket. No installed USD replacement, Isaac physics/render
run, or full-load capacity recertification has been performed for this revision.

## Full mixed load

The preserved v2 release with the **earlier 32-tine upper rack, 56-tine lower rack,
and previous basket pose** validated
**67 objects**: 23 in the lower rack, 28 in the upper rack, and 16 utensils in the
removable basket. Those counts do not certify either current rack revision. The
[count table and measured results](#revision-validation) describe this specific
load and its validation limits.

`load_frigidaire.py` creates `full_load_manifest.json` by checking a finite set of
placements against the actual authored contact geometry using FCL. It does not
modify the existing benchmark props or caches. The separate tableware catalog uses
these fixed modeling dimensions throughout packing and physics:

| Object | Fixed size | Estimated mass |
|---|---|---:|
| Dinner plate | Ø260 × 20 mm overall profile | 650 g |
| Salad plate | Ø205 × 18 mm | 400 g |
| Saucer | Ø150 × 18 mm | 200 g |
| Bowl | Ø140 × 65 mm | 350 g |
| Handled mug | Ø85 × 100 mm body; 120 mm overall width | 300 g |
| Tumbler | Ø80 mm rim, Ø75 mm base, 160 mm tall | 250 g |
| Fork | 195 × 25 × 12 mm | 45 g |
| Table knife | 215 × 18 × 6 mm | 55 g |
| Tablespoon | 190 × 40 × 18 mm | 50 g |
| Teaspoon | 155 × 30 × 12 mm | 25 g |

The photographs establish plausible loading patterns, not these exact object
dimensions or masses. Plates retain curved profiles; bowls, mugs, and tumblers
retain cavities; mug handles retain an open aperture. Every counted object is a
free rigid body. There are no hidden dish supports or fixing joints, stacked
cups, nested bowls, or object resizing during packing.

The deterministic planner reserves lower-front bowl positions and attempts balanced
tableware rounds before filling remaining candidates in a fixed order.
Residual cutlery is added as complete four-piece bundles. Packing stops when its
finite patterns admit no further ordinary items or complete cutlery bundles;
additional individual utensils can still fit after bundle saturation. Physics-rejected
candidate keys are supplied through the release's portable
`full_load_banned_candidates.json` file; the manifest records rejected candidates
and reasons. Use the `--ban-file` argument shown in the build commands to reproduce
the release's candidate exclusions. The resulting count is a mixed load saturated within
the documented placement patterns, not a globally maximum capacity or a verified
manufacturer place-setting rating.

Validate and render the load in separate runs:

```bash
scripts/run_kit.sh scripts/evaluation/frigidaire_full_load_evidence.py --headless --device cpu \
    --physics-only --usd assets/models/frigidaire_fdpc4221as_v2/fdpc4221as.usdc \
    --manifest assets/models/frigidaire_fdpc4221as_v2/full_load_manifest.json
scripts/run_py.sh scripts/setup/configure_frigidaire_rendering.py
scripts/run_kit.sh scripts/evaluation/frigidaire_full_load_render.py --headless --device cpu \
    --enable_cameras --render-only --usd assets/models/frigidaire_fdpc4221as_v2/fdpc4221as.usdc \
    --manifest assets/models/frigidaire_fdpc4221as_v2/full_load_manifest.json \
    --rendering_mode quality --kit_args='--/rtx/raytracing/fractionalCutoutOpacity=true'
scripts/run_py.sh scripts/setup/configure_frigidaire_rendering.py --require-render-pass
```

The rendering configuration step enables fractional opacity and translucency in
the loaded scene's root-layer `renderSettings` metadata. The render command also
uses Isaac Lab's quality preset because its default balanced preset disables
translucency. The render-only wrapper executes the unchanged validator, reapplies
the visual settings after simulation initialization and before each frame, and
records actual renderer readbacks and tumbler shader inputs in
`render_runtime.json`. Inventory photographs use a charcoal studio floor so the
translucent glass shell and rim remain visible; the assembled views retain the
original floor. This changes only the rendering studio's appearance and is also
recorded in the runtime report. [NVIDIA's RTX documentation](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/rtx-renderer_rt_legacy.html)
describes fractional cutout opacity; its [raw USD example](https://docs.omniverse.nvidia.com/workflows/latest/rtx_rt-dh-setup.html#setup-post-processing-and-render-settings)
shows the metadata format. The Kit-free configuration tool preserves all geometry,
physics attributes, poses, and the physics report. Its `render_settings.json`
sidecar fingerprints the current scene, sources, manifest, physics, and image
report. Repeat it after rendering to verify the final image hashes before the
standalone smoke check and packaging.

For placement diagnostics, append `--settle-only` to the physics command. A
successful settlement diagnostic does not certify capacity and never populates
`validated_counts`.

The complete run first initializes a natural resting configuration through physical
motion. Manifest placements receive 3 mm of additional hover; the four
collision-cleared, physically refined teaspoon poses and one rear-bank salad plate
receive none. That salad plate starts at Y=125 mm, between floor wires at Y=108
and 135 mm, so its rim settles into the wire valley. After twelve
seconds of settling, the dishwasher opens, extends both racks, holds for two
seconds, retracts both racks, closes, and holds for four seconds. The dishes remain
free rigid bodies throughout: initialization does not reset their poses or
velocities, attach fixing joints, or apply dish-holding forces.

Initialization is reported separately. Every 120 Hz sample must keep each actor
origin above its supporting rack's low floor and within that component's authored
XY envelope; settled holds also require support contacts. The report preserves all
raw initialization poses, rack frames, joint trajectories, file hashes, and the
objects' net displacement. This phase allows pieces of cutlery to find stable
contacts together and supplies no validated counts.

Initialization then includes a separate readiness wait: at least twelve additional
seconds, followed by one-second observation windows, with a total wait capped at
thirty seconds. Two consecutive windows must pass the same mesh-speed, position-span,
quaternion-span, support, and contact criteria used by measured holds. Waiting
keeps the closed joint targets and applies no commands to the dishes. Its duration,
raw pose traces, and unsuccessful windows are recorded separately; a timeout fails
the run.

Only after readiness succeeds does the measured validation begin. The unchanged
twelve-second closed hold establishes the retention reference. The measured cycle then opens
the door, extends both racks, retracts them, closes the door, and reopens and
extends them again. Six settled states
record each object's world and rack-relative pose, stability, support contacts,
and failures. Stability requires a final-second actor-position span below 5 mm,
quaternion span below 3°, and unsmoothed maximum speed below 0.03 m/s across six
actual mesh extrema transformed by every 120 Hz actor pose. This includes
rotation about the actor origin. Rack-relative displacement through the cycle must remain
within 10 mm per axis relative to the initialized resting configuration. The
removable basket must also remain stable relative to
the lower rack. A fallen object cannot pass by settling on the ground.
Scripted moves use cubic smoothstep ramps with zero endpoint commanded velocity,
at most 0.10 m/s commanded rack speed and 0.35 rad/s commanded door speed. Ramp
duration is at least 3.5 seconds and otherwise follows 1.5 × travel / speed limit.
The reports include each duration and measured peak joint speed. Retention is
certified for this gentle motion profile; abrupt movements are not covered.
The report preserves raw solver velocities separately. PhysX documents that its
split-impulse solver can report velocities that differ from consecutive body-pose
displacements, so the stability gate measures observed mesh motion directly.
[PhysX solver iteration documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html#solver-iterations)
explains this distinction. Per-hold files under `full_load/pose_traces/` save every
measured position and quaternion, the timestep, and a file hash. The report also
records the six local mesh points, allowing independent recomputation of the
motion measurements without rerunning physics.

Continuous collision detection is enabled on the scene and moving bodies to
protect thin wire contacts during settling. Detailed PhysX contacts include all
appliance and tableware body pairs during
the settled holds. Peak penetration must remain below 2 mm and median worst
penetration below 1 mm. Invalid contact data or a full 262144-contact buffer fails
validation. These are simulation separation measurements; they do not establish
manufacturing tolerances. The run does not certify a complete dish unloading
sequence, washing effectiveness, spray access, or robot reachability.

Only a complete passing run exports `full_load.usda`, `full_load_settled.json`, and
validated per-type/per-rack counts. Keep `tableware/` and the component USDs beside
the loaded scene to preserve its relative references. The standalone scene starts
closed, with free dishes and appliance bodies at their measured poses. It includes
a 120 Hz CPU TGS physics scene, gravity, a static floor, lights, camera, and finite
scripted drives holding the closed state. Continuous collision detection is enabled
for the scene and moving bodies so narrow basket wires can arrest moving cutlery.
These drives use the same gains as the
Python helper. USD door target positions use degrees; rack targets use metres.
Open the door before extending racks, and use the documented gradual motion
profile when reproducing the validated cycle. The raw component appliance USD
retains its passive defaults.

The portable loaded scene is mounted at the world origin. Moving only an ancestor
Xform does not move the appliance's world-side fixed-joint anchor. To relocate the
scene, update that anchor and all dish poses consistently, or use the Python
spawning helper and manifest placements at the requested world pose.

To open the exported USD directly and check its authored closed hold without the
Python appliance controller:

```bash
scripts/run_kit.sh scripts/evaluation/frigidaire_loaded_scene_smoke.py --headless --device cpu \
    --scene assets/models/frigidaire_fdpc4221as_v2/full_load.usda
```

For Isaac Sim Core 4.5, this check refreshes the existing-stage registry so Core
selects the authored physics scene and timestep. It does not change USD physics,
drive, or body sleep attributes. Support contacts are observed from the first
physics step. If sleeping bodies stop reporting contacts, a previously measured
contact connection is retained only while both bodies' surface-motion bounds
remain within 1 µm of their last observed contact poses at every later step.

The [full-load gallery](../../../build/frigidaire_collection/history/v2/gallery/full_load/index.html)
is saved under `build/frigidaire_collection/history/v2/gallery/full_load/`.
It contains assembly views, isolated loaded-rack overhead and oblique views, and
an inventory contact sheet labeled with actual validated counts. Rendering requires
a complete passing physics report with matching asset, source, and manifest
hashes; it replays saved physical states. A picture cannot silently replace a
failed physics check. Reports include candidate counts, accepted counts, rejected
placements, and per-object failure reasons even when a run fails.

## Use in Isaac Lab

`example_scene.usda` is a portable scene that references the appliance. The raw USD
supplies passive joint damping; the Python helper below adds the estimated door
counterbalance and slide resistance during simulation. Opening the USD alone does
not execute that helper.

Launch `AppLauncher` before importing Isaac scene modules, then create the asset
before resetting the simulation:

```python
from dishsim.frigidaire_asset import spawn, apply_mode, step_passive

appliance, basket = spawn(
    "/World/Dishwasher", position=(0.0, 0.0, 0.0), yaw=0.0, mode="passive"
)
sim.reset()

# During each physics step in passive mode:
step_passive(appliance)
appliance.write_data_to_sim()
sim.step()
appliance.update(sim.get_physics_dt())
basket.update(sim.get_physics_dt())
```

`spawn` returns an Isaac Lab `Articulation` and a separate basket `RigidObject`.
It updates the fixed joint's world anchor for the requested position and yaw.
The requested pose is in world coordinates, including beneath a translated or
rotated parent. Scaled, sheared or reflected parents raise `ValueError`.
Use `mode="scripted"` for finite-force joint drives and set joint-position targets
through the ordinary Isaac Lab articulation API. Call `apply_mode` after reset to
switch between the two modes. For standalone scripts, call
`dishsim.media.release_sim_for_close()` immediately before closing the app.

The default prim is `/FrigidaireFDPC4221AS`. Units are meters and kilograms, Z is up,
X runs across the width, and the appliance faces −Y. Its origin is the center of
the floor footprint. The default state is closed.

| Joint | Body | Axis | Travel |
|---|---|---|---|
| `door_hinge` | `Door` | +X | 0–90°; radians in Isaac Lab |
| `lower_slide` | `LowerRack` | Y | −0.49–0 m |
| `upper_slide` | `UpperRack` | Y | −0.44–0 m |

Negative rack displacement extends the rack. Open the door before extending racks;
retract racks before closing it. Retract the upper rack for vertical access to the
lower rack. The rear-right basket remains partly beneath the upper rack when the
lower rack is extended. The selected removal probe lifts the basket 140 mm above
the lower tines, draws it forward 140 mm, and then completes a further 90 mm lift.
Replacement reverses that path and releases the basket 10 mm above its seat.
The silverware basket is a free rigid body supported by the lower rack.
The installed racks remain constrained to their slides. Separate `cabinet.usdc`,
`door.usdc`, `lower_rack.usdc`, `upper_rack.usdc`, and `silverware_basket.usdc` files
provide loose component geometry with local component origins.

Wheel contact uses estimated low friction (0.06 static / 0.04 dynamic, minimum
combination) to approximate rolling while wheels remain rigidly attached to their
rack. Dish-support wire contact retains its separate coating friction. This avoids
making a normal rack pull overcome the sliding friction of eight locked wheels.

Each relevant body's `Manipulation` children expose handle centers and candidate
fixture centers. `quatWXYZ` is a component-local orientation hint in Isaac Lab's
quaternion convention; compose it with the body's current world rotation. The
site Xform's own rotation stays identity. These are planning hints; a particular dish or gripper still needs collision
and stability checking against the actual wire and tine geometry.

## Validation meaning

Physics evidence exercises empty full travel, loaded retraction and closure,
reopening, settling and retention of a 260 mm plate, a 140 mm bowl, an 80 × 100 mm
cup, and a 180 mm utensil. It also probes basket lifting/replacement, passive handle
forces, door obstruction with an extended rack, and a translated/yawed spawn.
Endpoint tolerances are 0.5° for the door and 5 mm for slides.
The validated utensil pose places its broad end down across the basket's floor ribs;
its narrower handle remains accessible above the rim. A handle-down drop can lodge
in a floor opening, so that placement is not certified by the successful run.

Insertion and retrieval use an ideal grasp fixture that commands a dynamic object's
velocity while preserving contacts. Position is not overwritten during the motion.
The fixture compensates gravity; tracking error and rack disturbance are measured. This validates support and a
selected access path; it does not validate a robot arm, finger grasp, or arbitrary
dish placement. Images use selected settled inspection poses; their presence alone
does not certify physics. `contacts.json` measures signed PhysX contact separations
over one-second settled holds in the closed, open, extended and loaded inspection
states. Its limits are 2 mm peak penetration and 1 mm median worst penetration; a
full contact buffer fails the measurement instead of silently truncating contacts.
These are inspection-state contact checks, separate from the motion measurements.
Measured results and all gate failures remain in the JSON.

The specification anchors the 609.6 mm width, 635 mm depth, 850.9 mm default height,
and 1250.95 mm open-door depth. Wire diameters, tine spacing, floor contours, internal
clearances, rack travel, masses, inertias, friction and counterbalance behavior are
estimates documented with the reconstruction parameters. Concealed hinge hardware,
wheel rotation, latch release, tine flex, washing systems and control operation are
simplified. The released passive door uses an approximate spring counterbalance.

The supplied specification labels the upper rack's value as “Minimum Height
Clearance” of 8 in (203.2 mm), and lists lower-rack minimum/maximum clearances of
11/13 in. It does not identify the measurement datum. The reconstructed upper
rack's floor-to-ceiling space depends on the contoured floor location and
is an estimate; it should not be interpreted as verified dish capacity. Upper-rack
placements taller than 203.2 mm remain uncalibrated against the physical appliance.

## Revision validation

The results in this section describe the preserved v2 release with its earlier
32-tine upper rack, 56-tine lower rack, and previous basket pose. They are historical
validation records, not a validation of the current rack revisions and moved
basket. Executable-source hash checks against the
new source must report the old evidence as stale; do not refresh old report hashes
to make them appear current.

The preserved [component and assembly gallery](../../../build/frigidaire_collection/history/v2/gallery/index.html)
and its [combined evidence report](../../../build/frigidaire_collection/history/v2/gallery/evidence.json)
passed with the release's asset and executable-source hashes. All 36 base physics checks,
six contact inspections, and 31 rendered-image checks passed. The largest measured
door endpoint error was 0.1411°; the largest slide endpoint error was 0.0509 mm.
The component contact inspections recorded at most 0.00199 mm penetration and
488 contacts out of an 8192-contact buffer. These simulator separation values do
not imply comparable manufacturing accuracy.

The [full-load physics report](../../../build/frigidaire_collection/history/v2/gallery/full_load/physics.json)
passed all 37 checks with the release's matching asset, manifest, and executable-source hashes.
Its accepted counts are:

| Object | Lower rack | Upper rack | Basket | Total |
|---|---:|---:|---:|---:|
| Dinner plate | 11 | 0 | 0 | 11 |
| Salad plate | 10 | 0 | 0 | 10 |
| Saucer | 0 | 10 | 0 | 10 |
| Bowl | 2 | 0 | 0 | 2 |
| Handled mug | 0 | 6 | 0 | 6 |
| Tumbler | 0 | 12 | 0 | 12 |
| Fork | 0 | 0 | 4 | 4 |
| Table knife | 0 | 0 | 4 | 4 |
| Tablespoon | 0 | 0 | 4 | 4 |
| Teaspoon | 0 | 0 | 4 | 4 |
| **Total** | **23** | **28** | **16** | **67** |

After the physical initialization cycle, readiness required 15 additional seconds.
The first eligible window failed the unchanged speed threshold; the following
two windows passed. The separate measured cycle then passed all six holds,
including loaded rack travel, door closure, reopening, and full extension.
Maximum object displacement relative to its supporting rack was **0.458 mm**
against the 10 mm limit. The largest measured contact penetration was
**0.0693 mm**, with a maximum median-worst penetration of 0.0674 mm; no contact
buffer overflow occurred. Maximum final-second object position span was
0.0759 mm, mesh-point speed was 13.0 mm/s, and orientation span was 0.246°.
The authoritative physics log is `logs/frigidaire_v2_full_physics_ready.log`.

All 20 [full-load images](../../../build/frigidaire_collection/history/v2/gallery/full_load/index.html)
passed, bringing the revised evidence to **51 rendered views**: 31 component and
fixture views plus 20 loaded-assembly and inventory views. The
[combined full-load evidence](../../../build/frigidaire_collection/history/v2/gallery/full_load/evidence.json)
and [rendering provenance](../../../build/frigidaire_collection/history/v2/gallery/full_load/render_settings.json)
verified the release's physics, asset, manifest, executable sources, and image hashes.
The images show the accepted physical states; inventory views use the recorded
charcoal studio backdrop to make the translucent tumbler's rim and shell clear.
The final render log is `logs/frigidaire_v2_full_renders_studio.log`.

The [standalone scene smoke check](../../../build/frigidaire_collection/history/v2/gallery/full_load/standalone_smoke.json)
passed with 360 actual PhysX events at 120 Hz, covering three seconds of the
authored closed hold. All 67 objects maintained contact paths to their intended
supports; maximum per-axis displacement from the exported poses was 2.175 mm.
The support graph retained 92 measured connections from step 51 after sleeping
bodies stopped issuing contact reports. Each retained connection satisfied the
1 µm surface-motion condition; the largest observed bound was 0.497 µm. Final
contact reports were empty, so those retained connections are identified
separately in the report. Authored physics, drives, and sleep attributes remained
unchanged.

The [measured bowl non-nesting audit](../../../build/frigidaire_collection/history/v2/gallery/full_load/non_nesting.json)
also passed in all six recorded load states. It uses the bowls' actual visual
meshes: neither bowl's foot center enters the other's inner cavity, and shallow
rim/base intrusion remains within the existing 2 mm contact tolerance.

This is a validated mixed load for the stated fixed-size objects, estimated rack
geometry, initialized resting poses, and gentle motion profile. Saturation applies
to the documented placement patterns and complete cutlery bundles; it does not
establish a global maximum or a manufacturer place-setting rating.

All 86 repository tests passed before the final single-plate placement refinement;
the 21 affected loading tests were rerun and passed afterward. The tests include fixed-size
tableware geometry and cavity checks, malformed/stale manifest rejection,
self-contact exclusion, and motion-metric checks for brief oscillations and
rotation about a stationary actor origin. The preserved test log is
`logs/frigidaire_v2_pytest_release.log`, with the targeted rerun in
`logs/frigidaire_v2_loading_valley_tests.log`. The base evidence log is
`logs/frigidaire_v2_base_final.log`.

## Previous release measurements

The preserved first-release reports in `build/frigidaire_collection/history/v1/gallery/` record a
combined `PASS` with matching asset and executable-source hashes:

- All 36 physics gates passed. The largest measured endpoint errors were 0.281°
  for the door and 0.079 mm for a rack. Maximum fixture drift relative to its rack
  during loaded retraction was 0.464 mm against the 10 mm limit.
- The selected insertion/retrieval paths passed with at most 0.218 mm tracking
  error. Plate, bowl and inverted-cup axis errors stayed below 7.443° against the
  12° limit. Basket lift, replacement and head-down utensil retrieval passed.
- Passive probes moved the lower rack 42.1 mm under 18 N for 0.3 s, the upper rack
  42.3 mm under 14 N for 0.3 s, and the door 11.75° under a 15 N handle force for
  0.35 s. Translated/yawed anchoring and full travel passed.
- All six settled contact inspections passed. The largest reported penetration
  was 0.00102 mm; the largest median worst penetration was 0.000172 mm. The maximum
  contact count was 362 of 8192, with no overflow or invalid contact values. These
  are simulator separation measurements, not manufacturing accuracy claims.
- All 31 PNG captures passed their image checks at 1920 × 1440. The gallery also
  includes a labeled six-view `contact_sheet.png`. The separate scripted and passive
  demonstrations passed, as did all 41 automated tests (7 USD acceptance tests and
  34 existing regressions).

Release physics ran on CPU PhysX at 120 Hz in the pinned Isaac Sim 4.5.0 / Isaac Lab
2.1.1 environment; RTX camera rendering used the GPU. Isaac Lab's internal Python
package metadata reports `0.41.3`, which is distinct from its repository release
tag. The measured evidence phases took 126.03 s for physics and 38.04 s for
rendering/contact inspection, excluding application startup. Logs are
`logs/frigidaire_physics_release_clean.log`, `logs/frigidaire_render_release_clean.log`,
`logs/frigidaire_demo_scripted.log`, and `logs/frigidaire_demo_passive.log`.

## 2026-09-21/22: tape-measured racks and basket (v4 source; v3 archived)

The sections above describe the photo-fitted geometry (v1 to v3). On 2026-09-21 the
user tape-measured the real racks and basket. The source `geometry.py` was rewritten
to those numbers, the USD collection was rebuilt on 2026-09-22, and the previous build
was archived first as `build/frigidaire_collection/history/v3` (165 files, 301.6 MB).
The current geometry page is [docs/geometry.md](../geometry.md); this section is the
record of what was measured, decided and run.

### Tape measurements (cm)

| Part | Quantity | Tape |
|---|---|---|
| Upper rack | outer W x D x H | 48 x 51.5 x 12.5 |
| Upper rack | tine columns | 4, 9 apart |
| Upper rack | tines per column, pitch | 13 at 3.7 in the outer columns, 11 in the middle two (two missing at the centre) |
| Upper rack | margins left / right / front / rear | 12 / 12 / 8 / 5.5 |
| Lower rack | outer W x D x H | 52.5 x 56.3 x 11.5 |
| Lower rack | grid, pitch | 12 columns x 6 rows; 3.6 left-right, 8 front-back |
| Lower rack | tine height | 9.5; the two middle rows 4.5 |
| Lower rack | margins left / right / front / rear | 7.7 / 10.5 / 10.5 / 12 |
| Basket | body L x W x depth | 32 x 13 x 9.5 |
| Basket | height at the handle | 22 |
| Basket | compartments | four in a row (three cross partitions) |

The user said the basket is the only part that matters; the rest of the appliance may
relax in realism.

### Decisions confirmed with the user

| Id | Decision |
|---|---|
| D1 | Outer size, tine count and pitch are trusted. Margins are DERIVED: side margin = symmetric leftover; front/rear split the leftover depth by the tape ratio. Reported as derived. |
| D2 | Upper middle columns lack positions 6 and 7 of 13 from the front. |
| D3 | Lower rows 3 and 4 (all their tines) are 45 mm. |
| D4 | Basket rear-right inside the right rim, long axis along Y; tines under its footprint removed and base rails truncated; wire-lattice walls; handle arch along the long axis to 220 mm. |
| D5 | Scope: geometry + USD rebuild + validation + assembly evidence + cutlery pose pool regenerated + HOTEC re-plan/re-render. Claims manifests, initial states, organized states, planner pool/results, random-pose metadata and exposure results are NOT regenerated and are stale. |
| D6 | New revision ids (`upper_tines_4x13_v2`, `lower_tines_6x12_v2`, `basket_1x4_320x130_v1`); v3 archived. |
| D7 | Tests re-pinned to the new geometry. |
| D8 | Claims variant A resized: front bank 6 dinner + 2 salad + 2 bowls, rear bank 6 dinner + 2 salad, 5 tumblers per side. Variant B retired (no bank has 11 usable gaps). |

### Derivations (mm)

| Rack | Leftover | Split | Result |
|---|---|---|---|
| Upper, width | 480 - 3 x 90 = 150 | symmetric | side 105 (tape 120) |
| Upper, depth | 515 - 12 x 37 = 71 | 80 : 55 | front 42.074, rear 28.926 (tape 80 / 55) |
| Lower, width | 525 - 11 x 36 = 129 | 77 : 105 | left 54.577, right 74.423 (tape 77 / 105) |
| Lower, depth | 563 - 5 x 80 = 163 | 105 : 120 | front 76.067, rear 86.933 (tape 105 / 120) |

The tape margins over-determine the field by 30 mm (X) and 64 mm (Y) upstairs and
53 mm (X) and 62 mm (Y) downstairs. Upper columns x = +-45, +-135; positions
y = -215.426 + 37 i. Lower columns x = -207.923 + 36 i; rows y = -205.433 + 80 j;
heights 95, 95, 45, 45, 95, 95 on a base at z = 6. The basket footprint x [104, 234],
y [-64, 256] plus 6 mm clearance removes columns 10 to 12 (1-based) of rows 3 to 6:
12 tines, leaving 60 (12, 12, 9, 9, 9, 9). Plate banks sit at y = -165.433 (front, 11
gaps) and y = 154.567 (rear, 8 gaps). A 260 mm dinner plate centred on a bank would cut
the rim wire (rows 80 mm apart in a 563 mm rack), so plate candidates seat 25 mm
rearward in the front bank and 15 mm forward in the rear bank (claims), 10 mm rearward
for the HOTEC front-bank plates, and the plate fixture site moved to y = -150.

### Upper shelf redesign

The photo-fitted mug valley and crest were dropped. In a 480 mm rim the crest
(outer column + 20.5 = 155.5) left 79.5 mm to the wall at 235, less than an 80 mm
tumbler. Each side floor is now one V-shelf: central ridge +-53 at z -10 carrying the
inner columns, shoulders at 62 (z +5) and 72 (z -5), floor z -12.2 at the outer column
(x 135), the glass trough at x 185 (z -18), rising at the same 6.6 deg gradient to x 225
(z -13.4), then the wall foot at 232 (z +2), the wall at 235 (z 61) and the rim at 237.8
(z 125). Cup channel site x +-103.5, glass trough site x +-185. An inverted 80 mm
tumbler stands upright in the trough with about 8 mm to the outer tine column and
5 mm to the wall foot; the 125 mm wall caps its outward lean at about 4 deg. The tine
columns, not floor crests, separate the cup channel from the glass channel.

### Basket (own frame, origin at the bottom face centre)

Envelope 130 x 320 x 95 body, handle arch top 220 (feet at y +-154, z 92; flat top
half-span 40 at z 214.5, wire r 5.5); rims r 3 inside the envelope; three cross
partitions at y = -80, 0, +80 (compartment centres y = -120, -40, 40, 120); lattice
floor 45 cross x 17 long ribs (r 1.1, floor top z 5.6); long walls 31 uprights, end walls
12 uprights, 8 courses; corner posts r 2.5; 252 wire paths. Seat in the lower rack
[169, 96, 7.5]; assembly origin [169, 104, 222.5]. Clearance audit (`[RESULT] PASS`):
tines and base rails 14.03 mm, right wall 3.5 mm, floor 1.26 mm, remaining rack
1.64 mm, visible wheel-bracket z gap 0.42 mm, no rails under the basket, 12 removed tines.

### Commands run, in order

```bash
# 1. archive the previous build as v3
python3 frigidaire/scripts/setup/stage_frigidaire_collection.py --archive-current v3 \
    --out-dir build/frigidaire_collection
# 2. previews (48 upper tines; 60 lower tines + basket_front/top/oblique.png)
python3 frigidaire/scripts/evaluation/frigidaire_upper_rack_preview.py \
    --out-dir build/frigidaire_collection/images/upper_rack
python3 frigidaire/scripts/evaluation/frigidaire_lower_rack_preview.py \
    --out-dir build/frigidaire_collection/images/lower_rack
# 3. clearance audit -> validation/lower_rack_clearance.json
python3 frigidaire/scripts/evaluation/frigidaire_lower_rack_clearance.py
# 4. USD rebuild
scripts/run_py.sh frigidaire/scripts/setup/build_frigidaire.py
# 5. composition inspection
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_collection_inspect.py \
    --collection-dir build/frigidaire_collection
# 6. cutlery pose pool (Kit-free FCL), copied to src/dishsim_frigidaire/cutlery_candidates.json
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_cutlery_pose_search.py \
    --write-candidates build/frigidaire_diagnostics/cutlery_candidates.json
# 7. claims variant A dry run (Kit-free FCL; a diagnostic, not the claims evidence)
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_claim_layouts.py --variant A \
    --out-dir build/frigidaire_diagnostics/claims_v4
# 8. tests
scripts/run_py.sh -m pytest frigidaire/tests -q
scripts/run_py.sh -m pytest tests -q
# 9. assembly physics evidence (Kit)
scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_asset_evidence.py --headless --device cpu \
    --assembly-only --physics-only --usd build/frigidaire_collection/usd/fdpc4221as.usdc \
    --out-dir build/frigidaire_collection/images/assembly
```

| Step | Result | Record |
|---|---|---|
| archive | 165 files, 301.6 MB | `build/frigidaire_collection/history/v3/archive_manifest.json` |
| previews | 48 / 60 tines drawn | `images/upper_rack/`, `images/lower_rack/measurements.json` |
| clearance | `[RESULT] PASS` | `validation/lower_rack_clearance.json` |
| build | `[RESULT] PASS` | `logs/frigidaire_v4_build.log`; `usd/parameters.json`, `usd/geometry_validation.json` |
| inspect | `[RESULT] PASS` | `logs/frigidaire_v4_inspect.log` |
| cutlery pose pool | free poses fork 1031, knife 1132, tablespoon 1440, teaspoon 1495; greedy simultaneous 9 / 9 / 6 / 9 (gate >= 4), one kind per compartment; `claims.BASKET_KEYS` = the first four greedy poses per kind; no physics refinement | `logs/frigidaire_v4_cutlery_search.log` |
| claims A dry run | `[RESULT] PASS`, 49 objects: 12 dinner + 4 salad + 2 lower bowls; 10 tumblers (all upright, 10 mm hover) + 2 saucers + 3 upper bowls; 16 cutlery | `build/frigidaire_diagnostics/claims_v4/claim_A_geometry.json`, `logs/frigidaire_v4_claims_dry_run.log` |
| tests | `frigidaire/tests` 421 passed, 6 skipped (the planner pool tests skip as stale: the cached organized pool records v3 usdc hashes and the old basket seat); Bosch `tests/` 60 passed | `logs/frigidaire_v4_pytest.log`, `logs/bosch_tests_2026-09-22.log` |
| assembly physics | `[RESULT] PASS`, all `[OK]` gates including basket lift and replacement (tracking error < 0.03 mm, rack disturbance < 4 um); combined certification INCOMPLETE until the render pass below | `logs/frigidaire_v4_assembly_physics.log` |
| assembly render | `[RESULT] PASS`, all `[OK]` image and settled-contact gates (peak penetration about 1.3 um); combined certification PASS, `evidence.json.result == "PASS"` (scope assembly) | `logs/frigidaire_v4_assembly_render.log`, `images/assembly/evidence.json` |
| HOTEC v2 layout (Kit-free) | `[RESULT] PASS`: plates 8/8, bowls 6/8 (lower 2, upper 4), cups 8/8, counter 2 bowls (v1 on the old racks: bowls 4/8, counter 4); cups near-upright like the tumblers | `results/hotec/frigidaire/v2/layout.json`, `logs/hotec_v2_layout.log` |
| HOTEC v2 settle and orbit (Kit) | `[RESULT] PASS`: settle passed after 6.25 s simulated (no restarts), max placed-to-settled displacement 23 mm / 13 deg (a lower plate), all 24 pieces contained, assets unchanged, 288-frame orbit; wall time 109 s | `results/hotec/frigidaire/v2/hotec_v2_{settle,evidence}.json`, `logs/hotec_v2_load.log`; `docs/figures/hotec_loaded_fdpc4221as.png` regenerated |
| package check | `[RESULT] PASS: Frigidaire collection check` (history v1, v2, v3 listed) | `python3 frigidaire/scripts/setup/package_frigidaire.py --collection-dir build/frigidaire_collection --check` |
| history README | regenerated from `stage_frigidaire_collection.history_readme()` so it names the v2 / v2 / basket_1x4 revisions as current | `build/frigidaire_collection/history/README.md` |

### Stale after this revision

Everything recorded against the v3 hashes describes the archived geometry only:

- `build/frigidaire_collection/validation/claims/` (the claims manifests; the v4 dry run above is a diagnostic under `build/frigidaire_diagnostics/claims_v4`)
- `results/initial_states/frigidaire/` (packing states and their organized counterparts)
- `results/planner/frigidaire/` (candidate pool and planner results)
- `results/random_poses/frigidaire/` (random-pose metadata)
- `results/exposure/frigidaire/` (every recorded exposure score)
- `results/hotec/frigidaire/v1/` (the HOTEC v1 load)

The authoritative list is `stale_results` in
`build/frigidaire_collection/history/v3/archive_manifest.json`.

## 2026-09-22 (later): v3 outer floor restored inside the tape rim (`upper_tines_4x13_v3`)

After the tape rebuild the user compared the racks with the previous build and said the outer
wire structure of the upper rack was further from the real rack than v3 had been. Read-only
comparison of the committed (v3) and working-tree generators showed identical wire families
and counts (21 cross loops, 9 longitudinal loops, top and mid rims, grip, carriers) and
vertical lower-rack walls in both; the v3 upper side wall was also near-vertical (3.5 deg).
What the V-shelf of `upper_tines_4x13_v2` had removed was the v3 signature: the mug valley and
ridge beside the outer column and the glass slope descending outward to a deep trough against
the wall. Inside the 480 mm rim a verbatim v3 crest (+18 mm at column + 20.5 = 155.5) leaves
75 mm to the wall foot, which no 80 mm tumbler mouth passes at floor level.

### Decisions (pop-up)

| Question | Answer |
|---|---|
| Which rack | upper only |
| What looked wrong | the outer floor/wall shape (flare framing corrected: it is the slope-to-trough shape), rim height and loop count queried |
| Widths | keep the tape rims (480 x 515, rim 125); re-fit the v3 shape inside them |
| Flare vs the 80 mm prototype tumbler | realistic shape first; the tumbler count is whatever FCL places |
| Rim height / loops | keep 125 mm from the tape; keep 21 / 9 loops |
| Section | v3 look with a LOW ridge (+4 mm nominal instead of +18) so near-upright 80 mm tumblers and 71 mm cups still fit |
| Validation | full pipeline again (build, inspect, claims dry run, tests, assembly physics + render, HOTEC run v3, docs) |
| Archive | overwrite the one-day-old v4 build, no archive; only the upper revision id changes |

Rollers and wheels stay on the cabinet tracks (not raised by the user).

### Section (half, nominal corners, mm)

ridge 53 @ -10, shoulder 62 @ +5 and 72 @ -5, valley 145.5 @ -18 (column + 10.5), ridge 155.5 @ +4
(column + 20.5; about +0.3 after the 9 mm fillet), trough 220 @ -18 (rim - 17.8), wall foot 231 @ +2
(rim - 6.8), wall 235 @ 61, rim 237.8 @ 125. Longitudinal cradles at +-220, +-187.75, +-145.5,
+-72, 0. Sites: cup channel +-108.75, glass channel +-187.75. `channel_profile` keys:
`mug_valley_offset_from_column`, `ridge_offset_from_column`, `ridge_z`, `trough_inset_from_rim`,
`wall_foot_inset_from_rim`, `central_ridge_half_width`. Consumers: `claims._glass_channel_x`
(mid slope), `UPPER_GLASS_FLOOR -.002` (the higher, wall-foot mouth contact), `TUMBLER_VARIANTS`
near-upright family (0 / 4 / 8 deg, insets 0 / +3 / -3 / +5 mm, lifts 10-12 mm), `loading.candidates`
mirror, `organized_candidates` xs +-.18775 / +-.10875, HOTEC `CUP_VARIANTS`.

### Commands and results, in order

| Step | Result | Record |
|---|---|---|
| source smoke | 48 / 60 tines; section minimum -16.8 mm in the valley, trough -14.8 mm, ridge +0.3 mm, wall foot +3.2 mm | host python3 |
| upper preview | `[RESULT] GENERATED: 48 tines`; front view shows valley, low ridge, outward slope, trough at the wall | `images/upper_rack/` |
| asset tests | 14 passed (section test re-pinned: trough within 3 mm of the minimum and against the wall, ridge > 10 mm above the valley and below +6 mm, slope monotonic) | |
| build | `[RESULT] PASS`; `lower_rack.usdc` and `silverware_basket.usdc` sha256 unchanged (325671ed..., 70e93880...), `upper_rack.usdc` fdde95e4... | `logs/frigidaire_v5_build.log` |
| inspect | `[RESULT] PASS` | `logs/frigidaire_v5_inspect.log` |
| claims A dry run | `[RESULT] PASS`, 49 objects; all 10 tumblers upright (`x0.18275_lift0.012`) | `build/frigidaire_diagnostics/claims_v5/`, `logs/frigidaire_v5_claims_dry_run.log` |
| tests | `frigidaire/tests` 421 passed, 6 skipped (planner pool stale) | `logs/frigidaire_v5_pytest.log` |
| lower preview, clearance, history README | regenerated for the new source hash; clearance `[RESULT] PASS` (unchanged numbers) | `images/lower_rack/`, `validation/lower_rack_clearance.json`, `history/README.md` |
| HOTEC v3 layout (Kit-free) | `[RESULT] PASS`: plates 8/8, bowls 6/8, cups 8/8, counter 2 | `results/hotec/frigidaire/v3/layout.json`, `logs/hotec_v3_layout.log` |
| assembly physics | `[RESULT] PASS`, all 14 `[OK]` gates including basket lift and replacement; combined certification INCOMPLETE until the render pass below | `logs/frigidaire_v5_assembly_physics.log` |
| assembly render | `[RESULT] PASS`, combined certification PASS, `evidence.json.result == "PASS"` (scope assembly) | `logs/frigidaire_v5_assembly_render.log`, `images/assembly/evidence.json` |
| package check | `[RESULT] PASS: Frigidaire collection check` | `python3 frigidaire/scripts/setup/package_frigidaire.py --collection-dir build/frigidaire_collection --check` |
| HOTEC v3 settle and orbit (Kit) | `[RESULT] PASS`: settle 6.24 s (no restarts), max displacement 25 mm / 14 deg (cups sliding down the restored glass slope), all 24 pieces contained, assets unchanged, 288-frame orbit; wall time 114 s | `results/hotec/frigidaire/v3/hotec_v3_{settle,evidence}.json`, `logs/hotec_v3_load.log`; `docs/figures/hotec_loaded_fdpc4221as.png` regenerated |

Run v2 (`results/hotec/frigidaire/v2`) and the v4 assembly evidence
are superseded by the above; the v4 usd was overwritten in place (no archive, per the user).

## 2026-09-22 (later still): the v3 basket design at the tape dimensions (`basket_1x4_320x130_v2`)

After the upper-rack fix the user asked for the basket to be "the old design with the new
dimensions". Read-only comparison of the v3 generator (`git show HEAD:...geometry.py`) with the
tape rebuild's box basket, plus the basket photos (front, top, top_down, bottom), settled what
"the old design" means: a body that tapers toward the floor (v3: 74.6 x 296.9 mm floor under an
88 x 312 mm rim), a substantial double bottom edge and a double top lip, a dense 7 mm square
lattice, corner posts, three tapered cross partitions with a top edge, and a solid elongated
loop handle with an open aperture lying over one long wall on two support straps.

### Decisions (pop-up)

| Question | Answer |
|---|---|
| Handle | the v3 loop over one long wall (not the centred arch); the OUTER (+X) wall toward the rack's right side, as v3 |
| Handle top | 220 mm as measured, so the v3 loop shape rides on taller legs (the photos' one-third proportion noted, tape wins) |
| Taper | the v3 floor/rim ratios (0.848 / 0.952) under the 130 x 320 rim |
| Partitions | equal quarters at y = -80, 0, +80 (top-down photo) |
| Other v3 details | all back: double rims and lip, 7 mm lattice, tapered partitions with top edge, corner posts, straps |
| Validation | full pipeline again, overwrite the same-day build in place, no archive |

### Numbers (mm, basket frame)

Top rim centreline +-62 / +-157 at z 92; bottom rim +-52.1 / +-149.3 at z 5; reinforcement
0.5 mm outside at z 14 (r 2.3); lip 0.5 mm inside at z 87 (r 2.4); floor ribs 43 x 15 (r 1.1) at
z 3 / 4.5; walls 43 / 15 uprights and 8 courses (z 21 to 80); corner posts r 2.5; partitions 15
uprights + 9 courses + top edge (z 90) at y = -80 / 0 / +80; handle plane x = 58, legs y = +-109
from the rim to z 163.5, flat top z 213.5 (surface 220) spanning +-75, lower rail z 172 / 176,
straps at y = +-80. Lattice counts derive from the 7 mm pitch in `_derive_parameters`.
The seat is unchanged (`[169, 96, 7.5]`): the top-rim footprint, removed tines and rails stay;
the tapered bottom rim clears the right wall and the rear floor bend by about 8 mm (was 3.5 / 1.6).

### Commands and results, in order

| Step | Result | Record |
|---|---|---|
| source smoke | basket 297 wires, centreline extent +-62 / +-157 / 3 to 213.5, families as designed | host python3 |
| basket tests | asset, clearance, random-pose tests 54 passed after re-pinning the wall probe (+18 mm) and adding `test_basket_keeps_the_v3_design_at_the_tape_size` | |
| lower preview | `[RESULT] GENERATED: 60 tines` + `basket_front/top/oblique.png` | `images/lower_rack/` |
| clearance | `[RESULT] PASS`: tines 14.03, right wall 8.05, floor 3.4, remaining 8.48, bracket 1.92 mm | `validation/lower_rack_clearance.json` |
| build | `[RESULT] PASS`; racks unchanged (`upper_rack.usdc` fdde95e4..., `lower_rack.usdc` 325671ed...), `silverware_basket.usdc` f6b59b82... | `logs/frigidaire_v6_build.log` |
| inspect | `[RESULT] PASS` | `logs/frigidaire_v6_inspect.log` |
| cutlery pose pool | free poses fork 997, knife 1518, tablespoon 1391, teaspoon 1304; greedy simultaneous 9 / 9 / 6 / 9; `claims.BASKET_KEYS` re-picked | `logs/frigidaire_v6_cutlery_search.log`, `cutlery_candidates.json` |
| claims A dry run | `[RESULT] PASS`, 49 objects, all 16 cutlery | `build/frigidaire_diagnostics/claims_v6/`, `logs/frigidaire_v6_claims_dry_run.log` |
| tests | `frigidaire/tests` 422 passed, 6 skipped | `logs/frigidaire_v6_pytest.log` |
| HOTEC v4 layout (Kit-free) | `[RESULT] PASS`: plates 8/8, bowls 6/8, cups 8/8, counter 2 | `results/hotec/frigidaire/v4/layout.json`, `logs/hotec_v4_layout.log` |
| assembly physics | `[RESULT] PASS`, all 14 `[OK]` gates including basket lift and replacement with the loop handle; combined certification INCOMPLETE until the render pass below | `logs/frigidaire_v6_assembly_physics.log` |
| assembly render | `[RESULT] PASS`, combined certification PASS, `evidence.json.result == "PASS"` (scope assembly) | `logs/frigidaire_v6_assembly_render.log`, `images/assembly/evidence.json` |
| package check | first `[RESULT] FAIL` (`Stale upper_rack source geometry measurements`: the upper preview's recorded source hash predated the basket edit of the shared generator), `[RESULT] PASS` after regenerating `images/upper_rack/` | `python3 frigidaire/scripts/setup/package_frigidaire.py --collection-dir build/frigidaire_collection --check` |
| HOTEC v4 settle and orbit (Kit) | `[RESULT] PASS`: settle 6.23 s (no restarts), max displacement 25 mm / 15 deg (cups down the glass slope), all 24 pieces contained, assets unchanged, 288-frame orbit; wall time 122 s | `results/hotec/frigidaire/v4/hotec_v4_{settle,evidence}.json`, `logs/hotec_v4_load.log`; `docs/figures/hotec_loaded_fdpc4221as.png` regenerated |
| history README | regenerated via `history_readme()` (names `basket_1x4_320x130_v2` as current) | `build/frigidaire_collection/history/README.md` |

Runs v2 and v3 of the HOTEC load and the earlier same-day assembly evidence are superseded; the box-basket usd was overwritten in place (no archive, per the user).

## 2026-09-22 (evening): basket axes corrected to 320 x 95 x 130, re-seated (`basket_1x4_320x95_v3`)

The user, on seeing the `basket_1x4_320x130_v2` renders: "Swap the length and depth the handle
side is 22cm the other side should be 13cm. Use the front view x Y should be 13cm and handle add
up to 22 and Z should be 9.5. Fix the asset". Read from the FRONT view of the real basket: it is
32 cm long (Y), 9.5 cm wide (X) and 13 cm tall (Z, body), with the handle top at 22 cm. The two
earlier same-day readings (130 wide x 95 tall, the `basket_1x4_320x130_v1` box and the
`basket_1x4_320x130_v2` v3-design body) had the width and the height swapped. The v3
photo-fitted design decided in the previous step is unchanged (tapered lattice body, double bottom
edge and top lip, corner posts, three tapered cross partitions with a top edge, elongated loop
handle with an open aperture over the +X long wall on two straps); only the size and the seat
changed. Source of truth: `PARAMETERS["silverware_basket"]` in `geometry.py` (`length_y` .320,
`width_x` .095, `body_height` .130, `handle_top_z` .220; taper .848 / .952; rims; 7 mm floor and
wall lattice pitch, 9.2 mm course pitch; counts derived in `_derive_parameters`) and
`PARAMETERS["lower_rack"]` (`basket_reserved_x/y` derived from the seat).

### Decisions (pop-up)

| Question | Answer |
|---|---|
| Dimensions | 320 long (Y) x 95 wide (X) top rim, 130 mm body height (Z), handle top 220 mm; the v3 design as is |
| Seat | re-seat against the right wall for the narrower body (x 184, sweep below) rather than keep the 130 mm basket's x 169 |

### Derivations (mm)

- Seat sweep along X inside the right rim: the tapered bottom rim keeps 8.0 mm to the sloping
  right wall fillet at x 184 (was 169 for the 130 mm wide basket); seat `[184, 96, 7.5]`, assembly
  origin `[184, 104, 222.5]`. The rear floor bend stays at about 8 mm.
- Reserved bay x [130.5, 237.5], y [-70, 262] (the 95 x 320 footprint x [136.5, 231.5],
  y [-64, 256] plus 6 mm). Tine mask 64 present (12, 12, 10, 10, 10, 10): columns 11-12
  (1-based; x 152.08 and 188.08) of rows 3-6 removed, 8 tines; the base rails of rows 3-6 end at
  column 10 (x 116.08). Plate banks: front 11 gaps (9 left of the bowl zone), rear 9 gaps. The
  lower-rack revision id stays `lower_tines_6x12_v2` (the mask is derived from the seat, not a
  design change).
- Basket body (own frame): top rim centreline +-44.5 / +-157 at z 127 (r 3); floor envelope
  80.6 x 304.6 (bottom rim +-37.3 / +-149.3 at z 5); reinforcement z 14 (r 2.3, 0.5 outside);
  top lip z 122 (r 2.4, 0.5 inside); floor lattice 43 cross x 11 long ribs at 7 mm (r 1.1),
  apertures about 4.8 mm; long walls 43 uprights, end walls 10 uprights; corner posts r 2.5;
  partitions at y = -80, 0, +80 with 10 tapered uprights and a top edge at z 125 (r 2.1).
- Course counts from the 9.2 mm pitch in `_derive_parameters`: walls
  round((127 - 12 - 21) / 9.2) + 1 = 11 courses (z 21 to 115), partitions
  round((127 - 9 - 14) / 9.2) + 1 = 12 courses (z 14 to 118).
- Handle: plane x = 40.5 (4 mm inside the top rim), legs at y = +-109 from the rim (z 127) to
  z 163.5, the v3 bends to a flat top at z 213.5 (surface 220) spanning y = +-75 (r 6.5), lower
  rail r 5.5 at z 172 (middle) / 176 (ends), aperture about 30 mm, straps at y = +-80 from the
  floor up the outer wall to the lower rail. Sites: handle_center [40.5, 0, 213.5],
  handle_aperture [40.5, 0, 192], utensil [0, -120, 96], compartments at y -120 / -40 / 40 / 120.
  289 wire paths.

### Consumer and test changes

- `claims.py` docstring: the rear bank has 9 gaps (the basket bay omits columns 11-12); variant A
  claims 8 of them (6 dinner + 2 salad). `claims.BASKET_KEYS` re-picked from the new pose pool
  (the first four greedy poses per kind).
- `frigidaire_hotec_load.py` docstring: the basket bay removes columns 11-12 of rows 3-6;
  `REAR_GAPS` is every second of the 9 gaps the bay leaves.
- Tests re-pinned: tines per row `[12, 12, 10, 10, 10, 10]` and the seat `[.184, .096, .0075]`
  (`test_frigidaire_asset.py`), basket extent `.095 x .320` and the v3 taper of it
  (`test_frigidaire_asset.py`), seat x 184.0, 8 removed tines and the 18 mm wall-probe shift
  (`test_frigidaire_lower_clearance.py`), plate gaps `(9, 9)` front-usable / rear
  (`test_frigidaire_lower_loading.py`), the claims blocked-gap comment (`lower_rear_07`, the last
  claimed gap of the rear bank: 9 gaps, 8 claimed; `test_frigidaire_claims.py`).

### Commands and results, in order

| Step | Result | Record |
|---|---|---|
| lower preview (host) | 64 tines (12, 12, 10, 10, 10, 10), 8 removed, basket block at seat `[184, 96, 7.5]`, 289 wires; `basket_front/top/oblique.png` | `images/lower_rack/measurements.json` |
| clearance audit | `[RESULT] PASS`: tines and base rails 14.41, right wall 8.0, floor 3.4, remaining rack 8.48, bracket z gap 1.92 mm, no rails under the basket, 8 removed tines | `validation/lower_rack_clearance.json` |
| build | `[RESULT] PASS`; `silverware_basket.usdc` 80225c34... (`basket_1x4_320x95_v3`, 289 wire paths), `lower_rack.usdc` d9adf05a... (`lower_tines_6x12_v2`), `upper_rack.usdc` unchanged fdde95e4... (`upper_tines_4x13_v3`) | `logs/frigidaire_v7_build.log` |
| inspect | `[RESULT] PASS` | `logs/frigidaire_v7_inspect.log` |
| cutlery pose pool | free poses fork 908, knife 1413, tablespoon 755, teaspoon 1137; greedy simultaneous 9 / 9 / 5 / 9 (gate >= 4) | `logs/frigidaire_v7_cutlery_search.log`, `cutlery_candidates.json` |
| claims A dry run | `[RESULT] PASS`, 49 objects: lower 12 dinner + 4 salad + 2 bowls, upper 10 tumblers + 2 saucers + 3 bowls, basket 16 cutlery | `build/frigidaire_diagnostics/claims_v7/claim_A_geometry.json`, `logs/frigidaire_v7_claims_dry_run.log` |
| tests | `frigidaire/tests` 422 passed, 6 skipped (planner pool stale); re-run after the docstring fixes below with the same result | `logs/frigidaire_v7_pytest.log` |
| HOTEC v5 layout (Kit-free) | `[RESULT] PASS`: plates 8/8, bowls 6/8, cups 8/8, counter 2 (same as v2-v4: no HOTEC piece touches the basket) | `results/hotec/frigidaire/v5/layout.json`, `logs/hotec_v5_layout.log` |
| HOTEC v5 settle and orbit (Kit) | `[RESULT] PASS`: settle 6.23 s (no restarts), max displacement 25 mm / 14 deg, all 24 pieces contained, assets unchanged, 288-frame orbit; wall time 120 s | `results/hotec/frigidaire/v5/hotec_v5_{settle,evidence}.json`, `logs/hotec_v5_load.log`; `docs/figures/hotec_loaded_fdpc4221as.png` regenerated |
| history README, staged README | regenerated via `history_readme()`; `build/frigidaire_collection/README.md` refreshed from `docs/collection_readme.md` (the staged copy still described the v3 racks) | `build/frigidaire_collection/{history/README.md,README.md}` |
| docs verification (workflow) | a read-only verifier re-derived every number (consistent) and found stale comment text in `geometry.py` (130 x 320 envelope), `claims.py` (12 tines, 320 x 130 x 95), `frigidaire_claim_report.py` (60-tine) and two test comments; all fixed, which changed the hashed source, so inspect and the assembly evidence were re-run (rows below) | `logs/frigidaire_v7_inspect.log` |
| inspect (after the docstring fixes) | `[RESULT] PASS` | `logs/frigidaire_v7_inspect.log` |
| assembly physics (final sources) | `[RESULT] PASS`, all 14 `[OK]` gates including basket lift and replacement with the 95 mm basket at seat x 184 | `logs/frigidaire_v7_assembly_physics.log` |
| assembly render (final sources) | `[RESULT] PASS`, combined certification PASS, `evidence.json.result == "PASS"` (scope assembly) | `logs/frigidaire_v7_assembly_render.log`, `images/assembly/evidence.json` |
| package check | `[RESULT] PASS: Frigidaire collection check` (after regenerating both previews for the current source hash; the check had first flagged the stale upper preview and then the edited sources) | `python3 frigidaire/scripts/setup/package_frigidaire.py --collection-dir build/frigidaire_collection --check` |

The same-day builds with the 130 mm-wide baskets (v1 box, v2 v3-design) and their evidence are
superseded; the usd was overwritten in place (no archive, per the user). HOTEC runs v2 to v4 were kept
on disk (moved to the hold folder on 2026-09-29) but superseded by v5.

The two earlier same-day baskets (v1 box 130 x 320 x 95 with a centred arch, v2 = the v3 design
at 130 x 320 x 95) are superseded; the builds were overwritten in place, no archive.

## Outside heights (2026-09-23)

Prompted by the cm dimension drawings (`docs/figures/frigidaire_{upper,lower}_rack_cm.png`,
`frigidaire_rack_dimensions_cm.py`): the build had placed each rack's rim wire CENTRE at the tape
height above an internal datum, while the user measured outside, bottom to top (pop-up answer).
Measured on the generated wires, the upper rack was 145.9 mm outside (its floor dips 16.8 mm below
the datum), the lower 119.4 mm and the basket 128.5 / 218.5 mm from its lowest wire. User choices:
lower the upper rim (keep floor, tines, rollers), fix the lower rim and the basket too, rebuild and
re-run HOTEC. Done: upper `rim_height` .125 -> .1041, lower .115 -> .1106 (new `outer_height_tape`
parameters, pinned by `test_outer_heights_match_the_tape`); basket bottom features 1.5 mm lower in
its own frame and seat z 7.5 -> 9 mm, so it rests where it did and measures 130 / 220; revisions
`upper_tines_4x13_v4`, `lower_tines_6x12_v3`, `basket_1x4_320x95_v4`; the mm-for-cm typo in the
margin notes fixed. Pipeline: `--archive-current v4`, previews, clearance PASS (tine clearance 14.41 -> 14.63 mm, resting clearances unchanged), build,
cutlery search (counts 908/1413/750/1108, simultaneous 9/9/5/9; two teaspoon `BASKET_KEYS`
re-picked), claims A dry run `claims_v8` (PASS, 49), tests 423 passed / 6 skipped + Bosch 61, inspect,
assembly evidence physics 14/14 and render 34/34, combined PASS, package check PASS. HOTEC on the
new twin: v11 (v8 poses) PASS, v12 (v9 poses) PASS, v13 (sequential) FAIL at the known mid-zone bowl.
