# Frigidaire FDPC4221AS dishwasher asset

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

Open `media/frigidaire_fdpc4221as_v2/index.html` for the revised labeled full-resolution image
gallery. Each component has front, right, front-left and overhead views. Additional
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

The [source clearance audit](../build/frigidaire_lower_rack_polish/source_clearance.json)
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

The [full-load gallery](../media/frigidaire_fdpc4221as_v2/full_load/index.html)
is saved under `media/frigidaire_fdpc4221as_v2/full_load/`.
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

The preserved [component and assembly gallery](../media/frigidaire_fdpc4221as_v2/index.html)
and its [combined evidence report](../media/frigidaire_fdpc4221as_v2/evidence.json)
passed with the release's asset and executable-source hashes. All 36 base physics checks,
six contact inspections, and 31 rendered-image checks passed. The largest measured
door endpoint error was 0.1411°; the largest slide endpoint error was 0.0509 mm.
The component contact inspections recorded at most 0.00199 mm penetration and
488 contacts out of an 8192-contact buffer. These simulator separation values do
not imply comparable manufacturing accuracy.

The [full-load physics report](../media/frigidaire_fdpc4221as_v2/full_load/physics.json)
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

All 20 [full-load images](../media/frigidaire_fdpc4221as_v2/full_load/index.html)
passed, bringing the revised evidence to **51 rendered views**: 31 component and
fixture views plus 20 loaded-assembly and inventory views. The
[combined full-load evidence](../media/frigidaire_fdpc4221as_v2/full_load/evidence.json)
and [rendering provenance](../media/frigidaire_fdpc4221as_v2/full_load/render_settings.json)
verified the release's physics, asset, manifest, executable sources, and image hashes.
The images show the accepted physical states; inventory views use the recorded
charcoal studio backdrop to make the translucent tumbler's rim and shell clear.
The final render log is `logs/frigidaire_v2_full_renders_studio.log`.

The [standalone scene smoke check](../media/frigidaire_fdpc4221as_v2/full_load/standalone_smoke.json)
passed with 360 actual PhysX events at 120 Hz, covering three seconds of the
authored closed hold. All 67 objects maintained contact paths to their intended
supports; maximum per-axis displacement from the exported poses was 2.175 mm.
The support graph retained 92 measured connections from step 51 after sleeping
bodies stopped issuing contact reports. Each retained connection satisfied the
1 µm surface-motion condition; the largest observed bound was 0.497 µm. Final
contact reports were empty, so those retained connections are identified
separately in the report. Authored physics, drives, and sleep attributes remained
unchanged.

The [measured bowl non-nesting audit](../media/frigidaire_fdpc4221as_v2/full_load/non_nesting.json)
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

The preserved first-release reports in `media/frigidaire_fdpc4221as/` record a combined `PASS` with
matching asset and executable-source hashes:

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
