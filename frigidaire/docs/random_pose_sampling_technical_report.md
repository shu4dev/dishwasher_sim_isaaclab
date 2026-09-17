# Technical report: sampling dish poses in a dishwasher

Experiment: `complete_20260910_seed0` · Frigidaire FDPC4221AS · Isaac Sim 4.5 · completed 10 September 2026

**The experiment uses stratified Monte Carlo sampling of single-object release poses.** It evaluates every pairing of three object types and two racks, but only a finite random sample of positions and rotations within each pairing. It is neither a permutation enumeration nor an exhaustive search of all feasible placements.

The completed scan contains **600 distinct proposal IDs: 100 per object/rack pairing**. Of these, **176 produced accepted final placements (29.33%)**, including **168 strictly contained final placements (28.00%)**. Eight accepted placements use the configured 1 mm boundary tolerance. Two trials remain numerically unresolved. These counts describe successful trials; they are not counts of unique arrangements or simultaneous dishwasher capacity. The [finalized experiment report](../../results/random_poses/frigidaire/complete_20260910_seed0/experiment_report.md) and [raw trial records](../../results/random_poses/frigidaire/complete_20260910_seed0/trials.jsonl) supply the underlying evidence.

**A pose is a position together with an orientation.** A rigid dish has three translational and three rotational degrees of freedom. The implementation records a translation vector `t = (x, y, z)` in metres and a unit quaternion `q = (qx, qy, qz, qw)`. Four quaternion components encode three rotational degrees of freedom because they satisfy a unit-length constraint; `q` and `−q` describe the same physical orientation. The pose space is continuous, so it cannot be exhausted by listing 600 points.

For a fixed object kind `k` and rack `r`, the proposal coordinates belong to `B_r × SO(3)`: a three-dimensional center box and the space of three-dimensional rotations. Here the center is the center of the rotated visual mesh's axis-aligned bounding box, not necessarily the object's actor origin or center of mass. The overall experiment samples the six separate object/rack cases. Each trial contains exactly one active dish.

**All categorical pairings were tested; the continuous poses were sampled.** The factor design is the Cartesian product:

```text
{dinner plate, bowl, mug} × {lower rack, upper rack}
                           = 6 experiment cells

6 cells × 100 jointly sampled poses per cell = 600 proposals
```

All three kinds were tested on both racks, including plates on the upper rack. Catalog recommendations, slot patterns and uprightness were not used to restrict the proposals. “Combination” is reasonable informal language for an object/rack pairing. It does not mean the binomial-coefficient operation “choose k objects from n.” The [sampler and cell definitions](../src/dishsim_frigidaire/random_poses.py) establish this design.

**The rotation is sampled uniformly over orientations, not by drawing three uniform Euler angles.** The actual algorithm draws four independent standard normal values and normalizes their vector:

```text
g = (g1, g2, g3, g4),  each gi independently distributed as N(0, 1)
q = g / ||g||₂
```

The isotropic Gaussian has no preferred direction in four dimensions, so normalization produces a uniform point on the unit three-sphere. Mapping antipodal quaternion points to the same rotation gives the Haar-uniform distribution on `SO(3)`. This is the appropriate rotation-invariant meaning of “uniform random orientation.” The implementation uses its own NumPy-based routine; it does not call SciPy's rotation sampler. For the mathematical distinction between uniform orientations and uniform Euler coordinates, see [LaValle, Planning Algorithms, §5.2.2](https://lavalle.pl/planning/book.pdf); the same Haar-uniform target is specified by [SciPy's rotation documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.random.html).

Euler XYZ angles in the HTML viewer are a display conversion of the stored quaternion. Their histograms are not expected to be flat. They were not the quantities used to generate the rotations.

**The position sampler compensates for object geometry and actor-origin conventions.** Let `V_k` contain the object's original visual vertices, including the mug handle. For each drawn rotation, the code rotates these vertices and finds their bounding-box center. Independently, it draws a target center uniformly within the rack's fixed box:

```text
R = rotation matrix represented by q
a_k(q) = [coordinatewise min(R V_k) + coordinatewise max(R V_k)] / 2
c_j = lower_j + u_j (upper_j − lower_j),  uj independently uniform in [0, 1)
t = c − a_k(q)
```

Consequently, the placed visual bounding box is centered at `c`. The sampled center `c` and rotation `q` are independent. The actor translation `t` generally depends on `q`; it would be inaccurate to describe actor X/Y/Z as three independent coordinates drawn from the same fixed box for every orientation. This correction matters for the bowl's base origin and the mug's asymmetric handle.

The recorded center domains are below. Values are rack-local metres, rounded for display; [metadata.json](../../results/random_poses/frigidaire/complete_20260910_seed0/metadata.json) preserves full precision.

| Selected rack | Center X range (m) | Center Y range (m) | Center Z range (m) |
| --- | --- | --- | --- |
| Lower | −0.274320 to 0.274320 | −0.290830 to 0.290830 | 0.002000 to 0.356657 |
| Upper | −0.254000 to 0.254000 | −0.274320 to 0.274320 | −0.014543 to 0.227000 |

X/Y follow the generated rack rim dimensions. The lower Z datum comes from generated floor wires. The upper datum is the underside of the upper rack for lower-rack proposals, and the tub ceiling for upper-rack proposals. These are source-derived modeled bounds, not a proof that every feasible release lies in them. The upper rack's negative lower Z bound reflects its local frame and contoured floor.

The domain is **not shrunk according to object size or orientation**. Some proposed meshes therefore intersect wires or protrude beyond the center domain. Initial geometry collisions are counted as rejected proposals; the sampler does not repeatedly redraw until it obtains a collision-free or successful pose. Overhang beyond a rack rim is not automatically rejected: a dish may settle inside the enclosure later.

**Each proposal is transformed into the measured extended-rack frame and physically evaluated.** If the rack world transform is `(t_r, R_r)`, the release pose is:

```text
t_world = t_r + R_r t
R_world = R_r R(q)
```

The [runtime implementation](../src/dishsim_frigidaire/random_pose_runtime.py) resets to a validated empty-rack baseline before each trial. It checks the proposed dish against the measured dishwasher components using FCL. A collision-free proposal is released with zero initial linear and angular velocity, allowed to settle under gravity, and carried inward as the selected rack retracts. A final hold provides the endpoint measurements. The other rack stays retracted; the door stays open.

The implemented procedure can be summarized as follows:

```text
Create one reproducible random stream for each object/rack cell.
For sample index 0 through 99:
    For each of the six cells, in a fixed round-robin order:
        Draw one orientation and one bounding-box center.
        Derive the actor pose; transform it using the measured open rack.
        Count this proposal, even if the initial collision check rejects it.
        Otherwise: release → settle → retract rack → final hold → classify.
```

Root seed `0` is expanded with NumPy `SeedSequence.spawn(6)` into separate per-cell streams. The order is plate/lower, plate/upper, bowl/lower, bowl/upper, mug/lower, mug/upper. Round-robin scheduling distributes partial-run effort across all six cells. It does not mean the six streams use identical random poses. Reproducibility here means the recorded generator implementation, seed, domains and mesh sources reproduce the proposal sequence; bitwise-identical physical trajectories across unrelated runtime versions are not claimed.

The continuation preserved 260 classified records and performed 340 new evaluations. The previously interrupted proposal was retried at its original seeded pose. Thus there are **600 logical proposals and 601 cumulative evaluation attempts**, not 601 independent samples. The smoke run repeats early seed-0 proposals and is not added to this denominator. These accounting rules are implemented in the [scheduler](../src/dishsim_frigidaire/random_pose_experiment.py) and [resume validation](../src/dishsim_frigidaire/random_pose_resume.py).

**Acceptance describes a successful release-and-closure trajectory, with additional physical gates.** The experiment did not preserve the original pose rigidly during settling. A randomly tilted bowl may rotate substantially and end in a different accepted orientation. A starting proposal can also be outside the closed cabinet while the selected rack is extended. Final containment is tested after retraction.

| Implemented check | Recorded rule |
| --- | --- |
| Initial geometry | Proposed dish must pass the collision check before physical release. |
| Rack closure | Endpoint error at most 5 mm, with the prescribed other-rack and open-door state. |
| Final containment | Every visual mesh vertex must lie inside the modeled envelope, allowing 1 mm. An independent audit also checks the measured Cabinet frame. |
| Rest | A trailing 1 s window, root position span below 5 mm, orientation span below 3°, and tracked-point speed below 0.03 m/s. |
| Time allowance | Up to 12 simulated seconds for each dish settling/final-hold stage. |
| Support | Sustained contact-graph support from the selected rack, or via the basket to the lower rack; the basket must remain supported and at rest. |
| Contact validity | Rest-window peak penetration below 2 mm and median maximum penetration below 1 mm; peak below 2 mm throughout retraction and final hold. |
| Physics | CPU PhysX, 120 Hz, continuous collision detection; commanded rack speed capped at 0.10 m/s. |

Containment uses all visual vertices. The rest-speed metric uses six source-derived local vertex extrema; it is not an exhaustive speed calculation over every mesh vertex. Contact-based support plus observed rest is an operational acceptance criterion, not a separate force/wrench stability certificate. Initial impact penetration is recorded separately from the retraction limit.

These gates are stronger than the literal static conditions “inside the dishwasher and the rack can close.” They were used to require a supported, settled, numerically valid simulated placement. A configuration that fails this particular release, finite settling time or numerical gate is not thereby proved geometrically impossible. Door closability, robot access, insertion-path feasibility and multi-object capacity were not measured.

**A simulated acceptance does not establish that a real drop will reach the same placement.** These trials did execute in Isaac Sim, but the dishwasher and tableware have not been calibrated against physical drop experiments. The [staged model parameters](../../build/frigidaire_collection/usd/parameters.json) explicitly identify mass, friction, inertia and concealed mechanical resistance as requiring physical calibration. Tableware masses are estimates; the center of mass uses the visual bounds center, except for a prescribed 5 mm handle-side shift for the mug. Inertia is approximated by a solid bounding box rather than measured for the hollow dish. The authored tableware static/dynamic friction values are 0.48/0.34, with zero restitution; appliance contact values are 0.45/0.32, also with zero restitution. These are model settings, not measured properties of the user's dishes. See [tableware authoring](../src/dishsim_frigidaire/tableware.py) and [appliance authoring](../src/dishsim_frigidaire/asset.py).

The rack and dish collision shapes are also approximations, and the accepted support test establishes contact plus a short rest window. It does not test whether a marginal perch survives a small disturbance or repeats reliably across slightly different releases. These limitations are possible sources of unrealistic-looking placements, not a diagnosis of any particular trial. A disputed final placement should be reviewed by its trial ID: replay its original release in Isaac with video and visible collision shapes, extend the hold, and assess repeatability and small perturbations before treating it as robust. The HTML gallery reconstructs saved poses; its playback advances between independent trials and is not a drop-trajectory video.

**The specifically questioned `bowl_LowerRack_00010` was never physically dropped.** Its saved outcome is `initial_collision`, with both `settled_pose` and `final_pose` null. All ten trace rows are empty-rack initialization observations (`trial_reset`, `dish = null`); no dish settling or loaded retraction occurred. The raw field named `released_pose` stores the proposed world transform computed before the collision check, so that field's presence is not proof of an actual release. This case is excluded from the 176 accepted trials. The viewer formerly substituted this rejected proposal when Final or Settled was unavailable. That display behavior has been corrected: unavailable stages stay empty, and an explicitly selected rejected Sampled proposal is a red ghost labeled “Never dropped.” This finding explains this example without attributing it to a failure of the drop physics. It also reinforces the separate limitation that the experiment does not validate an ordinary loading path from outside the rack into every proposed starting configuration. See the [case evidence](../../outputs/sampling_technical_report/bowl_LowerRack_00010_review.json).

**The results estimate a probability under the chosen proposal distribution.** For cell `h`, define `p_h` as the probability that this sampling law and reset/physics protocol produce a confirmed accepted result. The measured estimate is the number of confirmed accepted trials divided by 100. The overall 29.33% is the equal-weight average of the six cell estimates. It is not a percentage of all distinct resting arrangements, and it need not equal a household loading success rate with different object/rack frequencies.

| Object | Rack | Confirmed accepted / raw attempts | 95% Wilson interval | Strict final placements | Unresolved |
| --- | --- | --- | --- | --- | --- |
| Dinner plate | Lower | 15 / 100 = 15% | 9.31–23.28% | 14 | 0 |
| Dinner plate | Upper | 10 / 100 = 10% | 5.52–17.44% | 10 | 0 |
| Bowl | Lower | 44 / 100 = 44% | 34.67–53.77% | 42 | 1 |
| Bowl | Upper | 32 / 100 = 32% | 23.67–41.66% | 32 | 1 |
| Mug | Lower | 46 / 100 = 46% | 36.56–55.74% | 41 | 0 |
| Mug | Upper | 29 / 100 = 29% | 21.01–38.54% | 29 | 0 |

![Confirmed acceptance probability by object and rack, with marginal 95% Wilson intervals](../../outputs/sampling_technical_report/acceptance_by_cell.png)

The intervals are newly calculated from the saved counts, using the [Wilson method described by NIST](https://www.itl.nist.gov/div898/handbook/prc/section2/prc241.htm). For `p̂ = a/n` and `z = 1.9599639845`, the interval is:

```text
[p̂ + z²/(2n) ± z √(p̂(1 − p̂)/n + z²/(4n²))] / [1 + z²/n]
```

These are approximate marginal 95% intervals for **confirmed acceptance under this protocol**, assuming independent proposals and sufficiently reproducible resets. They are not simultaneous 95% guarantees for all six cells, bounds on physical-model error, or certificates that unresolved proposals are impossible. The single seed and simulator/contact approximations remain limitations.

For the equal-weight overall estimate, the strata can have different success probabilities. An approximate stratified variance estimate is `Σ_h (1/6)² p̂_h(1 − p̂_h)/(100 − 1)`. It gives a standard error of **1.79 percentage points** and an approximate normal 95% interval of **25.83–32.83%** for confirmed acceptance. Treating all 600 trials as identically distributed Bernoulli trials would ignore the fixed six-cell allocation. These derived values and assumptions are preserved in [statistics.json](../../outputs/sampling_technical_report/statistics.json).

There were 395 initial collisions, six closure failures, six losses of support, 11 final containment failures, four settling timeouts and two numerical errors. Of the 205 proposals that entered physical dish simulation, 176 were accepted: 85.85% conditional on passing the initial geometry check. This conditional rate has a different denominator and does not replace 176/600.

Both unresolved cases were bowls, and neither reached rack retraction because settled contact penetration was invalid. If both eventually passed under a valid resolution, this completed sample's accepted count could rise at most to 178/600 = 29.67%. The corresponding strict-containment ceiling is 170/600 = 28.33%. These are completion ceilings for the two unknown records, not new observed successes or confidence limits. Excluding the unknowns and silently using 598 as the main denominator would hide part of the protocol's outcome distribution.

**Permutations and combinations describe different discrete problems.** The distinction is about what is being selected or assigned, not whether a dish is rotated.

| Problem | Mathematical operation | Relation to this experiment |
| --- | --- | --- |
| Test each object kind on each rack | Cartesian product: 3 × 2 | All six categorical pairings were tested. |
| Draw one position and orientation | One joint sample from a continuous product space | Performed 100 times per cell. |
| Test every member of finite position and orientation lists | Cartesian product: `N_position × N_orientation` | Not performed. |
| Put `k` distinguishable objects into `M` distinct slots, no shared slots | Ordered assignment: `M! / (M − k)!` | A different, discrete multi-object experiment. |
| Occupy `k` of `M` slots with indistinguishable objects | Combination: `M! / [k!(M − k)!]` | Also not performed. |
| Change the loading order of `k` distinct objects | `k!` orders before feasibility checks | Not tested; loading sequence is distinct from final arrangement. |

For example, 100 candidate positions and 100 candidate orientations would produce **10,000 pairs per cell** if every position were paired with every orientation. This run instead generated **100 jointly drawn position/orientation pairs per cell**. It did not generate two reusable lists and cross every element. “Some permutations and some combinations” is therefore not the right description of these 600 trials.

If a future finite search used 10 × 10 × 10 positions and 72 specified orientations, it would contain `6 × 1,000 × 72 = 432,000` single-object candidates before physics checks. Enumerating them would be exhaustive only over that declared discrete set. Valid poses between grid points could still be missed. The orientation set must itself be defined; a rectangular grid of Euler angles is not a uniform orientation grid.

For several objects at once, the continuous configuration space grows to six pose dimensions per rigid object, with object-object collision and support constraints. Individually accepted single-dish poses cannot simply be combined: dishes can overlap or alter each other's behavior. Neither multiplying the six accepted counts nor taking “176 choose k” establishes a valid capacity.

**The sampling strategy should be chosen from the quantity you want to measure.** The present design is a useful baseline for random-drop acceptance and comparisons between object/rack cases. Equal counts prevent rare-looking or difficult categories from receiving little effort, and Haar-uniform orientations avoid privileging upright placement. The geometry-based center definition keeps actor-origin conventions from arbitrarily shifting the sampled volume. Those benefits do not make the sampler efficient at discovering every kind of stable placement.

| Intended objective | Recommended design for a subsequent experiment | What must be reported |
| --- | --- | --- |
| Estimate success under unconstrained random drops | Retain fixed-domain, stratified independent sampling; increase the count according to precision. | All raw attempts, sampling law, per-cell rates and uncertainty. |
| Build a diverse catalogue of acceptable placements | Cover positions and orientations systematically, then validate and cluster final poses. A correctly transformed scrambled Sobol design is one option. | Coverage at declared resolution, distinct-pose rule, proposal count and acceptance outcomes. |
| Find many accepted examples quickly | Use geometry/support-guided proposals or adapt sampling near successful regions. Keep a separate unchanged random baseline. | Changed sampling distribution; guided-sample success is a different estimand. |
| Test all poses in a finite specification | Choose position spacing and an explicit finite orientation set, then evaluate their Cartesian product. | Exact grid/set, count, and completeness limited to those candidates. |
| Find maximum simultaneous loading | Search joint multi-object configurations and simulate their interactions and required loading sequence. | Object identities, joint constraints, capacity objective and search limits. |

For a Sobol-based design, map three coordinates to the center box and three to a rotation-preserving quaternion construction. Do not map three uniform coordinates to uniform Euler angles and call the result Haar-uniform. Use complete power-of-two sample sets per independent scramble, such as 256 or 512 candidates per cell, to preserve the sequence's balance properties; the [SciPy Sobol documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.qmc.Sobol.html) explains this requirement. This is a proposed alternative, not the sampler that generated the reported 600 trials. Contact discontinuities mean improved physical coverage or acceptance is not guaranteed by low discrepancy alone, and IID binomial intervals should not be transferred unchanged to correlated quasi-random points.

Changing the proposal box according to orientation, restricting poses near supports, rejecting every initial overhang, or keeping only successful proposals changes the population being explored. For this run some initially protruding poses can settle inside; excluding them is not merely a harmless speed optimization. Likewise, increasing the percentage accepted by choosing easier proposals does not demonstrate that a greater fraction of the original pose space is feasible.

**Choose the sample count from precision or discovery requirements, not from a claim of exhaustive coverage.** The repository uses 100 per cell as a configurable default in a time-limited exploratory experiment. There is no recorded power or precision calculation establishing 100 as sufficient. At `n = 100`, the worst-case normal approximation to a marginal 95% half-width is about 9.8 percentage points. This supports a coarse initial comparison, not precise ranking of every close pair.

For independent-proposal acceptance estimation, a conservative approximate planning formula is `n ≈ z² / (4e²)`, using worst-case probability 0.5 and a desired absolute half-width `e`:

| Desired approximate marginal 95% precision | Planned raw proposals per cell | Six-cell total |
| --- | --- | --- |
| ±5 percentage points | 385 | 2,310 |
| ±3 percentage points | 1,068 | 6,408 |

These are planning approximations, not exact guarantees or six-cell simultaneous coverage. For a new confirmatory run, predeclare counts and seeds. Choose smaller uncertainty only if its value justifies the additional simulation cost. The 74.8-minute cumulative wall time for this run includes initialization, baseline checks and an interrupted attempt; runtime is outcome-dependent, so scaling it by sample count is only a rough forecast.

A discovery objective has a different calculation. If a target region has proposal probability at least `p_min`, independent sampling needs `n ≥ log(0.05) / log(1 − p_min)` to have at least 95% probability of visiting it once. For a region with 1% proposal mass this gives 299 samples. It does not ensure discovery of every region, and it cannot guarantee visiting an isolated zero-measure feasible pose. The region's probability must refer to the actual chosen sampling distribution.

**For the original goal of showing diverse feasible positions and rotations, the next step should be a coverage catalogue with explicit pose equivalence.** Keep this run as the random-drop baseline. For a separately identified coverage run, distribute proposals across the recorded position domain and the rotation space, validate each trajectory, and cluster accepted final poses in the measured rack frame. Preserve an unconstrained component of the search if support-guided proposals are added, so unusual placements are not deliberately excluded.

A candidate equivalence rule can compare rack-relative translation distance and quaternion angular distance:

```text
position distance = ||t1 − t2||₂
rotation distance = 2 arccos(min(1, |q1 · q2|))   [normalized quaternions]
```

For example, 5 mm and 5° could be an initial reporting resolution, but these would be newly chosen catalogue tolerances, not the experiment's closure or rest thresholds. They must be justified by the downstream task and examined at several resolutions. Specify the clustering method as well: a representative-radius rule and connected-component clustering can produce different counts. Reduce orientations by object symmetry only where the modeled geometry, collision shapes and relevant appearance actually support that symmetry; a mug handle cannot be ignored.

No such clustering or symmetry reduction was performed in this run. Different releases may converge to almost the same final state, so the present defensible statement is **176 accepted trials, of which 168 have strictly contained final poses**. A claim such as “176 different feasible arrangements” would require additional analysis and a declared definition of “different.”

If the intended question is instead whether an exact specified pose remains unchanged while the rack closes, the protocol must add an allowed pose-change limit relative to that target. Current rest gates measure motion within the final rest window, not displacement from the initial random sample. Such a study would answer a different question and should be reported separately.

The [interactive viewer](../../outputs/random_pose_viewer/index.html) exposes starting, settling and final scenes with their exact saved coordinates. The [method and reproduction documentation](random_pose_experiment.md), [sampler source](../src/dishsim_frigidaire/random_poses.py), and [runtime source](../src/dishsim_frigidaire/random_pose_runtime.py) make the design reviewable. This technical report and its statistical calculations are derived from the completed records; no new physical trials were run to produce them.
