# Literature Search Report: feasibility-aware packing and rearrangement for a fixed-base arm (2026-11-05)

The final set has 24 sources (2007 to 2026, one preprint). Each one was checked against a DOI record or an arXiv listing. The closest precedents for the planned pack, gate, forbid, re-pack loop are four papers. Dantam et al. (2018) set the cut strength from the motion planner's collision set. Quintero-Peña et al. (2023) shrink each cut to an irreducible infeasible subset. Wang et al. (2022) read order rules from arm overlap at fixed grasp poses. Iori et al. (2007) add lazy loading-feasibility cuts inside an exact solver. No source packs a goal load with an exact solver and re-packs it after an arm-and-hand check fails, and none of the three dishwasher or rack-loading sources models arm feasibility.

Configuration (confirmed 2026-11-05, 05:05 EST): reachability- and feasibility-constrained packing and rearrangement for a fixed-base manipulator. Focus 1 compares lazy constraint generation, cheap pre-filters and learned feasibility classifiers. Focus 2 covers task and motion planning (TAMP) for packing, loading and shelf or bin placement where the arm's access depends on order and on neighbouring objects. Focus 3 covers dishwasher or rack loading with robots. Evidence profile cs_ml (peer-reviewed venues plus peer-reviewed-equivalent preprints, tagged). Years 2012 to 2026 plus foundational works. APA 7, English.

The setting served: a UR5e with a side-mounted Robotiq 2F-85 places plates, bowls and cups into a simulated dishwasher rack. The goal load is an exact CP-SAT packing of discrete candidate poses with pairwise collision and insertion-cycle conflicts. A Kit-free robot gate judges each dish in build order, with the earlier dishes present, on pick, carry and a direct release with the open hand 2 mm clear. Three of 15 goal poses fail the gate, and the plan is a lazy loop: pack, gate, forbid the failing poses (a blocked pair becomes an order conflict), re-pack.

## Search Strategy

### Databases and access

| Source | Access path | Used for | Hit counts |
|---|---|---|---|
| Semantic Scholar Graph API | `/paper/search/bulk` (Boolean), `/references`, `/recommendations` | Layers 1, 2 and 4 | recorded; two strings failed with HTTP 429 |
| arXiv API | `export.arxiv.org/api/query` | Layer 1 | recorded; one string failed with HTTP 429 |
| OpenAlex API | `api.openalex.org/works` | DOI verification, Layer 3 cited-by | recorded |
| Crossref API | `api.crossref.org/works/<doi>` | DOI verification | not applicable |
| Web search engine | 26 queries | topical recall and known-item checks | links only |
| Publisher, repository and lab pages | SAGE, INFORMS (via RePEc), AAAI OJS, PMLR, MIT DSpace, Cambridge Apollo, Potsdam, NSF PAR, UniMoRe IRIS, lab sites, ar5iv, arXiv HTML | full text and abstracts | not applicable |

IEEE Xplore, ACM DL, Scopus and Google Scholar were not queried directly. Their search interfaces need API keys or refuse automated clients, so no hit counts exist for them. Records from IEEE, ACM and Elsevier venues were reached through OpenAlex and Crossref DOI records and the web search engine.

### Strings, date range and filters

Date range 2012 to 2026. Foundational works before 2012 were admitted when they originate a method this report transfers (Hooker, 2007; Iori et al., 2007). Last searched 2026-11-05. Filters: English; Semantic Scholar fields of study Computer Science and Engineering on S2a'; peer-reviewed venues plus arXiv preprints tagged [preprint].

```
S2a  Semantic Scholar /paper/search/bulk, year=2012-2026
     (packing | "bin packing" | "object placement" | rearrangement)
     + (reachability | "kinematic feasibility" | "grasp feasibility" | "placement feasibility")
     + (manipulator | "robot arm")
S2a' S2a with fieldsOfStudy=Computer Science,Engineering
S2b  ("task and motion planning" | "manipulation planning")
     + (lazy | incremental | "constraint generation" | "failure explanation" | Benders | "no-good")
     year=2012-2026, fieldsOfStudy=Computer Science,Engineering
S2c  (dishwasher | "dish rack" | dishrack) + (robot | manipulator | grasp | loading), year=2012-2026
AX1  arXiv: (abs:packing OR abs:rearrangement OR abs:"object placement")
     AND (abs:reachability OR abs:"kinematic feasibility" OR abs:"grasp feasibility" OR abs:"placement feasibility")
     AND (abs:manipulator OR abs:"robot arm" OR abs:robot)
AX2  arXiv, relaxed: (all:packing OR all:rearrangement OR all:placement)
     AND (all:reachability OR all:feasibility) AND (all:manipulator OR all:"robot arm")
AX3  arXiv: (all:dishwasher OR all:"dish rack") AND (all:robot OR all:manipulator OR all:grasp)
W    26 web queries: lazy constraint generation and logic-based Benders in TAMP; robotic packing with
     MIP or CP and lazy motion checks; learned feasibility for TAMP; culprit detection; dishwasher and
     dish-rack loading with robots; known-item title queries for candidates found in other layers
L3a  OpenAlex: filter=cites:W2141841102,publication_year:2020-2026,
     title_and_abstract.search:packing|dishwasher|shelf|placement
L3b  OpenAlex: filter=cites:W2962872943,publication_year:2020-2026
L4   Semantic Scholar /recommendations/v1/papers/forpaper/arXiv:2203.10379
```

### The four layers with hit counts

| Layer | String or seed | Records | Outcome |
|---|---|---|---|
| 1 Boolean | S2a | 80 | stemming matched chemistry and physics papers; field filter added |
| 1 Boolean | S2a' | 49 | 15 robotics planning records kept for abstract screening |
| 1 Boolean | S2b, S2c | not retrieved | HTTP 429 on both; their concepts were covered by W |
| 1 Boolean | AX1 | 9 | under 20, so relaxed to AX2 |
| 1 Boolean | AX2 | 80 | inside the 20 to 500 band; 20 kept for abstract screening |
| 1 Boolean | AX3 | not retrieved | HTTP 429; covered by W |
| 1 Boolean | W | about 50 scholarly records | most core sources first surfaced here as known items |
| 2 Backward | reference list of Wang et al. (2022) | 26 | foundational lines confirmed (Stilman et al., 2007; lazy PRM; Krontiris and Bekris, 2016) |
| 2 Backward | reference sections of 6 core full texts, 15 foundational patterns | not applicable | co-citation counts below |
| 3 Forward | L3a, cited-by of Srivastava et al. (2014), 453 citing works | 19 | added Quintero-Peña et al. (2023) |
| 3 Forward | L3b, cited-by of Han et al. (2018), 62 citing works | 56 | no addition; 2 records deferred |
| 4 Semantic | L4 | 30 | no addition (mostly 2026 navigation and path-finding preprints) |

Layer 2 co-citation was counted by pattern search over the reference sections of six core full texts: Srivastava et al. (2014), Dantam et al. (2018), Lagriffoul and Andres (2016), Wells et al. (2019), Driess et al. (2020), and Lozano-Pérez and Kaelbling (2014). Lines cited by four or more of the six count as foundational.

| Foundational line | Core papers citing it (of 6) |
|---|---|
| Srivastava et al. (2014), interface layer | 5 of the 5 other papers |
| Kaelbling and Lozano-Pérez (2011), hierarchical planning in the now | 5 |
| Lagriffoul et al. (2012, 2014), geometric constraints and backtracking | 5 |
| Cambon et al. (2009), aSyMov | 5 |
| Erdem et al. (2011), causal reasoning with feasibility checks | 4 |
| Stilman et al. (2007), manipulation among movable obstacles | 4 |
| Hauser, multi-modal planning and minimum constraint removal | 4 |

The Erdem (2011) line led to Erdem et al. (2016), the journal analysis included here. The Lagriffoul line led to Lagriffoul and Andres (2016).

### Stopping rule and coverage audit

The search stopped when every required concept had support or a disclosed gap and the last round added nothing. Layer 3 added one source and Layer 4 added none.

| Required concept | Included support | Status |
|---|---|---|
| Lazy constraint generation in packing or TAMP | Dantam 2018; Srivastava 2014; Lagriffoul and Andres 2016; Quintero-Peña 2023; Iori 2007; Côté 2014; Hooker 2007; Wang 2022 | covered; packing with an arm in the loop is a gap |
| Feasibility pre-filters and reachability maps | Erdem 2016; Lozano-Pérez and Kaelbling 2014; Makhal and Goins 2018; Wang and Hauser 2019; Haustein 2019; Qin 2026; Shome 2019 | covered; one peer-reviewed reachability-map source |
| Learned feasibility classifiers | Wells 2019; Driess 2020; Yang 2023 | covered |
| Access order and neighbour blocking | Han 2018; Wang 2022; Srivastava 2014; Hartmann 2023; Tian 2024; Iori 2007; Côté 2014 | covered |
| Dishwasher or rack loading | Jiang 2012; Voysey 2021; Jain 2023 | thin; none models arm feasibility (gap disclosed) |

## Coverage Distribution Advisory

| Years | Sources | Share |
|---|---|---|
| 2007 to 2011 (foundational) | 2 | 8% |
| 2012 to 2016 | 6 | 25% |
| 2017 to 2021 | 9 | 38% |
| 2022 to 2026 | 7 | 29% |

| Method family | Sources | Share |
|---|---|---|
| Exact combinatorial solver in the loop (SMT, CSP, ASP, ILP, MILP with CP) | 9 | 38% |
| Search and sampling planners, reachability maps | 8 | 33% |
| Learned models | 6 | 25% |
| Penalty optimization | 1 | 4% |

```
DISTRIBUTIONAL_SKEW_ADVISORY
dimension: venue tier
value:     top robotics and planning venues (ICRA, IROS, RSS, CoRL, ICAPS, IJRR, T-RO, RA-L)
share:     17 of 24 (71%)
signal:    the evidence follows what these venues publish: planners judged in simulation with short
           hardware demonstrations. Industrial loading practice enters only through three OR journals
           (Operations Research twice, Transportation Science) and one engineering journal.
           This is a signal for reading the numbers, not a defect of the set.
```

Geography is near the threshold. First authors at US institutions hold 16 of 24 sources (67%), Europe 6, Turkey 1, Canada 1. Only 3 of 24 sources concern dishwasher or rack loading, and 1 of 24 is a preprint.

## Screening Results

| Stage | Records |
|---|---|
| Identified across the four layers | about 320 (138 from S2a', AX1 and AX2; about 50 from W; 26, 75 and 30 from Layers 2, 3 and 4) |
| After removing duplicates | about 285 |
| Excluded at title and abstract (off domain, non-manipulation, no feasibility element) | about 243 |
| Passed title and abstract (Include or Maybe) | 42 |
| Maybe records deferred without full-text assessment (time cap) | 8 |
| Assessed in full text or detailed abstract | 34 |
| Excluded at that stage | 10 |
| Final included | 24 (23 peer reviewed, 1 preprint) |

Exclusions after assessment:

| Record | Reason |
|---|---|
| Garrett, Lozano-Pérez and Kaelbling, PDDLStream (arXiv 1802.08705) | results appear only in figures; its retry rule for failed samplers repeats the retraction rule of Dantam et al. (2018) |
| WARA Robotics mobile manipulation challenge report (arXiv 2505.06919) | no venue; no quantitative dishwasher-loading results |
| RM4D reachability map (arXiv 2410.06968) | preprint under review; abstract gives no figures; Makhal and Goins (2018) kept as the peer-reviewed map |
| Ortiz-Haro et al., constraint graph sampling (arXiv 2011.04828) | learns a sampling order; no conflict cuts found in the abstract |
| Lazy-DaSH (arXiv 2504.05552) | multi-robot hypergraph TAMP preprint; overlaps Wang et al. (2022) |
| Iwao et al., packing with multiple grippers (JSME Robomec 2021) | domestic proceedings; full text restricted |
| Bipedal STL TAMP with Benders decomposition (arXiv 2508.13407) | navigation domain; noted under gaps |
| Lazy probabilistic roadmaps revisited (arXiv 2209.14471) | path planning only |
| "The Robotic Busboy" (catalog record only) | authors, year and venue not confirmed; dropped under the no-fabrication rule |
| Partial-order execution interface for online 3D bin packing and downstream TAMP (DOI 10.1109/ascc70886.2026.11701811) | title record only; no abstract reachable |

Deferred Maybe records, listed for a follow-up check:

| Record | Identifier |
|---|---|
| Semantic- and density-aware planning for accessibility-preserving multi-object placement | arXiv 2608.16741 |
| Policy-guided lazy search with feedback for TAMP | 10.1109/ICRA48891.2023.10161109 |
| A sampling and learning framework to prove motion planning infeasibility | 10.1177/02783649231154674 |
| Hierarchical bin packing with dual manipulators | arXiv 2506.01628 |
| Optimal order pick-and-place of objects in a cluttered scene | 10.1109/LRA.2021.3093021 |
| Dynamic buffers for tabletop rearrangement with stacking | arXiv 2509.22828 |
| Placement-position search for packing with a robot manipulator | 10.1109/SMC.2016.7844249 |
| Lazy evaluation of multi-grasp goals guided by motion planning | 10.1109/ACCESS.2026.3739509 |

## Annotated Bibliography

Each entry gives the APA 7 reference, how existence was verified, and an annotation.

### A. Lazy constraint generation and exact decomposition

#### 1. Srivastava et al. (2014)
Srivastava, S., Fang, E., Riano, L., Chitnis, R., Russell, S., & Abbeel, P. (2014). Combined task and motion planning through an extensible planner-independent interface layer. In *2014 IEEE International Conference on Robotics and Automation (ICRA)* (pp. 639-646). IEEE. https://doi.org/10.1109/ICRA.2014.6906922

Verification: DOI records in OpenAlex and Semantic Scholar; full text read (author PDF, aima.eecs.berkeley.edu).

A PR2 picks a target from clutter or lays out a dinner table, with off-the-shelf task and motion planners used as black boxes. An interface layer tries candidate values for pose references. When no candidate yields a collision-free path, it removes the movable objects, plans again, reads the colliding objects off that trajectory, and adds facts such as "b2 obstructs the trajectory to the grasp pose of b1" before replanning. Within 600 s it solved 100% of Clutter-15 problems (mean 32 s), 63% of Clutter-40 (mean 68 s) and 100% of Dinner-6 (mean 209 s), over 100 random problems per clutter size. This is the "blocked pair becomes an order conflict" move, with the blocker read from a relaxed motion plan. The relaxation can name a blocker that is not the minimal cause.

#### 2. Dantam et al. (2018)
Dantam, N. T., Kingston, Z. K., Chaudhuri, S., & Kavraki, L. E. (2018). An incremental constraint-based framework for task and motion planning. *The International Journal of Robotics Research*, *37*(10), 1134-1151. https://doi.org/10.1177/0278364918761570

Verification: OpenAlex DOI record; full text read (Kavraki lab PDF).

An incremental SMT task planner (Z3) proposes plans, and a sampling-based motion planner refines them. A failed refinement pushes one of three constraints onto the solver stack: forbid that exact plan, forbid the operator from that state at every step, or forbid the operator while the objects in the motion planner's collision set stay put. Failure constraints are popped when the step and sampling horizons deepen, because a timeout never proves infeasibility. Task planning for tens of objects took about 1 s and a 10-action plan about 10 s, against 275 s average task planning for the He et al. (2015) baseline in the scalability tests. The collision constraint made motion-planning time grow sub-exponentially in the number of blocking objects (tested up to 10). The method ran on a Baxter. The three strengths map onto three cut strengths for the packer. The domains are blocks and shelves with small location sets and no packing objective.

#### 3. Lagriffoul and Andres (2016)
Lagriffoul, F., & Andres, B. (2016). Combining task and motion planning: A culprit detection problem. *The International Journal of Robotics Research*, *35*(8), 890-927. https://doi.org/10.1177/0278364915619022

Verification: OpenAlex DOI record; postprint full text read (Potsdam repository).

When a symbolic plan has no geometric solution, the planner must find the culprit actions to backtrack to. A three-layer geometric reasoner (spatial relations checked by linear programming, unavoidable-collision tests, then full geometric backtracking) returns integrity constraints to the answer set programming task planner. The explanations leave out unrelated objects and the order of unrelated actions. Without culprit detection no instance of scenario 1 was solved, not even with three blocks; with it, 91.7% of runs were solved in one reported scenario. The cheap layers took 0.07 s to 7 s against 45 s to 1 min for geometric backtracking on a 30-action plan. Finding the minimal culprit set is an IIS set-cover problem, NP-hard in general. The geometric layers rest on bounding boxes and a line-network assumption.

#### 4. Quintero-Peña et al. (2023)
Quintero-Peña, C., Kingston, Z., Pan, T., Shome, R., Kyrillidis, A., & Kavraki, L. E. (2023). Optimal grasps and placements for task and motion planning in clutter. In *2023 IEEE International Conference on Robotics and Automation (ICRA)* (pp. 3707-3713). IEEE. https://doi.org/10.1109/ICRA48891.2023.10161455

Verification: DOI and pages from the IEEE proceedings entry and the Kavraki lab page (web search); accepted manuscript full text read (NSF PAR).

Z3 proposes plan skeletons, and a Gurobi optimization grounds every grasp and placement of a skeleton at once in dense planar clutter. When the optimization is infeasible, the method computes an irreducible inconsistent subsystem (IIS) and turns the conflicting constraints into symbolic expressions that block those actions and groundings. The runs used 50 trials per problem and a 1,000 s timeout. On the 14-action problem OP-7 the optimizer without IIS feedback solved 4% (mean 301.80 s) and with IIS feedback 100% (mean 19.98 s). Sampling baselines solved 0% from OP-6 on, and the IIS variant still solved 98% at 16 actions. This is the nearest robotics case of an exact solver fed minimal infeasibility explanations as cuts. Objects and hand are conservative planar projections, and arm motion is checked afterwards by RRT-Connect.

#### 5. Erdem et al. (2016)
Erdem, E., Patoğlu, V., & Schüller, P. (2016). A systematic analysis of levels of integration between high-level task planning and low-level feasibility checks. *AI Communications*, *29*(2), 319-349. https://doi.org/10.3233/AIC-150697

Verification: OpenAlex DOI record and Sabancı repository record. The journal full text is closed, so the numbers come from the arXiv version (arXiv 1307.7461, full text read via ar5iv), which may differ from the journal.

The paper compares four couplings of answer set planning with feasibility checks: precompute every check (Pre), check during search (Int), filter enumerated plans (Filt), and replan with the failing actions forbidden (Repl). In robotic manipulation (10 instances, 29,282 possible checks) the first plan took 888 s with Pre, 238 s of it precomputation. Int took 1,007 s with 467 checks, Filt 1,716 s, and Repl 2,007 s with only 139 checks and 2 timeouts. In legged locomotion the balance check has 10^10 instances, precomputation was infeasible, and Pre on the leg-reach check plus Int gave the fastest first plan (65 s). This supports precomputing whatever is enumerable and context free and checking the rest lazily. The domains are grids.

#### 6. Hooker (2007), foundational
Hooker, J. N. (2007). Planning and scheduling by logic-based Benders decomposition. *Operations Research*, *55*(3), 588-602. https://doi.org/10.1287/opre.1060.0371

Verification: OpenAlex DOI record; abstract read (EconPapers).

Tasks are assigned to facilities by an MILP master and scheduled by a CP subproblem; logic-based Benders decomposition returns a cut to the master whenever a subproblem is infeasible or worse than its bound. The abstract reports speedups of several orders of magnitude over the best MILP and CP methods for the cost and makespan objectives. The packer and gate loop has the same shape, with CP-SAT as master, the robot gate as subproblem and a forbidden pose or order as the cut. The subproblems here are exact CP schedules, while the robot gate is a sampling planner that cannot prove infeasibility.

#### 7. Iori et al. (2007), foundational
Iori, M., Salazar-González, J.-J., & Vigo, D. (2007). An exact approach for the vehicle routing problem with two-dimensional loading constraints. *Transportation Science*, *41*(2), 253-264. https://doi.org/10.1287/trsc.1060.0165

Verification: OpenAlex DOI record; abstract (EconPapers); full text read (UniMoRe repository).

Each vehicle route must carry its customers' rectangular items on a 2D loading surface and unload them in visit order without moving other items. Branch-and-cut handles routing, and an exact loading check (branch-and-bound, Check-2L) runs only when the LP solution is integer. An infeasible route gets an infeasible-path cut that forbids that customer sequence. Tested routes, feasible and infeasible, are kept in a pool so no loading instance is solved twice, and a check stopped at its time or backtrack limit leaves the final solution without an optimality proof. The method solved instances with up to 35 customers and more than 100 items to optimality under a one-day CPU limit. The loading check is order dependent, so its pool and its integer-only separation carry over to an insertion-order gate. There is no robot.

#### 8. Côté et al. (2014)
Côté, J.-F., Gendreau, M., & Potvin, J.-Y. (2014). An exact algorithm for the two-dimensional orthogonal packing problem with unloading constraints. *Operations Research*, *62*(5), 1126-1141. https://doi.org/10.1287/opre.2014.1307

Verification: OpenAlex DOI record; abstract read (IDEAS/RePEc). The full text was not read because the CIRRELT report PDF is encrypted.

The problem is 2D orthogonal packing with unloading constraints, the loading subproblem of routing-and-loading models. Branch-and-cut solves a one-dimensional relaxation; at each integer node a subproblem looks for a 2D layout without overlap that respects the unloading order, and cuts are added when it fails. Preprocessing shrinks the search space and detects infeasibility early. No figure was retrieved: the abstract reports a comparison with the best exact method on benchmark instances. It is the closest OR analogue of packing under an insertion-order rule with lazy feasibility cuts. The items are rectangles.

### B. Pre-filters, reachability and placement search

#### 9. Lozano-Pérez and Kaelbling (2014)
Lozano-Pérez, T., & Kaelbling, L. P. (2014). A constraint-based method for solving sequential manipulation planning problems. In *2014 IEEE/RSJ International Conference on Intelligent Robots and Systems* (pp. 3684-3691). IEEE. https://doi.org/10.1109/IROS.2014.6943079

Verification: DOI and pages from IEEE and MIT DSpace records (web search); full text read (CSAIL PDF).

A symbolic search proposes pick and place skeletons, and each skeleton becomes a discrete CSP over grasps, placements, robot configurations and paths. Configurations are six discrete values, free space is precomputed as configuration-space slices, pairwise Disjoint constraints keep placed objects apart, and an off-the-shelf CSP solver (Chen and Van Beek) does the search. In one reported problem the task search called the CSP 35 times, 34 times on unsatisfiable skeletons, and the whole search took 192 s. Pure backtracking runs were stopped at 500 s on instances the CSP solved in seconds. This is the closest robotics precedent for a discrete solver over candidate poses with pairwise conflicts. Precomputation grows with the discretization, and the robot is a simplified mobile manipulator.

#### 10. Makhal and Goins (2018)
Makhal, A., & Goins, A. K. (2018). Reuleaux: Robot base placement by reachability analysis. In *2018 Second IEEE International Conference on Robotic Computing (IRC)* (pp. 137-142). IEEE. https://doi.org/10.1109/IRC.2018.00028

Verification: OpenAlex record with DOI; full text read (arXiv 1710.01328 via ar5iv).

The paper precomputes a voxelized reachability map (0.08 m voxels, 1 m radius) from IK samples and inverts it to place a robot base. The UR5 map took 143.27 min to build from 763,600 poses. Base placement for 2 or 4 task poses took 1.45 s to 2.79 s. For a fixed-base UR5e this is a cheap, context-free filter on candidate release poses. The map ignores obstacles, so it cannot see the rack or earlier dishes, and its accuracy against IK is not reported.

#### 11. Wang and Hauser (2019)
Wang, F., & Hauser, K. (2019). Stable bin packing of non-convex 3D objects with a robot manipulator. In *2019 International Conference on Robotics and Automation (ICRA)* (pp. 8698-8704). IEEE. https://doi.org/10.1109/ICRA.2019.8794049

Verification: Crossref DOI record; full text read (arXiv 1812.04093 via ar5iv).

Irregular objects are packed one at a time. Candidates satisfy only non-overlap and containment, are ranked by heightmap minimization, and up to 100 top candidates are checked in order for stability and manipulation feasibility: a top-down path per grasp, with IK, joint limits and collision checked at every pose. If none passes, a fallback retries perturbed roll and pitch or changes the packing sequence. Over 1,000 ten-item sets, planning succeeded for 97.1% with all constraints (99.9% with non-overlap only) and simulated execution for 79.1% against 17.11% without the constraints. The fallback served 3.2% of items, and changing the sequence fixed half of those. A 10-item plan took 34.9 s on average. The packer is greedy, with no global re-pack.

#### 12. Haustein et al. (2019)
Haustein, J. A., Hang, K., Stork, J. A., & Kragic, D. (2019). Object placement planning and optimization for robot manipulators. In *2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)* (pp. 7417-7424). IEEE. https://doi.org/10.1109/IROS40897.2019.8967732

Verification: OpenAlex DOI record; full text read (arXiv 1907.02555 via ar5iv).

The robot must place a held object in clutter at a pose that is collision free, stable, reachable and good under a user objective such as tight packing. An anytime hierarchical search over placement regions uses Monte Carlo tree search; each candidate passes stability and collision checks, then IK, then a bidirectional RRT, and the IK and collision outcomes feed the tree's rewards. Initial solutions arrived within a few seconds in all 24 scene, object and objective combinations; solution quality is shown only in plots. It shows reachability folded into placement search, with check outcomes steering where the search samples next. It places one object at a time.

#### 13. Qin et al. (2026) [preprint]
Qin, T., McCann, C., Calli, B., & Xiao, J. (2026). *Gripper-aware automatic dense packing of irregular objects* (arXiv:2609.22062) [Preprint]. arXiv. https://doi.org/10.48550/arXiv.2609.22062

Verification: arXiv listing and HTML full text read; no venue stated.

A placement optimizer treats the held object and the gripper as one composite body of sphere trees, with gripper collision checked against placed objects and against the walls below the rim. Infeasibility enters as weighted penalties, and execution uses force-limited descent and a push after release. On a Franka Panda (5 trials per configuration) the composite model placed 35 of 35 objects at 95.59% density. Leaving the gripper out of the planner dropped that to 7 of 35 and 26.09%. The optimizer took 3.56 s per object against 0.42 s to 1.04 s for heightmap minimization. A hand-in-shape clearance model decides most of the outcome, which supports a hull filter on candidate poses before the solver runs. The sets have 7 or 8 objects, and one failed placement ends a run.

#### 14. Shome et al. (2019)
Shome, R., Tang, W. N., Song, C., Mitash, C., Kourtev, H., Yu, J., Boularias, A., & Bekris, K. E. (2019). Towards robust product packing with a minimalistic end-effector. In *2019 International Conference on Robotics and Automation (ICRA)* (pp. 9007-9013). IEEE. https://doi.org/10.1109/ICRA.2019.8793966

Verification: OpenAlex DOI record; full text read (arXiv 1903.00984 via ar5iv).

A suction cup that also pushes packs products into a bin toward a given grid layout. The arm lowers to a pre-push pose, shifts it in 5 mm steps until an expanded footprint (object plus margin) clears the placed objects, then pushes into place, with monitored corrections afterwards. In simulation, success rose from 7.2% with no footprint expansion to 100% once the margin reached 1.25 times the noise level. The ar5iv rendering duplicated some digits, so these values should be checked against the PDF before reuse. A clearance margin tied to measured error, applied to object and tool together, mirrors the 2 mm open-hand rule. The target layout is an input, and real-robot success rates appear only in plots.

### C. Learned feasibility

#### 15. Wells et al. (2019)
Wells, A. M., Dantam, N. T., Shrivastava, A., & Kavraki, L. E. (2019). Learning feasibility for task and motion planning in tabletop environments. *IEEE Robotics and Automation Letters*, *4*(2), 1255-1262. https://doi.org/10.1109/LRA.2019.2894861

Verification: OpenAlex DOI record; full text read (Kavraki lab PDF).

Most TAMP runtime in tabletop domains goes to motions that turn out infeasible. An RBF SVM (LIBSVM), trained on scenes of two rectangular prisms with a table-mounted UR5, predicts motion feasibility. Scenes with more objects are split into pairs, and the classifier orders the task-motion search. After a cutoff time the planner ignores it, which keeps probabilistic completeness, and the labels are biased toward false positives on purpose. The classifier was correct about 96% of the time on held-out two-prism scenes and gave order-of-magnitude runtime improvements. Its pairwise decomposition matches the packer's pairwise conflict model, and the false-positive bias leaves the exact gate as the final judge. Objects are approximated by inscribed prisms.

#### 16. Driess et al. (2020)
Driess, D., Oguz, O. S., Ha, J.-S., & Toussaint, M. (2020). Deep visual heuristics: Learning feasibility of mixed-integer programs for manipulation planning. In *2020 IEEE International Conference on Robotics and Automation (ICRA)*. IEEE. https://doi.org/10.1109/ICRA40945.2020.9197291

Verification: OpenAlex DOI record (no page numbers listed); full text read (TU Berlin PDF).

Most discrete choices in a mixed-integer manipulation program are infeasible, and each infeasible nonlinear program is slow to reject. A CNN over images rendered per target object predicts the feasibility of each of 24 grasp actions, orders the search, and can declare a scene infeasible below a threshold. Only 11.2% of 462,672 training actions were feasible. The network cut the programs solved per feasible scene by a factor of 4.8 against random order (1.1 on average). It detected infeasible scenes 48 times faster at a 0.3% false-infeasible rate. An infeasible program took up to 17 s against 0.5 s to 1.1 s for a feasible one, so ordering pays most on poses that will fail. A model trained on single-object scenes gave 70% false infeasibles once a second object appeared.

#### 17. Yang et al. (2023)
Yang, Z., Garrett, C. R., Lozano-Pérez, T., Kaelbling, L. P., & Fox, D. (2023). Sequence-based plan feasibility prediction for efficient task and motion planning. In *Robotics: Science and Systems XIX*. https://doi.org/10.15607/RSS.2023.XIX.061

Verification: OpenAlex DOI record; abstract read (arXiv 2211.01576).

PIGINet, a transformer over the plan, the goal and the initial state, predicts whether a candidate task plan will refine and reorders candidates for a TAMP planner in kitchen-like scenes. Runtime fell by 80% on problems with small state spaces and by 10% to 50% on larger ones, across seven problem families, with 150 to 600 training problems. Plan-level prediction is the learned counterpart of choosing which CP-SAT solution to gate first. This review read the abstract only.

### D. Access order and neighbour blocking

#### 18. Han et al. (2018)
Han, S. D., Stiffler, N. M., Krontiris, A., Bekris, K. E., & Yu, J. (2018). Complexity results and fast methods for optimal tabletop rearrangement with overhand grasps. *The International Journal of Robotics Research*, *37*(13-14), 1775-1795. https://doi.org/10.1177/0278364918780999

Verification: OpenAlex DOI record; full text read (arXiv 1711.07369 via ar5iv).

Tabletop rearrangement with overhand grasps is planned to minimize grasps and travel. A dependency graph gets an arc when one object's goal overlaps another's start, and the extra grasps needed equal a minimum feedback vertex set; exact ILPs and heuristics solve it, and a second ILP orders the moves. Exact ILPs found optimal feedback vertex sets for more than 30 objects within a few seconds. On a uArm with 12 objects the method used 16 actions and 299.95 s against 20 actions and 405.16 s for a greedy baseline. Each cycle in the blocking graph costs an extra grasp and a buffer, which is why the packer forbids insertion cycles. Blocking is modelled only between start and goal footprints, and the arm itself is not modelled.

#### 19. Wang et al. (2022)
Wang, R., Gao, K., Yu, J., & Bekris, K. E. (2022). Lazy rearrangement planning in confined spaces. *Proceedings of the International Conference on Automated Planning and Scheduling*, *32*(1), 385-393. https://doi.org/10.1609/icaps.v32i1.19824

Verification: OpenAlex DOI record; full text read (arXiv 2203.10379 via ar5iv).

Objects on a shelf are rearranged by an arm that reaches in from the side, where objects block each other. IK grasps (about 0.01 s each) are computed at starts and goals. Arm overlap with other objects at those static grasp poses becomes conditional rules such as "do not move o3 if o2 is at its goal and o1 is at its start." Forward checking prunes rule-breaking moves without motion planning, paths are verified only along a branch that reaches the goal, and a failed edge removes its subtree. On a Motoman SDA10F in PyBullet (80 trials per size), monotone 14-object problems reached 100% success in 16.8 s against 83.3% in 47.3 s for the best baseline. Non-monotone 16-object problems reached 82.4% against 25.8%. Path verification took 96% to 99% of planning time, and non-monotone plans averaged 1.8 extra actions. Some rules involve two other objects at once, so a pairwise conflict cannot express every blocker. The goal arrangement is fixed, and the rules hold only to the grasp sampling resolution.

#### 20. Hartmann et al. (2023)
Hartmann, V. N., Orthey, A., Driess, D., Oguz, O. S., & Toussaint, M. (2023). Long-horizon multi-robot rearrangement planning for construction assembly. *IEEE Transactions on Robotics*, *39*(1), 239-252. https://doi.org/10.1109/TRO.2022.3198020

Verification: OpenAlex record with DOI; full text read (arXiv 2106.02489 via ar5iv).

Several robots assemble structures part by part. A part becomes eligible once a connected part is placed. Keyframe optimization checks placement, kinematics and collisions; failed options are set aside and revisited at lower priority, and earlier placements are rewound when no robot can place the next part. The 113-part pavilion took a median of 1,597.7 s with one agent and 2,336.1 s with five, and no backtracking was needed in the experiments. Order chosen by a structure heuristic and repaired by deferral is a cheaper policy than re-packing, and it worked at this scale. The planners cannot prove infeasibility, and success rates are not reported.

#### 21. Tian et al. (2024)
Tian, Y., Willis, K. D. D., Al Omari, B., Luo, J., Ma, P., Li, Y., Javid, F., Gu, E., Jacob, J., Sueda, S., Li, H., Chitta, S., & Matusik, W. (2024). ASAP: Automated sequence planning for complex robotic assembly with physical feasibility. In *2024 IEEE International Conference on Robotics and Automation (ICRA)* (pp. 4380-4386). IEEE. https://doi.org/10.1109/ICRA57147.2024.10611595

Verification: OpenAlex record with DOI; full text read (arXiv 2309.16909v2 HTML).

ASAP searches a disassembly tree with a geometric or GNN part-selection policy. Each step needs a collision-free path and a stability check with up to M held parts. For robot execution it samples 100 antipodal grasps in 5 orientations, solves IK along the assembly path, and iterates until a collision-free, IK-feasible plan exists. On 240 test assemblies (3 to more than 50 parts) it reached 82.08% success with the GNN at the high budget and 4 held parts, against 56.67% for Assemble Them All. Median runtime was 43 s for 5 parts or fewer and 4,190 s above 30 parts. Feasibility of each step depends on the parts already present, as in the rack. The benchmark numbers exclude manipulator constraints; grasp and IK checks appear only in one qualitative demonstration.

### E. Dishwasher and rack loading

#### 22. Jiang et al. (2012)
Jiang, Y., Lim, M., Zheng, C., & Saxena, A. (2012). Learning to place new objects in a scene. *The International Journal of Robotics Research*, *31*(9), 1021-1043. https://doi.org/10.1177/0278364912438781

Verification: OpenAlex DOI record; full text read (arXiv 1202.1694 via ar5iv).

The robot decides where and how to place a new object, including plates, bowls and martini glasses in dish racks. Sampled candidate placements are scored by a max-margin SVM over 145 features of the object and the placing area; a multi-object variant solves an ILP over sampled cells and orientations. Over 10 robot tasks, 6 of them dish racks, placements with object models were 98% stable and 98% preferred on seen objects and areas, and 96% and 92% on new ones; without models the figures were 94% and 92%, and 82% and 72%. Sixteen plates went into one rack with fewer than 5% of 80 placements failing. The score has no arm or hand term, and several failures came from rack spikes hit after incomplete point clouds.

#### 23. Voysey et al. (2021)
Voysey, I., Thuruthel, T. G., & Iida, F. (2021). Autonomous dishwasher loading from cluttered trays using pre-trained deep neural networks. *Engineering Reports*, *3*(5), Article e12321. https://doi.org/10.1002/eng2.12321

Verification: Crossref DOI record and Cambridge repository record. The Wiley full text returned HTTP 403, so the numbers come from search-engine excerpts of the full text and carry moderate confidence.

A dishwasher is loaded from cluttered canteen trays without task-specific training. The system uses a pre-trained object detector on RGB images, a simple pinch gripper and 1D force feedback, in a one-shot and a closed-loop strategy. One-shot runs reached 92.0% planning success and cleared 58.7% of objects; closed-loop runs reached 84.0% planning success and cleared 81.1%. It is the only peer-reviewed robot dishwasher-loading study found, and its metric is tray clearing. How rack poses are chosen was not visible to this review.

#### 24. Jain et al. (2023)
Jain, V., Lin, Y., Undersander, E., Bisk, Y., & Rai, A. (2023). Transformers are adaptable task planners. In *Proceedings of The 6th Conference on Robot Learning* (Vol. 205, pp. 1011-1037). PMLR. https://proceedings.mlr.press/v205/jain23a.html (arXiv:2207.02442)

Verification: PMLR proceedings page; full text read (arXiv 2207.02442 via ar5iv).

A transformer task planner learns dishwasher-loading preferences from demonstrations and adapts to a new user from one demonstration. Rack placements are a precomputed list of discrete candidate poses per dish category, built by dropping dishes at random into the racks and recording where they settled. Packing efficiency was 0.62 on held-out sessions of seen preferences and 0.54 on unseen preferences prompted with one demonstration. On a Franka Panda with four dishes it succeeded in 100% and 75% of two scene variations. The rack representation matches the setting's settled candidate poses. Reachability is not modelled, and the main results are in simulation.

## Literature Matrix

● marks a main theme of the source and ○ a secondary one. Quality codes: J journal, C peer-reviewed conference, P preprint; F full text read, A abstract read, E search-engine excerpts of the full text.

| Source | Lazy constraint generation | Pre-filter or reachability map | Learned feasibility | TAMP for packing or loading | Access order and neighbour blocking | Dishwasher or rack loading | Method | Quality |
|---|---|---|---|---|---|---|---|---|
| Srivastava et al. 2014 | ● | | | ○ | ● | | interface layer, failure facts | C, F |
| Dantam et al. 2018 | ● | | | ○ | ● | | incremental SMT | J, F |
| Lagriffoul and Andres 2016 | ● | ○ | | | ○ | | ASP with culprit detection | J, F |
| Quintero-Peña et al. 2023 | ● | | | ● | ○ | | SMT with MIP and IIS cuts | C, F |
| Erdem et al. 2016 | ● | ● | | | | | ASP, four integration levels | J, F (arXiv version) |
| Hooker 2007 | ● | | | | | | logic-based Benders, MILP with CP | J, A |
| Iori et al. 2007 | ● | | | ● | ● | | branch-and-cut, exact loading check | J, F |
| Côté et al. 2014 | ● | | | ● | ● | | branch-and-cut, 2D subproblem | J, A |
| Lozano-Pérez and Kaelbling 2014 | ○ | ● | | ○ | | | discrete CSP | C, F |
| Makhal and Goins 2018 | | ● | | | | | reachability map | C, F |
| Wang and Hauser 2019 | ○ | ● | | ● | ○ | | ranked candidates, post-check | C, F |
| Haustein et al. 2019 | | ● | | ○ | | | MCTS with IK and RRT | C, F |
| Qin et al. 2026 | | ● | | ● | | | composite-body optimization | P, F |
| Shome et al. 2019 | | ● | | ● | ○ | | pushing with footprint margin | C, F |
| Wells et al. 2019 | | | ● | | | | SVM feasibility | J, F |
| Driess et al. 2020 | | | ● | | | | CNN feasibility for MIP | C, F |
| Yang et al. 2023 | | | ● | | | | transformer plan feasibility | C, A |
| Han et al. 2018 | | | | ○ | ● | | dependency graph, ILP | J, F |
| Wang et al. 2022 | ● | ○ | | ● | ● | | lazy search with order rules | C, F |
| Hartmann et al. 2023 | ○ | | | ● | ● | | keyframe optimization, deferral | J, F |
| Tian et al. 2024 | | | ○ | ● | ● | | tree search with stability check | C, F |
| Jiang et al. 2012 | | | ● | | | ● | learned placement scores | J, F |
| Voysey et al. 2021 | | | ○ | | | ● | pre-trained detector pipeline | J, E |
| Jain et al. 2023 | | | ● | | ○ | ● | transformer task planner | C, F |

## Synthesis

### Focus 1: lazy constraint generation, pre-filters and learned classifiers

| Approach | Representative | The number that matters |
|---|---|---|
| Lazy cuts from a failing check, strength set by the collision set | Dantam et al. (2018) | tens of objects task-planned in about 1 s; motion time sub-exponential in blocking objects (up to 10) with collision cuts |
| Lazy cuts shrunk to an IIS | Quintero-Peña et al. (2023) | 14-action problem: 100% solved (mean 19.98 s) with IIS cuts, 4% (mean 301.80 s) without |
| Exact decomposition with a pooled check (OR) | Iori et al. (2007) | optimal up to 35 customers and more than 100 items; checks only at integer solutions |
| Precompute every enumerable check | Erdem et al. (2016) | first plan 888 s with 29,282 precomputed checks; 2,007 s with replanning (139 checks, 2 timeouts); infeasible at 10^10 checks |
| Cheap checks over ranked candidates | Wang and Hauser (2019) | 97.1% planning success on 10-item sets; fallback for 3.2% of items |
| Hand in the placement shape | Qin et al. (2026) [preprint] | 35 of 35 placements with the gripper modelled, 7 of 35 without |
| Learned ordering of checks | Driess et al. (2020) | 4.8 times fewer programs solved; infeasibility found 48 times faster at 0.3% false infeasible |

Lazy cuts dominate when checks are expensive and most candidates pass. The size of each cut decides convergence: Lagriffoul and Andres (2016) solved nothing in scenario 1 without culprit detection, and Quintero-Peña et al. (2023) went from 4% to 100% by shrinking cuts. Precomputation wins whenever the check set is enumerable and does not depend on context (Erdem et al., 2016; Lozano-Pérez & Kaelbling, 2014). Learned classifiers in this set only order or prune the search; every one of them keeps an exact check as the final judge (Wells et al., 2019; Driess et al., 2020; Yang et al., 2023).

### Focus 2: TAMP for packing and loading where packing and access interact

| Approach | Representative | The number that matters |
|---|---|---|
| Order rules from arm overlap at grasp poses, lazy path checks | Wang et al. (2022) | 14 objects: 100% in 16.8 s against 83.3% in 47.3 s; 16 objects non-monotone: 82.4% against 25.8% |
| Blocking graph with cycles priced as buffers | Han et al. (2018) | optimal feedback vertex sets for more than 30 objects in seconds; 16 against 20 actions with 12 objects |
| Obstruction facts from collision sets | Srivastava et al. (2014) | 63% of 40-object clutter problems solved within 600 s |
| Order heuristic with deferral and rewind | Hartmann et al. (2023) | 113 parts in 1,597.7 s median with one agent; no backtracking needed |
| Sequence search with per-step feasibility | Tian et al. (2024) | 82.08% of 240 assemblies; manipulator constraints only in a demo |
| Loading order inside exact packing (OR) | Iori et al. (2007); Côté et al. (2014) | 35 customers and more than 100 items solved to optimality (Iori) |

Access order enters these planners as pairwise or conditional precedence between objects, read from where the arm or hand overlaps a neighbour. Wang et al. (2022) is the closest to the setting: rules come from static overlap at fixed grasp poses and motion planning runs only on branches that reach the goal, which spent 96% to 99% of planning time on verification. The robotics planners take the goal arrangement as given (Han et al., 2018; Wang et al., 2022) or build it greedily (Wang & Hauser, 2019; Hartmann et al., 2023). Only the OR papers choose the packing and its order jointly under an order-dependent feasibility check.

### Focus 3: dishwasher and rack loading

| Representative | What it models | The number that matters | Arm feasibility |
|---|---|---|---|
| Jiang et al. (2012) | learned placement scores in dish racks | 98% stable with object models on seen objects; 82% stable without models on new ones (all 10 tasks) | none in the score |
| Voysey et al. (2021) | tray clearing into a dishwasher | 81.1% of objects cleared in closed loop | not reported in what this review could read |
| Jain et al. (2023) | discrete settled rack poses and user preferences | packing efficiency 0.62; 100% and 75% on hardware | not modelled |

Rack loading work treats placement as a learned score or a slot choice. Jain et al. (2023) build their rack poses the way the setting does, by settling dropped dishes and storing the poses, and leave feasibility to the policy. No source reports whether a fixed-base arm can reach and release into a given rack pose.

### What transfers to a CP-SAT packer with lazy feasibility conflicts

1. Treat the loop as logic-based Benders. CP-SAT is the master, the robot gate is the subproblem, and each gate failure returns a cut (Hooker, 2007). Iori et al. (2007) run the expensive check only on integer solutions and pool every verdict, feasible or not. The analogue here is to gate only complete loads and cache verdicts keyed by dish, pose and the earlier neighbours inside the hand's reach volume.
2. Pick the cut from the collision set, weakest last (Dantam et al., 2018; Srivastava et al., 2014). A failure with no neighbour involved is a unary cut that forbids that pose for that dish in any order. A failure that names one earlier dish is the pairwise order conflict the packer already supports. A failure that names two or more dishes needs a clause over several order literals, since Wang et al. (2022) found rules that depend on two other objects at once. A no-good on the whole load is the fallback when the gate returns no collision set. Each added order conflict has to keep the build order acyclic, because the rack has no buffer and Han et al. (2018) price every cycle at an extra grasp.
3. Shrink each cut before adding it. Quintero-Peña et al. (2023) went from 4% to 100% solved with IIS-based cuts, and Lagriffoul and Andres (2016) solved nothing in scenario 1 without culprit detection. For the pairwise case a deletion filter is enough: re-run the gate with one named neighbour removed at a time and keep only the neighbours whose removal lets the pose pass. Exact minimal sets are NP-hard in general, so cap the filter at a fixed number of gate calls.
4. Precompute the context-free part and put the hand into it. Erdem et al. (2016) found precomputation fastest when the checks are enumerable (29,282 checks, 238 s). Candidate poses times dishes is enumerable here, so empty-rack IK and reach can be checked once per candidate (Lozano-Pérez & Kaelbling, 2014; Makhal & Goins, 2018). The same pass can build a pairwise table from the open-hand hull at each release pose, inflated by the 2 mm clearance, against every other candidate pose. Qin et al. (2026) placed 35 of 35 objects with the gripper modelled and 7 of 35 without, and Shome et al. (2019) report 7.2% success with no footprint margin and 100% once the margin covered 1.25 times the error.
5. Keep timeouts retractable and learned models advisory, and log the iterations. Dantam et al. (2018) pop failure constraints when horizons deepen, Wells et al. (2019) drop the classifier after a cutoff time, and Iori et al. (2007) report that a stopped check leaves optimality unproved. Cuts from a planner timeout should be marked retractable, separate from proved failures such as an IK miss or a hull collision. A classifier can set CP-SAT solution hints or objective penalties without adding hard constraints (Driess et al., 2020, kept false infeasibles at 0.3%). The only iteration data in this set are 35 CSP calls in one search (Lozano-Pérez & Kaelbling, 2014) and 139 against 467 checks (Erdem et al., 2016), so the run should log re-packs, cuts by type and gate time per iteration.

### Open gap

No source in this set chooses a packing with an exact solver and re-packs it after an arm-and-hand check fails. The robotics papers with lazy cuts plan task skeletons or rearrange toward a fixed goal (Dantam et al., 2018; Quintero-Peña et al., 2023; Wang et al., 2022). The OR papers that re-choose the packing under lazy order-dependent checks have no arm (Iori et al., 2007; Côté et al., 2014). Release with an open hand is not checked as its own test anywhere. Qin et al. (2026) model the gripper during descent with penalties, and Shome et al. (2019) use footprint margins.

## Identified Gaps

1. Goal re-selection after arm failures. No robotics source changes the goal packing when the arm cannot build it; Wang and Hauser (2019) fall back greedily (3.2% of items) and Wang et al. (2022) fix the goal. The OR precedent (Iori et al., 2007) re-chooses routes after loading failures with no arm model.
2. Release feasibility. No source tests the open-hand release clearance as a separate check or counts release failures. Qin et al. (2026) penalize gripper collision during descent, Shome et al. (2019) inflate footprints, and none of them reports placements lost at release between wires or tines.
3. Rack loading with arm feasibility. The three rack sources score placements (Jiang et al., 2012), clear trays (Voysey et al., 2021) or pick settled slots by preference (Jain et al., 2023). None reports which rack poses a fixed-base arm can reach and release into, or how often a learned rack placement is unreachable.
4. Iteration counts for lazy loops in packing. No robotics source reports how many re-packs a lazy loop needs to converge. The available data points are CSP call counts (35 calls, 34 unsatisfiable; Lozano-Pérez & Kaelbling, 2014) and check counts (139 against 467; Erdem et al., 2016).
5. Proofs of infeasibility for the gate. The sampling-based checks in this set cannot prove failure, so their cuts are retractable (Dantam et al., 2018; Hartmann et al., 2023). The deferred Layer 3 record on proving motion-planning infeasibility (DOI 10.1177/02783649231154674) was not assessed and is the lead for turning gate timeouts into permanent cuts.

## Recommended Sources by Plan Section (Goal, Decisions, Change, Run)

| Plan section | Sources | What each gives the plan |
|---|---|---|
| Goal | Hooker (2007); Iori et al. (2007); Wang et al. (2022) | frame the loop as logic-based Benders with an order-dependent check; state success as samples solved within a budget at a given roster size |
| Decisions | Dantam et al. (2018); Quintero-Peña et al. (2023); Lagriffoul and Andres (2016); Erdem et al. (2016); Wells et al. (2019) | cut strength from the collision set; minimal cuts by a capped deletion filter; precompute the enumerable part; timeouts retractable; classifiers as hints only |
| Change | Srivastava et al. (2014); Lozano-Pérez and Kaelbling (2014); Makhal and Goins (2018); Qin et al. (2026); Shome et al. (2019); Wang and Hauser (2019) | blocker read from the gate's collision set; precomputed reach per candidate pose; open-hand hull table with the 2 mm margin; ranked post-check with a sequence fallback |
| Run | Erdem et al. (2016); Wang et al. (2022); Quintero-Peña et al. (2023); Han et al. (2018); Jain et al. (2023) | report time to first feasible load, gate calls and timeouts; success by roster size with the verification share; success within a fixed timeout with and without minimal cuts; settled candidate poses as the rack representation |
