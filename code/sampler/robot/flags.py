"""Named run flags of the robot episode and its probes (Kit-free).

Plan ``plans/2026-09-29-easy-s0.md`` Section 3 and its "## 0. Decisions" (D1-D19). Every threshold or rule the
previous session changed to make the 3-bowl test pass is a named field here; call sites read the resolved
``Flags`` and never a literal, and every run records ``to_dict(flags)``.

Profiles:

``headline``
    Every benchmark relaxation OFF (R4 test case, R5 place-down, R6 racked-in judge); the benchmark's disturbance
    rule (D1), end check (D2) and per-move settle (D3); the rig's pre-R7 blocked-motion limits (D8); the drift hold
    gate (D9); the counter cap (D19); R3 kept only as a declared, scripted rack motion with both racks back at the
    benchmark's extension before scoring.
``legacy_upright3``
    The settings of the 2026-09-29 upright3 PASS frozen exactly (data/results/robot/episodes/robot_upright3.json), the
    regression test's "old settings".

Harness behaviour (sleep threshold, contact logging, invariants, media) is the same in every profile; the
legacy profile records invariant violations without aborting, so the old run can reproduce while the harness
verdict still reports them.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace

# The privileged simulator information the controller uses (plan Section 3: allowed for now, must be visible).
PRIVILEGED = (
    "dish poses read from the simulator (grasp candidates, park/goal targets, disturbance and at-goal judging)",
    "the in-hand dish pose measured from the simulator after every lift (placement uses it)",
    "gravity disabled on every robot link: idealised gravity compensation (plan decision D6)",
    "the analytic IK / FCL arm model is calibrated to the simulated links (collide.calibrate)",
)


@dataclass(frozen=True)
class Flags:
    profile: str
    # R4 (benchmark relaxation): the 3-upright-bowl test case replaces the instance; None = the instance itself
    test_case: str | None = None
    # R5 (benchmark relaxation): release at a goal. "goal_pose" = the goal's settled pose (episode place());
    #     "lower_until_contact" = the dish lowered as it hangs until contact (episode place_down())
    place_mode: str = "goal_pose"
    # R6 (benchmark relaxation): move judge. "at_goal" = the benchmark tolerance (frigidaire_bench.within_goal);
    #     "racked_in" = in the intended rack
    judge: str = "at_goal"
    # D1: "benchmark" = a displaced neighbour is fatal only if it was at its goal before the move or ends outside
    #     the counter and the racks, else a counted nudge; "any_neighbour" = any neighbour > 10 mm / 20 deg fails
    disturbance: str = "benchmark"
    # D2: "benchmark" = park the arm, re-extend the racks, final hold, then tub wall + retract both + containment;
    #     "racked_in" = the old episode end check (every dish in some rack)
    end_check: str = "benchmark"
    # D3: per-move settle that gates the verdict. "benchmark" = 150 ticks with the last 60 as drift window
    #     (5 mm / 3 deg) and the contact-depth limit; "legacy" = the old episode's ungated 120 ticks
    move_settle: str = "benchmark"
    final_hold_observation_s: float = 2.0        # D3: backend.hold() with LIMITS and a 2 s observation before scoring
    # R3 (procedure): the upper rack is pushed in while the lower rack is loaded (declared scripted rack motion)
    upper_in_for_lower: bool = True
    reextend_before_scoring: bool = True         # both racks back at the benchmark's extension before scoring
    # R1 / D9: the in-hand hold gate. "drift" = the dish touches only the gripper and drifts <= hold_drift_max_m over
    #     hold_window_s after the in-hand settle; "legacy_rise" = rose > 5 cm and within 20 cm of the TCP
    hold_gate: str = "drift"
    in_hand_settle_ticks: int = 90
    hold_window_s: float = 2.0
    hold_drift_max_m: float = .005
    legacy_rise_min_m: float = .05
    legacy_near_tcp_m: float = .20
    # R2: finger_joint drive (Phase 0 keeps the measured 8 N m / 400; Phase 1 calibrates within the 2F-85's limits)
    gripper_effort: float = 8.
    gripper_stiffness: float = 400.
    gripper_damping: float = 1.                  # plans/2026-11-04-grip-force.md: finger_joint damping and the inner finger
    finger_effort: float = 10.                   #     joints' drive (ur5e.robot_cfg gripper_finger), the measured sweep's knobs
    finger_stiffness: float = 10.
    finger_damping: float = .05
    grip_family: str | None = None               # R2 per hold (plans/2026-11-04-fifteen-of-fifteen.md): the grasp family whose
                                                 #     GRIP_DRIVE_BY_FAMILY drive the gripper_* fields carry (resolve's
                                                 #     grip_family); None = the profile's fields as written
    # R7 / D8: blocked-motion limits. None = the rig's settled default (the pre-R7 value, rig.SETTLED_LAG_RAD 0.03)
    lag_max_rad: float = .15                     # rig.LAG_MAX_RAD, in motion (never widened)
    settled_lag_rad: float = .03                 # rig.SETTLED_LAG_RAD, after the last waypoint
    tol_approach: float | None = None            # pick: the approach line (legacy 0.06)
    tol_joint_move_held: float | None = None     # plan_to: the straight joint move while carrying (legacy 0.08)
    tol_rrt: float | None = None                 # plan_to: RRT-Connect paths, empty-handed too (legacy 0.08)
    tol_rise: float | None = None                # transit_to: the straight rise to Z_SAFE, empty-handed too (legacy 0.08)
    tol_lift: float | None = None                # pick: the lift after the close (legacy 0.08)
    tol_reaim: float | None = None               # place_down: the re-aim above the spot (legacy 0.08)
    place_down_contact_lag: float = .05          # place_down: lag read as "the dish met the rack" (legacy detector)
    jaw_wall_rad: tuple = (.45, .795)            # jaw-angle filter: stand-in for the gripper's object detection
    # D17: pedestal (both profiles, Phases 0-1)
    mount: str = "data/results/robot/mount/easy_s0_any.json"
    # D19: the benchmark's counter cap (a park beyond it is refused and counted)
    counter_cap: bool = True
    # plans/2026-11-04-fifteen-of-fifteen.md, basket out (the user, 2026-11-04 19:20 EST; the real basket lifts out): False = the
    #     twin without its SilverwareBasket in the Kit scene and in the arm's FCL world (the episode's --no-basket / --basket out, or
    #     a --place-goal load's top-level "basket": false); s0's goals, the benchmark samples and S_ref keep the basket
    basket: bool = True
    # D18 (diagnostic, NOT headline): skip a dish after its two attempts and continue with the next move
    continue_after_failed_dish: bool = False
    # geometry of the motion primitives (unchanged from the upright3 PASS)
    z_safe_m: float = 1.25
    hover_m: float = .10
    release_above_m: float = .006
    park_spots: tuple = ((.30, .12), (.50, .12), (.30, -.10), (.50, -.10), (.70, .12))
    # harness
    sleep_threshold: float | None = 0.           # dishes and robot links never sleep (plan Phase 0.3, D10); None = asset
                                                 #     values untouched (ANALYSIS only: attribution of a changed result)
    invariants_abort: bool = True                # an auto-fail invariant ends the trial (legacy: record only)
    deinstance_root: str = "/World/Robot/Gripper"  # D12: "/World/Robot" once the press test shows dead arm colliders
    diag_settle_active: bool = True              # D3: run the plan's velocity settle routine (steps up to 5 s) after
                                                 #     each move as a diagnostic column; legacy measures it passively


PROFILES = {
    "headline": Flags(profile="headline"),
    "legacy_upright3": Flags(
        profile="legacy_upright3", test_case="upright3", place_mode="lower_until_contact", judge="racked_in",
        disturbance="any_neighbour", end_check="racked_in", move_settle="legacy", final_hold_observation_s=0.,
        reextend_before_scoring=False, hold_gate="legacy_rise",
        tol_approach=.06, tol_joint_move_held=.08, tol_rrt=.08, tol_rise=.08, tol_lift=.08, tol_reaim=.08,
        counter_cap=False, invariants_abort=False, diag_settle_active=False),
}

# R2 per grasp family (plans/2026-11-04-fifteen-of-fifteen.md, the user 2026-11-04): finger_joint's effort limit, stiffness and
# damping follow the dish's hold, as the real 2F-85 takes a force and a speed with each grip command
# (results/2026-11-04-grip-force.md). The bowls' ring holds and the cups' side pinches close at the calibrated 22 N squeeze
# (8 / 4 / 2.5: 22.5 N per pad in the probe, 22.3 to 23.2 N in the side ring gate, 20.4 N in the cup gate); the plates' pinches
# keep 8 / 400 / 1 (76 to 99 N at the wall, 10 of 10 on the plank; 2 of 10 at the soft drive, which closes too slowly). A family
# not listed (rim, wall, foot-in: never measured at another drive) keeps the profile's fields. "side-down" is the mouth-down cup
# side pinch (--grasp-family side with --hold side-down or --mouth-down). The inner finger joints keep the profile's fields.
GRIP_SQUEEZE_22N = {"gripper_effort": 8., "gripper_stiffness": 4., "gripper_damping": 2.5}
GRIP_WALL_90N = {"gripper_effort": 8., "gripper_stiffness": 400., "gripper_damping": 1.}
GRIP_DRIVE_BY_FAMILY = {"foot-out": GRIP_SQUEEZE_22N, "foot-side": GRIP_SQUEEZE_22N, "side": GRIP_SQUEEZE_22N,
                        "side-down": GRIP_SQUEEZE_22N, "plate-wall": GRIP_WALL_90N, "edge": GRIP_WALL_90N}
GRIP_DRIVE_FIELDS = ("gripper_effort", "gripper_stiffness", "gripper_damping", "finger_effort", "finger_stiffness",
                     "finger_damping")

CHOICES = {"test_case": (None, "upright3"), "place_mode": ("goal_pose", "lower_until_contact"),
           "judge": ("at_goal", "racked_in"), "disturbance": ("benchmark", "any_neighbour"),
           "end_check": ("benchmark", "racked_in"), "move_settle": ("benchmark", "legacy"),
           "hold_gate": ("drift", "legacy_rise")}

# Which plan item each field implements (the report's deviation table reads this).
FIELD_ITEM = {"test_case": "R4", "place_mode": "R5", "judge": "R6", "disturbance": "D1", "end_check": "D2",
              "move_settle": "D3", "final_hold_observation_s": "D3", "upper_in_for_lower": "R3",
              "reextend_before_scoring": "R3/D2", "hold_gate": "R1/D9", "in_hand_settle_ticks": "R1/D9",
              "hold_window_s": "R1/D9", "hold_drift_max_m": "R1/D9", "legacy_rise_min_m": "R1",
              "legacy_near_tcp_m": "R1", "gripper_effort": "R2", "gripper_stiffness": "R2", "gripper_damping": "R2",
              "finger_effort": "R2", "finger_stiffness": "R2", "finger_damping": "R2", "grip_family": "R2",
              "lag_max_rad": "R7/D8",
              "settled_lag_rad": "R7/D8", "tol_approach": "R7/D8", "tol_joint_move_held": "R7/D8", "tol_rrt": "R7/D8",
              "tol_rise": "R7/D8", "tol_lift": "R7/D8", "tol_reaim": "R7/D8", "place_down_contact_lag": "R7/D8",
              "jaw_wall_rad": "R7", "mount": "D17", "counter_cap": "D19", "basket": "fifteen-of-fifteen: basket out",
              "continue_after_failed_dish": "D18",
              "sleep_threshold": "D10", "invariants_abort": "Phase 0.4", "deinstance_root": "D12",
              "diag_settle_active": "D3"}

# Fields whose headline value departs from the benchmark's teleport rules and must be labelled in every report.
BENCHMARK_RELAXATIONS = ("test_case", "place_mode", "judge", "disturbance", "end_check", "move_settle",
                         "reextend_before_scoring", "counter_cap", "continue_after_failed_dish", "basket")


def resolve(profile="headline", grip_family=None, **overrides):
    """The profile's Flags with explicit overrides (validated against CHOICES). ``grip_family`` (R2 per hold): finger_joint's
    drive is that family's GRIP_DRIVE_BY_FAMILY entry (a family not listed keeps the profile's fields), the explicit
    overrides on top (the episode's --grip-drive pairs override the family's values)."""
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {sorted(PROFILES)}")
    names = {f.name for f in fields(Flags)}
    bad = set(overrides) - names
    if bad:
        raise ValueError(f"unknown flag(s) {sorted(bad)}")
    flags = replace(PROFILES[profile], grip_family=grip_family, **{**GRIP_DRIVE_BY_FAMILY.get(grip_family, {}), **overrides})
    for name, allowed in CHOICES.items():
        if getattr(flags, name) not in allowed:
            raise ValueError(f"{name}={getattr(flags, name)!r} not in {allowed}")
    return flags


def to_dict(flags):
    d = asdict(flags)
    d["jaw_wall_rad"] = list(d["jaw_wall_rad"])
    d["park_spots"] = [list(p) for p in d["park_spots"]]
    return d


def deviations(flags):
    """[(field, value, headline value, plan item)] where ``flags`` differs from the headline profile at the same grasp family
    (R2 per hold: the family's own drive is the headline's, an explicit override of it is a deviation)."""
    head = resolve("headline", grip_family=flags.grip_family)
    return [(f.name, getattr(flags, f.name), getattr(head, f.name), FIELD_ITEM.get(f.name, ""))
            for f in fields(Flags) if f.name not in ("profile", "grip_family")
            and getattr(flags, f.name) != getattr(head, f.name)]


def grip_drive(flags):
    """The finger drive a run spawns with (flags R2): finger_joint's and the inner finger joints' fields, the family and, when
    the family has a GRIP_DRIVE_BY_FAMILY entry, that entry (an explicit override shows as a difference to it)."""
    return {**{k: getattr(flags, k) for k in GRIP_DRIVE_FIELDS}, "family": flags.grip_family,
            "family_drive": dict(GRIP_DRIVE_BY_FAMILY[flags.grip_family]) if flags.grip_family in GRIP_DRIVE_BY_FAMILY else None}


def is_headline(flags):
    """True when no benchmark relaxation and no diagnostic mode is on (the numbers may be quoted as headline)."""
    head = PROFILES["headline"]
    return all(getattr(flags, name) == getattr(head, name) for name in BENCHMARK_RELAXATIONS)


def hold_verdict(flags, *, rise_m=None, dist_to_tcp_m=None, in_hand=None, window_s=None, window_drift_m=None):
    """The ONE in-hand hold rule of a profile (D9), shared by the episode, the grip probe and the grasp test.

    ``drift`` gate: held = the dish is in the hand (``in_hand``: the caller's evidence, e.g. it touches only the
    gripper during the window) and drifts <= ``hold_drift_max_m`` over a window of >= ``hold_window_s`` that
    starts after the in-hand settle. ``legacy_rise`` gate (the episode's upright3 rule): rose > 5 cm and lies
    within 20 cm of the TCP after the in-hand settle. Returns (held, rule text)."""
    if flags.hold_gate == "legacy_rise":
        held = (rise_m is not None and dist_to_tcp_m is not None
                and rise_m > flags.legacy_rise_min_m and dist_to_tcp_m < flags.legacy_near_tcp_m)
        return bool(held), (f"legacy_rise: rose > {flags.legacy_rise_min_m * 1e3:.0f} mm and within "
                            f"{flags.legacy_near_tcp_m * 1e3:.0f} mm of the TCP after {flags.in_hand_settle_ticks} ticks")
    held = (bool(in_hand) and window_s is not None and window_drift_m is not None
            and window_s >= flags.hold_window_s - 1e-9 and window_drift_m <= flags.hold_drift_max_m)
    return bool(held), (f"drift: in the hand and <= {flags.hold_drift_max_m * 1e3:.0f} mm drift over "
                        f">= {flags.hold_window_s:.1f} s after the {flags.in_hand_settle_ticks}-tick in-hand settle")
