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
    The settings of the 2026-09-29 upright3 PASS frozen exactly (results/robot/episodes/robot_upright3.json), the
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
    mount: str = "results/robot/mount/easy_s0_any.json"
    # D19: the benchmark's counter cap (a park beyond it is refused and counted)
    counter_cap: bool = True
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

CHOICES = {"test_case": (None, "upright3"), "place_mode": ("goal_pose", "lower_until_contact"),
           "judge": ("at_goal", "racked_in"), "disturbance": ("benchmark", "any_neighbour"),
           "end_check": ("benchmark", "racked_in"), "move_settle": ("benchmark", "legacy"),
           "hold_gate": ("drift", "legacy_rise")}

# Which plan item each field implements (the report's deviation table reads this).
FIELD_ITEM = {"test_case": "R4", "place_mode": "R5", "judge": "R6", "disturbance": "D1", "end_check": "D2",
              "move_settle": "D3", "final_hold_observation_s": "D3", "upper_in_for_lower": "R3",
              "reextend_before_scoring": "R3/D2", "hold_gate": "R1/D9", "in_hand_settle_ticks": "R1/D9",
              "hold_window_s": "R1/D9", "hold_drift_max_m": "R1/D9", "legacy_rise_min_m": "R1",
              "legacy_near_tcp_m": "R1", "gripper_effort": "R2", "gripper_stiffness": "R2", "lag_max_rad": "R7/D8",
              "settled_lag_rad": "R7/D8", "tol_approach": "R7/D8", "tol_joint_move_held": "R7/D8", "tol_rrt": "R7/D8",
              "tol_rise": "R7/D8", "tol_lift": "R7/D8", "tol_reaim": "R7/D8", "place_down_contact_lag": "R7/D8",
              "jaw_wall_rad": "R7", "mount": "D17", "counter_cap": "D19", "continue_after_failed_dish": "D18",
              "sleep_threshold": "D10", "invariants_abort": "Phase 0.4", "deinstance_root": "D12",
              "diag_settle_active": "D3"}

# Fields whose headline value departs from the benchmark's teleport rules and must be labelled in every report.
BENCHMARK_RELAXATIONS = ("test_case", "place_mode", "judge", "disturbance", "end_check", "move_settle",
                         "reextend_before_scoring", "counter_cap", "continue_after_failed_dish")


def resolve(profile="headline", **overrides):
    """The profile's Flags with explicit overrides (validated against CHOICES)."""
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile!r}; choose from {sorted(PROFILES)}")
    names = {f.name for f in fields(Flags)}
    bad = set(overrides) - names
    if bad:
        raise ValueError(f"unknown flag(s) {sorted(bad)}")
    flags = replace(PROFILES[profile], **overrides)
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
    """[(field, value, headline value, plan item)] where ``flags`` differs from the headline profile."""
    head = PROFILES["headline"]
    return [(f.name, getattr(flags, f.name), getattr(head, f.name), FIELD_ITEM.get(f.name, ""))
            for f in fields(Flags) if f.name != "profile" and getattr(flags, f.name) != getattr(head, f.name)]


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
