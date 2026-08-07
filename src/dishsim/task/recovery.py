# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""What to do when objects remain but none of them is pickable (Stage C).

The situation is real and not rare: every remaining object either has something resting on it or
has no collision-free grasp in the current world. Declaring failure immediately would be honest
but premature — "no collision-free grasp" is frequently an artifact of how thin the candidate set
was, and a leaning pile often relaxes on its own given a moment of physics.

So the response is a **bounded ladder**, cheapest first, and a loud failure at the end:

1. **widen the grasp search** — re-probe with a much denser yaw sweep. No physics, no motion,
   costs milliseconds, and resolves the common case where the nominal grasp is blocked from one
   side while a rotated one is clear.
2. **re-settle** — run physics for a moment and re-read the world. Objects mid-topple settle,
   a leaning stack finds its rest, and the support graph and grasp availability are both rebuilt
   from what is actually there.
3. **give up loudly** — end the episode as ``deadlock`` naming every remaining object and the
   reason it could not be picked, rather than looping or reporting a bare failure.

Each rung is bounded and each returns a bool meaning "something changed, try again", so the
sequencer's loop stays a loop rather than a recursion. The cap
(``config.TASK["max_recovery_attempts"]``) is what guarantees termination: a rung that claims
progress it did not make can only waste that many iterations.

Deliberately NOT here: a non-prehensile nudge or push. That is the textbook next rung and it is a
genuinely different capability — this repository's entire safety model (calibrated pad-force
bands, the hidden wrist weld) is built around pinch grasps, and an open-jaw push needs its own
force thresholds and its own validation. It belongs behind this same interface when it is
justified, which is why strategies are a registry rather than a hard-coded sequence.
"""

from typing import Callable

from .. import config

#: A recovery strategy: ``(sequencer, remaining) -> bool``. True means "state changed, re-evaluate".
#: The sequencer re-derives the support graph and grasp availability after any True.
RecoveryFn = Callable[[object, list], bool]


def widen_grasp_search(sequencer, remaining) -> bool:
    """Re-probe grasps with a much denser yaw sweep.

    The cheapest rung by a wide margin — pure IK and collision queries against the world as it
    already is. It fires once per episode: having widened the sweep there is nothing further to
    widen, so claiming progress a second time would burn a recovery attempt for nothing.
    """
    if getattr(sequencer, "_grasp_widened", False):
        return False
    wide = int(config.TASK["grasp_yaw_samples_wide"])
    base = getattr(sequencer, "grasp_fn", None)
    if base is None or not hasattr(base, "set_yaw_samples"):
        return False
    base.set_yaw_samples(wide)
    sequencer._grasp_widened = True
    return True


def resettle(sequencer, remaining) -> bool:
    """Run physics briefly and re-read the world.

    Objects caught mid-topple come to rest, a leaning pile relaxes, and both the support graph
    and grasp availability are rebuilt from measured state on the next iteration. Unlike rung 1
    this can be repeated — each settle genuinely changes the scene — so it is bounded only by the
    attempt cap.
    """
    motion = getattr(sequencer, "motion", None)
    if motion is None or not hasattr(motion, "hold"):
        return False
    motion.hold(int(config.TASK["recovery_settle_steps"]), phase="recovery-settle")
    return True


#: Registry key -> strategy. Order matters: the ladder is tried top to bottom and stops at the
#: first rung that reports a change.
RECOVERY_STRATEGIES: dict = {
    "widen_grasp_search": widen_grasp_search,
    "resettle": resettle,
}

#: The default ladder, cheapest rung first.
DEFAULT_LADDER = ("widen_grasp_search", "resettle")


def available_recoveries() -> list:
    """Registered strategy names, sorted."""
    return sorted(RECOVERY_STRATEGIES)


def make_recovery(names=None) -> RecoveryFn:
    """Build a ladder that tries each named strategy in order.

    Args:
        names: Strategy names, cheapest first; defaults to :data:`DEFAULT_LADDER`.

    Returns:
        A callable matching :data:`RecoveryFn` that returns True as soon as one rung reports a
        change, and False when every rung is exhausted — which the sequencer turns into a
        ``deadlock``.
    """
    names = list(DEFAULT_LADDER if names is None else names)
    unknown = [n for n in names if n not in RECOVERY_STRATEGIES]
    if unknown:
        raise ValueError(f"unknown recovery strategy {unknown} (choices: {available_recoveries()})")

    def ladder(sequencer, remaining) -> bool:
        for name in names:
            if RECOVERY_STRATEGIES[name](sequencer, remaining):
                sequencer._emit("recovery_step", {"strategy": name,
                                                  "remaining": [i.item_id for i in remaining]})
                return True
        return False

    return ladder
