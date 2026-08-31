# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Initial-arrangement sampling for benchmark instances — Kit-free.

Lives in the package rather than in ``scripts/setup/gen_instances.py`` because that script
launches ``AppLauncher`` at module scope, so importing it boots Kit; anything that wants to
build instances without Isaac (an adversarial-family generator, a test) could otherwise only
copy the sampler, and a second copy is a second thing to drift.

The sampler decides only what an item is COMMANDED to; whether the arrangement then settles
reproducibly is a physics question the Kit generator answers with its reproduction gate.
"""

import numpy as np

from . import capacity, config


def author_cycles(roster, lengths, rng):
    """Author disjoint same-class swap cycles over goal slots.

    Member ``k`` of a cycle is forced to START on member ``k+1``'s goal slot (wrapping) — a
    true derangement, so no member can go home before another leaves: solving it requires
    evicting through the buffer band or a spare in-machine cell, which is exactly what the
    counter cap then rations.

    Args:
        roster: Planned items (``item_id``, ``object_class``, ``slot_id``).
        lengths: Cycle lengths to author, e.g. ``(2, 3)``; members are disjoint across cycles.
        rng: ``numpy.random.Generator``.

    Returns:
        ``(forced_slots, cycles)`` — ``forced_slots`` maps item_id -> the slot_id it must
        start on, ``cycles`` is ``[[item_id, ...], ...]`` for the instance meta — or
        ``(None, None)`` when no class has enough unused members for some requested length.
    """
    by_class: dict = {}
    for it in roster:
        by_class.setdefault(it.object_class, []).append(it)
    used: set = set()
    forced: dict = {}
    cycles: list = []
    for length in lengths:
        pools = {c: [it for it in v if it.item_id not in used] for c, v in by_class.items()}
        ok = sorted(c for c, v in pools.items() if len(v) >= length)
        if not ok:
            return None, None
        cls = ok[int(rng.integers(len(ok)))]
        members = [pools[cls][j] for j in rng.permutation(len(pools[cls]))[:length]]
        for k, it in enumerate(members):
            forced[it.item_id] = members[(k + 1) % length].slot_id
        used.update(it.item_id for it in members)
        cycles.append([it.item_id for it in members])
    return forced, cycles


def sample_initials(roster, tables, world, rng, n_displace, *,
                    avoid_goal_slots: bool = False, max_counter: int | None = None,
                    forced_slots: dict | None = None):
    """Commanded initial pose per item, or ``(None, displaced)`` on a dead end.

    Displaced items go first-FCL-free into some OTHER placeable slot of their own class or
    onto the buffer band; the rest start at their own target, so an instance's difficulty is
    exactly the displaced set.

    Args:
        roster: Planned items for the state (``capacity.PlannedPlacement``-like: ``item_id``,
            ``object_class``, ``slot_id``, ``T_base_obj``).
        tables: ``capacity.load_state_tables`` result — ``slots`` and ``placeable`` per class.
        world: A ``rearrange.ArrangementWorld`` (or anything with ``clear``/``sync``/
            ``move_collides``/``buffer_poses``).
        rng: ``numpy.random.Generator``; the only source of randomness here.
        n_displace: How many items start away from their target (includes the forced ones).
        avoid_goal_slots: Displaced items may not start on ANY roster item's goal slot
            (the easy tier's no-goal-squatting rule).
        max_counter: Max displaced items placed on the counter band, or ``None`` = no cap.
        forced_slots: ``{item_id: slot_id}`` — cycle members committed FIRST, deterministically
            (an FCL rejection there dead-ends the whole draw so the caller re-rolls early).

    Returns:
        ``(initials, displaced)`` where ``initials`` maps item id to a base-frame 4x4, or
        ``(None, displaced)`` if some item had no free candidate (the caller re-rolls).
    """
    world.clear()
    forced = dict(forced_slots or {})
    free_pool = [it.item_id for it in roster if it.item_id not in forced]
    n_random = n_displace - len(forced)
    assert n_random >= 0, "forced cycle members exceed n_displace"
    displaced = set(forced) | {str(s) for s in
                               rng.choice(free_pool, size=n_random, replace=False)}
    goal_slot_ids = ({cls: {it.slot_id for it in roster if it.object_class == cls}
                      for cls in {it.object_class for it in roster}}
                     if avoid_goal_slots else {})
    by_id = {it.item_id: it for it in roster}
    initials: dict = {}
    n_buf = 0
    # forced (cycle) members first, then the rest in roster order
    order = [by_id[i] for i in forced] + [it for it in roster if it.item_id not in forced]
    for it in order:
        cls = it.object_class
        if it.item_id in forced:
            with config.active_object(cls):
                T = capacity._nominal_release_pose(tables.slots[cls][forced[it.item_id]])
            if world.move_collides(it.item_id, T, object_class=cls):
                return None, displaced  # cycle slot blocked — re-roll the whole draw
        elif it.item_id not in displaced:
            T = np.asarray(it.T_base_obj)
        else:
            with config.active_object(cls):
                slot_poses = [capacity._nominal_release_pose(tables.slots[cls][sid])
                              for sid, ok in tables.placeable[cls].items()
                              if ok and sid != it.slot_id
                              and (not avoid_goal_slots or sid not in goal_slot_ids[cls])]
            candidates = ([(c, False) for c in slot_poses]
                          + [(c, True) for c in world.buffer_poses(cls)])
            # Generator.shuffle silently shuffles a stacked COPY of a list of arrays —
            # permute indices instead
            candidates = [candidates[j] for j in rng.permutation(len(candidates))]
            T = None
            for cand, is_buf in candidates:
                if is_buf and max_counter is not None and n_buf >= max_counter:
                    continue
                if not world.move_collides(it.item_id, cand, object_class=cls):
                    T = cand
                    n_buf += int(is_buf)
                    break
            if T is None:
                return None, displaced
        world.sync({it.item_id: T}, {it.item_id: it.object_class})
        initials[it.item_id] = T
    return initials, displaced
