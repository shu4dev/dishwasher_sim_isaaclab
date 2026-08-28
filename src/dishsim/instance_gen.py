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


def sample_initials(roster, tables, world, rng, n_displace):
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
        n_displace: How many items start away from their target.

    Returns:
        ``(initials, displaced)`` where ``initials`` maps item id to a base-frame 4x4, or
        ``(None, displaced)`` if some item had no free candidate (the caller re-rolls).
    """
    world.clear()
    displaced = {str(s) for s in
                 rng.choice([it.item_id for it in roster], size=n_displace, replace=False)}
    initials = {}
    for it in roster:
        if it.item_id not in displaced:
            T = np.asarray(it.T_base_obj)
        else:
            cls = it.object_class
            with config.active_object(cls):
                slot_poses = [capacity._nominal_release_pose(tables.slots[cls][sid])
                              for sid, ok in tables.placeable[cls].items()
                              if ok and sid != it.slot_id]
            candidates = slot_poses + world.buffer_poses(cls)
            # Generator.shuffle silently shuffles a stacked COPY of a list of arrays —
            # permute indices instead
            candidates = [candidates[j] for j in rng.permutation(len(candidates))]
            T = next((c for c in candidates
                      if not world.move_collides(it.item_id, c, object_class=cls)), None)
            if T is None:
                return None, displaced
        world.sync({it.item_id: T}, {it.item_id: it.object_class})
        initials[it.item_id] = T
    return initials, displaced
