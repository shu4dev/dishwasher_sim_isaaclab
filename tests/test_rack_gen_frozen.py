# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The v1 rack-geometry byte-stability pin — one of the repo's two frozen-invariant tripwires.

A code change to rack_gen that alters geometry WITHOUT changing config params keeps the
config_hash unchanged, so every baked collision cache would silently disagree with the
generator. This is the only check that catches it, and it runs cache-free.

(Known local artifact: fails under non-pinned Python/numpy float stacks — e.g. py3.14 on
macOS — from last-ulp float-byte drift; it passes on the pinned Isaac box.)
"""

import hashlib

from dishsim import config, rack_gen

# Recorded 2026-08-10 against the pre-Bosch generator (RACK_GEN_VERSION 4, v1 machine). The
# v1 racks are FROZEN: any drift here silently invalidates every baked collision cache and
# the frozen mug baseline — this test must only ever fail because someone changed v1 geometry
# or the builder's v1 code path, and both are bugs.
_V1_DIGESTS = {
    "E_shelf_1_04": "95146089757e0b34b2bfe51caa322af9061a47512cdfe50dd78c6d73c56a2597",
    "E_shelf_03": "d9d80e69f36043485aa4c19838fdbd80b2b2e27af4367124152661d9fffe9438",
}
_V1_PARAMS_HASH = "d6df107a9bca8510"
_V1_PART_COUNTS = {"E_shelf_1_04": 421, "E_shelf_03": 166}


def test_v1_geometry_byte_identical():
    """build_rack on the v1 dicts reproduces the recorded pre-Bosch geometry byte-for-byte,
    and params_hash of the v1 generator config is unchanged."""
    config.apply_machine(config.MACHINE_BASELINE_NAME)  # hermetic: never trust test order
    for name, want in _V1_DIGESTS.items():
        parts = rack_gen.build_rack(config.RACK_GEN[name])
        assert len(parts) == _V1_PART_COUNTS[name], f"{name}: v1 part count drifted"
        h = hashlib.sha256()
        for part in parts:
            h.update(part.name.encode())
            h.update(part.zone.encode())
            h.update(part.group.encode())
            h.update(part.mesh.vertices.tobytes())
            h.update(part.mesh.faces.tobytes())
        assert h.hexdigest() == want, f"{name}: v1 rack geometry drifted"
    assert rack_gen.params_hash(config.RACK_GEN, config.RACK_GEN_VERSION) == _V1_PARAMS_HASH
