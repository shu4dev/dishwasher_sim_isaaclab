# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Shared machine/object/scenario/placement CLI flags for the setup scripts.

Stdlib-only ON PURPOSE: the Kit-side scripts import this BEFORE ``AppLauncher`` boots, where
any ``dishsim``/``isaaclab`` import is a boot-order landmine (docs/environment.md). The
matching ordered apply — machine, then object, then scenario, then placement — lives in
:func:`dishsim.config.apply_selection`; pass the four parsed flags to it verbatim.
"""


def add_selector_args(parser, scenario_default: str | None = "both_out",
                      scenario_help: str | None = None) -> None:
    """Add the four selection flags every setup script shares.

    Args:
        parser: The script's ``argparse`` parser.
        scenario_default: Default for ``--scenario``; ``None`` means "the active machine's
            placement state, resolved after ``--machine`` is applied".
        scenario_help: Override for the ``--scenario`` help text.
    """
    parser.add_argument("--placement", type=str, default=None,
                        help="Named base placement (see config.BASE_PLACEMENTS); default: the machine's.")
    parser.add_argument("--machine", type=str, default=None,
                        help="Machine name (see config.MACHINES); default: the v1 baseline.")
    parser.add_argument("--object", type=str, default="mug",
                        help="Carried object class (see config.OBJECTS).")
    parser.add_argument("--scenario", type=str, default=scenario_default,
                        help=scenario_help or "Rack-state scenario (see config.SCENARIOS).")
