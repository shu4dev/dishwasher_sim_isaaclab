"""Frigidaire FDPC4221AS source, independent of simulation startup.

Submodules load on demand; importing the package does not import NumPy, USD,
PhysX, FCL, or Isaac Lab. USD authoring and simulation require their runtimes.
"""
from importlib import import_module

__all__ = [
    "asset", "claims", "geometry", "loading", "load_validation", "lower_rack_asset", "paths", "tableware",
    "usd_bootstrap",
]


def __getattr__(name):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = import_module(f"{__name__}.{name}")
    globals()[name] = module
    return module


def __dir__():
    return sorted(set(globals()) | set(__all__))
