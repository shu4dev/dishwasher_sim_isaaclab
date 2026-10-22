"""Slot finder: candidate sites of each dish kind from the machine manifest's tine grids x the dish manifest's allowed
orientations. Standard library only; rack-local metres (each rack's link frame), as the manifests.

A site is where a dish of that kind may go, not a pose: {kind, rack, slot, mode, x_m, y_m} (x_m is None for a zone that
spans the rack in x). Poses inside a site (leans, offsets, lifts, tilts) stay with the dish set's pose families.
Basket containers are skipped this round (admitted by the schema, unimplemented).

Modes: between_tines (a gap between consecutive tines present in both rows of an outer bank: front = rows 0-1,
rear = the last two; front gaps keep ``front_keep_out_margin_m`` ahead of the rack's keep-out), over_tines (a zone over
a band of two tine rows), centre_gaps (the gaps between consecutive present tines of one column, centred at x = 0),
glass_channel (the channel ladder on each side).
"""

BANDS = {"front": (0, 1), "mid": (2, 3), "rear": (-2, -1)}


def _mid(a, b):
    return (a + b) / 2


def _between_tines(grid, params):
    xs, ys, present = grid["columns_x_m"], grid["rows_y_m"], grid["present"]
    keep_x = min((k["x_m"][0] for k in grid.get("keep_out", ())), default=None)
    out = []
    for bank in params["banks"]:
        r0, r1 = BANDS[bank]
        both = [present[r0][i] and present[r1][i] for i in range(len(xs))]
        for i in range(len(xs) - 1):
            x = _mid(xs[i], xs[i + 1])
            if not (both[i] and both[i + 1]):
                continue
            if bank == "front" and keep_x is not None and x > keep_x - params.get("front_keep_out_margin_m", 0.):
                continue
            out.append({"slot": f"lower_{bank}_{i:02d}", "x_m": x, "y_m": _mid(ys[r0], ys[r1])})
    return out


def _over_tines(grid, params):
    ys = grid["rows_y_m"]
    out = []
    for zone, band in params["zones"].items():
        r0, r1 = BANDS[band]
        out.append({"slot": f"lower_{zone}", "x_m": None, "y_m": _mid(ys[r0], ys[r1]),
                    "band_y_m": [ys[r0], ys[r1]]})
    return out


def _centre_gaps(grid, params):
    col, ys = params["column"], grid["rows_y_m"]
    rows = [i for i in range(len(ys)) if grid["present"][i][col]]
    gaps = [(a, b) for a, b in zip(rows[:-1], rows[1:]) if not params.get("regular_only", True) or b - a == 1]
    return [{"slot": f"upper_bowl_{a:02d}", "x_m": 0., "y_m": _mid(ys[a], ys[b])} for a, b in gaps]


def _glass_channel(rack, params):
    g = rack["glass_channels"]
    return [{"slot": f"glass_{'left' if side < 0 else 'right'}_{i}", "x_m": side * g["x_abs_m"],
             "y_m": g["ladder_y0_m"] + g["ladder_pitch_m"] * i}
            for i in range(g["ladder_count"]) for side in params["sides"]]


def find(machine, dishes):
    """{kind: [site]} for every dish manifest in ``dishes`` ({kind: manifest}) against ``machine``."""
    racks = {c["name"]: c for c in machine["containers"] if c["kind"] == "rack"}
    out = {}
    for kind, dish in dishes.items():
        sites = []
        for allowed in dish["allowed_orientations"]:
            rack = racks.get(allowed["rack"])
            if allowed["container_kind"] != "rack" or rack is None:
                continue                                        # baskets: unimplemented this round
            mode, params = allowed["mode"], allowed.get("params", {})
            found = (_glass_channel(rack, params) if mode == "glass_channel"
                     else {"between_tines": _between_tines, "over_tines": _over_tines,
                           "centre_gaps": _centre_gaps}[mode](rack["tine_grid"], params))
            sites += [{"kind": kind, "rack": rack["name"], "mode": mode, **s} for s in found]
        out[kind] = sites
    return out
