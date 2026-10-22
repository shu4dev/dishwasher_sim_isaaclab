"""Dependency graph of a certified goal: physical_graph (moved from frigidaire_dependency_kit.py, which delegates here).

Support edges come from the certify run's confirmation drops (Kit, frigidaire_dependency_kit.py --certify), access
edges from its path check; this module only draws the graph from them. Standard library only.
"""


def physical_graph(order, confirmations, hits):
    """The drawn graph of a certified load (plans/2026-10-16-physical-cause-graph.md; only HARD requirements, coordinator
    2026-10-16), standard library only. Support: a -> b for each a in a confirmation's ``hard`` (b fails without a: its
    only support removed, or with several, that one removed and the others in, while it lands with them all); a dish whose
    supports are needed only as a disjunction (outcome disjunctive: it fails without all of them, no single one is
    necessary) gets no edge, kept under ``disjunctions`` {b, supports, suffice}. Access: a -> b for every hit (b's final
    pose covers a's insertion path). A pair with arcs both ways loses both (conflicts: nothing is claimed about it); an edge
    the witness order breaks is not drawn (against_witness: the witness built the load that way). So every drawn edge
    follows the witness order and the graph is a DAG: layers = 0 for a dish with no prerequisite, else 1 + the deepest
    prerequisite's layer. Edges {a, b, kind support | access, conf | hit (index)}: a must be in place before b."""
    pos = {i: n for n, i in enumerate(order)}
    support = {(a, c["dish"]): k for k, c in enumerate(confirmations) if c.get("outcome") == "hard" for a in c["hard"]}
    access = {(h["a"], h["b"]): k for k, h in enumerate(hits)}
    arcs = set(support) | set(access)
    conflicts = sorted({tuple(sorted(p)) for p in arcs if p[::-1] in arcs})
    gone = {p for c in conflicts for p in (c, c[::-1])}
    edges, against = [], []
    for kind, found, key in (("support", support, "conf"), ("access", access, "hit")):
        for (a, b), k in sorted(found.items(), key=lambda x: (pos[x[0][1]], pos[x[0][0]])):
            if (a, b) in gone:
                continue
            (against if pos[a] > pos[b] else edges).append({"a": a, "b": b, "kind": kind, key: k})
    used = {i for e in edges for i in (e["a"], e["b"])}
    layer = {}
    for i in order:                                          # every drawn edge goes forward in the witness order
        layer[i] = 1 + max((layer[e["a"]] for e in edges if e["b"] == i), default=-1)
    counts = {"support": sum(e["kind"] == "support" for e in edges), "access": sum(e["kind"] == "access" for e in edges),
              "conflicts": len(conflicts), "against_witness": len(against),
              "disjunctive": sum(c.get("outcome") == "disjunctive" for c in confirmations)}
    return {"edges": edges, "conflicts": [list(c) for c in conflicts], "against_witness": against,
            "disjunctions": [{"b": c["dish"], "supports": c["supports"], "suffice": c.get("suffice") or []}
                             for c in confirmations if c.get("outcome") == "disjunctive"],
            "free": [i for i in order if i not in used], "layers": layer, "counts": counts}


def stage_graph(state, params):
    """graph stage (Kit-free): the graph of the certified goal from its stored confirmations and access hits.
    Variant graph=none stores no graph."""
    goal = state["goal"]
    if params.get("graph", "physical") == "none":
        goal["graph"] = None
        return state, "SKIP", "graph=none"
    record = goal.get("certify") or {}
    if not record.get("valid"):
        return state, "FAIL", "goal not certified (no valid certify record)"
    goal["graph"] = physical_graph(record["order"], record["confirmations"], record["hits"])
    return state, "PASS", f"{goal['graph']['counts']}"
