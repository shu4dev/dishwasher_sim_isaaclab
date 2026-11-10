"""Dependency graph of a certified goal: physical_graph (moved from frigidaire_dependency_kit.py, which delegates here).

Support edges come from the certify run's confirmation drops (Kit, frigidaire_dependency_kit.py --certify), access
edges from its path check; this module only draws the graph from them (read from the build record in _work) and writes
the sample's dag block. start_dag draws the start's graph (initial.dag) from the Isaac probes of
frigidaire_bench_kit.py --start-graph. Standard library only.
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
    """graph stage (Kit-free): the graph of the certified goal from the certify run's witness order, confirmations and
    access hits in the build record (metadata.provenance.goal_record) -> the dag block (sample.dag_of); its counts go to
    metadata.provenance.certify. Variant graph=none stores no dag."""
    import json
    from . import ROOT
    from .sample import dag_of
    if params.get("graph", "physical") == "none":
        state["dag"] = None
        return state, "SKIP", "graph=none"
    prov = state["metadata"]["provenance"]
    if not (prov.get("certify") or {}).get("valid"):
        return state, "FAIL", "goal not certified (no valid certify record)"
    dep = json.loads((ROOT / prov["goal_record"]).read_text())
    order = (dep.get("witness") or {}).get("order")
    graph = physical_graph(order, dep.get("confirmations") or [], (dep.get("access") or {}).get("hits") or [])
    state["dag"] = dag_of(graph, order)
    prov["certify"]["counts"] = graph["counts"]
    return state, "PASS", f"{graph['counts']}"


def start_dag(table):
    """The start's dag (plans/2026-11-10-start-dependency-graph.md) from the probe table of frigidaire_bench_kit.py
    --start-graph ({ids, rows: {probe support | lift, b, a, displaced}}): a -> b (a is moved before b) support when parking
    b displaced a, access when lifting b displaced a and parking b did not. A pair with arcs both ways loses both
    (conflicts, as in physical_graph); the edges are the transitive reduction, so a three-dish stack reads as a two-edge
    chain; layers as physical_graph's (0 = nothing has to move first); order by layer, then the table's dish order.
    Returns nodes, edges {a, b, kind}, layers, order (the dag shape) and conflicts. Raises ValueError on a longer cycle."""
    nodes, hit = list(table["ids"]), {}
    for r in table["rows"]:
        if r["displaced"] and hit.get((r["a"], r["b"])) != "support":
            hit[(r["a"], r["b"])] = "support" if r["probe"] == "support" else "access"
    conflicts = sorted({tuple(sorted(p)) for p in hit if p[::-1] in hit})
    arcs = {p: kind for p, kind in hit.items() if tuple(sorted(p)) not in conflicts}
    after = {i: {b for a, b in arcs if a == i} for i in nodes}
    for m in nodes:                                          # Warshall: after[i] = every dish reachable from i
        for i in nodes:
            if m in after[i]:
                after[i] |= after[m]
    if any(i in after[i] for i in nodes):
        raise ValueError(f"cycle among {sorted(i for i in nodes if i in after[i])}")
    edges = [{"a": a, "b": b, "kind": kind} for (a, b), kind in sorted(arcs.items())
             if not any(b in after[c] for c in after[a] if c != b)]
    layer = {}
    for i in sorted(nodes, key=lambda i: sum(i in after[j] for j in nodes)):   # fewer ancestors first: topological
        layer[i] = 1 + max((layer[e["a"]] for e in edges if e["b"] == i), default=-1)
    return {"nodes": nodes, "edges": edges, "layers": {i: layer[i] for i in nodes},
            "order": sorted(nodes, key=lambda i: (layer[i], nodes.index(i))), "conflicts": [list(c) for c in conflicts]}
