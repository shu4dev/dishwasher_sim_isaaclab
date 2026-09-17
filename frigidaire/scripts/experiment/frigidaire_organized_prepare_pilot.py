#!/usr/bin/env python3
"""Prepare a complete inventory for an independently coordinated physics pilot."""
import argparse
from collections import Counter
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'frigidaire/src')]
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.organized_candidates import solve_inventory
from dishsim_frigidaire.organization import OrganizationGeometry, evaluate_organization
from frigidaire_organized_experiment import assign_identities, counts, digest, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--source-state-id', required=True)
    args = parser.parse_args()
    root = args.run_dir.resolve()
    summary = json.loads((root / 'summary.json').read_text())
    row = next(row for row in summary['states'] if row['source_state_id'] == args.source_state_id)
    source_path = Path(row['source_state'])
    source = json.loads(source_path.read_text())
    baseline = json.loads((root / 'baseline.json').read_text())
    pilot = root / ('pilot_' + args.source_state_id.replace('_', ''))
    pilot.mkdir(exist_ok=False)
    shutil.copy2(__file__, pilot / 'generator.py')
    catalog_path, graph_path = root / summary['candidate_file'], root / summary['compatibility_file']
    catalog, graph = json.loads(catalog_path.read_text()), json.loads(graph_path.read_text())
    shutil.copy2(catalog_path, pilot / 'catalog_snapshot.json')
    shutil.copy2(graph_path, pilot / 'compatibility.json')
    desired = dict(Counter(obj['kind'] for obj in source['objects']))
    geometry = OrganizationGeometry(ROOT / 'build/frigidaire_collection/usd')
    exclusions, records = [], []
    for iteration in range(8):
        solver = solve_inventory(catalog, graph, desired, seed=summary['seed'] + 4400 + iteration,
                                 time_limit_s=10, excluded_sets=exclusions)
        records.append({'solver': solver})
        if not solver.get('selected_indices'):
            break
        objects = assign_identities([catalog['candidates'][i] for i in solver['selected_indices']], source['objects'], baseline)
        assessment = evaluate_organization(objects, geometry=geometry, component_frames=baseline['poses'],
                                           direct_support={obj['object_id']: True for obj in objects})
        assessment['scope'] = 'Static aggregate geometry; direct support from isolated screens only. Full joint physics and closure remain unvalidated.'
        records[-1]['static_organization'] = assessment
        save(pilot / 'solver.json', records)
        if assessment['valid']:
            manifest = dict(schema_version=1, seed=solver['seed'], purpose='organized_pilot',
                source_state_id=args.source_state_id, source_state=str(source_path),
                source_state_sha256=digest(source_path), policy=catalog['policy'], objects=objects,
                counts=counts(objects), baseline=baseline, order='upper_first', reproduction_of=None,
                pilot_source_catalog='catalog_snapshot.json', pilot_source_catalog_sha256=digest(catalog_path),
                pilot_graph='compatibility.json', pilot_solver='solver.json',
                candidate_indices=solver['selected_indices'], static_organization=assessment,
                scope='Complete inventory proposal only; no joint physics or closure acceptance claim')
            save(pilot / 'candidate_manifest.json', manifest)
            print('[PILOT READY] ' + str(pilot / 'candidate_manifest.json'), flush=True)
            return
        exclusions.append(solver['selected_indices'])
    save(pilot / 'solver.json', records)
    print('[PILOT UNRESOLVED] No static aggregate passing proposal', flush=True)


if __name__ == '__main__':
    main()
