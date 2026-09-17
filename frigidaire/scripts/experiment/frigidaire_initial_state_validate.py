#!/usr/bin/env python3
"""One fresh Isaac process for a measured baseline or joint load validation."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]


def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--usd', type=Path, default=ROOT/'build/frigidaire_collection/usd/fdpc4221as.usdc')
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--out-dir', type=Path, required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=240)
    parser.add_argument('--order', choices=['upper_first', 'lower_first'], default='upper_first')
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device='cpu', headless=True)
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    app = None
    result = {'outcome': 'simulation_error', 'started_utc': datetime.now(timezone.utc).isoformat()}
    source_files = [Path(__file__), ROOT/'frigidaire/src/dishsim_frigidaire/initial_state_runtime.py',
                    ROOT/'frigidaire/src/dishsim_frigidaire/initial_state_candidates.py']
    result['execution_source_hashes'] = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                         for path in source_files}
    try:
        app = AppLauncher(args).app
        sys.path[:0] = [str(ROOT/'src'), str(ROOT/'frigidaire/src')]
        from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend
        from dishsim_frigidaire.random_poses import source_geometry_domains
        manifest = json.loads(args.manifest.read_text()) if args.manifest else {'objects': []}
        candidates = manifest['objects']
        backend = IsaacInitialStateBackend(args.usd, args.out_dir, device=args.device,
            domains=source_geometry_domains(), deadline=started+args.max_wall_seconds,
            app=app, candidates=candidates)
        if args.manifest:
            order = ('UpperRack', 'LowerRack') if args.order == 'upper_first' else ('LowerRack', 'UpperRack')
            result.update(backend.evaluate(order=order, baseline=manifest.get('baseline')))
        else:
            result.update(backend.prepare_baseline())
            result['outcome'] = 'baseline_ready' if result.get('result') == 'PASS' else 'baseline_failed'
        result['runtime_settings'] = backend.runtime_settings
        result['isaac_sim_version'] = Path('/isaac-sim/VERSION').read_text().strip()
    except Exception as exc:
        traceback.print_exc()
        result.update(outcome='timeout' if isinstance(exc, TimeoutError) else 'simulation_error', error=repr(exc))
    finally:
        result['wall_seconds'] = time.monotonic()-started
        result['finished_utc'] = datetime.now(timezone.utc).isoformat()
        (args.out_dir/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print('[RESULT] '+result['outcome']+' '+str(args.out_dir/'result.json'), flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == '__main__':
    main()
