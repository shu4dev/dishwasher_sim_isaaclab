"""Kit-free cutlery orientation and basket-support diagnostic.

Head-down cutlery poses over the tape-measured silverware basket. Every kind takes one
compartment (geometry.PARAMETERS["silverware_basket"]["compartment_centres_xy"], in order);
inside it a 3x3 head-anchor grid derived from the cell size, four yaws, small X/Y tilts
and 2 mm shifts are FCL-checked against the closed appliance, required to touch the floor
lattice when lowered 4 mm, and flagged when a wall wire is within 2.5 mm. A greedy pass
then picks one pose per slot, and ``--write-candidates`` freezes the full per-kind choice
lists in the cutlery_candidates.json schema (no physics refinements).

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_cutlery_pose_search.py \\
        --write-candidates build/frigidaire_diagnostics/cutlery_candidates.json
"""
import argparse, sys, json, math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.asset import BODY_POSITIONS
from dishsim_frigidaire.geometry import PARAMETERS
from dishsim_frigidaire import loading
from dishsim_frigidaire.loading import CollisionWorld, collision_parts, rotation, quaternion_xyzw
import numpy as np
import fcl
from pxr import Usd

KINDS = ('fork', 'knife', 'tablespoon', 'teaspoon')          # compartment 0..3, in this order
TILTS = [('X', 0), ('X', -4), ('X', 4), ('X', -8), ('X', 8), ('Y', -4), ('Y', 4), ('Y', -8), ('Y', 8)]
SHIFTS = [(0, 0), (.002, 0), (-.002, 0), (0, .002), (0, -.002)]
CELL_FRACTION, CELL_OFFSET_MAX = .45, .030                  # anchors at 0 and +-0.45 of the half cell, capped at 30 mm
HEAD_ABOVE_FLOOR = .0011                                    # lowest vertex above the floor lattice top
CANDIDATE_KEYS = ('kind', 'rack', 'slot', 'variant', 'position', 'quaternion_xyzw')   # cutlery_candidates.json entry schema
CANONICAL = Path(loading.__file__).with_name('cutlery_candidates.json')
MIN_SIMULTANEOUS = 4


def cell_offsets(basket):
    """(x offsets, y offsets) of the 3x3 head-anchor grid inside one compartment cell.

    The layout string is columns x rows: "2x2" is 2 columns (X) by 2 rows (Y), "1x4" is
    one column by four rows.
    """
    columns, rows = (int(n) for n in basket['compartment_layout'].split('x'))
    half_x, half_y = basket['width_x'] / columns / 2, basket['length_y'] / rows / 2
    dx, dy = (round(min(CELL_FRACTION * h, CELL_OFFSET_MAX), 6) for h in (half_x, half_y))
    return (-dx, 0., dx), (-dy, 0., dy)


def hits(manager, objects):
    data = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
    for obj in objects:
        manager.collide(obj, data, fcl.defaultCollisionCallback)
        if data.result.is_collision:
            return True
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out-dir', type=Path, default=ROOT / 'build/frigidaire_diagnostics',
                        help='where cutlery_pose_search.json (full diagnostic) is written')
    parser.add_argument('--write-candidates', type=Path, default=None, metavar='PATH',
                        help='also write the per-kind choice lists in the cutlery_candidates.json schema')
    args = parser.parse_args(argv)

    basket = PARAMETERS['silverware_basket']
    centres = basket['compartment_centres_xy']
    floor_top_z = basket['floor_lattice']['floor_top_z']
    xs, ys = cell_offsets(basket)
    print('basket', basket['geometry_revision'], 'layout', basket['compartment_layout'],
          'centres', [[round(v, 5) for v in c] for c in centres],
          'cell offsets x', [round(v, 5) for v in xs], 'y', [round(v, 5) for v in ys],
          'floor_top_z', floor_top_z, flush=True)

    world = CollisionWorld(ASSET_DIR)
    stage = Usd.Stage.Open(str(ASSET_DIR / 'silverware_basket.usdc'))
    floor_parts, wall_parts = [], []
    for p in collision_parts(ASSET_DIR / 'silverware_basket.usdc'):
        family = stage.GetPrimAtPath(p.name).GetAttribute('wireFamily').Get() or ''
        (floor_parts if family.startswith(('BottomCrossRib', 'BottomLongRib')) else wall_parts).append(p)
    managers, references = {}, {}
    for name, parts in [('floor', floor_parts), ('wall', wall_parts)]:
        objs = world.transform(parts, np.eye(3), BODY_POSITIONS['SilverwareBasket'])
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(objs)
        manager.setup()
        managers[name], references[name] = manager, objs
    print('basket parts: floor', len(floor_parts), 'wall', len(wall_parts), flush=True)

    results = {}
    for comp, kind in enumerate(KINDS):
        cx, cy = (float(v) for v in centres[comp])
        choices = []
        for yaw in (90, 270, 0, 180):
            for axis, angle in TILTS:
                orient = rotation(axis, math.radians(angle)) @ rotation('Z', math.radians(yaw)) @ rotation('X', math.pi)
                pts = world.points[kind] @ orient.T
                bottom = pts[pts[:, 2] < pts[:, 2].min() + .0015, :2].mean(0)
                z = float(floor_top_z + HEAD_ABOVE_FLOOR - pts[:, 2].min())
                for ix, x in enumerate(xs):
                    for iy, y in enumerate(ys):
                        for sx, sy in SHIFTS:
                            anchor = (cx + x + sx, cy + y + sy)
                            position = [float(anchor[0] - bottom[0]), float(anchor[1] - bottom[1]), z]
                            c = {'kind': kind, 'rack': 'SilverwareBasket', 'slot': f'cutlery_{comp}_{ix}_{iy}',
                                 'variant': f'yaw{yaw}_{axis}{angle}_shift{sx}_{sy}', 'position': position,
                                 'quaternion_xyzw': quaternion_xyzw(orient),
                                 'head_anchor_xy': [float(anchor[0]), float(anchor[1])]}
                            objs = world.candidate_objects(c)
                            if world.collides(objs):
                                continue
                            lower = dict(c, position=[position[0], position[1], position[2] - .004])
                            if not hits(managers['floor'], world.candidate_objects(lower)):
                                continue
                            # Nearby upper-wall support is useful for a leaning shaft.
                            # The wall test excludes all actual floor ribs.
                            near = False
                            for dx, dy in [(-.0025, 0), (.0025, 0), (0, -.0025), (0, .0025)]:
                                shifted = dict(c, position=[position[0] + dx, position[1] + dy, position[2]])
                                if hits(managers['wall'], world.candidate_objects(shifted)):
                                    near = True
                                    break
                            c['wall_within_2p5mm'] = near
                            choices.append(c)
            print(kind, 'yaw', yaw, 'valid_so_far', len(choices), flush=True)
        # Prefer near-wall lean, then narrow-X spoon orientations, then small shifts.
        choices.sort(key=lambda c: (not c['wall_within_2p5mm'],
                                    '_X0_' in c['variant'], not c['variant'].startswith('yaw90_')))
        world.reset_load([])
        used, accepted = set(), []
        for c in choices:
            if c['slot'] in used or world.collides(world.candidate_objects(c)):
                continue
            world.add(c)
            used.add(c['slot'])
            accepted.append(c)
        results[kind] = {'free_with_floor_support': len(choices),
                         'with_near_wall': sum(c['wall_within_2p5mm'] for c in choices),
                         'greedy_pattern': accepted, 'all_choices': choices}
        print('CUTLERY_RESULT', kind, 'free', len(choices), 'nearwall', results[kind]['with_near_wall'],
              'simultaneous', len(accepted), flush=True)
        world.reset_load([])

    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    (out / 'cutlery_pose_search.json').write_text(json.dumps(results, indent=2))
    print('wrote', out / 'cutlery_pose_search.json', flush=True)

    if args.write_candidates is not None:
        # Same top-level schema as the shipped file (physics_refinements dropped, nothing else);
        # entries carry exactly the shipped entry keys, so the helper keys above are stripped.
        candidates = json.loads(CANONICAL.read_text()) if CANONICAL.exists() else {'schema_version': 1}
        candidates.pop('physics_refinements', None)
        candidates['basis'] = {'basket_geometry_revision': basket['geometry_revision'],
                               'generator': Path(__file__).name,
                               'note': 'regenerated for the tape-measured basket; no physics refinements'}
        candidates['patterns'] = {kind: [{key: c[key] for key in CANDIDATE_KEYS} for c in results[kind]['all_choices']]
                                  for kind in KINDS}
        args.write_candidates.parent.mkdir(parents=True, exist_ok=True)
        args.write_candidates.write_text(json.dumps(candidates, indent=2) + '\n')
        print('wrote', args.write_candidates, {k: len(v) for k, v in candidates['patterns'].items()}, flush=True)

    simultaneous = {kind: len(results[kind]['greedy_pattern']) for kind in KINDS}
    ok = all(n >= MIN_SIMULTANEOUS for n in simultaneous.values())
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} simultaneous {simultaneous} (need >= {MIN_SIMULTANEOUS} per kind)", flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
