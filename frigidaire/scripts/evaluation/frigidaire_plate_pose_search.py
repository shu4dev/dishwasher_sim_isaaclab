"""Kit-free diagnostic for full-size tableware pose seeds against authored colliders."""
import sys, math, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.loading import CollisionWorld, _candidate, rotation
import numpy as np

world=CollisionWorld(ASSET_DIR)
found={}
for kind,axis,rack,x,y,floors in [
    ('dinner_plate','Y','LowerRack',-.16129,-.140,[.004,.006,.008,.012]),
    ('salad_plate','Y','LowerRack',-.16129,-.140,[.004,.006,.008,.012]),
    ('saucer','X','UpperRack',0.,-.09525,[-.0081,-.006,-.002,.002,.006,.012]),
]:
    free=[]
    for floor in floors:
        for lean in [-12,-8,-4,0,4,8,12]:
            orient=rotation(axis,math.radians(90-lean))
            for offset in np.arange(-.018,.01801,.002):
                c=_candidate(kind,rack,'probe',x+(offset if axis=='Y' else 0),
                             y+(offset if axis=='X' else 0),orient,world.points[kind],floor,
                             f'lean{lean}_offset{offset:.4f}_floor{floor:.4f}')
                if not world.collides(world.candidate_objects(c)):
                    free.append(c)
        print(kind,'floor',floor,'cumulativefree',len(free),flush=True)
    found[kind]=free
    print(kind,'free',len(free),'examples',free[:12],flush=True)
out=Path(__file__).resolve().parents[3] / "build/frigidaire_diagnostics"
out.mkdir(parents=True, exist_ok=True)
(out/'plate_pose_search.json').write_text(json.dumps(found,indent=2))

patterns=[]
teeth=(np.arange(16)-7.5)*.032258
mids=(teeth[:-1]+teeth[1:])/2
for kind,offset in [('dinner_plate',.008),('salad_plate',.004)]:
    for bank,y,slots in [('front',-.140,mids),('rear',.140,mids[:11])]:
        for i,x in enumerate(slots):
            if bank=='front' and x>.105:
                continue
            c=_candidate(kind,'LowerRack',f'{bank}_{i}',x+offset,y,
                         rotation('Y',math.radians(94)),world.points[kind],.006,'lean-4')
            c['static_free']=not world.collides(world.candidate_objects(c))
            patterns.append(c)
for i,y in enumerate(np.arange(-.22225,.22226,.03175)):
    c=_candidate('saucer','UpperRack',f'upper_{i}',0,y+.010,
                 rotation('X',math.radians(82)),world.points['saucer'],-.0081,'lean8')
    c['static_free']=not world.collides(world.candidate_objects(c))
    patterns.append(c)
for kind in ['dinner_plate','salad_plate','saucer']:
    rows=[c for c in patterns if c['kind']==kind]
    print('PATTERN',kind,'static_free',sum(c['static_free'] for c in rows),'of',len(rows),flush=True)
    world.reset_load([])
    accepted=[]
    for c in rows:
        if not world.collides(world.candidate_objects(c)):
            accepted.append(c);world.add(c)
    print('PATTERN',kind,'simultaneously_free',len(accepted),flush=True)
    world.reset_load([])
(out/'plate_patterns.json').write_text(json.dumps(patterns,indent=2))
