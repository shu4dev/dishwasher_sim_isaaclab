"""Qualify exact high-wall contacts for the cutlery candidate diagnostic."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.asset import BODY_POSITIONS
from dishsim_frigidaire.loading import CollisionWorld,collision_parts
from pxr import Usd
import numpy as np
import fcl

out=Path(__file__).resolve().parents[3] / "build/frigidaire_diagnostics"
raw=json.loads((out/'cutlery_pose_search.json').read_text())
world=CollisionWorld(ASSET_DIR)
stage=Usd.Stage.Open(str(ASSET_DIR/'silverware_basket.usdc'))
parts=[]
for p in collision_parts(ASSET_DIR/'silverware_basket.usdc'):
    family=stage.GetPrimAtPath(p.name).GetAttribute('wireFamily').Get() or ''
    if not family.startswith(('BottomCrossRib','BottomLongRib')):parts.append(p)
objects=world.transform(parts,np.eye(3),BODY_POSITIONS['SilverwareBasket'])
manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects(objects);manager.setup()
result={}
for kind,info in raw.items():
    choices=info['all_choices']
    for c in choices:
        max_z=0.
        if c['wall_within_2p5mm']:
            p=c['position']
            for dx,dy in [(-.0025,0),(.0025,0),(0,-.0025),(0,.0025)]:
                shifted=dict(c,position=[p[0]+dx,p[1]+dy,p[2]])
                data=fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=64,enable_contact=True))
                for obj in world.candidate_objects(shifted):manager.collide(obj,data,fcl.defaultCollisionCallback)
                if data.result.contacts:
                    max_z=max(max_z,max(float(contact.pos[2])-BODY_POSITIONS['SilverwareBasket'][2] for contact in data.result.contacts))
        c['maximum_near_wall_contact_height_m']=max_z
        c['upper_wall_braced']=max_z>.060
    choices.sort(key=lambda c:(not c['upper_wall_braced'], '_X0_' in c['variant'],not c['variant'].startswith('yaw90_')))
    # A family needs a reproducible order, not a different tune for each count.
    world.reset_load([]);accepted=[];used=set()
    for c in choices:
        if c['slot'] in used or world.collides(world.candidate_objects(c)):continue
        world.add(c);used.add(c['slot']);accepted.append(c)
    print(kind,'upper_braced_choices',sum(c['upper_wall_braced'] for c in choices),
          'simultaneous',len(accepted),'braced_accepted',sum(c['upper_wall_braced'] for c in accepted),flush=True)
    result[kind]={'accepted':accepted,'ranked_choices':choices}
    world.reset_load([])
(out/'cutlery_supported_patterns.json').write_text(json.dumps(result,indent=2))
world.reset_load([]);used=set();combined=[]
for _ in range(9):
    for kind,info in result.items():
        for c in info['ranked_choices']:
            if c['slot'] in used or world.collides(world.candidate_objects(c)):continue
            world.add(c);used.add(c['slot']);combined.append(c)
            break
print('COMBINED',{kind:sum(c['kind']==kind for c in combined) for kind in result},flush=True)
(out/'cutlery_combined_pattern.json').write_text(json.dumps(combined,indent=2))
