"""Kit-free cutlery orientation and basket-support diagnostic."""
import sys, json, math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.asset import BODY_POSITIONS
from dishsim_frigidaire.loading import CollisionWorld, collision_parts, rotation, quaternion_xyzw
import numpy as np
import fcl
from pxr import Usd

world=CollisionWorld(ASSET_DIR)
stage=Usd.Stage.Open(str(ASSET_DIR/'silverware_basket.usdc'))
floor_parts=[];wall_parts=[]
for p in collision_parts(ASSET_DIR/'silverware_basket.usdc'):
    family=stage.GetPrimAtPath(p.name).GetAttribute('wireFamily').Get() or ''
    (floor_parts if family.startswith(('BottomCrossRib','BottomLongRib')) else wall_parts).append(p)
managers={};references={}
for name,parts in [('floor',floor_parts),('wall',wall_parts)]:
    objs=world.transform(parts,np.eye(3),BODY_POSITIONS['SilverwareBasket'])
    manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects(objs);manager.setup()
    managers[name]=manager;references[name]=objs

def hits(manager,objects):
    data=fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
    for obj in objects:
        manager.collide(obj,data,fcl.defaultCollisionCallback)
        if data.result.is_collision:return True
    return False

results={}
for comp,kind in enumerate(('fork','knife','tablespoon','teaspoon')):
    center=(-.1115625,-.0375,.0375,.1115625)[comp]
    choices=[]
    for yaw in (90,270,0,180):
        for axis,angle in [('X',0),('X',-4),('X',4),('X',-8),('X',8),('Y',-4),('Y',4),('Y',-8),('Y',8)]:
            orient=rotation(axis,math.radians(angle))@rotation('Z',math.radians(yaw))@rotation('X',math.pi)
            pts=world.points[kind]@orient.T
            bottom=pts[pts[:,2]<pts[:,2].min()+.0015,:2].mean(0)
            for ix,x in enumerate((-.021,0.,.021)):
                for iy,y in enumerate((-.018,0.,.018)):
                    for sx,sy in [(0,0),(.002,0),(-.002,0),(0,.002),(0,-.002)]:
                        position=[float(x+sx-bottom[0]),float(center+y+sy-bottom[1]),float(.0067-pts[:,2].min())]
                        c={'kind':kind,'rack':'SilverwareBasket','slot':f'cutlery_{comp}_{ix}_{iy}',
                           'variant':f'yaw{yaw}_{axis}{angle}_shift{sx}_{sy}','position':position,
                           'quaternion_xyzw':quaternion_xyzw(orient),'head_anchor_xy':[x+sx,center+y+sy]}
                        objs=world.candidate_objects(c)
                        if world.collides(objs):continue
                        lower=dict(c,position=[position[0],position[1],position[2]-.004])
                        if not hits(managers['floor'],world.candidate_objects(lower)):continue
                        # Nearby upper-wall support is useful for a leaning shaft.
                        # The wall test excludes all actual floor ribs.
                        near=False
                        for dx,dy in [(-.0025,0),(.0025,0),(0,-.0025),(0,.0025)]:
                            shifted=dict(c,position=[position[0]+dx,position[1]+dy,position[2]])
                            if hits(managers['wall'],world.candidate_objects(shifted)):
                                near=True;break
                        c['wall_within_2p5mm']=near
                        choices.append(c)
        print(kind,'yaw',yaw,'valid_so_far',len(choices),flush=True)
    # Prefer near-wall lean, then narrow-X spoon orientations, then small shifts.
    choices.sort(key=lambda c:(not c['wall_within_2p5mm'],
                  '_X0_' in c['variant'],not c['variant'].startswith('yaw90_')))
    world.reset_load([]);used=set();accepted=[]
    for c in choices:
        if c['slot'] in used or world.collides(world.candidate_objects(c)):continue
        world.add(c);used.add(c['slot']);accepted.append(c)
    results[kind]={'free_with_floor_support':len(choices),'with_near_wall':sum(c['wall_within_2p5mm'] for c in choices),
                   'greedy_pattern':accepted,'all_choices':choices}
    print('CUTLERY_RESULT',kind,'free',len(choices),'nearwall',results[kind]['with_near_wall'],'simultaneous',len(accepted),flush=True)
    world.reset_load([])
out=Path(__file__).resolve().parents[3] / "build/frigidaire_diagnostics";out.mkdir(parents=True, exist_ok=True)
(out/'cutlery_pose_search.json').write_text(json.dumps(results,indent=2))
