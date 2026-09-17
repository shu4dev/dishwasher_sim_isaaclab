"""Release the four observed CCD-settled teaspoons with at most2mm translation."""
import sys,json,hashlib,itertools
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, IMAGE_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.asset import BODY_POSITIONS
from dishsim_frigidaire.loading import CollisionWorld,collision_parts
from scipy.spatial.transform import Rotation
from pxr import Usd
import numpy as np
import fcl

rp=IMAGE_DIR / "full_load/physics.json"
mp=ASSET_DIR/'full_load_manifest.json'
report_text=rp.read_text();manifest_text=mp.read_text()
report=json.loads(report_text);manifest=json.loads(manifest_text)
state=report['states']['loaded_closed'];basket=state['basket']
def quat(rec):
    w,x,y,z=rec['quaternion_wxyz'];return Rotation.from_quat([x,y,z,w])
basket_rot=quat(basket)
originals=[c for c in manifest['objects'] if c['kind']=='teaspoon']
others=[c for c in manifest['objects'] if c['kind']!='teaspoon']
world=CollisionWorld(ASSET_DIR);world.reset_load(others)
bases=[]
for old in originals:
    rec=state['objects'][old['id']]
    local=(basket_rot.inv()*quat(rec)).as_quat()
    p=np.asarray(rec['rack_relative_position_m'])
    check=basket_rot.inv().apply(np.asarray(rec['position_m'])-basket['position_m'])
    assert np.max(np.abs(p-check))<1e-6
    bases.append({**old,'position':p.tolist(),'quaternion_xyzw':local.tolist()})

def make(base,shift):
    dx,dy,dz=shift
    c={**base,'variant':f'observed_ccd_closed_dx{dx:g}_dy{dy:g}_dz{dz:g}',
       'position':(np.asarray(base['position'])+shift).tolist(),
       'observed_release_offset_m':list(shift)}
    c['candidate_key']=f"{c['kind']}:{c['slot']}:{c['variant']}"
    return c

def pair_hit(a,b):
    manager=fcl.DynamicAABBTreeCollisionManager();manager.registerObjects(a);manager.setup()
    data=fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
    for obj in b:
        manager.collide(obj,data,fcl.defaultCollisionCallback)
        if data.result.is_collision:return True
    return False

def common_free(candidates):
    objects=[world.candidate_objects(c) for c in candidates]
    if any(world.collides(o) for o in objects):return False
    return not any(pair_hit(objects[i],objects[j]) for i in range(4) for j in range(i))

common_trials=[];selected=None
for dz in (0.,.00025,.0005,.00075,.001,.0015,.002):
    proposal=[make(c,(0.,0.,dz)) for c in bases]
    clear=common_free(proposal)
    common_trials.append({'lift_m':dz,'all_clear':clear})
    print('COMMON',dz,clear,flush=True)
    if clear:selected=proposal;break

options=[];counts=[]
if selected is None:
    shifts=[(x,y,z) for x,y,z in itertools.product(np.arange(-.002,.00201,.0005),
              np.arange(-.002,.00201,.0005),np.arange(0,.00201,.0005))
              if x*x+y*y+z*z<=.002**2+1e-15]
    shifts.sort(key=lambda v:(np.linalg.norm(v),abs(v[0])+abs(v[1]),-v[2]))
    for base in bases:
        free=[]
        for shift in shifts:
            c=make(base,shift);o=world.candidate_objects(c)
            if not world.collides(o):free.append((c,o))
        options.append(free);counts.append(len(free))
        print('STATIC_FREE',base['id'],len(free),flush=True)
    # Minimum largest translation first; tie-break total release distance.
    for cap in sorted(set(round(float(np.linalg.norm(c['observed_release_offset_m'])),10)
                      for choices in options for c,_ in choices)):
        limited=[[(c,o) for c,o in choices if np.linalg.norm(c['observed_release_offset_m'])<=cap+1e-9]
                 for choices in options]
        if any(not c for c in limited):continue
        order=sorted(range(4),key=lambda i:len(limited[i]));picked={}
        def search(depth):
            if depth==4:return True
            index=order[depth]
            for c,o in limited[index]:
                if any(pair_hit(o,p[1]) for p in picked.values()):continue
                picked[index]=(c,o)
                if search(depth+1):return True
                del picked[index]
            return False
        if search(0):
            selected=[picked[i][0] for i in range(4)];break

floor=[]
stage=Usd.Stage.Open(str(ASSET_DIR/'silverware_basket.usdc'))
for p in collision_parts(ASSET_DIR/'silverware_basket.usdc'):
    family=stage.GetPrimAtPath(p.name).GetAttribute('wireFamily').Get() or ''
    if family.startswith(('BottomCrossRib','BottomLongRib')):floor.append(p)
floor_objects=world.transform(floor,np.eye(3),BODY_POSITIONS['SilverwareBasket'])
floor_gaps={}
if selected:
    assert common_free(selected)
    for c in selected:
        objects=world.candidate_objects(c)
        minimum=float('inf')
        for a in floor_objects:
            for b in objects:
                distance=fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult())
                minimum=min(minimum,float(distance))
        low,high=0.,.008
        def floor_hit(drop):
            p=np.asarray(c['position'])-[0,0,drop]
            return pair_hit(floor_objects,world.candidate_objects(dict(c,position=p.tolist())))
        if floor_hit(high):
            for _ in range(15):
                mid=(low+high)/2
                if floor_hit(mid):high=mid
                else:low=mid
            drop=high
        else:drop=None
        floor_gaps[c['id']]={'minimum_euclidean_floor_gap_m':minimum,'vertical_drop_to_first_floor_contact_m':drop}
        print('FLOOR',c['id'],floor_gaps[c['id']],flush=True)

result={'purpose':'Observed four-spoon arrangement seed; not physics validation',
        'observation_state':'loaded_closed','report_sha256':hashlib.sha256(report_text.encode()).hexdigest(),
        'manifest_sha256':hashlib.sha256(manifest_text.encode()).hexdigest(),
        'other_initial_objects':len(others),'common_trials':common_trials,'static_free_option_counts':counts,
        'all_appliance_other_initial_and_mutual_collisions_clear':selected is not None,
        'maximum_translation_norm_m':max(np.linalg.norm(c['observed_release_offset_m']) for c in selected) if selected else None,
        'candidates':selected,'floor_gaps':floor_gaps}
out=Path('outputs/frigidaire_v2_diagnostics/teaspoon_group_preflight.json')
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2),flush=True)
