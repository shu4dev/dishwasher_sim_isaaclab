"""Preflight an observed stable teaspoon pose against the original complete load."""
import sys,json,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, IMAGE_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.loading import CollisionWorld
import numpy as np
from scipy.spatial.transform import Rotation

report_path=IMAGE_DIR / "full_load/physics.json"
manifest_path=ASSET_DIR/'full_load_manifest.json'
report=json.loads(report_path.read_text());manifest=json.loads(manifest_path.read_text())
state=report['states']['final_extended'];record=state['objects']['teaspoon_002'];basket=state['basket']
def rot(record):
    w,x,y,z=record['quaternion_wxyz'];return Rotation.from_quat([x,y,z,w])
local_q=(rot(basket).inv()*rot(record)).as_quat()
calculated=rot(basket).inv().apply(np.asarray(record['position_m'])-basket['position_m'])
position=np.asarray(record['rack_relative_position_m'])
assert np.max(np.abs(calculated-position))<1e-6
old=next(c for c in manifest['objects'] if c['id']=='teaspoon_002')
world=CollisionWorld(ASSET_DIR)
world.reset_load([c for c in manifest['objects'] if c['id']!='teaspoon_002'])
trials=[];clear=[]
for dz in [.001,.002,.003]:
    for dx,dy in [(0.,0.),(.001,0.),(-.001,0.),(0.,.001),(0.,-.001),
                  (.002,0.),(-.002,0.),(0.,.002),(0.,-.002)]:
        candidate={**old,'variant':f'observed_final_rest_lift{dz:g}_dx{dx:g}_dy{dy:g}',
                   'position':(position+[dx,dy,dz]).tolist(),'quaternion_xyzw':local_q.tolist()}
        candidate['candidate_key']=f"{candidate['kind']}:{candidate['slot']}:{candidate['variant']}"
        collision=world.collides(world.candidate_objects(candidate))
        trials.append({'lift_m':dz,'dx_m':dx,'dy_m':dy,'collision':collision})
        if not collision:clear.append(candidate)
    if clear:break
result={'purpose':'Candidate pose correction; this is not physics validation',
        'failed_report_sha256':hashlib.sha256(report_path.read_bytes()).hexdigest(),
        'manifest_sha256':hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        'other_initial_objects':len(manifest['objects'])-1,
        'measured_basket_local_position':position.tolist(),'measured_basket_local_quaternion_xyzw':local_q.tolist(),
        'local_position_reconstruction_error_m':float(np.max(np.abs(calculated-position))),
        'trials':trials,'clear_candidates':clear,'selected_candidate':clear[0] if clear else None}
out=Path(__file__).resolve().parents[3] / "build/frigidaire_diagnostics/teaspoon_rest_preflight.json"
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2),flush=True)
