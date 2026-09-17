"""Exercise the export function without Kit; synthetic poses are NOT validation."""
import ast
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd
ensure_usd()
from dishsim_frigidaire.asset import BODY_POSITIONS
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

scratch=ROOT/'build/frigidaire_diagnostics/export_preflight'
copied=scratch/'asset'
copied.mkdir(parents=True,exist_ok=True)
for path in ASSET_DIR.rglob('*'):
    if not path.is_file() or path.suffix not in ('.usda','.usdc','.usd','.json','.md'):
        continue
    dest=copied/path.relative_to(ASSET_DIR)
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(path,dest)

manifest=json.loads((copied/'full_load_manifest.json').read_text())
# One object per type is sufficient to exercise each portable reference.
seen=set();selected=[]
for entry in manifest['objects']:
    if entry['kind'] not in seen:
        selected.append(entry);seen.add(entry['kind'])
manifest={**manifest,'objects':selected,'preflight_only':True}
state={'appliance_bodies':{},'objects':{}}
for body,position in BODY_POSITIONS.items():
    state['appliance_bodies'][body]={'position_m':list(position),'quaternion_wxyz':[1.,0.,0.,0.]}
state['basket']=dict(state['appliance_bodies']['SilverwareBasket'])
for entry in selected:
    q=entry['quaternion_xyzw']
    state['objects'][entry['id']]={
        'position_m':[x+y for x,y in zip(entry['position'],BODY_POSITIONS[entry['rack']])],
        'quaternion_wxyz':[q[3],*q[:3]],
    }
report={'states':{'loaded_closed':state},'manifest_sha256':'SYNTHETIC_PREFLIGHT_NOT_VALIDATED',
        'validated_counts':None,'scripted_drives':{
            'door_hinge':{'stiffness':300.,'damping':45.,'effort_limit':30.,'velocity_limit':.65},
            'lower_slide':{'stiffness':1600.,'damping':180.,'effort_limit':60.,'velocity_limit':.15},
            'upper_slide':{'stiffness':1600.,'damping':180.,'effort_limit':60.,'velocity_limit':.15},
        }}
source=ROOT/'frigidaire/scripts/evaluation/frigidaire_full_load_evidence.py'
text=source.read_text()
node=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name=='export_scene')

def diagnostic_json_write(path,data):
    # The production export assumes its caller passed validated data. Override
    # only the test artifact's claim, never the exported function's USD calls.
    data={**data,'physics_result':'NOT_VALIDATED_PREFLIGHT_ONLY','validated_counts':None}
    path.write_text(json.dumps(data,indent=2)+'\n')

namespace={'args':SimpleNamespace(usd=copied/'fdpc4221as.usdc'),'HZ':120,'math':math,
           'Gf':Gf,'Sdf':Sdf,'Usd':Usd,'UsdGeom':UsdGeom,'UsdPhysics':UsdPhysics,
           'json_write':diagnostic_json_write}
exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),namespace)
namespace['export_scene'](report,manifest)
path=copied/'full_load.usda'
stage=Usd.Stage.Open(str(path))
default=stage.GetDefaultPrim()
data=dict(default.GetCustomData());data.pop('validated_object_count',None)
data['validation_status']='NOT VALIDATED: synthetic export preflight only'
data['diagnostic_object_count']=len(selected)
default.SetCustomData(data);stage.GetRootLayer().Save()

layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(path))
assert not unresolved,unresolved
absolute=[];bad_relationships=[]
for prim in stage.Traverse():
    for spec in prim.GetPrimStack():
        for ref in spec.referenceList.GetAddedOrExplicitItems():
            if ref.assetPath and Path(ref.assetPath).is_absolute():absolute.append(ref.assetPath)
    for rel in prim.GetRelationships():
        for target in rel.GetTargets():
            if target.IsPrimPath() and not stage.GetPrimAtPath(target):bad_relationships.append(str(target))
assert not absolute,absolute
assert not bad_relationships,bad_relationships
scene=stage.GetPrimAtPath('/LoadedFrigidaire/Physics')
assert scene.GetAttribute('physxScene:enableGPUDynamics').Get() is False
assert scene.GetAttribute('physxScene:timeStepsPerSecond').Get()==120
assert scene.GetAttribute('physxScene:broadphaseType').Get()=='MBP'
assert scene.GetAttribute('physxScene:solverType').Get()=='TGS'
assert UsdGeom.GetStageMetersPerUnit(stage)==1
assert UsdGeom.GetStageUpAxis(stage)=='Z'
joint_measurements={}
for name,values in report['scripted_drives'].items():
    prim=stage.GetPrimAtPath('/LoadedFrigidaire/Appliance/Joints/'+name)
    drive=UsdPhysics.DriveAPI(prim,'angular' if name=='door_hinge' else 'linear')
    scale=math.pi/180 if name=='door_hinge' else 1.
    assert math.isclose(drive.GetStiffnessAttr().Get(),values['stiffness']*scale,rel_tol=1e-6)
    assert math.isclose(drive.GetDampingAttr().Get(),values['damping']*scale,rel_tol=1e-6)
    assert drive.GetMaxForceAttr().Get()==values['effort_limit']
    joint_measurements[name]={'stiffness':drive.GetStiffnessAttr().Get(),'damping':drive.GetDampingAttr().Get(),
        'velocity_limit':prim.GetAttribute('physxJoint:maxJointVelocity').Get()}
summary={'status':'EXPORT_API_AND_COMPOSITION_CHECKS_COMPLETED','physics_validation':'NOT PERFORMED',
         'export_source_sha256':hashlib.sha256(text.encode()).hexdigest(),'default_prim':str(default.GetPath()),
         'synthetic_object_count':len(selected),'unresolved_dependencies':unresolved,
         'absolute_references':absolute,'invalid_relationship_targets':bad_relationships,
         'referenced_layers':len(layers),'joint_drives':joint_measurements,'output':str(path)}
(scratch/'preflight.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
