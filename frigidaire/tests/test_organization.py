"""Organization gates, semantic symmetry and exact finite-inventory constraints."""
import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from dishsim_frigidaire.organization import OrganizationGeometry, evaluate_organization, orientation_metrics, ray_blocked
from dishsim_frigidaire.organized_candidates import solve_inventory, _patterns, pair_compatible


def pose(p=(0,0,0),q=(1,0,0,0)):
    return {'position_m':list(p),'quaternion_xyzw':list(q)}


def obj(oid='m',kind='mug',p=(0,0,0),q=(1,0,0,0),rack='UpperRack'):
    return {'object_id':oid,'kind':kind,'rack':rack,'pose_world':pose(p,q)}


@pytest.fixture(scope='module')
def geometry():return OrganizationGeometry()


@pytest.mark.parametrize('kind,limit',[('mug',45),('bowl',75)])
def test_downward_orientation_boundary_and_quaternion_sign(kind,limit):
    for delta,valid in [(-.001,True),(0,True),(.001,False)]:
        q=Rotation.from_euler('x',180-limit-delta,degrees=True).as_quat()
        assert orientation_metrics(kind,'UpperRack',pose(q=q))['valid']==valid
        assert orientation_metrics(kind,'UpperRack',pose(q=-q))['valid']==valid


def test_plate_normal_symmetry_and_rack():
    for angle in (75,90,105,-75,-90,-105):
        q=Rotation.from_euler('y',angle,degrees=True).as_quat()
        assert orientation_metrics('dinner_plate','LowerRack',pose(q=q))['valid']
        assert not orientation_metrics('dinner_plate','UpperRack',pose(q=q))['valid']
    assert not orientation_metrics('dinner_plate','LowerRack',pose(q=Rotation.from_euler('y',74.99,degrees=True).as_quat()))['valid']


def test_missing_or_indirect_support_fails(geometry):
    vessel=obj()
    assert not evaluate_organization([vessel],geometry=geometry)['valid']
    assert not evaluate_organization([vessel],geometry=geometry,direct_support={'m':False})['valid']
    assert evaluate_organization([vessel],geometry=geometry,direct_support={'m':True})['valid']


def test_exact_spacing_and_handle_geometry(geometry):
    a=obj('a'); b=obj('b',p=(0,.09,0))
    result=evaluate_organization([a,b],geometry=geometry,direct_support={'a':True,'b':True})
    assert result['valid']
    assert result['separation']['minimum_certified_clearance_m']==pytest.approx(.005,abs=1e-6)
    b['pose_world']['position_m'][1]=.089
    result=evaluate_organization([a,b],geometry=geometry,direct_support={'a':True,'b':True})
    assert any(v['rule']=='separation' for v in result['violations'])
    # Center distance alone cannot ignore a handle extending along X.
    b['pose_world']['position_m']=[.11,0,0]
    assert not pair_compatible(geometry,geometry.body('mug',a['pose_world']),geometry.body('mug',b['pose_world']))[0]


def test_cavity_detects_nested_shell_without_surface_contact(geometry):
    bowl=obj('bowl','bowl',q=(0,0,0,1))
    # A 20 mm elevated identical bowl can fit partly in a cavity regardless of orientation gate.
    other=obj('intruder','bowl',p=(0,0,.02),q=(0,0,0,1))
    a=geometry.body('bowl',bowl['pose_world']); b=geometry.body('bowl',other['pose_world'])
    assert geometry.nested(a,b)
    far=geometry.body('bowl',pose((.4,0,0),(0,0,0,1)))
    assert not geometry.nested(a,far)


def test_opening_exposure_detects_noncontact_plate(geometry):
    mug=obj('mug',p=(0,0,.2))
    plate=obj('plate','dinner_plate',p=(0,0,.09),q=(0,0,0,1),rack='LowerRack')
    result=evaluate_organization([mug,plate],geometry=geometry,direct_support={'mug':True,'plate':True})
    assert result['per_object']['mug']['opening_exposure']['unobstructed_rays']==0
    assert any(v['rule']=='opening_exposure' for v in result['violations'])
    plate['pose_world']['position_m'][2]=-.01
    result=evaluate_organization([mug,plate],geometry=geometry,direct_support={'mug':True,'plate':True})
    assert result['per_object']['mug']['opening_exposure']['unobstructed_rays']==64


def test_ray_is_two_sided_and_finite():
    triangles=np.array([[[-1,-1,.05],[1,-1,.05],[0,1,.05]]],float)
    origins=np.array([[0,0,0],[2,0,0]],float)
    assert ray_blocked(origins,np.array([0,0,1]),triangles,.1).tolist()==[True,False]
    assert ray_blocked(origins,np.array([0,0,1]),triangles[:,::-1],.1).tolist()==[True,False]
    assert not ray_blocked(origins,np.array([0,0,1]),triangles,.04).any()


def catalog(kinds):
    return {'candidates':[{'kind':kind,'rack':'UpperRack' if i%2==0 else 'LowerRack',
       'position_xy_m':[0,i*.1],'quaternion_xyzw':[1,0,0,0],
       'row_metadata':{'row_id':kind}} for i,kind in enumerate(kinds)]}


def graph(n,edges=()):return {'complete':True,'allowed_indices':list(range(n)),'conflict_pairs':list(edges)}


def test_milp_preserves_exact_kinds_and_upper_mug_preference():
    c=catalog(['mug','mug','bowl','bowl','mug'])
    r=solve_inventory(c,graph(5),{'mug':2,'bowl':1},time_limit_s=2)
    assert r['count']==3
    assert 0 in r['selected_indices'] and 4 in r['selected_indices']
    r2=solve_inventory(c,graph(5),{'mug':2,'bowl':1},time_limit_s=2,excluded_sets=[r['selected_indices']])
    assert r2['selected_indices']!=r['selected_indices']
    assert r2['count']==3


def test_milp_reports_finite_infeasibility_without_reducing_inventory():
    c=catalog(['mug','bowl'])
    r=solve_inventory(c,graph(2),{'mug':2,'bowl':1},time_limit_s=2)
    assert r['selected_indices'] is None
    assert r['geometric_feasibility']=='infeasible_finite_catalog'
    r=solve_inventory(c,graph(2,[(0,1)]),{'mug':1,'bowl':1},time_limit_s=2)
    assert r['selected_indices'] is None


def test_pattern_families_use_required_semantic_directions_and_refinements():
    base=_patterns(0); refined=_patterns(1)
    assert len(refined)>len(base)
    assert all(orientation_metrics(p['kind'],p['rack'],pose(q=p['quaternion_xyzw']))['valid'] for p in base)
    assert set(p['kind'] for p in base)=={'dinner_plate','bowl','mug'}
    assert set(p['generation_tier'] for p in refined)=={0,1}


def test_collision_bounds_include_capsule_envelope_at_rotated_pose(geometry):
    from dishsim_frigidaire.tableware import tableware_geometry
    data=tableware_geometry('mug')
    q=Rotation.from_euler('xyz',[23,41,17],degrees=True).as_quat()
    r=Rotation.from_quat(q).as_matrix();p=np.array([.13,-.2,.45])
    body=geometry.body('mug',pose(p,q))
    for a,b,radius in data['capsules']:
        for endpoint in (a,b):
            center=r@endpoint+p
            assert np.all(center-radius>=body['bounds'][0]-1e-12)
            assert np.all(center+radius<=body['bounds'][1]+1e-12)


def test_catalog_resume_does_not_repeat_processed_support_search(monkeypatch):
    import dishsim_frigidaire.organized_candidates as module
    from copy import deepcopy
    class Checker:
        asset_sha256={'asset':'same'}
        def __init__(self,*args):pass
        def update_components(self,*args):pass
    class Geometry:
        vertices={'mug':np.array([[0.,0.,0.]])}
        def __init__(self,*args,**kwargs):pass
    def patterns(level):
        return [{'kind':'mug','rack':'UpperRack','slot_id':f'slot{i}',
                 'position_xy_m':[i*.02,0.],'quaternion_xyzw':[1.,0.,0.,0.],
                 'generation_tier':int(i>=3),'row_metadata':{'row_id':'same'}}
                for i in range(3+level)]
    calls=[]
    def support(checker,geometry,pattern,frames):
        calls.append(pattern['slot_id'])
        return (pose((*pattern['position_xy_m'],.1)),pose((*pattern['position_xy_m'],.1))), {'reason':None}
    monkeypatch.setattr(module,'InitialCollisionChecker',Checker)
    monkeypatch.setattr(module,'OrganizationGeometry',Geometry)
    monkeypatch.setattr(module,'_patterns',patterns)
    monkeypatch.setattr(module,'_support_pose',support)
    frames={name:pose(q=(0,0,0,1)) for name in ('Cabinet','Door','LowerRack','UpperRack','SilverwareBasket')}
    first=module.generate_catalog('unused',frames,max_candidates=2)
    original=deepcopy(first)
    assert calls==['slot0','slot1'] and first['processed_patterns']==2
    second=module.generate_catalog('unused',frames,max_candidates=10,refinement_level=1,previous_catalog=first)
    assert calls==['slot0','slot1','slot2','slot3']
    assert second['complete'] and second['processed_patterns']==4
    assert second['resumed_processed_patterns']==2 and second['new_candidate_count']==2
    assert second['candidates'][:2]==first['candidates']
    assert first==original
    frames['UpperRack']['position_m'][0]=.1
    with pytest.raises(ValueError,match='same measured baseline'):
        module.generate_catalog('unused',frames,refinement_level=1,previous_catalog=first)


def test_incomplete_graph_cannot_reach_inventory_solver():
    with pytest.raises(ValueError,match='incomplete'):
        solve_inventory(catalog(['mug']),dict(graph(1),complete=False),{'mug':1},time_limit_s=1)


def test_orientation_family_preference_dominates_compactness_with_many_families():
    # Only the two remote candidates share a family. Many near-center choices
    # each use a different family; their compactness must not override grouping.
    candidates=[]
    for i in range(100):
        candidates.append({'kind':'bowl','rack':'UpperRack',
            'position_xy_m':[.23,.23] if i<2 else [0.,0.],
            'quaternion_xyzw':Rotation.from_euler('x',180 if i<2 else 170-i*.1,degrees=True).as_quat().tolist(),
            'row_metadata':{'row_id':'one_row'}})
    result=solve_inventory({'candidates':candidates},graph(len(candidates)),{'bowl':2},time_limit_s=3)
    assert result['selected_indices']==[0,1]


def test_screening_order_interleaves_groups_and_covers_slots_before_variants():
    from dishsim_frigidaire.organized_candidates import screening_order
    candidates=[]
    for kind,rack in [('bowl','LowerRack'),('bowl','UpperRack'),('mug','UpperRack')]:
        for slot in range(3):
            for variant,height in [(0,.15),(1,.10)]:
                index=len(candidates)
                candidates.append({'candidate_index':index,'kind':kind,'rack':rack,
                    'slot_id':f'{kind}_{rack}_{slot}', 'generation_tier':0,
                    'rack_local_pose':pose((slot*.1,0,height))})
    result=screening_order({'candidates':candidates})
    assert sorted(result)==list(range(18))
    first=[candidates[i] for i in result[:9]]
    assert len({c['slot_id'] for c in first})==9
    assert all(c['rack_local_pose']['position_m'][2]==.10 for c in first)
    assert len({(c['kind'],c['rack']) for c in first[:3]})==3
    assert result==screening_order({'candidates':candidates})


def test_measured_rotation_jitter_does_not_split_intended_row_family():
    from dishsim_frigidaire.organized_candidates import _semantic_orientation_family
    a={'kind':'bowl','quaternion_xyzw':[1,0,0,0],
       'row_metadata':{'opening_normal_rack':[0,0,-1]}}
    b={**a,'quaternion_xyzw':Rotation.from_euler('x',177,degrees=True).as_quat().tolist()}
    assert _semantic_orientation_family(a)==_semantic_orientation_family(b)
    p={**a,'kind':'dinner_plate','row_metadata':{'opening_normal_rack':[1,0,0]}}
    q={**p,'row_metadata':{'opening_normal_rack':[-1,0,0]}}
    assert _semantic_orientation_family(p)==_semantic_orientation_family(q)
    m={**a,'kind':'mug','row_metadata':{'opening_normal_rack':[0,0,-1],'handle_direction_rack':[1,0,0]}}
    n={**m,'row_metadata':{'opening_normal_rack':[0,0,-1],'handle_direction_rack':[-1,0,0]}}
    assert _semantic_orientation_family(m)!=_semantic_orientation_family(n)
