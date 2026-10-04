"""Organization gates, semantic symmetry and exact finite-inventory constraints."""
import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from dishsim_frigidaire.organization import OrganizationGeometry, evaluate_organization, orientation_metrics, ray_blocked


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
