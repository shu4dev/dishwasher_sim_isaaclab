"""Screening identity isolation and measured/common-rack pose transfer."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('organized_screen',
    ROOT/'frigidaire/scripts/experiment/frigidaire_organized_screen.py')
screen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(screen)


def candidate():
    pose = {'position_m': [.1, .2, .3], 'quaternion_xyzw': [1., 0., 0., 0.]}
    return {'candidate_id': 'original', 'candidate_index': 9, 'object_id': 'original',
            'kind': 'mug', 'rack': 'UpperRack', 'pose_world': pose,
            'rack_local_pose': deepcopy(pose), 'row_metadata': {'row_id': 'row_a',
                'opening_normal_rack': [0., 0., -1.]}}


def test_active_screen_identity_is_stable_and_source_is_not_mutated():
    source = candidate()
    before = deepcopy(source)
    entry = screen.active_entry(source)
    assert entry['object_id'] == 'screen_mug'
    assert entry['candidate_id'] == 'original'
    entry['pose_world']['position_m'][0] = 9.
    assert source == before


def test_screened_pose_uses_measured_rack_then_common_baseline():
    source = candidate()
    before = deepcopy(source)
    identity = [0., 0., 0., 1.]
    snapshot = {'poses': {'UpperRack': {'position_m': [1., 2., 3.], 'quaternion_xyzw': identity},
                         'screen_mug': {'position_m': [1.1, 2.2, 3.3], 'quaternion_xyzw': [1., 0., 0., 0.]}}}
    baseline = {'poses': {'UpperRack': {'position_m': [4., 5., 6.], 'quaternion_xyzw': identity}}}
    result = screen.screened_candidate(source, snapshot, baseline, 0, 'trials/screen_0000/result.json')
    assert result['candidate_id'] == 'original_settled'
    assert result['source_candidate_id'] == 'original'
    assert result['source_candidate_index'] == 9
    assert result['candidate_index'] == 0
    assert result['support_screen']['outcome'] == 'screening_passed'
    np.testing.assert_allclose(result['pose_world']['position_m'], [4.1, 5.2, 6.3])
    np.testing.assert_allclose(result['rack_local_pose']['position_m'], [.1, .2, .3])
    assert result['row_metadata'] == source['row_metadata']
    assert source == before
