"""Recovery must use confirmed views without admitting unseen or weak masks."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime_app'))
from text_sam.pipeline import TrackingPipeline
from text_sam.trust import validate
from test_viewpoint_tracking import observation,track


def recovering(adapt=True):
    target=track(adapt)
    target.update(state='VERIFYING',recovering=True,verify_count=0,
        candidate_box=[20,20,120,120],box=[20,20,120,120],empty_streak=0,
        motion_frame=90,motion_box=[20,20,120,120],velocity=[0.,0.],
        first_frame=0,last_seen=90,grounding_score=.9)
    target['online']=observation(1.25)
    target['view_references']=[observation(.3)]
    return target


class KnownViewTests(unittest.TestCase):
    def test_confirmed_view_can_recover_with_stale_online_template(self):
        target=recovering();obs=observation(.05)
        valid,evidence=validate(target,obs,'sam2',True)
        self.assertTrue(valid,evidence)
        self.assertLess(evidence['online_similarity'],.54)
        self.assertTrue(evidence['known_view_recovery'])
        self.assertFalse(validate(target,obs,'sam2',False)[0])

    def test_fallback_cannot_bypass_visibility_identity_or_scale(self):
        for change in ('visibility','appearance','scale'):
            target=recovering();obs=observation(.05)
            if change=='visibility':obs['visibility']=.7
            if change=='appearance':obs=observation(1.5)
            if change=='scale':obs['area']=100
            self.assertFalse(validate(target,obs,'sam2',True)[0],change)
        target=recovering(False)
        self.assertFalse(validate(target,observation(.05),'sam2',True)[0])
        target=recovering()
        self.assertFalse(validate(target,observation(.05),'motion',True)[0])

    def test_recovery_of_full_object_is_not_delayed_by_tiny_part_box(self):
        pipe=TrackingPipeline(SimpleNamespace(locate=lambda *a:None),
            SimpleNamespace(reset=lambda:None),viewpoint_adaptation=True)
        target=recovering();target['box']=[90,20,100,30]
        target['last_seen']=10;target['viewpoint_adaptation']=True
        proposal=dict(box=[20,20,120,120])
        score=pipe.candidate_affinity(target,proposal,observation()['appearance'],12)
        self.assertGreater(score,.43)
        proposal['box']=[220,20,320,120]
        self.assertEqual(pipe.candidate_affinity(target,proposal,observation()['appearance'],12),0.)
        pipe.viewpoint_adaptation=False;proposal['box']=[20,20,120,120]
        self.assertEqual(pipe.candidate_affinity(target,proposal,observation()['appearance'],12),0.)

    def test_recovery_selects_a_view_with_matching_scale_before_colour_ranking(self):
        target=recovering();target['online']=observation(.1)
        target['view_references']=[observation(.1,area=10),observation(.2,area=5000)]
        valid,evidence=validate(target,observation(.1),'sam2',True)
        self.assertTrue(valid,evidence);self.assertEqual(evidence['matched_area_ratio'],1.)
        target['state']='OCCLUDED'
        valid,evidence=validate(target,observation(.1,area=500000),'sam2',True)
        self.assertFalse(valid);self.assertIn('identity_scale',evidence['reason'])

    def test_verified_component_can_supply_missing_parent_identity_appearance(self):
        target=recovering();target['online']=observation()
        target['component_references']=[observation(1.3)]
        self.assertTrue(validate(target,observation(1.3),'sam2',True)[0])
        target['viewpoint_adaptation']=False
        self.assertFalse(validate(target,observation(1.3),'sam2',True)[0])

    def test_coarse_recovery_also_ignores_an_incompatible_tiny_reference(self):
        pipe=TrackingPipeline(SimpleNamespace(locate=lambda *a:None),
            SimpleNamespace(reset=lambda:None),viewpoint_adaptation=True)
        target=recovering();small=observation(.1);small['box']=[90,20,92,22]
        target['view_references']=[small,observation(.2)]
        self.assertGreater(pipe.candidate_affinity(target,dict(box=[20,20,120,120]),observation(.1)['appearance'],100),.43)

    def test_current_verified_component_supports_parent_recovery_after_motion(self):
        pipe=TrackingPipeline(SimpleNamespace(locate=lambda *a:None),
            SimpleNamespace(reset=lambda:None),viewpoint_adaptation=True)
        target=recovering();target['box']=[300,20,310,30];target['last_seen']=10
        item=dict(box=[20,20,120,120]);descriptor=observation()['appearance']
        self.assertEqual(pipe.candidate_affinity(target,item,descriptor,12),0.)
        target['linked_component_box']=[40,40,80,100]
        self.assertGreater(pipe.candidate_affinity(target,item,descriptor,12),.43)

    def test_a_clipped_part_does_not_define_the_motion_scale_of_a_full_proposal(self):
        pipe=TrackingPipeline(SimpleNamespace(locate=lambda *a:None),
            SimpleNamespace(reset=lambda:None),viewpoint_adaptation=True)
        target=recovering();target['box']=[130,130,140,140];target['last_seen']=10
        item=dict(box=[0,0,100,100]);descriptor=observation()['appearance']
        self.assertEqual(pipe.candidate_affinity(target,item,descriptor,12),0.)
        target['image_shape']=(140,140)
        self.assertGreater(pipe.candidate_affinity(target,item,descriptor,12),.43)
        self.assertEqual(pipe.candidate_affinity(target,item,observation(1.5)['appearance'],12),0.)

    def test_two_frames_required_before_output_memory_and_online_rebase(self):
        commits=[]
        engine=SimpleNamespace(reset=lambda:None,prediction_source='sam2',history_frames=2,
            predict_many=lambda *a:{1:(np.ones((140,140),bool),.99)},
            drop_objects=lambda *a:None,commit_many=lambda ids:commits.append(list(ids)))
        pipe=TrackingPipeline(SimpleNamespace(locate=lambda *a:None),engine,viewpoint_adaptation=True)
        pipe.command=SimpleNamespace(quantity='one');pipe.seen_any=True
        target=recovering();pipe.tracks={1:target}
        initial=copy.deepcopy(target['reference']);online=target['online']['appearance'].copy()
        candidate=observation(.05);candidate['box']=[30,30,110,110]
        with patch('text_sam.pipeline.observe',return_value=candidate):
            first=pipe.process(np.zeros((140,140,3),np.uint8),100,200)
            self.assertFalse(first['masks'])
            self.assertEqual(commits[-1],[])
            self.assertEqual(target['candidate_box'],[20,20,120,120])
            np.testing.assert_array_equal(target['online']['appearance'],online)
            second=pipe.process(np.zeros((140,140,3),np.uint8),101,200)
        self.assertEqual(second['recovered_ids'],[1])
        self.assertIn(1,second['masks']);self.assertEqual(commits[-1],[1])
        np.testing.assert_array_equal(target['online']['appearance'],observation(.05)['appearance'])
        np.testing.assert_array_equal(target['reference']['appearance'],initial['appearance'])


if __name__=='__main__':unittest.main()
