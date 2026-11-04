import copy
from pathlib import Path
import sys
import unittest

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime_app'))
from text_sam.trust import remember_viewpoint, validate, update_online


def observation(angle=0, area=5000):
    vector=np.zeros(192,dtype=np.float32)
    vector[0],vector[64]=np.cos(angle),np.sin(angle)
    return dict(box=[20,20,120,120],area=area,aspect=1.,appearance=vector,visibility=.99)


def track(adapt=True):
    reference=observation()
    return dict(id=1,state='TRACKING',verified=True,reference=reference,
        online=copy.deepcopy(reference),trusted_observation=reference,
        first_frame=0,last_seen=0,viewpoint_adaptation=adapt)


class ViewpointTests(unittest.TestCase):
    def test_gradual_accepted_views_keep_identity_without_lower_thresholds(self):
        target=track()
        for frame,angle in enumerate(np.linspace(0,1.45,121)[1:],1):
            obs=observation(angle)
            valid,evidence=validate(target,obs,'sam2')
            self.assertTrue(valid,(frame,evidence))
            remember_viewpoint(target,obs,frame)
            target['last_seen']=frame;target['trusted_observation']=obs
            update_online(target,obs)
        self.assertLess(evidence['initial_anchor_similarity'],.43)
        self.assertGreater(evidence['anchor_similarity'],.9)
        self.assertLessEqual(len(target['view_references']),12)
        self.assertTrue(validate(target,observation(1.45),'sam2',recovery=True)[0])
        self.assertEqual(target['reference']['appearance'][0],1)

    def test_sudden_different_appearance_is_rejected_and_not_learned(self):
        target=track();valid,evidence=validate(target,observation(1.5),'sam2')
        self.assertFalse(valid)
        self.assertIn('fixed_appearance',evidence['reason'])
        for frame in range(1,15):remember_viewpoint(target,observation(1.5),frame)
        self.assertFalse(target.get('view_references'))

    def test_area_jumps_and_missing_visibility_still_rejected(self):
        target=track()
        self.assertIn('area_jump',validate(target,observation(area=50),'sam2')[1]['reason'])
        self.assertFalse(validate(target,None,'sam2')[0])
        obs=observation();obs['visibility']=.1
        self.assertFalse(validate(target,obs,'sam2')[0])

    def test_lost_gap_cannot_admit_a_new_view(self):
        target=track();target['stable_view_frames']=10
        remember_viewpoint(target,observation(.25),100)
        self.assertEqual(target['stable_view_frames'],0)
        self.assertFalse(target.get('view_references'))

    def test_stable_scale_changes_are_retained_even_with_the_same_colour(self):
        target=track()
        for frame in range(1,81):
            obs=observation(area=int(5000*1.03**frame));side=int(np.sqrt(obs['area']))
            obs['box']=[20,20,20+side,20+side]
            remember_viewpoint(target,obs,frame)
            target['last_seen']=frame;target['trusted_observation']=obs
        self.assertTrue(target.get('view_references'))
        self.assertGreater(max(ref['area']for ref in target['view_references']),10000)
        self.assertTrue(validate(target,obs,'sam2',True)[0])

    def test_other_backends_keep_fixed_reference_behavior(self):
        target=track(False);target['view_references']=[observation(1.5)]
        self.assertFalse(validate(target,observation(1.5),'sam2')[0])


if __name__=='__main__':unittest.main()
