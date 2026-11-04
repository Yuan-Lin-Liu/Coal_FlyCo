"""A negative-point refinement may only decode the same pending RGB frame."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime_app'))
from text_sam.vision import StreamingSAM2


class RefinementFrameTests(unittest.TestCase):
    def test_another_image_cannot_refine_or_discard_pending_memory(self):
        engine=StreamingSAM2.__new__(StreamingSAM2)
        rgb=np.zeros((20,20,3),np.uint8);pending=object()
        engine.last_frame=5;engine.last_prediction_rgb=rgb;engine.pending={1:pending}
        with self.assertRaises(RuntimeError):
            engine.predict_many(rgb.copy(),5,10,{1:[2,2,10,10]},refine=True)
        self.assertIs(engine.pending[1],pending)

    def test_regular_prediction_cannot_repeat_a_frame(self):
        engine=StreamingSAM2.__new__(StreamingSAM2)
        rgb=np.zeros((20,20,3),np.uint8)
        engine.last_frame=5;engine.last_prediction_rgb=rgb;engine.pending={1:object()}
        with self.assertRaises(RuntimeError):engine.predict_many(rgb,5,10,{1:[2,2,10,10]})
        self.assertIn(1,engine.pending)


if __name__=='__main__':unittest.main()
