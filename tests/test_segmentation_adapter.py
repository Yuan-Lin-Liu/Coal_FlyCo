"""HTTP/ROS boundary regressions without GPU weights or a ROS master."""
import base64
import hashlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
import cv2
import numpy as np

SOURCE=Path(__file__).resolve().parents[1]/'third_party/flyco_ros2/vendor/perception_predictor/src/sam_tracking/src'
sys.path.insert(0,str(SOURCE))
from sam_tracking.providers.http_text_sam_provider import HTTPTextSAMProvider
from sam_tracking.providers.factory import make_camera_provider
from sam_tracking.core.tracking_pipeline import TrackingPipeline

class BoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frames=[];cls.corrupt=False
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                self.respond({'protocol':'flyco-text-sam-v1','service_revision':'20261026-r3','backend':'A','status':'ready'})
            def do_POST(self):
                p=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                if self.path=='/session':out={'session':'owned-session'}
                elif self.path=='/hint':out={'advisory':True,'deferred':True}
                elif self.path=='/reset':out={}
                else:
                    rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(p['image_png']),np.uint8),1),cv2.COLOR_BGR2RGB)
                    cls.frames.append(rgb.copy());mask=(rgb[:,:,0]>100).astype(np.uint8)*255
                    ok,png=cv2.imencode('.png',mask);assert ok
                    out={'session':p['session'],'sequence':p['sequence'],
                         'input_rgb_sha256':hashlib.sha256(rgb.tobytes()).hexdigest(),
                         'mask_png':base64.b64encode(png).decode(),'mask_sha256':hashlib.sha256(mask.tobytes()).hexdigest()}
                    if not mask.any():out['mask_png']=None;out['mask_sha256']=None
                    if cls.corrupt:out['input_rgb_sha256']='wrong'
                self.respond(out)
            def respond(self,data):
                raw=json.dumps(data).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        cls.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
        cls.endpoint='http://127.0.0.1:'+str(cls.server.server_port)
    @classmethod
    def tearDownClass(cls):cls.server.shutdown();cls.server.server_close();cls.thread.join()
    def setUp(self):
        self.__class__.frames=[];self.__class__.corrupt=False
        self.provider=make_camera_provider('A',None,self.endpoint)
        self.rgb=np.zeros((20,30,3),np.uint8);self.rgb[4:14,6:18]=[220,50,10]
        self.prompt={'text':'an arbitrary target','coords':np.empty((0,2)),'labels':np.empty(0)}
    def tearDown(self):self.provider.reset()
    def test_rgb_and_binary_mask_round_trip_is_exact(self):
        mask=self.provider.initialize(self.rgb,self.prompt)
        np.testing.assert_array_equal(self.frames[-1],self.rgb)
        np.testing.assert_array_equal(mask,(self.rgb[:,:,0]>100).astype(np.uint8)*255)
    def test_same_callback_only_runs_inference_once(self):
        self.provider.initialize(self.rgb,self.prompt);self.provider.track(self.rgb)
        self.assertEqual(len(self.frames),1)
    def test_reused_camera_buffer_new_stamp_requires_new_inference(self):
        self.provider.begin_frame((1,0));self.provider.initialize(self.rgb,self.prompt)
        self.rgb[:]=[10,50,220];self.provider.begin_frame((2,0));mask=self.provider.track(self.rgb)
        self.assertEqual(len(self.frames),2);self.assertIsNone(mask)
    def test_depth_advisory_preserves_only_the_exact_current_rgb_mask(self):
        mask=self.provider.initialize(self.rgb,self.prompt)
        held=self.provider.reinitialize_from_prompt(self.rgb,self.prompt,'depth_low_consistency_reinit')
        np.testing.assert_array_equal(mask,held)
        self.assertIsNone(self.provider.reinitialize_from_prompt(self.rgb.copy(),self.prompt,'depth_low_consistency_reinit'))
        self.assertEqual(self.provider.stats.reinitialization_count,0)
    def test_different_pixels_in_service_response_are_rejected(self):
        self.__class__.corrupt=True
        with self.assertRaisesRegex(RuntimeError,'different RGB'):self.provider.initialize(self.rgb,self.prompt)
    def test_clicks_cannot_silently_change_the_text_target(self):
        prompt=dict(self.prompt,coords=np.array([[1,2]]),labels=np.array([1]))
        self.assertFalse(self.provider.can_initialize_from_prompt(prompt))
    def test_depth_advisory_does_not_clear_pipeline_state_or_count_as_recovery(self):
        self.provider.initialize(self.rgb,self.prompt)
        class Guidance:
            def make_reinit_decision(self,**kwargs):
                return SimpleNamespace(should_reinitialize=True,prompt={'text':''},reason='depth_low_consistency_reinit',consistency=.1,bbox=[1,1,4,4],depth_area=9)
            def note_reinit_deferred(self,reason):self.deferred=reason
            def note_reinitialized(self,reason):raise AssertionError('Advisory incorrectly counted as reinitialization')
            def snapshot(self):return {}
        guidance=Guidance();prompts=SimpleNamespace(query_text=self.prompt['text'])
        pipe=TrackingPipeline(self.provider,prompts,None,None,guidance);pipe._empty_mask_streak=2
        result=pipe._maybe_depth_reinitialize(self.rgb,np.ones((20,30),np.uint8))
        self.assertEqual(result['state'],'reinit_pending');self.assertEqual(pipe.empty_mask_streak,2)
        self.assertEqual(guidance.deferred,'depth_low_consistency_reinit')

if __name__=='__main__':unittest.main()
