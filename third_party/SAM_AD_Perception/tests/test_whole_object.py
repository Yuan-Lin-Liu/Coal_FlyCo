"""One generic policy must preserve the original query and verify fresh masks."""
from pathlib import Path
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'runtime_app'))
from text_sam.commands import parse_command
from text_sam.whole_object import WholeObjectPipeline,LinkedObjectGrounder,connected
from text_sam.vision import ImageSAM2

class Engine:
    prediction_source='sam2';has_neural_memory=True;history_frames=1
    def __init__(self,mask,visibility=1):self.mask=mask;self.visibility=visibility;self.calls=0;self.commits=[]
    def reset(self):self.histories={}
    def predict_many(self,rgb,frame,frames,boxes=None,**kwargs):
        self.calls+=1
        for i in boxes or {}:self.histories[i]={}
        return {i:(self.mask.copy(),self.visibility)for i in self.histories}
    def drop_objects(self,ids):
        for i in ids:self.histories.pop(i,None)
    def commit_many(self,ids):self.commits.append(list(ids))

class Grounder:
    def __init__(self):self.box=[10,10,90,100];self.available=True;self.queries=[]
    def locate(self,*args):return self.locate_guarded(*args)
    def locate_guarded(self,rgb,command,threshold,**kwargs):
        self.queries.append(command.query)
        candidates=[dict(id=1,box=self.box.copy(),score=.95,category_ok=True)]if self.available else []
        return dict(candidates=candidates,selected=candidates,matched=bool(candidates))

def fixture():
    main=np.zeros((140,140),bool);main[10:35,10:90]=True;main[10:100,47:53]=True
    part=np.zeros_like(main);part[30:100,35:65]=True
    rgb=np.full((140,140,3),[20,180,20],np.uint8);rgb[part]=[160,60,30];rgb[main]=[220,220,210]
    a,b=Engine(main),Engine(part);ground=Grounder()
    pipe=WholeObjectPipeline(ground,a,b,.3,30,1,90,1,True,viewpoint_adaptation=True)
    return pipe,rgb,main,part,a,b,ground

class WholeObjectTests(unittest.TestCase):
    def test_refresh_verified_before_merging_under_one_id(self):
        pipe,rgb,main,part,a,b,_=fixture()
        first=pipe.process(rgb,0,10,parse_command('target'))
        self.assertFalse(first['masks']);self.assertEqual(b.commits[-1],[]);self.assertFalse(pipe.component_references)
        second=pipe.process(rgb,1,10)
        self.assertEqual(list(second['masks']),[1]);np.testing.assert_array_equal(second['masks'][1],main|part)
        self.assertTrue(pipe.component_references);self.assertEqual(b.commits[-1],[1])

    def test_exact_same_policy_and_original_text_for_different_categories(self):
        for query in ('windmill','seated Buddha statue','stone church','bicycle','red chair'):
            pipe,rgb,main,part,a,b,ground=fixture();command=parse_command(query)
            pipe.process(rgb,0,10,command);result=pipe.process(rgb,1,10)
            np.testing.assert_array_equal(result['masks'][1],main|part)
            self.assertEqual(b.calls,2);self.assertTrue(ground.queries)
            self.assertTrue(all(q==command.query for q in ground.queries))

    def test_low_quality_refresh_never_fills_detection_rectangle(self):
        pipe,rgb,main,part,a,b,_=fixture();b.visibility=.1
        pipe.process(rgb,0,10,parse_command('target'));result=pipe.process(rgb,1,10)
        np.testing.assert_array_equal(result['masks'][1],main)
        self.assertFalse(result['masks'][1][80,40])

    def test_fresh_refresh_bridges_video_loss_then_stops_when_unreliable(self):
        pipe,rgb,main,part,a,b,_=fixture()
        pipe.process(rgb,0,30,parse_command('target'));pipe.process(rgb,1,30);a.visibility=.1
        for frame in range(2,23):
            result=pipe.process(rgb,frame,30);np.testing.assert_array_equal(result['masks'][1],part)
        self.assertEqual(pipe.tracks[1]['state'],'OCCLUDED')
        b.visibility=.1;self.assertFalse(pipe.process(rgb,23,30)['masks'])

    def test_no_detection_cannot_reuse_the_previous_image_mask(self):
        pipe,rgb,main,part,a,b,ground=fixture()
        pipe.process(rgb,0,10,parse_command('target'));pipe.process(rgb,1,10)
        a.visibility=.1;ground.available=False
        self.assertFalse(pipe.process(rgb,2,10)['masks'])

    def test_refresh_pixels_cannot_leak_outside_current_detection(self):
        pipe,rgb,main,part,a,b,_=fixture();b.mask[108:120,20:60]=True
        pipe.process(rgb,0,10,parse_command('target'));result=pipe.process(rgb,1,10)
        self.assertFalse(result['masks'][1][110,40]);self.assertTrue(result['masks'][1][80,40])

    def test_disconnected_visible_parts_can_belong_to_one_fresh_semantic_envelope(self):
        pipe,rgb,main,part,a,b,_=fixture();a.mask[35:100,:]=False;b.mask[:50,:]=False
        self.assertFalse(connected(a.mask,b.mask))
        pipe.process(rgb,0,10,parse_command('target'));result=pipe.process(rgb,1,10)
        self.assertTrue(result['masks'][1][80,40]);self.assertTrue(result['masks'][1][20,20])

    def test_verified_refresh_uses_actual_current_prompt_extent(self):
        pipe,rgb,main,part,a,b,ground=fixture();ground.box=[35,30,65,100];rgb[20:100,25:75]=[160,60,30]
        pipe.process(rgb,0,10,parse_command('target'));pipe.process(rgb,1,10)
        track=next(iter(pipe.refresh.tracks.values()));track.update(state='VERIFYING',verify_count=1,recovering=True)
        ground.box=[25,20,75,100];b.mask[20:100,25:75]=True
        result=pipe.process(rgb,2,10)
        self.assertEqual(track['state'],'TRACKING');self.assertTrue(result['masks'][1][70,27])

    def test_enclosing_candidate_must_be_confident_and_same_instance(self):
        ground=LinkedObjectGrounder(Grounder());ground.command=parse_command('target')
        candidates=[dict(id=1,box=[35,65,65,100],score=.6,category_ok=True),
            dict(id=2,box=[30,20,70,100],score=.51,category_ok=True),
            dict(id=3,box=[10,10,90,100],score=.2,category_ok=True),
            dict(id=4,box=[95,10,135,130],score=.9,category_ok=True)]
        ground.snapshot={'objects':[dict(verified=True,box=[35,65,65,100])],
            'detection':dict(candidates=candidates,selected=candidates)}
        selected=ground.locate_guarded(np.zeros((140,140,3),np.uint8),ground.command)['selected']
        self.assertEqual([item['id']for item in selected],[2])

    def test_failed_video_search_retries_next_frame(self):
        pipe,rgb,main,part,a,b,_=fixture()
        pipe.process(rgb,0,10,parse_command('target'));pipe.process(rgb,1,10)
        pipe.tracks[1]['state']='OCCLUDED';pipe.tracker.drop_objects([1]);pipe.last_detection_frame=2
        pipe.process(rgb,3,10);self.assertEqual(pipe.tracks[1]['state'],'VERIFYING')
        pipe.process(rgb,4,10);self.assertEqual(pipe.tracks[1]['state'],'TRACKING')

class FakePredictor:
    def reset_predictor(self):self.calls=0
    def set_image(self,rgb):self.calls+=1
    def predict(self,**kwargs):
        masks=np.zeros((3,20,20),bool);masks[0,:,:]=True;masks[1,4:10,4:10]=True;masks[2,2:15,2:15]=True
        return masks,np.array([.51,.95,.8]),None

class ImageRefreshTests(unittest.TestCase):
    def fixture(self):
        engine=ImageSAM2.__new__(ImageSAM2);engine.predictor=FakePredictor();engine.reset();return engine
    def test_mask_quality_wins_over_larger_mask_area(self):
        engine=self.fixture();out=engine.predict_many(np.zeros((20,20,3),np.uint8),0,10,{1:[0,0,20,20]})
        self.assertEqual(int(out[1][0].sum()),36);self.assertEqual(out[1][1],.95)
    def test_rejected_image_mask_never_enters_history(self):
        engine=self.fixture();engine.predict_many(np.zeros((20,20,3),np.uint8),0,10,{1:[0,0,20,20]});engine.commit_many([])
        self.assertFalse(engine.histories);self.assertEqual(engine.history_frames,0)
    def test_every_new_rgb_is_encoded_even_if_appearance_and_box_are_unchanged(self):
        engine=self.fixture();rgb=np.zeros((20,20,3),np.uint8)
        out=engine.predict_many(rgb,0,10,{1:[0,0,20,20]});engine.commit_many(out)
        engine.predict_many(rgb.copy(),1,10,{1:[0,0,20,20]});self.assertEqual(engine.predictor.calls,2)

    def refined_fixture(self,quality=.94,drift=False):
        class RefiningPredictor(FakePredictor):
            def __init__(self):self.boxes=[]
            def predict(self,**kwargs):
                self.boxes.append(kwargs['box']);masks=np.zeros((3,20,20),bool)
                masks[0,2:18,2:18]=True
                if len(self.boxes)==2 and drift:masks[0]=False;masks[0,:8,:8]=True
                return masks,np.array([.7 if len(self.boxes)==1 else quality,.2,.1]),None
        engine=ImageSAM2.__new__(ImageSAM2);engine.predictor=RefiningPredictor();engine.reset();return engine

    def test_low_quality_prompt_refines_without_encoding_rgb_twice(self):
        engine=self.refined_fixture();out=engine.predict_many(np.zeros((20,20,3),np.uint8),0,10,{1:[0,0,20,20]})
        self.assertEqual(out[1][1],.94);self.assertEqual(len(engine.predictor.boxes),2)
        self.assertEqual(engine.predictor.calls,1);self.assertGreater(engine.predictor.boxes[1][0],0)

    def test_refinement_needs_quality_gain_and_must_retain_the_same_foreground(self):
        for quality,drift in ((.71,False),(.99,True)):
            engine=self.refined_fixture(quality,drift)
            out=engine.predict_many(np.zeros((20,20,3),np.uint8),0,10,{1:[0,0,20,20]})
            self.assertEqual(out[1][1],.7);self.assertEqual(int(out[1][0].sum()),256)

if __name__=='__main__':unittest.main()
