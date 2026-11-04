"""Class-independent completion using fresh grounding of the original target text."""
import cv2
import numpy as np
from .pipeline import TrackingPipeline
from .targets import box_iou
from .trust import identity_references


def inclusion(inner,outer):
    intersection=max(0,min(inner[2],outer[2])-max(inner[0],outer[0]))*max(0,min(inner[3],outer[3])-max(inner[1],outer[1]))
    return intersection/max(1,(inner[2]-inner[0])*(inner[3]-inner[1]))


def connected(main,part):
    radius=max(3,round(.0125*np.hypot(*main.shape)))
    nearby=cv2.dilate(main.astype(np.uint8),cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2*radius+1,2*radius+1)))>0
    return np.count_nonzero(nearby&part)>=min(16,max(1,int(part.sum()*.01)))


class LinkedObjectGrounder:
    """Associate current text detections with the already admitted target instance."""
    def __init__(self,grounder):
        self.grounder=grounder;self.command=self.snapshot=None
        self.rgb=None;self.cached=None;self.component_boxes=[];self.parent_boxes=[]
    def __getattr__(self,key):return getattr(self.grounder,key)
    def locate(self,rgb,command,threshold=.3):return self.locate_guarded(rgb,command,threshold)
    def locate_guarded(self,rgb,command,threshold=.3):
        if self.rgb is rgb and self.cached is not None:return self.cached
        snap=self.snapshot
        detection=snap.get('detection')
        if detection is None:detection=self.grounder.locate_guarded(rgb,self.command,threshold)
        observed=[t['box']for t in snap['objects']if t.get('verified')]+self.component_boxes
        proposals=detection['candidates']if observed else detection['selected']
        candidates=[c for c in proposals if c.get('category_ok',True)
            and (not observed or any(box_iou(old,c['box'])>=.2 or inclusion(old,c['box'])>=.6 for old in observed))]
        selected=sorted(candidates,key=lambda c:c['score'],reverse=True)[:1]
        if selected:
            best=selected[0]
            enclosing=[c for c in candidates if c['score']>=.8*best['score']and inclusion(best['box'],c['box'])>=.85]
            selected=[max(enclosing,key=lambda c:(c['box'][2]-c['box'][0])*(c['box'][3]-c['box'][1]))]
        self.parent_boxes=[c['box']for c in selected];self.rgb=rgb
        # One instance, never concatenate detections of neighbouring objects.
        self.cached=dict(detection,candidates=selected,selected=selected,matched=bool(selected),
            box=selected[0]['box']if selected else None,selected_ids=[c['id']for c in selected],
            selected_id=selected[0]['id']if selected else None)
        return self.cached


class GroundedRefreshEngine:
    """Use only a new original-query box and masks decoded from this exact RGB."""
    def __init__(self,engine,grounder):self.engine,self.grounder=engine,grounder
    def __getattr__(self,key):return getattr(self.engine,key)
    def predict_many(self,rgb,frame_index,total_frames,boxes=None):
        detection=self.grounder.locate_guarded(rgb,self.grounder.command,.3)
        ids=set(self.engine.histories)|set(boxes or {})
        if not detection['selected']:
            self.engine.drop_objects(list(ids))
            return self.engine.predict_many(rgb,frame_index,total_frames,{})
        box=detection['selected'][0]['box']
        predictions=self.engine.predict_many(rgb,frame_index,total_frames,{i:box for i in ids})
        h,w=rgb.shape[:2]
        x1,y1=max(0,int(np.floor(box[0]))-2),max(0,int(np.floor(box[1]))-2)
        x2,y2=min(w,int(np.ceil(box[2]))+2),min(h,int(np.ceil(box[3]))+2)
        extent=np.zeros((h,w),bool);extent[y1:y2,x1:x2]=True
        return {i:(mask&extent,quality)for i,(mask,quality)in predictions.items()}


class EnginePair:
    def __init__(self,main,refresh):self.main,self.refresh=main,refresh
    def __getattr__(self,key):return getattr(self.main,key)
    def reset(self):self.main.reset();self.refresh.reset()


class WholeObjectPipeline(TrackingPipeline):
    """The same original-query refresh and verification policy for every category."""
    def __init__(self,grounder,engine,image_engine,*args,**kwargs):
        super().__init__(grounder,EnginePair(engine,image_engine),*args,**kwargs)
        self.linked_grounder=LinkedObjectGrounder(grounder)
        self.refresh=TrackingPipeline(self.linked_grounder,GroundedRefreshEngine(image_engine,self.linked_grounder),*args,**kwargs)
        self.component_references=[]

    def process(self,rgb,frame_index,total_frames,command=None):
        if command is not None:
            self.linked_grounder.command=command;self.component_references=[]
        recent=[t['box']for t in self.refresh.tracks.values()
            if self.component_references and t.get('verified')and frame_index-t['last_seen']<=2]
        self.linked_grounder.component_boxes=recent
        for track in self.tracks.values():
            track['component_references']=self.component_references
            track['linked_component_box']=recent[0]if recent else None
        snap=super().process(rgb,frame_index,total_frames,command)
        self.linked_grounder.snapshot=snap;self.linked_grounder.rgb=None;self.linked_grounder.cached=None
        detection=self.linked_grounder.locate_guarded(rgb,self.command,.3)
        if not detection['selected']:
            for track in self.refresh.tracks.values():self.refresh.state(track,'OCCLUDED',frame_index,'target_not_detected')
            self.refresh.tracker.drop_objects(list(self.refresh.tracks));self.refresh.last_detection_frame=None
        else:
            for track in self.refresh.tracks.values():
                if track['state']=='VERIFYING':track['candidate_box']=detection['selected'][0]['box']
        counters=('memory_updates','sam_encoder_calls','detection_count')
        before={key:getattr(self.refresh,key)for key in counters}
        fresh=self.refresh.process(rgb,frame_index,total_frames,command)
        for key in counters:setattr(self,key,getattr(self,key)+getattr(self.refresh,key)-before[key])
        for event in fresh['audit']:
            event=dict(event,component='grounded_refresh')
            if event.get('action')=='reject_prediction':event['action']='reject_component_prediction'
            self.audit.append(event)
        self.transitions.extend(dict(event,component='grounded_refresh')for event in fresh['transitions'])
        if not snap['masks']and fresh['masks']and len(self.tracks)==1:
            object_id,track=next(iter(self.tracks.items()))
            verified=any(t.get('verified')and t['last_seen']==frame_index for t in self.refresh.tracks.values())
            if track.get('verified')and verified:
                part=next(iter(fresh['masks'].values()));snap['masks'][object_id]=part.copy();snap['phase']='tracking'
                x,y,w,h=cv2.boundingRect(part.astype(np.uint8))
                for row in snap['objects']:
                    if row['id']==object_id:row.update(state='TRACKING',prediction_only=False,box=[x,y,x+w,y+h],observed_box=[x,y,x+w,y+h])
                self.audit.append(dict(id=object_id,action='verified_refresh_during_recovery',component='grounded_refresh',reliable_output=True,main_memory_commit=False))
        if snap['masks']and fresh['masks']:
            object_id,main=next(iter(snap['masks'].items()));part=next(iter(fresh['masks'].values()))
            mx,my,mw,mh=cv2.boundingRect(main.astype(np.uint8));px,py,pw,ph=cv2.boundingRect(part.astype(np.uint8))
            same_instance=any(inclusion([px,py,px+pw,py+ph],parent)>=.85
                and inclusion([mx,my,mx+mw,my+mh],parent)>=.60 for parent in self.linked_grounder.parent_boxes)
            if connected(main,part)or same_instance:
                self.component_references=[ref for track in self.refresh.tracks.values()if track.get('verified')for ref in identity_references(track)]
                merged=main|part;snap['masks'][object_id]=merged;x,y,w,h=cv2.boundingRect(merged.astype(np.uint8))
                for row in snap['objects']:
                    if row['id']==object_id:row['box']=row['observed_box']=[x,y,x+w,y+h]
                self.audit.append(dict(id=object_id,action='merge_verified_component',component='grounded_refresh',
                    added_pixels=int(np.count_nonzero(part&~main)),refresh_pixels=int(part.sum()),reliable_output=True))
            else:
                self.audit.append(dict(id=object_id,action='reject_component',reason='different_instance',component='grounded_refresh'))
                for track in self.refresh.tracks.values():self.refresh.state(track,'OCCLUDED',frame_index,'different_instance')
                self.refresh.tracker.drop_objects(list(self.refresh.tracks));self.refresh.last_detection_frame=None
        return snap
