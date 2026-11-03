"""Batch rays against the existing static USD colliders using bundled NVIDIA Warp.

This module only implements sensor rays. This is
only a sensor implementation, like PhysX raycasts, over the same warehouse.
The mesh is frozen at startup; dynamic obstacle scenes must use another backend.
"""
import numpy as np
import warp as wp


@wp.kernel
def cast(mesh: wp.uint64, starts: wp.array(dtype=wp.vec3),
         directions: wp.array(dtype=wp.vec3), max_distance: float,
         distances: wp.array(dtype=float)):
    i=wp.tid()
    hit=wp.mesh_query_ray(mesh,starts[i],directions[i],max_distance)
    distances[i]=-1.0
    if hit.result:
        distances[i]=hit.t


def collider_triangles(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    root=stage.GetPrimAtPath('/World/CoalWarehouse')
    if not root:
        raise ValueError('Expected existing CoalWarehouse root')
    cache=UsdGeom.XformCache()
    points,indices,paths=[],[],[]
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        ancestor=prim
        while ancestor and not ancestor.IsPseudoRoot():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError('Batch static sensor cannot include a rigid body: '+str(prim.GetPath()))
            ancestor=ancestor.GetParent()
        if prim.IsA(UsdGeom.Cube):
            s=UsdGeom.Cube(prim).GetSizeAttr().Get()/2
            local=np.array([[-s,-s,-s],[s,-s,-s],[s,s,-s],[-s,s,-s],
                            [-s,-s,s],[s,-s,s],[s,s,s],[-s,s,s]])
            faces=[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
        elif prim.IsA(UsdGeom.Mesh):
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                approximation=UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
                if approximation not in ('none',None):
                    raise ValueError('Unsupported collider approximation: '+str(prim.GetPath()))
            mesh=UsdGeom.Mesh(prim)
            local=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
            counts=mesh.GetFaceVertexCountsAttr().Get()
            ids=mesh.GetFaceVertexIndicesAttr().Get()
            faces=[]
            cursor=0
            for count in counts:
                if count not in (3,4):
                    raise ValueError('Only triangle/quad static colliders are supported')
                faces.append(list(ids[cursor:cursor+count]))
                cursor+=count
        else:
            raise ValueError('Unsupported collider type '+prim.GetTypeName()+': '+str(prim.GetPath()))
        transform=np.asarray(cache.GetLocalToWorldTransform(prim))
        world=np.c_[local,np.ones(len(local))]@transform
        offset=len(points)
        points.extend(world[:,:3].tolist())
        for face in faces:
            for i in range(1,len(face)-1):
                indices.extend([offset+face[0],offset+face[i],offset+face[i+1]])
        paths.append(str(prim.GetPath()))
    if not points:
        raise ValueError('No collision geometry found')
    return np.asarray(points,dtype=np.float32),np.asarray(indices,dtype=np.int32),paths


class BatchRaycast:
    def __init__(self,stage):
        wp.init()
        self.device='cuda:0'
        self.stream=wp.Stream(self.device)
        self.buffers={}
        points,indices,self.paths=collider_triangles(stage)
        self.mesh=wp.Mesh(points=wp.array(points,dtype=wp.vec3,device=self.device),
                          indices=wp.array(indices,dtype=int,device=self.device))
        self.triangles=len(indices)//3
        self.raycast_batch(np.array([[0.,0.,1.]]),np.array([[0.,0.,-1.]]),40.)

    def raycast_batch(self,starts,directions,max_distance):
        origins=np.asarray(starts,dtype=np.float32)
        dirs=np.asarray(directions,dtype=np.float32)
        count=len(origins)
        if count not in self.buffers:
            host_origins=wp.empty(count,dtype=wp.vec3,device='cpu',pinned=True)
            host_dirs=wp.empty(count,dtype=wp.vec3,device='cpu',pinned=True)
            host_depth=wp.empty(count,dtype=float,device='cpu',pinned=True)
            self.buffers[count]=(host_origins,host_dirs,host_depth,
                wp.empty(count,dtype=wp.vec3,device=self.device),
                wp.empty(count,dtype=wp.vec3,device=self.device),
                wp.empty(count,dtype=float,device=self.device),
                host_origins.numpy(),host_dirs.numpy(),host_depth.numpy())
        ho,hd,hz,go,gd,gz,ov,dv,depth=self.buffers[count]
        # Reuse pinned staging and GPU buffers to avoid repeated allocation
        # and default-stream synchronization in the sensor loop.
        np.copyto(ov,origins)
        np.copyto(dv,dirs)
        wp.copy(go,ho,stream=self.stream)
        wp.copy(gd,hd,stream=self.stream)
        wp.launch(cast,dim=count,inputs=[self.mesh.id,go,gd,float(max_distance),gz],
                  device=self.device,stream=self.stream)
        wp.copy(hz,gz,stream=self.stream)
        wp.synchronize_stream(self.stream)
        hits=origins+dirs*depth[:,None]
        hits[depth<0]=np.nan
        return hits
