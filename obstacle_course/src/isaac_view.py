"""Render the shared boxes and compare a preview TCP path with a straight reference."""
import numpy as np
from pxr import Gf, UsdGeom

class View:
    def __init__(self, stage, course, kinematics, model):
        self.stage, self.course, self.kinematics, self.model = stage,course,kinematics,model
        UsdGeom.Xform.Define(stage,'/World/ObstacleCourse')
        self.boxes = {}
        for box in course['obstacles']:
            cube = UsdGeom.Cube.Define(stage,'/World/ObstacleCourse/'+box['id'])
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*box['position']))
            x,y,z,w = box['quaternion_xyzw']
            cube.AddOrientOp().Set(Gf.Quatf(w,Gf.Vec3f(x,y,z)))
            cube.AddScaleOp().Set(Gf.Vec3f(*box['size']))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*box['color'])])
            self.boxes[box['id']] = cube

    def paths(self, trajectory):
        times = np.linspace(0,trajectory['points'][-1]['time_from_start_s'],201)
        xyz = [self.kinematics.world_transforms(dict(zip(trajectory['joint_names'],
            self.model.sample_trajectory(trajectory,t))))['tool0'][:3,3].tolist() for t in times]
        for name,points,color,width in [('PlannedTCP',xyz,(1.,.65,.05),.007),
                ('StraightReference',[xyz[0],np.asarray(trajectory['target_tcp']['matrix_column_vectors'])[:3,3].tolist()],(.7,.7,.7),.003)]:
            curve = UsdGeom.BasisCurves.Define(self.stage,'/World/ObstacleCourse/'+name)
            curve.CreateTypeAttr('linear');curve.CreateCurveVertexCountsAttr([len(points)])
            curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
            curve.CreateWidthsAttr([width]);curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        start = UsdGeom.Sphere.Define(self.stage,'/World/ObstacleCourse/StartTCP')
        start.CreateRadiusAttr(.015);start.CreateDisplayColorAttr([Gf.Vec3f(.1,.8,1.)])
        start.ClearXformOpOrder();start.AddTranslateOp().Set(Gf.Vec3d(*xyz[0]))

    def clear_path(self):
        for name in ('PlannedTCP','StraightReference','StartTCP'):
            self.stage.RemovePrim('/World/ObstacleCourse/'+name)

    def verify(self):
        errors = []
        for box in self.course['obstacles']:
            cube = self.boxes[box['id']]
            transform = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(cube.GetPrim())).T
            errors.append(float(np.max(np.abs(transform[:3,3]-np.asarray(box['position'])))))
            actual_size = np.linalg.norm(transform[:3,:3],axis=0)*cube.GetSizeAttr().Get()
            errors.append(float(np.max(np.abs(actual_size-np.asarray(box['size'])))))
            rotation = transform[:3,:3] / actual_size
            from world import matrix
            errors.append(float(np.max(np.abs(rotation-matrix(box['position'],box['quaternion_xyzw'])[:3,:3]))))
        if max(errors,default=0) > 1e-6:
            raise RuntimeError('Isaac box geometry differs from the shared YAML')
        return {'obstacles':len(self.boxes),'max_geometry_error':max(errors,default=0)}
