"""Validate the shared geometry boundary, including rotated boxes and invalid inputs."""
import copy
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import yaml

course_root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(course_root/'src'))
from world import load, signed_box_distance

class WorldTests(unittest.TestCase):
    def setUp(self):
        self.filename=course_root/'config/obstacle_course.yaml'
        self.raw=yaml.safe_load(self.filename.read_text())

    def changed(self, value):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'course.yaml';path.write_text(yaml.safe_dump(value))
            return load(path,'single_wall')

    def test_scenarios_and_geometry_digest(self):
        counts=[len(load(self.filename,s)['obstacles']) for s in ('single_wall','corridor','gate_3d')]
        self.assertEqual(counts,[1,3,3])
        course=load(self.filename,'single_wall')
        changed=copy.deepcopy(self.raw)
        changed['scenarios']['single_wall']['obstacles']['wall']['size'][0]+=.01
        self.assertNotEqual(course['geometry_sha256'],self.changed(changed)['geometry_sha256'])

    def test_reject_invalid_geometry(self):
        for key,value in [('size',[.1,0.,.2]),('position',[float('nan'),0.,0.]),
                ('quaternion_xyzw',[0.,0.,0.,2.]),('type','mesh')]:
            raw=copy.deepcopy(self.raw);raw['scenarios']['single_wall']['obstacles']['wall'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.changed(raw)

    def test_oriented_box_distance(self):
        box={'position':[1.,2.,3.],'size':[2.,.2,.4],
             'quaternion_xyzw':[0.,0.,float(np.sin(np.pi/4)),float(np.cos(np.pi/4))]}
        self.assertAlmostEqual(signed_box_distance([1.,2.,3.],box),-.1)
        self.assertAlmostEqual(signed_box_distance([1.2,2.,3.],box),.1)
        self.assertAlmostEqual(signed_box_distance([1.,3.2,3.],box),.2)

if __name__=='__main__':unittest.main()
