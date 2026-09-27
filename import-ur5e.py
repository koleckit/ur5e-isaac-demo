"""Isaac Sim 6 importer; local UR description assets, no network asset dependency."""
import argparse
from pathlib import Path
parser = argparse.ArgumentParser()
parser.add_argument('--urdf', default='ur5e.urdf')
parser.add_argument('--asset-dir', default='assets')
parser.add_argument('--prefix', default='ur5e')
args = parser.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
import json
from pathlib import Path
import omni.kit.app
from pxr import Usd, UsdPhysics

directory = Path(__file__).resolve().parent
urdf = Path(args.urdf)
if not urdf.is_absolute():
    urdf = directory / urdf
report = {'result': 'FAIL', 'urdf': str(urdf), 'calibration': 'factory calibrated' if args.prefix != 'ur5e' else 'generic UR5e, uncalibrated'}
try:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate('isaacsim.robot.schema', True)
    manager.set_extension_enabled_immediate('omni.scene.optimizer.core', True)
    from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
    config = URDFImporterConfig(
        urdf_path=str(urdf), usd_path=str(directory / args.asset_dir),
        ros_package_paths=[{'name': 'ur_description', 'path': '/opt/ros/lyrical/share/ur_description'}],
        fix_base=True, merge_fixed_joints=False, joint_drive_type='acceleration',
        joint_target_type='position', override_joint_stiffness=10000.0,
        override_joint_damping=1000.0)
    asset = URDFImporter(config).import_urdf()
    if not asset:
        raise RuntimeError('URDF importer returned no asset')
    stage = Usd.Stage.Open(asset)
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints = [p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    if len(joints) != 6 or not roots:
        raise RuntimeError(f'Invalid imported robot: roots={roots}, joints={joints}')
    report.update(result='PASS', usd_path=asset, articulation_roots=roots, revolute_joints=joints)
    (directory / f'{args.prefix}-asset.json').write_text(json.dumps(report, indent=2) + '\n')
    print('UR5E_IMPORT_DOD:', json.dumps(report))
except Exception as exc:
    report.update(result='FAIL', error=str(exc))
    raise
finally:
    (directory / f'{args.prefix}-import-dod.json').write_text(json.dumps(report, indent=2) + '\n')
    app.close()
