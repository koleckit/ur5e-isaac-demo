"""Smoke test local Isaac Sim renderer and physics without online robot assets."""
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'renderer': 'RayTracedLighting'})
import json
from pathlib import Path
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

output = Path(__file__).resolve().parent / 'isaac-sim-dod.json'
report = {'result': 'FAIL'}
try:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    cube = world.scene.add(DynamicCuboid(prim_path='/World/DoDCube', name='dod_cube',
        position=np.array([0.0, 0.0, 1.0]), scale=np.array([0.1, 0.1, 0.1])))
    world.reset()
    initial_position, _ = cube.get_world_pose()
    for _ in range(120):
        world.step(render=True)
    final_position, _ = cube.get_world_pose()
    if not np.all(np.isfinite(final_position)) or not 0.02 < final_position[2] < 0.2:
        raise RuntimeError(f'Unexpected cube position after physics: {final_position}')
    report.update(result='PASS', rendered_steps=120,
                  initial_position=initial_position.tolist(), final_position=final_position.tolist())
    print('ISAAC_SIM_DOD:', json.dumps(report))
except Exception as exc:
    report['error'] = str(exc)
    raise
finally:
    output.write_text(json.dumps(report, indent=2) + '\n')
    app.close()
