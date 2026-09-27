"""Render the captured controller feedback; no interpolation of measured samples."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

directory = Path(__file__).resolve().parent
data = np.genfromtxt(directory / 'planned-vs-actual.csv', delimiter=',', names=True)
names = list(json.loads((directory / 'mock-execute-dod.json').read_text())['rmse_rad_by_joint'])
figure, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True, layout='constrained')
for index, (axis, name) in enumerate(zip(axes.flat, names)):
    planned = data[f'planned_{name}']
    actual = data[f'actual_{name}']
    axis.plot(data['time_s'], planned, color='#2563eb', linewidth=2, label='planned')
    axis.plot(data['time_s'], actual, color='#f97316', linestyle='--', linewidth=1.2, label='mock actual')
    axis.set_title(name.replace('_joint','').replace('_',' '), fontsize=11)
    axis.set_ylabel('position [rad]')
    axis.grid(alpha=0.2)
    axis.ticklabel_format(useOffset=False, axis='y')
    if index == 0:
        axis.legend(fontsize=9)
for axis in axes[-1]:
    axis.set_xlabel('controller trajectory time [s]')
figure.suptitle('UR5e mock — cuMotion plan vs FollowJointTrajectory feedback', fontsize=14)
figure.savefig(directory / 'planned-vs-actual.png', dpi=160)
plt.close(figure)
figure, axis = plt.subplots(figsize=(11,4), layout='constrained')
for name in names:
    axis.plot(data['time_s'], data[f'error_{name}']*1e6, label=name.replace('_joint',''))
axis.set(xlabel='controller trajectory time [s]', ylabel='actual − planned [µrad]', title='Mock tracking error — no physical UR connected')
axis.grid(alpha=0.2)
axis.legend(ncol=3, fontsize=9)
figure.savefig(directory / 'tracking-error.png', dpi=160)
plt.close(figure)
print('Saved planned-vs-actual.png and tracking-error.png')
