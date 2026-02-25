import numpy as np
import geometry
import open3d as o3d

x=0
y=0
z=0
roll=0
pitch=0
yaw=0
pose = [x, y, z, roll, pitch, yaw]

Lx = 30
resolution = 1500
terrain_inc = Lx / resolution


with open("data/terrains/terrain_seed_41.bin", "rb") as f:
    terrain = np.fromfile(f, dtype=np.float64)
    terrain = terrain.reshape(-1, 3)


def get_beam_vector(pose, theta, phi):
    roll_device, pitch_device, yaw_device = pose[3], pose[4], pose[5]
    return geometry.calculateBeamVector(roll_device, pitch_device, yaw_device, theta, phi)


def _3d_diff(x1, y1, z1, x2, y2, z2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)


