from Lissajous import Lissajous
import ScanSim as ScanSim
import numpy as np
import open3d as o3d


if __name__ == "__main__":
    scansim = ScanSim.ScanSimulator()
    scansim.terrain.generate(seed=12)
    scansim.rangefinder.pose = [0, 0, 4, 0, 0, 0]
    path = Lissajous()
    path.generate()

    point_cloud = []
    for scan_angle in path.points:
        theta, phi = scan_angle
        point = scansim.rangefinder.takeMeasurement(theta, phi)
        if point is not None:
            point_cloud.append(point)


    point_cloud = np.array(point_cloud)
    print(point_cloud.shape)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud.astype(np.float64))

    z = point_cloud[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
    darkness = 0.5
    r = np.clip(2 * z_norm, 0, 1) * darkness
    g = np.clip(2 * (1 - np.abs(z_norm - 0.5)), 0, 1) * darkness
    b = np.clip(2 * (1 - z_norm), 0, 1) * darkness
    pcd.colors = o3d.utility.Vector3dVector(np.column_stack((r, g, b)))

    o3d.visualization.draw_geometries(
        [pcd],
        window_name="Lissajous Scan Point Cloud",
        width=1024,
        height=768,
    )
