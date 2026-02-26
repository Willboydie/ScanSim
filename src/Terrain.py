import utils.defaults as defaults
import utils.constants as constants
import numpy as np
from noise import pnoise2 as noise
import open3d as o3d
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Terrain:
    def __init__(self):
        self.Lx = constants.SITE_LENGTH
        self.resolution = constants.RESOLUTION
        self.slope_x_deg = defaults.SLOPE_X_DEG
        self.slope_y_deg = defaults.SLOPE_Y_DEG
        self.base_wavelength = defaults.BASE_WAVELENGTH
        self.octaves = defaults.OCTAVES
        self.persistence = defaults.PERSISTENCE
        self.lacunarity = defaults.LACUNARITY
        self.roughness_amplitude = defaults.ROUGHNESS_AMPLITUDE
        
        self.points = None



    def generate(self, seed, plot=False, save=False):

        np.random.seed(seed)

        # Convert slope to gradient
        ax = np.tan(np.deg2rad(self.slope_x_deg))
        ay = np.tan(np.deg2rad(self.slope_y_deg))

        # Grid
        x = np.linspace(-self.Lx/2, self.Lx/2, self.resolution)
        y = np.linspace(-self.Lx/2, self.Lx/2, self.resolution)
        X, Y = np.meshgrid(x, y)

        # Base frequency scaling
        base_freq = 1.0 / self.base_wavelength

        # Generate fractal terrain
        Z_noise = np.zeros_like(X)

        for i in range(self.resolution):
            for j in range(self.resolution):
                nx = X[i, j] * base_freq
                ny = Y[i, j] * base_freq

                Z_noise[i, j] = noise(
                    nx, ny,
                    octaves=self.octaves,
                    persistence=self.persistence,
                    lacunarity=self.lacunarity,
                    repeatx=1024,
                    repeaty=1024,
                    base=seed
                )

        # Normalize noise to unit variance
        Z_noise -= np.mean(Z_noise)
        Z_noise /= np.std(Z_noise)

        # Scale roughness
        Z_noise *= self.roughness_amplitude

        # Add global slope
        Z_slope = ax * X + ay * Y

        # Final surface
        Z = Z_slope + Z_noise

        # Convention: x = North, y = East, z = Up (match geometry.py). Negate y so +y = East.
        self.points = np.column_stack((X.ravel(), -Y.ravel(), Z.ravel()))
        if save:
            self.save_to_file()
        if plot:
            self.plot()


    def save_to_file(self):
        if self.points is None:
            raise ValueError("Must generate a terrain first with generate().")
        timestamp = datetime.now().strftime("%H.%M_%d-%m")
        filename = f"../data/terrains/{timestamp}.bin"
        with open(filename, "wb") as f:
            for point in self.points:
                for component in point:
                    f.write(component.tobytes())
        print(f"Terrain saved to {filename}")


    def load_from_file(self, filename, plot=False):
        try:
            with open(f"../data/terrains/{filename}.bin", "rb") as f:
                self.points = np.fromfile(f, dtype=np.float64)
                self.points = self.points.reshape(-1, 3)
        except FileNotFoundError:
            print(f"Terrain file {f'../data/terrains/{filename}.bin'} not found")
            return None
        if plot:
            self.plot()


    def plot(self, points=None, beam_indices=None, point_index=None, path_indices=None, matplotlib_3d=False):
        if points is None:
            if self.points is None:
                raise ValueError("Must generate a terrain first with generate() or specify points using the 'points' argument.")
            points = self.points

        # Color by elevation (Z): low = blue/green, high = yellow/red
        end = len(points)
        if beam_indices is not None:
            end -= len(beam_indices)

        z_min, z_max = points[:end, 2].min(), points[:end, 2].max()
        z_norm = (points[:, 2] - z_min) / (z_max - z_min + 1e-8)
        # Simple colormap: blue (0) -> green -> yellow -> red (1)
        r = np.clip(2 * z_norm, 0, 1)
        g = np.clip(2 * (1 - np.abs(z_norm - 0.5)), 0, 1)
        b = np.clip(2 * (1 - z_norm), 0, 1)
        colors = np.column_stack((r, g, b))
        if beam_indices is not None:
            colors = colors.copy()
            colors[beam_indices] = [0.0, 0.0, 0.0]
        if point_index is not None:
            colors[point_index] = [1.0, 0.0, 0.0]

        if path_indices is not None:
            colors[path_indices] = [0.0, 0.0, 0.0]

        if matplotlib_3d:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection="3d")
            ax.scatter(
                points[:, 0], points[:, 1], points[:, 2],
                c=colors, s=1, alpha=0.8
            )
            ax.set_xlabel("x (North)")
            ax.set_ylabel("y (East)")
            ax.set_zlabel("z (Up)")
            ax.set_title(f"Synthetic {self.Lx}m x {self.Lx}m site Terrain (Point Cloud)")
            plt.tight_layout()
            plt.show()
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # Coordinate frame at origin (X=red, Y=green, Z=blue), size ~10% of site
        axis_size = self.Lx * 0.1
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=axis_size, origin=[0.0, 0.0, 0.0]
        )

        # Visualize
        o3d.visualization.draw_geometries(
            [pcd, coord_frame],
            window_name=f"Synthetic {self.Lx}m x {self.Lx}m site Terrain (Point Cloud)",
            width=1024,
            height=768,
        )


    def reduceResolution(self, divisor):
        """
        Downsample the point cloud on a regular 2D grid, reducing density
        equally in x and y by a factor of `divisor`.

        Assumes `self.points` represents a square grid flattened to shape
        (N, 3), ordered row major from `meshgrid` (as in `generate`).
        """
        
        if divisor <= 1:
            return self.points

        n_points = self.points.shape[0]
        # Infer current grid resolution (assumed square)
        res = int(round(np.sqrt(n_points)))
        if res * res != n_points:
            raise ValueError(
                f"Cannot reduce resolution: points ({n_points}) do not form a square grid"
            )

        # Reshape to (res, res, 3), then stride in both axes
        grid = self.points.reshape(res, res, 3)
        reduced_grid = grid[::divisor, ::divisor, :]

        # Flatten back to (N_reduced, 3)
        return reduced_grid.reshape(-1, 3)


    
    def createContours(self, divisor):
        f'''
        Creates a contour array with contour lines at intervals of {divisor} points.
        '''
        return self.points[::divisor]





if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Generate synthetic terrain point cloud.")
    parser.add_argument("--plot", action="store_true", default=False, help="Show Open3D point cloud viewer")
    parser.add_argument("--save", action="store_true", default=False, help="Save point cloud to binary file")
    args = parser.parse_args()

    terrain = Terrain()
    terrain.generate(seed=123, plot=args.plot, save=args.save)

    reduced_points = terrain.reduceResolution(20)
    terrain.plot(reduced_points)