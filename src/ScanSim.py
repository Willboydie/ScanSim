import Rangefinder
import ProjectionPath
import Terrain
from utils.constants import INCIDENCE_TOLERANCE_RADIUS

class ScanSimulator:
    def __init__(self):
        self.projectionPath = ProjectionPath.ProjectionPath()
        self.terrain = Terrain.Terrain()
        self.rangefinder = Rangefinder.Rangefinder(self.terrain)


    def plotProjectionPath(self):
        path_points = self.projectionPath.points
        if path_points is None:
            raise ValueError("First generate a projection path with projectionPath.generate()")
        terrain_points = self.terrain.points
        if terrain_points is None:
            raise ValueError("First generate a terrain with terrain.generate()")

        path_indices = []
        for point in self.projectionPath.points:
            x, y = point
            idx = np.flatnonzero(
            np.isclose(terrain_points[:, 0], x, atol=INCIDENCE_TOLERANCE_RADIUS/2) &
            np.isclose(terrain_points[:, 1], y, atol=INCIDENCE_TOLERANCE_RADIUS/2)
            )
            path_indices.extend(idx.tolist())
        path_indices = np.array(path_indices)

        self.terrain.plot(path_indices=path_indices)

    def plotTerrain(self):
        self.terrain.plot()

    def saveTerrain(self, filename=None):
        self.terrain.save_to_file(filename)

    def loadTerrain(self, filename, plot=False):
        self.terrain.load_from_file(filename, plot)



if __name__ == "__main__":

    import numpy as np
    scansim = ScanSimulator()
    scansim.terrain.octaves = 100
    scansim.terrain.roughness_amplitude = 0.5
    scansim.terrain.generate(seed=12)
    scansim.projectionPath.generate()
    scansim.plotProjectionPath()
    
    # scansim.rangefinder.pose = [0, 0, 4, 0, 0, 0]
    # scansim.rangefinder.takeMeasurement(0.0, -np.pi/4, plot=True)
