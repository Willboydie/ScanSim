from types import NoneType
import Terrain
import ProjectionPath
import BeamVector as Beam
import utils.geometry as geometry
import numpy as np
import utils.defaults as defaults
import time

class RangeFinder:
    def __init__(self, _terrain, _pose):
        self.terrain = _terrain
        self.pose = _pose
        self.resReductionFactor = defaults.RES_REDUCTION_FACTOR_FOR_TARGET_REGION
        if self.terrain.resolution % self.resReductionFactor != 0 or self.terrain.resolution % 2 != 0:
            raise ValueError("Terrain resolution must be divisible by the reduction factor, and must be even.")
        self.bin_offsets = self.precomputeBinOffsets()
        self.beam_plus_terrain_points = None
        self.reduced_terrain_points = None


    def takeMeasurement(self, theta, phi):
        if theta <= -np.pi/2 or theta >= np.pi/2:
            raise ValueError("Theta must be between -pi/2 and pi/2.")
        if phi <= -np.pi/2 or phi >= np.pi/2:
            raise ValueError("Phi must be between -pi/2 and pi/2.")

        beam = Beam.Beam(self.pose, theta, phi)
        reduced_res_terrain_points =self.terrain.reduceResolution(self.resReductionFactor)
        reduced_target_region_points, reduced_target_region_indices = self.getIncidentPoints(reduced_res_terrain_points, beam.origin, beam.vector)
        if reduced_target_region_points is None:
            return None, None
        target_region_indices = self.getFullDensityIndices(reduced_target_region_indices)
        target_region_points = self.terrain.points[target_region_indices]
        return self.getIncidentPoints(target_region_points, beam.origin, beam.vector, indices=target_region_indices)


    def getIncidentPoints(
            self,
            points,
            origin,
            beam_vector,
            indices=None,
            radius=defaults.INCIDENCE_TOLERANCE_RADIUS,
            max_range=np.inf
            ):

        d = beam_vector / np.linalg.norm(beam_vector)

        v = points - origin

        # --- projection ---
        t = v @ d

        forward = (t >= 0) & (t <= max_range)
        if not np.any(forward):
            return None, None

        v = v[forward]
        t = t[forward]
        idx_forward = np.nonzero(forward)[0]

        # --- perpendicular distance ---
        vv = np.einsum('ij,ij->i', v, v)
        dist2 = vv - t*t

        inside = dist2 <= radius*radius
        if not np.any(inside):
            return None, None

        idx_valid = idx_forward[inside]
        dist2 = dist2[inside]

        # --- select nearest ---
        local = np.argmin(dist2)

        if indices is not None:
            return points[idx_valid[local]], indices[idx_valid[local]]

        return points[idx_valid[local]], idx_valid[local]


    def getFullDensityIndices(self, reduced_indices):
        reduced_indices = np.atleast_1d(np.asarray(reduced_indices, dtype=int))
        res = self.terrain.resolution
        r = self.resReductionFactor

        # reduced grid width
        n = res // r

        # reduced index → reduced grid coordinates
        rr, rc = divmod(reduced_indices, n)

        # centre location in full grid
        centre_indices = (
            (rr * r + r // 2) * res +
            (rc * r + r // 2)
        )

        # broadcast: each base + all bin offsets -> (n_cells, r*r), then flatten
        return (centre_indices[:, np.newaxis] + self.bin_offsets).ravel().astype(int)


    def precomputeBinOffsets(self):

        r = self.resReductionFactor
        res = self.terrain.resolution

        half = r // 2

        dr, dc = np.meshgrid(
            np.arange(-half, half),
            np.arange(-half, half),
            indexing="ij"
        )

        return dr.ravel() * res + dc.ravel()

    
    def addBeamPointsForPlotting(self, theta, phi, terrain_points=None):
        if terrain_points is None:
            terrain_points = self.terrain.points
        beam = Beam.Beam(self.pose, theta, phi)
        self.beam_plus_terrain_points = np.vstack((terrain_points, beam.plotting_points))

    
    def runSpeedTest(self): # Average Measurement Time = 450us
        dt_sum = 0
        N = 10000
        for i in range(N):
            start = time.time()
            incident_point, incident_index = self.takeMeasurement(theta=theta, phi=phi)
            dt = time.time() - start
            dt_sum += dt

        print(f"Average time taken per measurement: {dt_sum/N:.6f}")



if __name__ == "__main__":

    theta = -0.5
    phi = -0.2

    terrain = Terrain.Terrain()
    terrain.generate(seed=123)
    pose = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
    rangefinder = RangeFinder(terrain, pose)
    rangefinder.runSpeedTest()
    incident_point, incident_index = rangefinder.takeMeasurement(theta=theta, phi=phi)
    rangefinder.addBeamPointsForPlotting(theta=theta, phi=phi)
    if incident_index is None:
        highlight_indices = np.arange(len(rangefinder.terrain.points), len(rangefinder.beam_plus_terrain_points))
    else:
        highlight_indices = np.concatenate((np.array([incident_index]), np.arange(len(rangefinder.terrain.points), len(rangefinder.beam_plus_terrain_points))))
    terrain.plot(points=rangefinder.beam_plus_terrain_points, highlight_indices=highlight_indices, matplotlib_3d=False)
    