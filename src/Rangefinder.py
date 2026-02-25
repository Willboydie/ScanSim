import Terrain
import ProjectionPath
import BeamVector
import utils.geometry as geometry
import numpy as np
import utils.defaults as defaults

class RangeFinder:
    def __init__(self, terrain, projection_path, pose):
        self.terrain = terrain
        self.projection_path = projection_path
        self.pose = pose
        self.resReductionFactor = defaults.RES_REDUCTION_FACTOR_FOR_TARGET_REGION
        if self.terrain.resolution % self.resReductionFactor != 0:
            raise ValueError("Terrain resolution must be divisible by the reduction factor")
        self.bin_offsets = self.precomputeBinOffsets()


    def takeMeasurement(self, theta, phi):
        beam = BeamVector.BeamVector(self.pose, theta, phi)
        reduced_target_region_indices = self.findReducedTargetRegion(beam.vector)        
        full_density_target_region_indices = self.getFullDensityIndices(reduced_target_region_indices)
        incident_point, incident_index = self.pinpointIncidentPoint(full_density_target_region_indices, beam.vector)
        if incident_index is not None:
            self.terrain.plot(points=self.terrain.points, highlight_indices=[incident_index])
        else:
            print("No incident point found")
        return incident_point, incident_index

    
    def findReducedTargetRegion(self, beam_vector):
        reducedResolutionTerrain = self.terrain.reduceRes(self.resReductionFactor)
        origin = np.array([self.pose[0], self.pose[1], self.pose[2]])
        half_bin_size = (self.terrain.Lx / self.terrain.resolution / 2) * self.resReductionFactor
        possible_indices, possible_points = geometry.findPointsOnBeamAngle(reducedResolutionTerrain, beam_vector, self.pose, half_bin_size)

        target_region_indices = np.array([])
        for i, point in enumerate(possible_points):
           if self.isInTargetRegion(point, beam_vector, origin, half_bin_size):
                target_region_indices = np.append(target_region_indices, possible_indices[i])          
        return target_region_indices

    def pinpointIncidentPoint(self, target_indices, beam_vector):
        if len(target_indices) == 0:
            return None, None
        target_points = self.terrain.points[target_indices]
        diff = target_points - self.pose[:3]
        idx = np.argmin(diff @ beam_vector)
        return target_points[idx], target_indices[idx]

    def getFullDensityIndices(self, reduced_indices):
        res = self.terrain.resolution
        r = self.resReductionFactor

        # reduced grid width
        n = res // r

        # reduced index → reduced grid coordinates
        rr, rc = divmod(reduced_indices, n)

        # top-left index of the bin in full resolution (one per reduced cell)
        base_indices = (rr * r) * res + (rc * r)

        # broadcast: each base + all bin offsets -> (n_cells, r*r), then flatten
        return (base_indices[:, np.newaxis] + self.bin_offsets).ravel().astype(int)


    def isInTargetRegion(self, point, beam_vector, origin, half_bin_size):
        v = point - origin
        d = np.dot(v, beam_vector)         # distance along beam (assuming unit beam_vector)
        ref = origin + d * beam_vector     # point on beam
        return np.all(np.abs(point - ref) < half_bin_size)


    def precomputeBinOffsets(self):
        r = self.resReductionFactor
        res = self.terrain.resolution

        dr, dc = np.meshgrid(
            np.arange(r),
            np.arange(r),
            indexing="ij"
        )

        return (dr.ravel() * res + dc.ravel())



if __name__ == "__main__":
    terrain = Terrain.Terrain()
    terrain.generate(seed=123)
    projection_path = ProjectionPath.ProjectionPath()
    projection_path.generate()
    pose = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]
    range_finder = RangeFinder(terrain, projection_path, pose)
    range_finder.takeMeasurement(theta=0.0, phi=-np.pi/4)