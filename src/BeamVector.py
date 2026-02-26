import numpy as np
import utils.geometry as geometry
import utils.constants as constants

class Beam:
    def __init__(self, pose, theta, phi):
        self.x_origin, self.y_origin, self.z_origin, self.roll, self.pitch, self.yaw = pose
        self.origin = np.array([self.x_origin, self.y_origin, self.z_origin])
        self.theta = theta
        self.phi = phi
        self.vector = geometry.calculateBeamVector(self.roll, self.pitch, self.yaw, self.theta, self.phi)
        self.plotting_points = self.getPlottingPoints()

    def getPlottingPoints(self, distance=constants.BEAM_PLOTTING_DISTANCE, density=constants.BEAM_PLOTTING_DENSITY):
        x_values = np.linspace(self.x_origin, self.x_origin + distance * self.vector[0], density)
        y_values = np.linspace(self.y_origin, self.y_origin + distance * self.vector[1], density)
        z_values = np.linspace(self.z_origin, self.z_origin + distance * self.vector[2], density)
        return np.column_stack((x_values, y_values, z_values)).astype(np.float64)