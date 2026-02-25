import numpy as np
import utils.geometry as geometry

class BeamVector:
    def __init__(self, pose, theta, phi):
        self.x_origin, self.y_origin, self.z_origin, self.roll, self.pitch, self.yaw = pose
        self.theta = theta
        self.phi = phi
        self.vector = geometry.calculateBeamVector(self.roll, self.pitch, self.yaw, self.theta, self.phi)
        self.front = np.array([self.x_origin, self.y_origin, self.z_origin])

    # def traverse(self, distance):
    #     self.front += distance * self.vector