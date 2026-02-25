import utils.defaults as defaults
import numpy as np
import matplotlib.pyplot as plt
from math import radians, cos, sin

class ProjectionPath:
    def __init__(self):
        self.centre = np.array(defaults.PROJECTION_PATH_CENTRE)
        self.width = defaults.PROJECTION_PATH_WIDTH
        self.height = defaults.PROJECTION_PATH_HEIGHT
        self.orientation = defaults.PROJECTION_PATH_ORIENTATION
        self.increment = defaults.PROJECTION_PATH_INCREMENT
        self.points = None

    def generate(self, plot=False):
        cx, cy = self.centre
        dx = self.width / 2
        dy = self.height / 2

        # Generate the rectangle
        x_range = np.arange(cx - dx, cx + dx, self.increment)
        y_range = np.arange(cy - dy, cy + dy, self.increment)
        side1 = np.column_stack((x_range, np.full_like(x_range, cy + dy)))
        side2 = np.column_stack((x_range, np.full_like(x_range, cy - dy)))
        side3 = np.column_stack((np.full_like(y_range, cx - dx), y_range))
        side4 = np.column_stack((np.full_like(y_range, cx + dx), y_range))
        self.points = np.vstack((side1, side2, side3, side4))

        # Rotate rectangle clockwise by orientation (0–360deg) about centre
        angle_rad = radians(self.orientation)
        r = np.array([[cos(angle_rad), sin(angle_rad)], [-sin(angle_rad), cos(angle_rad)]])
        self.points = (self.points - self.centre) @ r.T + self.centre

        if plot:
            self.plot()


    def plot(self):
        plt.scatter(self.points[:, 0], self.points[:, 1])
        plt.axis("equal")
        plt.show()


if __name__ == "__main__":
    projection_path = ProjectionPath()
    projection_path.generate(plot=True)