import numpy as np
from Path import Path
import matplotlib.pyplot as plt
import utils.defaults as defaults


class Lissajous(Path):
    def __init__(self):
        super().__init__()
        self.centre = defaults.PATH_CENTRE
        self.width = defaults.PATH_WIDTH
        self.height = defaults.PATH_HEIGHT
        self.theta_period = defaults.PATH_THETA_PERIOD
        self.phi_period = defaults.PATH_PHI_PERIOD
        self.phase_offset = defaults.PATH_PHASE_OFFSET
        self.num_points = defaults.PATH_NUM_POINTS
        self.duration = defaults.PATH_DURATION

    def generate(self):
        t = np.linspace(0, self.duration, self.num_points)

        omega_theta = 2 * np.pi / self.theta_period
        omega_phi = 2 * np.pi / self.phi_period

        x = self.centre[0] + (self.width / 2) * np.sin(omega_theta * t + self.phase_offset)
        y = self.centre[1] + (self.height / 2) * np.sin(omega_phi * t)

        self.points = np.column_stack((x, y))
        return self.points

    def plot(self):
        if self.points is None:
            raise ValueError("First generate a path with generate()")
        plt.scatter(self.points[:, 0], self.points[:, 1])
        plt.axis("equal")
        plt.show()

if __name__ == "__main__":
    lissajous = Lissajous()
    lissajous.generate()
    lissajous.plot()