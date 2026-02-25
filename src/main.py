import Terrain
import ProjectionPath

if __name__ == "__main__":
    terrain = Terrain.Terrain()
    projection_path = ProjectionPath.ProjectionPath()
    terrain.slope_x_deg = 30
    terrain.generate(seed=41, plot=True)