import Rangefinder
import ProjectionPath

class ScanSimulator:
    def __init__(self, _projection_path, _terrain, _pose):
        self.projection_path = _projection_path
        self.terrain = _terrain
        self.pose = _pose
        self.rangefinder = Rangefinder.Rangefinder(_terrain, _pose)
        
