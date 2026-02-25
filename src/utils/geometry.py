import numpy as np

# The following system makes the following assumptions:
# - X = North, Y = East, Z = Up
# - roll is anticlockwise about X+
# - pitch is anticlockwise about Y+
# - yaw is clockwise about Z+
# - The local forward "neutral" vector is [1,0,0]
# - Rotations are performed in the order of yaw (1st) -> pitch (2nd) -> roll (3rd).

# In this project, 
# - theta will refer to the yaw of the rangefinder beam
# - phi will refer to the pitch of the rangefinder beam
# - roll, pitch and yaw will refer to the device's orientation (as laid out above)
# - Note that for this arrangement, roll of the rangefinder beam has no effect on the beam's (world) vector


def Rx(phi):
    '''
    Anticlockwise rotation around X+
    '''
    c, s = np.cos(phi), np.sin(phi)
    return np.array([
        [1, 0, 0],
        [0, c, s],
        [0, -s,  c]
    ])

def Ry(theta):
    '''
    Anticlockwise rotation around Y+
    '''
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, -s],
        [ 0, 1, 0],
        [s, 0, c]
    ])

def Rz(psi):
    '''
    Clockwise rotation around Z+
    '''
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def rotation(phi, theta, psi):
    # yaw → pitch → roll
    return Rx(phi) @ Ry(theta) @ Rz(psi)

def calculateBeamVector(
    roll_device, pitch_device, yaw_device, theta, phi
):
    Rm = rotation(roll_device, pitch_device, yaw_device)
    Rg = rotation(0, phi, theta)

    forward = np.array([1,0,0])

    return Rm @ Rg @ forward


def findPointsOnBeamAngle(terrain_points, beam_vector, pose, half_bin_size):
    '''
    Finds the terrain bins which lie on the plane parallel to the X and Y components of the beam vector (those pertaining to theta)
    For beam vector [a, b, c] this plane is defined by the equation ay - bx = 0
    The plane is offset by the position of the device (ie the origin of the beam)
    Returns the indices of the terrain bins.
    '''
    a, b = beam_vector[0], beam_vector[1]
    offset_x, offset_y = pose[0], pose[1]
    indices = []
    points = []
    for i, point in enumerate(terrain_points):
        x, y, z = point
        if abs(a*(y - offset_y) - b*(x - offset_x)) < half_bin_size:
            indices.append(i)
            points.append(point)
    return indices, points


def _3d_diff(x1, y1, z1, x2, y2, z2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)




# if __name__ == "__main__":
    # print(calculateBeamVector(roll_device=0.3, pitch_device=0.0, yaw_device=0.0, theta=0.3, phi=0.0))

