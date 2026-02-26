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


def Rx(psi):
    '''
    Anticlockwise rotation around X+
    '''
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [1, 0, 0],
        [0, c, s],
        [0, -s,  c]
    ])

def Ry(phi):
    '''
    Clockwise rotation around Y+
    '''
    c, s = np.cos(phi), np.sin(phi)
    return np.array([
        [ c, 0, -s],
        [ 0, 1, 0],
        [s, 0, c]
    ])

def Rz(theta):
    '''
    clockwise rotation around Z+
    '''
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def rotation(psi, phi, theta):
    # yaw → pitch → roll
    return Rx(psi) @ Ry(phi) @ Rz(theta)

def calculateBeamVector(roll, pitch, yaw, theta, phi):
    Rm = rotation(roll, pitch, yaw)
    Rg = rotation(0, phi, theta)

    forward = np.array([1,0,0])

    return Rm @ Rg @ forward


