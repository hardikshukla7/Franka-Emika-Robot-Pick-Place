import numpy as np

def calcAngDiff(R_des, R_curr):
    """
    Helper function for the End Effector Orientation Task. Computes the axis of rotation
    from the current orientation to the target orientation.

    This data can also be interpreted as an end effector velocity which will
    bring the end effector closer to the target orientation.

    INPUTS:
    R_des - 3x3 numpy array representing the desired orientation from
    end effector to the world.
    R_curr - 3x3 numpy array representing the "current" end effector orientation.

    OUTPUTS:
    omega - 3-element numpy array containing the axis of the rotation from
    the current frame to the end effector frame. The magnitude of this vector
    must be sin(angle), where angle is the angle of rotation around this axis.
    """

    # Calculate the relative rotation matrix that describes the rotation from R_curr to R_des
    R_rel = np.dot(np.transpose(R_curr), R_des)
    S = (1/2) * (R_rel - R_rel.T)
    a = np.array([S[2, 1], S[0, 2], S[1, 0]])
    angle = np.arccos((np.trace(R_rel) - 1) / 2)  # Angle of rotation in radians
    omega_mag = np.sin(angle)
    omega = R_curr@a

    return omega
