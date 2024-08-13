import numpy as np
from lib.IK_velocity import IK_velocity
from lib.calcJacobian import calcJacobian

"""
Lab 3
"""

def IK_velocity_null(q_in, v_in, omega_in, b):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :param b: 7 x 1 Secondary task joint velocity vector
    :return:
    dq + null - 1 x 7 vector corresponding to the joint velocities + secondary task null velocities
    """

    ## STUDENT CODE GOES HERE
    null = np.zeros((1, 7))
    b = b.reshape((7, 1))

    v_in = np.array(v_in)

    omega_in = np.array(omega_in)

    dq = IK_velocity(q_in,v_in,omega_in)

    J = calcJacobian(q_in)

    v_in = v_in.reshape((3, 1))
    omega_in = omega_in.reshape((3, 1))
    E = np.vstack((v_in, omega_in))
    nan_indices = np.where(np.isnan(E))[0]
    J = np.delete(J, nan_indices, axis=0)
    J1 = np.linalg.pinv(J, rcond = 0)
    null_projection = np.identity(7) - np.dot(J1, J)
    null = np.dot(null_projection, b)
    print(null)
    print(dq)
    return dq.reshape((7,1)) + null
