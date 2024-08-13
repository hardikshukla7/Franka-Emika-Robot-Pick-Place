import numpy as np
from lib.calcJacobian import calcJacobian
def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
         are infeasible, then dq should minimize the least squares error. If v_in
         and omega_in have multiple solutions, then you should select the solution
         that minimizes the l2 norm of dq
    """

    dq = np.zeros((1, 7))

    v_in = v_in.reshape((3, 1))
    omega_in = omega_in.reshape((3, 1))
    J = calcJacobian(q_in)
    E = np.vstack((v_in, omega_in))
    nan_indices = np.where(np.isnan(E))[0]  # Find the row indices of NaN elements

# Delete rows with NaN elements from the Jacobian matrix
    J = np.delete(J, nan_indices, axis=0)

# Remove NaN elements from E
    V = E[~np.isnan(E)]

    dq, residuals, rank, s = np.linalg.lstsq(J, V, rcond=None)

    return dq
