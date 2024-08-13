import numpy as np
from lib.calculateFK import FK
from math import pi, cos, sin

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    fk = FK()
    jointPositions, a_all, T0e = fk.forward(q_in)
    Jv = np.zeros((7,3))
    num1 = jointPositions.shape[0]-1
    mat1 = jointPositions[num1]
    I = np.eye(3)
    for i in range(7):
        R = a_all[i][:-1]
        mat2 = jointPositions[i]
        if i==0 :
            Jv[0] = np.cross(I[:,-1],mat1-mat2) #joint 1 is jv[0]
        else:
            Jv[i] = np.cross(R[:,-2], mat1-mat2) #from joint 2
    Jw = np.zeros((7,3))
    for i in range(7):
        if i==0 :
            Jw[0] = np.array([0, 0, 1])
        else:
            R = a_all[i][:-1]
            Jw[i] = R[:,-2]

    J = np.vstack((Jv.transpose(),Jw.transpose()))
    return J


if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))
