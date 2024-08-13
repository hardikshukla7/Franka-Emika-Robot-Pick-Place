import numpy as np
from math import pi, cos, sin
import math

class FK():

    def __init__(self):
        self.b = np.array     ([0,     -pi/2,    pi/2,   pi/2,          pi/2,   -pi/2,         pi/2,     0           ])
        self.d = np.array     ([0.141, 0.192,    0,      (0.195+0.121), 0,      (0.259+0.125), 0,       (0.051+0.159)])
        self.a = np.array     ([0,     0,        0,      0.0825,        0.0825,  0,            0.088,    0           ])
        self.offset = np.array([0,     0,        0,      0,             pi/2,    0,           -pi/2,     0           ])
        self.q1 = np.array    ([0,     0,        0,      0,            -pi/2,    0,            pi/2,     pi/4        ])

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout
        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """
        # Your Lab 1 code starts here
        #theta = q, alpha = b, d and a
        q = np.insert(q,0,0)
        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)
        A = np.zeros((4,4))
        A1 = np.identity(4)
        a_all = np.zeros((8,4,4))
        for i in range(0,8):
            theta = q[i]-self.q1[i]+self.offset[i]
            beta = self.b[i]
            A[0] = [cos(theta), -sin(theta)*cos(beta), sin(theta)*sin(beta), self.a[i]*cos(theta)]
            A[1] = [sin(theta), cos(theta)*cos(beta), -cos(theta)*sin(beta), self.a[i]*sin(theta)]
            A[2] = [0, sin(beta), cos(beta), self.d[i]]
            A[3] = [0,0,0,1]
            T0e = np.dot(A1,A)
            A1=T0e
            a_all[i]=T0e
            col = T0e[:,-1]
            col = np.delete(col,3)
            jointPositions[i] = col
        jointPositions[2] = jointPositions[2] + 0.195*np.matmul(a_all[2,:-1,:-1],np.array([0,0,1]))
        jointPositions[4] = jointPositions[4] + 0.125*np.matmul(a_all[4,:-1,:-1],np.array([0,0,1]))
        jointPositions[5] = jointPositions[5] - 0.015*np.matmul(a_all[5,:-1,:-1],np.array([0,0,1]))
        jointPositions[6] = jointPositions[6] + 0.051*np.matmul(a_all[6,:-1,:-1],np.array([0,0,1]))

        # Your code ends here

        return jointPositions, a_all, T0e



if __name__ == "__main__":


    # matches figure in the handout
    fk = FK()
    q = np.array([0,0,0,0,0,0,0])
    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
