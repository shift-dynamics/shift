#!/usr/bin/env python

import numpy as np

if __name__ == "__main__":
    np.set_printoptions(suppress=True)

    # A = np.matrix([[1620.0, -675.0, -30.0],
    #                [-675.0, 300.0, 60],
    #                [-30.0, 60.0, 4.0]])
    #
    # D, v = np.linalg.eig(A)
    # print("A=", A)
    # print("D=", D)
    # print("v=", v)

    L = np.matrix([[5.0, 0.0, 0.0, 0.0],
                   [3.0, 4.0, 0.0, 0.0],
                   [5.0, 6.0, 3.0, 0.0],
                   [8.0, 9.0, 10.0, 2.0]])
    print('L=')
    print(L)

    print('L * L^T =')
    print(L * L.transpose())

    D = np.matrix(np.diag([i for i in range(1, 12)]))
    print('D =')
    print(D)

    Q = np.matrix([
        [4., 3.,  7.,  9.,  11., 16., 12., 15., 17., 19., 13.],
        [7., 13., 23., 17., 9.,  11., 17., 3.,  2.,  5.,  6.],
        [9., 2.,  17., 11., 5.,  8.,  13., 4.,  3.,  2.,  1.],
        [1., 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.]])

    X = Q * D * Q.transpose()
    print('Q * D * Q.transpose()')
    print(X)
