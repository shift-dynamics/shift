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

    D = np.matrix(np.diag([i for i in range(1, 12)]))
    print('D =')
    print(D)

    Q = np.matrix([
        [4., 3.,  7.,  9.,  11., 16., 12., 15., 17., 19., 13.],
        [7., 13., 23., 17., 9.,  11., 17., 3.,  2.,  5.,  6.],
        [9., 2.,  17., 11., 5.,  8.,  13., 4.,  3.,  2.,  1.],
        [1., 1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.,  1.]])

    X = Q * D * Q.transpose()
    print('X = Q * D * Q.transpose() =')
    print(X)

    print('m = diag(Q.transpose() * inv(X) * Q) =')
    print(np.diagonal(Q.transpose() * np.linalg.inv(X) * Q))

    print("L:")
    L = np.matrix([[1.0, 0.0, 0.0, 0.0], [2.0, 1.0, 0.0, 0.0],
                  [3.0, 4.0, 1.0, 0.0], [5.0, 6.0, 7.0, 1.0]])
    print(L)

    b = np.matrix([10.0, 20.0, 19.0, 4.0]).transpose()
    print('b:', b)

    print('L^-1 * b:')
    print(np.linalg.inv(L) * b)

    print('L^-T * b:')
    print(np.linalg.inv(L.transpose()) * b)

    D = np.matrix([[6.0, 0.0, 0.0, 0.0],
                   [0.0, 5.0, 0.0, 0.0],
                   [0.0, 0.0, 4.0, 0.0],
                   [0.0, 0.0, 0.0, 3.0]])

    A = L * D * L.transpose()
    print('A=', A)

    x = np.matrix([[1.0], [2.0], [3.0], [4.0]])
    print('A^-1 x:')
    print(np.linalg.inv(A) * x)

    print('A^-1:')
    print(np.linalg.inv(A))

    # y = np.matrix([[1.], [2.], [3.], [4.]])
    #
    # print('y=')
    # print(y)
    #
    # print('L=')
    # print(L)
    #
    # print('y.transpose() * np.linalg.inv(L) = ')
    # print(y.transpose() * np.linalg.inv(L))
    #
    # print('np.linalg.inv(L.transpose()) * y')
    # print(np.linalg.inv(L.transpose()) * y)
