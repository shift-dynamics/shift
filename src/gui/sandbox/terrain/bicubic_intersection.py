#!/usr/bin/python

import sympy

B00, B01, B02, B03 = sympy.symbols('B00 B01 B02 B03')
B10, B11, B12, B13 = sympy.symbols('B10 B11 B12 B13')
B20, B21, B22, B23 = sympy.symbols('B20 B21 B22 B23')
B30, B31, B32, B33 = sympy.symbols('B30 B31 B32 B33')
a0, a1, a2, a3, a4, a5, t = sympy.symbols('a0 a1 a2 a3 a4 a5 t')
u = sympy.Matrix([[(a0 * t + a1)**3],
                  [(a0 * t + a1)**2],
                  [(a0 * t + a1)],
                  [1]])
v = sympy.Matrix([[(a2 * t + a3)**3],
                  [(a2 * t + a3)**2],
                  [(a2 * t + a3)],
                  [1]])
B = sympy.Matrix([[B00, B01, B02, B03],
                  [B10, B11, B12, B13],
                  [B20, B21, B22, B23],
                  [B30, B31, B32, B33]])

for i in range(6, -1, -1):
    print('[', i, ']', sympy.latex((u.transpose() * B * v).expand()[0, 0].coeff(t, i)))
