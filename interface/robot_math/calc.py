from functools import reduce

import numpy as np
import pandas as pd
import scipy.optimize
import sympy as sym

b1 = np.arctan(148 / 28)
b2 = np.sqrt(148**2 + 28**2)
b3 = np.arctan(48 / 152)
b4 = np.sqrt(48**2 + 152**2)


def calculate_homogeneous_matrix(theta, alpha, a, d):
    ct = sym.cos(theta)
    st = sym.sin(theta)
    ca = sym.cos(alpha)
    sa = sym.sin(alpha)

    # Construct the homogeneous transformation matrix
    Ai = sym.Matrix(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ]
    )

    return Ai


# Denavit-Hartenberg Table
t1, t2, t3 = sym.symbols("theta_1, theta_2, theta_3")


DH = pd.DataFrame(
    data={
        "a": [0, b2, b4],
        "alpha": [sym.pi / 2, 0, 0],
        "d": [165, 0, 0],
        "theta": [t1, t2 + b1, t3 + b3 - b1],
    }
)


H_sym = reduce(
    lambda x, y: x * y,
    [calculate_homogeneous_matrix(**dict(DH.iloc[i])) for i in range(3)],
)


def R(A):
    return A[:3, :3]


def T(A):
    return A[:3, 3]


H = sym.lambdify([t1, t2, t3], H_sym)
pos = sym.lambdify([t1, t2, t3], T(H_sym))


def get_q(target, last_pos):
    func = lambda x: ((pos(*x).squeeze() - target) ** 2).sum()
    ans = scipy.optimize.minimize(func, last_pos, tol=1e-2)
    q = ans.x
    return q


get_q(np.array([180, 0, 361]), np.array([0, 0, 0]))
