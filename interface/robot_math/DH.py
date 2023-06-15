# %%

import numpy as np
import pandas as pd
import scipy as sp
import sympy as sym

b1 = np.arctan(148 / 28)
b2 = np.sqrt(148**2 + 28**2)
b3 = np.arctan(48 / 152)
b4 = np.sqrt(48**2 + 152**2)

joints = sym.symbols("theta_1:4")

# Denavit-Hartenberg Table
DH_table = pd.DataFrame(
    data={
        "a": [0, b2, b4],
        "alpha": [sym.pi / 2, 0, 0],
        "d": [165, 0, 0],
        "theta": [joints[0], joints[1] + b1, joints[2] + b3 - b1],
    }
)


def cum_prod(l):
    result = [l[0]]
    for expr in l[1:]:
        result.append(result[-1] * expr)
    return result


def d(A):
    """Returns the position vector of a homogenous vector"""
    return A[:3, 3]


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


class DH:
    def __init__(self):
        self.table = DH_table
        self.joints = joints

        self.homogeneus_matrices = [
            calculate_homogeneous_matrix(**dict(self.table.iloc[i]))
            for i in range(len(self.table))
        ]
        temp = cum_prod(self.homogeneus_matrices)
        self.A_0_i = [Expr(A, joints) for A in temp]

        # TODO change name to last joint pos
        self.__last_pos = Expr(d(self.A_0_i[-1].symbolic), joints)
        # self.last_pos = Expr(d(self.A_0_i[-1].symbolic), joints)

    def fw_kinematics(self, joints):
        return np.array([[0,0,0]]+[d(Ai(joints)) for Ai in self.A_0_i])

    def bw_kinematics(self, target, last_pos):
        a = sp.optimize.least_squares(lambda x: target - self.last_pos(x), last_pos)
        return a.x

    def last_pos(self, joints):
        return self.__last_pos.numeric(joints)


class Expr:
    def __init__(self, symbolic, vars_):
        self.symbolic = symbolic
        self.vars_ = vars_
        self._num_lambdify = sym.lambdify(vars_, symbolic)
        self.numeric = lambda x: self._num_lambdify(*x).squeeze()

    def __call__(self, x):
        return self.numeric(x)


# %%


def main() -> None:
    def line_points(p0, p1, num_points):
        (x0, y0, z0) = p0
        (x1, y1, z1) = p1
        x_vals = np.linspace(x0, x1, num_points)
        y_vals = np.linspace(y0, y1, num_points)
        z_vals = np.linspace(z0, z1, num_points)
        points = np.column_stack((x_vals, y_vals, z_vals))
        return points

    points = line_points([180, 0, 360], [181, 0, 360 / 2], 2000)

    q_old = [0, 0, 0]

    rb = DH()

    def ssd(a, b):
        diff = a - b
        return np.dot(diff, diff)

    for p in points:
        q = rb.bw_kinematics(p, q_old)
        q_old = q
        # print(ssd(p, rb.fw_kinematics(q)[-1]))


if __name__ == "__main__":
    main()
    pass

# %%
dh = DH()
dh.fw_kinematics((0,0,0))