#************************************************#
# File:        robot.py                          #
# Description: library with functions to control #
#              a RRR manipulator robot           #
# Created:     06/05/2023                        #
# Reviewed:    19/05/2023                        #
#************************************************#

# Import libraries
import math
import numpy as np
import sympy as sym
import pandas as pd

# Import functions
from numpy import sin, cos, sqrt

class Robot:

    def __init__(self, d1, a2, a3):
        # Robot dimensions
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3

        # Denavit-Hartenberg Table
        self.denavit_hartenberg = pd.DataFrame(data={
            "a":     [0, a2, a3],
            "alpha": [np.pi/2, 0, 0],
            "d":     [d1, 0, 0],
            "theta": sym.symbols("theta_1, theta_2, theta_3")
        })

    def inverse_kinematics(self, x: float, y: float, z: float) -> np.array:

        """
            Calculates the joint angles required for a robotic arm 
            to reach a specific point in 3D space.

            Args:
                - x: x-coordinate of the target point.
                - y: y-coordinate of the target point.
                - z: z-coordinate of the target point.

            Returns:
                A tuple of three float values representing the joint
                angles (in radians) required to reach the target point.
        """

        # Calculate the joint angles
        theta_1 = math.atan2(y, x)
        r2 = x**2 + y**2 + (z-self.d1)**2
        cos_theta_3 = (r2 - self.a2**2 - self.a3**2)/(2 * self.a2 * self.a3)
        theta_3 = (-1)*math.atan2(sqrt(1-cos_theta_3**2), cos_theta_3)
        temp1 = math.atan2((z - self.d1), sqrt(x**2 + y**2))
        temp2 = math.atan2((self.a3 * sin(theta_3)), (self.a2 + self.a3*cos(theta_3)))
        theta_2 = temp1 - temp2

        # Return the joint angles as a tuple
        return np.array((theta_1, theta_2, theta_3))

    def get_homogeneous_matrices(self, denavit_hartenberg: pd.DataFrame) -> list:

        """
            Computes a list of transformation matrices based on the 
            Denavit-Hartenberg convention.

            Args:
                - denavit_hartenberg: A pandas DataFrame containing the Denavit-Hartenberg parameters 
                                    for each joint in the robot arm. The DataFrame should have four 
                                    columns: 'a', 'alpha','d', and 'theta'. Each row represents one joint.

            Returns:
                A list of sympy Matrix, where each matrix represents the 
                transformation matrix for one joint. 
        """

        Hs = []
        for _, value in denavit_hartenberg.iterrows():
            a, alpha, d, theta = value.values
            Hs.append(sym.Matrix([
                [sym.cos(theta), -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha), a*sym.cos(theta)],
                [sym.sin(theta), sym.cos(theta)*sym.cos(alpha), -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)],
                [0, sym.sin(alpha), sym.cos(alpha), d],
                [0, 0, 0, 1]
            ]))
        return Hs

    def forward_kinematics(self, theta_1: float, theta_2: float, theta_3: float):

        """
            Compute the forward kinematics of a 3-joint robot arm.

            Args:
                - theta_1 (float): The angle (in radians) of joint 1.
                - theta_2 (float): The angle (in radians) of joint 2.
                - theta_3 (float): The angle (in radians) of joint 3.

            Returns:
                
        """
        
        A1, A2, A3 = self.get_homogeneous_matrices(self.denavit_hartenberg)

        # Define symbolic variables for the joint angles
        t1, t2, t3 = sym.symbols("theta_1, theta_2, theta_3")

        # Get transformation matrices from Denavit-Hartenberg
        T1 = A1.subs({t1: theta_1})
        T2 = (A1*A2).subs({t1: theta_1, t2: theta_2})
        T3 = (A1*A2*A3).subs({t1: theta_1, t2: theta_2, t3: theta_3})

        return self.homogenous_to_xyz(T1, T2, T3) 
    
    def homogenous_to_xyz(self, T_01, T_02, T_03):

        x = [0, T_01[0, 3], T_02[0, 3], T_03[0, 3]]
        y = [0, T_01[1, 3], T_02[1, 3], T_03[1, 3]]
        z = [0, T_01[2, 3], T_02[2, 3], T_03[2, 3]]

        return x, y, z

    def inv_time_cubic_trajectory(self, ti: float, tf: float) -> None:
        timeMatrix = sym.Matrix([[1, ti, ti**2,     ti**3],
                                 [0,  1,  2*ti, 3*(ti**2)],
                                 [1, tf, tf**2,     tf**3],
                                 [0,  1,  2*tf, 3*(tf**2)]])
        return timeMatrix.inv()
    
    def create_trajectory(self, trajectory: dict, rate: float):

        time    = []
        joint_1 = []
        joint_2 = []
        joint_3 = []

        for i in range(1, len(trajectory["j1"])):

            # Time to execute trajectory
            ti = trajectory["t"][i-1]
            tf = trajectory["t"][i]

            # Speed in target points
            vi = trajectory["vi"][i-1]
            vf = trajectory["vf"][i-1]

            # Set time vector
            delta_time = tf - ti
            number_of_points = int(delta_time*rate)
            t = np.linspace(ti, tf, number_of_points)
            time += list(t)
            
            # Get the inverse of trajectory time matrix
            time_inv = self.inv_time_cubic_trajectory(ti, tf)

            for key in ["j1", "j2", "j3"]:

                # Move points
                qi = trajectory[key][i-1]
                qf = trajectory[key][i]

                # Create the cubic trajectory
                trajectory_matrix = sym.Matrix([qi, vi, qf, vf])
                a0, a1, a2, a3 = list(time_inv*trajectory_matrix)

                if key == "j1":
                    joint_1 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                elif key == "j2":
                    joint_2 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                else:
                    joint_3 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))

        return time, joint_1, joint_2, joint_3
                    
if __name__ == "__main__":
    r = Robot(2,2,2)

    trajectory = {
            "j1": [0, 0], "j2": [0, 1.441244159646074], "j3": [0, -1.955193101290536], 
            "vi": [0, 0], "vf": [0, 0, 0], 
            "t": [0, 2]
        }
    
    t, j1, j2, j3 = r.create_trajectory(trajectory, 100)

    import matplotlib.pyplot as plt
    plt.plot(t, j2)
    plt.show()