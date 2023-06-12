# Libraries
import numpy  as np 
import sympy  as sym
import pandas as pd

# Function to inverse kinematics
from robo.calc import get_q

class Robo:


    def __init__(self):

        # Denavit Hartenberg Table
        t1, t2, t3 = sym.symbols("theta_1, theta_2, theta_3")
        self.denavit = pd.DataFrame(data = {
            "a":     [0, np.sqrt(148**2 + 28**2), np.sqrt(48**2 + 152**2)],
            "alpha": [np.pi/2, 0, 0],
            "d":     [165, 0, 0],
            "theta": [t1, t2 + np.arctan(148/28), t3 + np.arctan(48/152) - np.arctan(148/28)]
        })

        # Compute the homogeneous and transformation matrices
        A1, A2, A3 = self.get_homogeneous_matrices(self.denavit)
        self.transformation_j1 = A1
        self.transformation_j2 = A1*A2
        self.transformation_j3 = A1*A2*A3

        # Store a trajectory
        self.trajectory = {
            "j1": [], "j2": [], "j3": [],
            "si": [], "sf": [], "claw": [], "time": []
        }

        self.all_joints_position = []      # Current position of all joints
        self.xyz_position        = []      # Current (x,y,z) position of the manipulator
        self.joint_angles        = []      # Current joint angles
        self.claw_opening        = 0       # Current claw position

        # Set an initial position
        self.set_joint_angles(0, 0, 0)


    def forward_kinematics(self, j1: float, j2: float, j3: float):

        """
            Compute the forward kinematics of a 3-joint robot arm
            using the Denavit-Hartenberg table          
        """

        # Define symbolic variables for the joint angles
        j1_s, j2_s, j3_s = sym.symbols("theta_1, theta_2, theta_3")

        # Get transformation matrices from Denavit-Hartenberg
        T1 = self.transformation_j1.subs({j1_s: j1})
        T2 = self.transformation_j2.subs({j1_s: j1, j2_s: j2})
        T3 = self.transformation_j3.subs({j1_s: j1, j2_s: j2, j3_s: j3})

        # Get (x,y,z) joints position
        x = [0, T1[0, 3], T2[0, 3], T3[0, 3]]
        y = [0, T1[1, 3], T2[1, 3], T3[1, 3]]
        z = [0, T1[2, 3], T2[2, 3], T3[2, 3]]

        return x, y, z


    def inverse_kinematics(self, x: float, y: float, z: float) -> np.array:

        """
            Calculates the joint angles required for a robotic arm 
            to reach a specific point in 3D space.
        """

        try:
            joint_angles = get_q(np.array([x, y, z]), self.joint_angles)
        except:
            print("Error in calculating the inverse kinematics...")
            joint_angles = self.get_joint_angles()
            x, y, z = self.get_xyz_position()

        self.xyz_position = [x, y, z]
        self.joint_angles = joint_angles

        return joint_angles
    

    def get_homogeneous_matrices(self, denavit: pd.DataFrame) -> list:

        """
            Computes a list of transformation matrices based on the 
            Denavit-Hartenberg convention.
        """

        Hs = []
        for _, value in denavit.iterrows():
            a, alpha, d, theta = value.values
            Hs.append(sym.Matrix([
                [sym.cos(theta), -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha), a*sym.cos(theta)],
                [sym.sin(theta), sym.cos(theta)*sym.cos(alpha), -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)],
                [0, sym.sin(alpha), sym.cos(alpha), d],
                [0, 0, 0, 1]
            ]))
        return Hs
    
    
    def add_to_trajectory(self, speed_i: float, speed_f: float, time: float):

        """
            Add the current point to a trajectory 
        """

        j1, j2, j3 = self.get_joint_angles()
        self.trajectory["j1"].append(j1)
        self.trajectory["j2"].append(j2)
        self.trajectory["j3"].append(j3)
        self.trajectory["claw"].append(self.get_claw_opening())
        self.trajectory["si"].append(speed_i)
        self.trajectory["sf"].append(speed_f)
        self.trajectory["time"].append(time)


    def get_cubic_trajectory(self, trajectory: dict, rate: float):

        time       = []
        j1, j2, j3 = [], [], []

        # Iterates over all points
        for i in range(1, len(trajectory["j1"])):

            # Time to execute the trajectory
            ti, tf = trajectory["t"][i-1], trajectory["t"][i]

            # Speed in target points
            si, sf =  trajectory["si"][i], trajectory["sf"][i]

            # Compute the time list
            t = np.linspace(ti, tf, int((tf - ti)*rate))
            time += list(t)

            # Compute the inverse of time matrix
            timeMatrix = sym.Matrix([[1, ti, ti**2,     ti**3],
                                     [0,  1,  2*ti, 3*(ti**2)],
                                     [1, tf, tf**2,     tf**3],
                                     [0,  1,  2*tf, 3*(tf**2)]]).inv()
            
            for key in ["j1", "j2", "j3"]:

                # Move points
                qi = trajectory[key][i-1]
                qf = trajectory[key][i]

                # Create the cubic trajectory
                trajectory_matrix = sym.Matrix([qi, si, qf, sf])
                a0, a1, a2, a3 = list(timeMatrix * trajectory_matrix)

                if key == "j1":
                    j1 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                elif key == "j2":
                    j2 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                else:
                    j3 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))

        return time, j1, j2, j3
    

    def get_xyz_position(self):
        return self.xyz_position
    
    
    def set_xyz_position(self, x: float, y: float, z: float):
        j1, j2, j3 = self.inverse_kinematics(x, y, z)
        if not (np.isnan(j1) or np.isnan(j2) or np.isnan(j3)):
            self.xyz_position = [x, y, z]
            self.joint_angles = [j1, j2, j3]
            self.all_joints_position = self.forward_kinematics(j1, j2, j3)
            return True
        return False
    
    
    def get_joint_angles(self):
        return self.joint_angles
    

    def set_joint_angles(self, j1: float, j2: float, j3: float):
        x, y, z = self.forward_kinematics(j1, j2, j3)
        self.all_joints_position = [x, y, z]
        self.xyz_position = [x[-1], y[-1], z[-1]]
        self.joint_angles = [j1, j2, j3]
        return True

    
    def get_claw_opening(self):
        return self.claw_opening
    
    def get_all_joints_position(self):
        return self.all_joints_position