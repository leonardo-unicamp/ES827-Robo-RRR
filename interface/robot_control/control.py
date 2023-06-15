# Libraries
import numpy as np
import threading as th

# Robot communication
from robot_math.DH import DH
from Ev3.client import Ev3Client

# Functions 
from time import sleep
from math import degrees, radians
from pandas import isnull

class RobotControl:


    def __init__(self):

        # Robot Mathematics
        self.calculate = DH()

        # Connection with Ev3
        # self.ev3 = Ev3Client()

        # Robot Trajectory
        self.__trajectory = RobotTrajectory()

        # Robot parameters
        self.__all_joints_position  = []      # Current position of all joints
        self.__manipulator_position = []      # Current (x,y,z) position of the manipulator
        self.__joint_angles         = []      # Current joint angles
        self.__is_moving            = False   # If the robot is moving

        # Set an initial position
        self.set_joint_angles(0, 0, 0, 0)

    
    # Getters
    def get_is_moving(self):            return self.__is_moving
    def get_joint_angles(self):         return self.__joint_angles
    def get_manipulator_position(self): return self.__manipulator_position
    def get_all_joints_position(self):  return self.__all_joints_position

    # Setters
    def set_is_moving(self, value: bool):
        self.__is_moving = value

    def set_joint_angles(self, j1: float, j2: float, j3: float, j4: float) -> None:
        x, y, z = self.calculate.fw_kinematics((j1, j2, j3))
        self.__joint_angles         = (j1, j2, j3, j4)
        self.__all_joints_position  = (x, y, z)
        self.__manipulator_position = (x[-1], y[-1], z[-1])
        self.ev3_set_position(*self.__joint_angles)

    def set_manipulator_position(self, x: float, y: float, z: float) -> None:
        _, _, _, j4 = self.get_joint_angles()
        j1, j2, j3 = self.calculate.bw_kinematics(
                        target=(x, y, z),
                        last_pos=self.get_manipulator_position())
        if not (isnull(j1) or isnull(j2) or isnull(j3)):
            self.__joint_angles         = (j1, j2, j3, j4)
            self.__all_joints_position  = self.calculate.fw_kinematics((j1, j2, j3))
            self.__manipulator_position = (x, y, z)
            self.ev3_set_position(*self.__joint_angles)


    def ev3_set_position(self, j1: float, j2: float, j3: float, j4: float) -> None:

        """ Set a value in degrees to Ev3 motors """

        # Convert all joints to degrees
        joints_in_degrees = [degrees(j1), degrees(j2), degrees(j3), degrees(j4)]

        # Set position to Ev3 motors
        # self.ev3.set_position(*joints_in_degrees)


    def __thread_move_robot(self, j1: tuple, j2: tuple, j3: tuple, j4: tuple) -> None:

        """ Send commands to Ev3 to execute an trajectory """

        # Keep waiting while the robot is executing another movement
        while self.get_is_moving():
            pass

        # It's available execute the movement
        self.set_is_moving(True)

        # Iterates over all points, set and update
        for i in range(len(j1)):
            joints = (j1[i], j2[i], j3[i], j4[i])
            self.ev3_set_position(*joints)
            updater = th.Thread(target=lambda: self.set_joint_angles(*joints))
            updater.start()
            sleep()

        # Movement finished
        self.set_is_moving(False)
        print("Movement finished.")

    
    def move_robot(self, j1: tuple, j2: tuple, j3: tuple, j4: tuple) -> None:

        """ Create a thread that send commands to execute the trajectory """

        sender = th.Thread(target=lambda: self.__thread_move_robot(j1, j2, j3, j4))
        sender.start()


    def go_to(self, target_j1: float, target_j2: float, target_j3: float, target_j4: float, time: float):

        """
            Calculate the trajectory from the current point to another
            knowing the joints positions
        """
        
        # Get current joint angles
        j1, j2, j3, j4 = self.get_joint_angles()

        # Get the trajectory
        _, j1, j2, j3, j4 = self.__trajectory.go_to((j1, target_j1), (j2, target_j2),
                                                    (j3, target_j3), (j4, target_j4), time)

        self.move_robot(j1, j2, j3, j4)

    
    # Trajectory function
    def remove_trajectory(self): self.__trajectory.remove_all()
    def add_trajectory(self):    self.__trajectory.add_point() #TO DO


    def run_trajectory(self):

        """ Execute the created trajectory """

        if not self.__trajectory.is_empty():

            # Get the initial angles of trajectory
            j1, j2, j3, j4 = self.__trajectory.initial_point()

            # Move the robot to the initial point of trajectory
            self.go_to(j1, j2, j3, j4, 5)

            # Execute the trajectory (10 points per second)
            _, j1, j2, j3, j4 = self.__trajectory.cubic(10)
            self.move_robot(j1, j2, j3, j4)
        else:
            print("None trajectory created!")

    
class RobotTrajectory:

    def __init__(self)  -> None:

        # Store a trajectory
        self.trajectory = {
            "j1": [], "j2": [], "j3": [], "j4": [], 
            "si": [], "sf": [], "time": []
        }

    
    def is_empty(self):

        """ Check if a trajectory is empty """

        return self.trajectory["j1"] == []


    def add_point(self, joints: tuple, si: float, sf: float, time: float) -> None:

        """ Add a point to the trajectory """

        self.trajectory["j1"].append(joints[0])
        self.trajectory["j2"].append(joints[1])
        self.trajectory["j3"].append(joints[2])
        self.trajectory["j4"].append(joints[3])
        self.trajectory["si"].append(si)
        self.trajectory["sf"].append(sf)
        self.trajectory["time"].append(time)


    def remove_all(self) -> None:

        """ Reset the trajectory """

        self.trajectory = {
            "j1": [], "j2": [], "j3": [], "j4": [], 
            "si": [], "sf": [], "time": []
        }

    
    def initial_point(self) -> tuple:

        """ Return the initial point of the trajectory """

        j1 = self.trajectory["j1"][0]
        j2 = self.trajectory["j2"][0]
        j3 = self.trajectory["j3"][0]
        j4 = self.trajectory["j4"][0]

        return (j1, j2, j3, j4)


    def cubic(self, rate: float) -> tuple:
        
        """ Computes a cubic trajectory of all points """

        time = []
        j1, j2, j3, j4 = [], [], [], []

        # Iterates over all points
        for i in range(1, len(self.trajectory["j1"])):

            # Time to execute the self.trajectory
            ti, tf = self.trajectory["time"][i-1], self.trajectory["time"][i]

            # Speed in target points
            si, sf =  self.trajectory["si"][i], self.trajectory["sf"][i]

            # Compute the time list
            t = np.linspace(ti, tf, int((tf - ti)*rate))
            time += list(t)

            # Compute the inverse of time matrix
            timeMatrix = np.array([[1, ti, ti**2,     ti**3],
                                   [0,  1,  2*ti, 3*(ti**2)],
                                   [1, tf, tf**2,     tf**3],
                                   [0,  1,  2*tf, 3*(tf**2)]])
            timeMatrix = np.linalg.inv(timeMatrix)
            
            for key in ["j1", "j2", "j3", "j4"]:

                # Move points
                qi = self.trajectory[key][i-1]
                qf = self.trajectory[key][i]

                # Create the cubic self.trajectory
                self.trajectory_matrix = np.array([qi, si, qf, sf])
                a0, a1, a2, a3 = list(timeMatrix @ self.trajectory_matrix)

                if key == "j1":
                    j1 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                elif key == "j2":
                    j2 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                elif key == "j3":
                    j3 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))
                elif key == "claw":
                    j4 += list(a0 + a1*t + a2*(t**2) + a3*(t**3))

        return time, j1, j2, j3, j4


    def go_to(self, j1: tuple, j2: tuple, j3: tuple, j4: tuple, time: float):

        """
            Calculate the trajectory from a point to another (just 2 points)
        """

        trajectory = {
            "j1": j1, "j2": j2, "j3": j3, "j4": j4, 
            "si": [0, 0], "sf": [0, 0], "time": [0, time]
        }

        return self.cubic(trajectory, 10)
    