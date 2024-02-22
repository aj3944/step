import pinocchio as pin
import os
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
from numpy.linalg import solve, norm
import webbrowser
from pylx16a.lx16a import *

# REMOVE BEFORE PUSHING
import sys


class Motor:
    center = 0

    def __init__(self, ID, cent, min_ang=0, max_ang=240):
        # self.I
        self.center = cent
        self.servo = LX16A(ID)
        try:
            self.servo.set_id(ID)
            self.servo.set_angle_limits(min_ang, max_ang)
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")

    def move(self, offset=0):
        self.servo.move(self.center + offset)


class Inverse_kinematics_Solver:

    # constructor
    def __init__(self, urdf_file, mesh_dir):
        self.urdf_file = urdf_file
        self.mesh_dir = mesh_dir
        self.joint_trajectories = []
        self.actual_iterations = 0

        # solver params
        self.eps = 1e-3
        self.DT = 1e-1
        self.damp = 1e-1
        self.IT_MAX = 20000
        self.JOINT_ID = None

    # creating the key math model and associated data
    def create_model_and_data(self):
        self.model = pin.buildModelFromUrdf(self.urdf_file)
        self.data = self.model.createData()

    # create visual and/or collision model
    def create_geometric_model(self, geometry_type):
        if self.model != None:
            if geometry_type == "visual":
                self.visual_model = pin.buildGeomFromUrdf(
                    self.model,
                    self.urdf_file,
                    pin.GeometryType.VISUAL,
                    package_dirs=self.mesh_dir,
                )
            elif geometry_type == "collision":
                self.collision_model = pin.buildGeomFromUrdf(
                    self.model,
                    self.urdf_file,
                    pin.GeometryType.COLLISION,
                    package_dirs=self.mesh_dir,
                )
                self.collision_data = pin.GeometryData(self.collision_model)

    # create meshcat visualizer and open the link
    def create_visualizer(self):
        self.visualizer = MeshcatVisualizer(
            self.model, self.collision_model, self.visual_model
        )
        self.visualizer.initViewer()
        self.visualizer.loadViewerModel()
        webbrowser.open("http://127.0.0.1:7001/static/")

    def solve(self, R_des, T_des, foot):
        if foot == "right":
            self.JOINT_ID = 8
        elif foot == "left":
            self.JOINT_ID = 4
        i = 0
        oMdes = pin.SE3(
            R_des, T_des
        )  # think of oMdes as oTdes, the transfromation from desired to origin expressed in origin

        # CHECK THIS LOGIC
        if len(self.joint_trajectories) == 0:
            q = pin.neutral(self.model)

        else:
            q = self.joint_trajectories[-1][-1]
            print(q)

        trajectory = np.zeros((self.IT_MAX, 8), float)
        actual_iterations = 0

        while True:
            pin.forwardKinematics(self.model, self.data, q)

            iMd = self.data.oMi[self.JOINT_ID].actInv(
                oMdes
            )  # active inverse, chnages basis and moves the vector/point to the appropriate position in the new basis
            err = pin.log(
                iMd
            ).vector  # in joint frame  The logarithm map (log()) is a mathematical operation that maps elements from the group SE(3) to its associated Lie algebra, se(3). This operation effectively converts the complex rotational and translational transformation into a simpler vector representation that captures the "difference" or "error" between two poses in SE(3).
            norm_err = norm(err)
            if norm_err < self.eps:
                success = True
                break
            if i >= self.IT_MAX:
                success = False
                break
            J = pin.computeJointJacobian(
                self.model, self.data, q, self.JOINT_ID
            )  # jacobian computed in e-e frame
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            q_dot = -J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pin.integrate(self.model, q, q_dot * self.DT)

            # Collision check
            pin.updateGeometryPlacements(
                self.model, self.data, self.collision_model, self.collision_data, q
            )  # updates geom_data by reference

            # an active pair is a pair of bodies in a joint considered for collision
            collision_detected = pin.computeCollisions(
                self.model,
                self.data,
                self.collision_model,
                self.collision_data,
                q,
                True,
            )  # this returns data,geom_data. polymorphic function

            if not collision_detected:
                for idx in range(self.model.nq):
                    # Assuming model.lowerPositionLimit and model.upperPositionLimit store the limits
                    q[idx] = max(
                        self.model.lowerPositionLimit[idx],
                        min(q[idx], self.model.upperPositionLimit[idx]),
                    )
                trajectory[actual_iterations] = q
                actual_iterations += 1
            i += 1

        if success:
            print(i)
            print("Convergence achieved!")
            trajectory = trajectory[:actual_iterations]

            self.joint_trajectories.append(trajectory)
            return True
        else:
            print(
                "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
            )
            return False

    def visualize(self):
        try:
            while True:
                for i in range(len(self.joint_trajectories)):
                    for j in range(len(self.joint_trajectories[i])):
                        self.visualizer.display(self.joint_trajectories[i][j])
                        # print(type(self.joint_trajectories[i][j]))
        except KeyboardInterrupt:
            pass

    def march(
        self,
        foot,
        foot_x_movement: float = 0,
        foot_y_movement: float = 0,
        foot_z_movement: float = 0,
    ):
        if foot == "right":
            R_des_right_foot_neutral = np.array(
                [[1, 0, 0], [0, 0.000796327, -1], [0, 1, 0.000796327]]
            )
            T_des_right_foot_neutral = np.array(
                [
                    0.0235,
                    -0.000230935,
                    -0.29,
                ]
            )
            T_des_right_foot = np.array(
                [
                    0.0235 + foot_x_movement,
                    -0.000230935 + foot_y_movement,
                    -0.29 + foot_z_movement,
                ]
            )
            self.create_model_and_data()
            self.create_geometric_model("visual")
            self.create_geometric_model("collision")
            self.solve(R_des_right_foot_neutral, T_des_right_foot, foot)

        elif foot == "left":
            R_des_left_foot_neutral = np.array(
                [
                    [0.999995, 0, -0.0031853],
                    [-0.0031853, 0.000796327, -0.999995],
                    [2.53654e-06, 1, 0.000796323],
                ]
            )

            T_des_left_foot_neutral = np.array(
                [
                    0.1225 + foot_x_movement,
                    -0.000234916 + foot_y_movement,
                    -0.29 + foot_z_movement,
                ]
            )
            self.create_model_and_data()
            self.create_geometric_model("visual")
            self.create_geometric_model("collision")
            self.solve(
                R_des_left_foot_neutral, T_des_left_foot_neutral, foot
            )  # creates trajectory as numpy array and appends to list- not optimal


if __name__ == "__main__":
    os.environ["ROS_PACKAGE_PATH"] = "/home/adi/hum_rob_ws/src"

    py310_issue = input("ARE YOU HAVING TROUBLE WITH py310 AND MESHCAT?? [y/n] \n")
    if py310_issue == "y":
        sys.path = [
            "/home/adi/anaconda3/envs/robotics_course_py310/lib/python3.10/site-packages"
        ] + sys.path
        print("adjusted sys.path\n")
    else:
        print("great! moving on!\n")

    urdf_filename = "/home/adi/hum_rob_ws/src/six_dof/urdf/6dof_from_hip.urdf"
    mesh_dir = "/home/adi/hum_rob_ws/src/six_dof/meshes"
    ik_solver = Inverse_kinematics_Solver(urdf_filename, mesh_dir)
    # in y axis minus is forward , in z minus is upwards
    ik_solver.march("left", 0.0, -0.020, 0.010)
    ik_solver.march("left", 0.0, 0.0, 0.0)

    ik_solver.march("right", 0.0, -0.020, 0.010)
    ik_solver.march("right", 0.0, 0.0, 0.0)
    # visualize
    ik_solver.create_visualizer()
    ik_solver.visualize()
    print("\n" + str(len(ik_solver.joint_trajectories)))
