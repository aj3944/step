import pinocchio as pin
import os
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
from numpy.linalg import solve, norm
import webbrowser


class Inverse_kinematics_Solver:

    # constructor
    def __init__(self, urdf_file, mesh_dir):
        self.urdf_file = urdf_file
        self.mesh_dir = mesh_dir
        self.joint_configs = []
        self.actual_iterations = 0

        # solver params
        self.eps = 1e-3
        self.DT = 1e-1
        self.damp = 1e-1
        self.IT_MAX = 20000
        self.JOINT_ID = None

    # creating the key math model and associated data
    def create_model_and_data(self):
        self.model = pin.buildModelfromURDF(self.urdf_file)
        self.data = self.model.createData()

    # create visual and/or collision model
    def create_geometric_model(self, geometry_type):
        if self.model != None:
            if geometry_type == "visual":
                self.visual_model = pin.buildGeomFromUrdf(
                    self.model, self.data, pin.VISUAL, package_dirs=self.mesh_dir
                )
            elif geometry_type == "collision":
                self.collision_model = pin.buildGeomFromUrdf(
                    self.model, self.data, pin.COLLISION, package_dirs=self.mesh_dir
                )
                self.collision_data = pin.GeometryData(self.collision_model)

    # create meshcat visualizer and open the link
    def create_visualizer(self):
        self.visualizer = MeshcatVisualizer(
            self.model, self.collision_model, self.visual_model
        )
        self.visualizer.initViewer()
        self.visualizer.loadViewerModel()
        webbrowser.open("http://127.0.0.1:7014/static/")

    def solve(self, R_des, T_des, foot):
        if foot == "right":
            self.JOINT_ID = 8
        elif foot == "left":
            self.JOINT_ID = 4
        i = 0
        T_des = np.array([0.0235, -0.030, -0.265])
        oMdes = pin.SE3(
            R_des, T_des
        )  # think of oMdes as oTdes, the transfromation from desired to origin expressed in origin

        q = pin.neutral(self.model)

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
                self.model, self.data, self.collision_model, self.geom_data, q
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
            print("Convergence achieved!")
            trajectory = trajectory[:actual_iterations]
            self.joint_configs.append(trajectory)
            return True
        else:
            print(
                "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
            )
            return False

    def visualize(self):
        for i in range(self.joint_configs.shape[0]):
            self.visualizer.display(self.joint_configs[i])

    def run(self, foot_y_movement=0, foot_z_movement=0):
        R_des_right_foot_neutral = np.array(
            [[1, 0, 0], [0, 0.000796327, -1], [0, 1, 0.000796327]]
        )
        R_des_left_neutral = np.array(
            [
                [0.999995, 0, -0.0031853],
                [-0.0031853, 0.000796327, -0.999995],
                [2.53654e-06, 1, 0.000796323],
            ]
        )

        T_des_right_foot_neutral = np.array(
            [0.0235, -0.000230935 + foot_y_movement, -0.29 + foot_z_movement]
        )

        T_des_left_foot_neutral = np.array([0.1225, -0.000234916, -0.29])

        self.solve(R_des_right_foot_neutral, T_des_right_foot_neutral, "right")

        self.create_visualizer()

        self.visualize()
