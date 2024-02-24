import pinocchio as pin
import os
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
from numpy.linalg import solve, norm
import webbrowser
from test import Bot
import time 


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
        if len(self.joint_configs) == 0:
            q = pin.neutral(self.model)

        else:
            q = self.joint_configs[-1][-1]
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

            # # Collision check
            # pin.updateGeometryPlacements(
            #     self.model, self.data, self.collision_model, self.collision_data, q
            # )  # updates geom_data by reference

            # # an active pair is a pair of bodies in a joint considered for collision
            # collision_detected = pin.computeCollisions(
            #     self.model,
            #     self.data,
            #     self.collision_model,
            #     self.collision_data,
            #     q,
            #     True,
            # )  # this returns data,geom_data. polymorphic function

            if True or not collision_detected :
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
            last_value = trajectory[-1]  # Get the last value of the trajectory
            #last_value=np.array([np.rad2deg(i) for i in last_value])
            with open('trajectory_values.csv', 'a') as file:  # Open file in append mode
                # Convert the NumPy array to a string with commas separating values and append a newline
                np.savetxt(file, [last_value], delimiter=',', fmt='%s')
            self.joint_configs.append(trajectory)
            return True
        else:
            print(
                "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
            )
            return False

    def visualize(self):
        try:
            while True:
                for i in range(len(self.joint_configs)):
                    for j in range(len(self.joint_configs[i])):
                        self.visualizer.display(self.joint_configs[i][j])
                        # print(type(self.joint_configs[i][j]))
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


def motor_map(traj_8):
    return [np.rad2deg(traj_8[1])*-1,np.rad2deg(traj_8[2]),np.rad2deg(traj_8[5])*-1,np.rad2deg(traj_8[6]*-1)]


if __name__ == "__main__":
    os.environ["ROS_PACKAGE_PATH"] = "/home/va/stepws/src/six_dof"

    mark_4 = Bot()
    mark_4.home()
    urdf_filename = "/home/va/stepws/src/six_dof/urdf/6dof_from_hip.urdf"
    mesh_dir = "/home/va/stepws/src/six_dof/meshes"
    ik_solver = Inverse_kinematics_Solver(urdf_filename, mesh_dir)
    # in y axis minus is forward , in z minus is upwards
    ik_solver.march("left", 0.0, 0.030, 0.02)
    ik_solver.march("left", 0.0, 0.0, 0.0)
    

    ik_solver.march("right", 0.0, 0.030, 0.02)
    ik_solver.march("right", 0.0, 0.0, 0.0)
    # visualize
    # ik_solver.create_visualizer()
    # ik_solver.visualize()

    motor_traj_list =  [ [motor_map(y[-1])] for y in ik_solver.joint_configs]
    # motor_traj_list =  [[motor_map(x) for x in y] for y in ik_solver.joint_configs]
    # motor_traj_list =  [motor_map(x[-1]) for x  in ]
    print(motor_traj_list)

    for traj_list in motor_traj_list:
        for pose_4 in traj_list:
            mark_4.injest_ik(pose_4,0.150)
            # time.sleep(1)

    # print(motor_traj_list[0][9000])
    # print(len(ik_solver.joint_configs[1]))
    # print(len(ik_solver.joint_configs[2]))
    # print(len(ik_solver.joint_configs[3]))
