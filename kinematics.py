import pinocchio as pin
import os
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
from pinocchio.visualize import GepettoVisualizer
# from robot_descriptions.loaders.pinocchio import load_robot_description
import gepetto.corbaserver
from numpy.linalg import norm, solve
import sys

urdf_filename = "/home/adi/hum_rob_ws/src/six_dof/urdf/6dof_fixed.urdf"
mesh_dir = "/home/adi/hum_rob_ws/src/six_dof/meshes"

ros_package_path = "/home/adi/hum_rob_ws/src"  # Replace with your path
os.environ["ROS_PACKAGE_PATH"] = ros_package_path


# building model from URDF, model is an object that includes kinematic and ineratia params
model = pin.buildModelFromUrdf(urdf_filename)

visual_model = pin.buildGeomFromUrdf(
    model, urdf_filename, pin.GeometryType.VISUAL, package_dirs=mesh_dir
)  # very important

collision_model = pin.buildGeomFromUrdf(
    model, urdf_filename, pin.GeometryType.COLLISION, package_dirs=mesh_dir
)

geom_data = pin.GeometryData(collision_model)


# data is an object that holds values that are a result of computation
data = model.createData()


visualizer = MeshcatVisualizer(model, collision_model, visual_model)

visualizer.initViewer()
visualizer.loadViewerModel()

q = pin.neutral(model)
print(q)
eps = 1e-4
DT = 1e-1
damp = 1e-2
IT_MAX=2000
JOINT_ID = 6

oMdes = pin.SE3(np.eye(3), np.array([0.092, 0.030, 0.050])) #think of oMdes as oTdes, the transfromation from desired to origin expressed in origin
i = 0

while True:
    pin.forwardKinematics(model, data, q)
    iMd = data.oMi[JOINT_ID].actInv(oMdes) #active inverse, chnages basis and moves the vector/point to the appropriate position in the new basis
    #print(data.oMi[6])
    err = pin.log(iMd).vector  # in joint frame  The logarithm map (log()) is a mathematical operation that maps elements from the group SE(3) to its associated Lie algebra, se(3). This operation effectively converts the complex rotational and translational transformation into a simpler vector representation that captures the "difference" or "error" between two poses in SE(3).
    norm_err=norm(err)
    if norm_err < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pin.computeJointJacobian(
        model, data, q, JOINT_ID
    )  # jacobian computed in e-e frame
    J = -np.dot(pin.Jlog6(iMd.inverse()), J)
    q_dot = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pin.integrate(model, q, q_dot * DT)
   # Collision check
    pin.updateGeometryPlacements(
        model, data, collision_model, geom_data, q
    )  # updates geom_data by reference

    # an active pair is a pair of bodies in a joint considered for collision
    collision_detected = pin.computeCollisions(
        model, data, collision_model, geom_data, q, True
    )  # this returns data,geom_data. polymorphic function

    if not collision_detected:
        for idx in range(model.nq):
            # Assuming model.lowerPositionLimit and model.upperPositionLimit store the limits
            q[idx] = max(
                model.lowerPositionLimit[idx],
                min(q[idx], model.upperPositionLimit[idx]),
            )

    visualizer.display(q)
    #print(norm_err)
    i+=1
    #time.sleep(0.05)

# if success:
#     print("Convergence achieved!")
# else:
#     print(
#         "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
#     )

# print("\nresult: %s" % q.flatten().tolist())
# print("\nfinal error: %s" % err.T)
