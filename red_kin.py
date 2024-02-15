import pinocchio as pin
import os
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
from numpy.linalg import solve, norm


urdf_filename = "/home/adi/hum_rob_ws/src/six_dof/urdf/6dof_from_hip.urdf"
mesh_dir = "/home/adi/hum_rob_ws/src/six_dof/meshes"

ros_package_path = "/home/adi/hum_rob_ws/src"  # Replace with your path
os.environ["ROS_PACKAGE_PATH"] = ros_package_path

model = pin.buildModelFromUrdf(urdf_filename)
visual_model = pin.buildGeomFromUrdf(
    model, urdf_filename, pin.GeometryType.VISUAL, package_dirs=mesh_dir
)
collision_model = pin.buildGeomFromUrdf(
    model, urdf_filename, pin.GeometryType.COLLISION, package_dirs=mesh_dir
)
data = model.createData()

# list of joitns to lock
joints_to_lock = [
    "right_foot_shin",
    "right_shin_thigh",
    "right_thigh_body",
    "right_thigh_body_lateral",
]

IDs = []

for joint_name in joints_to_lock:
    if model.existJointName(joint_name):
        IDs.append(model.getJointId(joint_name))
print(IDs)

initial_joint_config = np.array([0, 0, 0, 0, 0, 0, 0, 0])

model_reduced, geom_models_reduced = pin.buildReducedModel(
    model,
    list_of_geom_models=[visual_model, collision_model],
    list_of_joints_to_lock=IDs,
    reference_configuration=initial_joint_config,
)
visual_model_reduced, collision_model_reduced = (
    geom_models_reduced[0],
    geom_models_reduced[1],
)


data_reduced = model_reduced.createData()
geom_data = pin.GeometryData(collision_model_reduced)

print("joints to lock (only ids):", IDs)
print("reduced model: dim=" + str(len(model_reduced.joints)))
print("-" * 30)

visualizer = MeshcatVisualizer(
    model_reduced, collision_model_reduced, visual_model_reduced
)


visualizer.initViewer()
visualizer.loadViewerModel()

q_red = pin.neutral(model_reduced)
print(q_red)
eps = 1e-3
DT = 1e-1
damp = 1e-1
IT_MAX = 20000
JOINT_ID = 4


R_des = np.array(
    [
        [0.980062, 0.0029636, -0.198672],
        [-0.0030271, 0.999995, -1.58699e-05],
        [0.198671, 0.000616952, 0.980066],
    ]
)
oMdes = pin.SE3(
    R_des, np.array([0.147782, -3.96171e-06, 0.029025])
)  # think of oMdes as oTdes, the transfromation from desired to origin expressed in origin
i = 0


visualizer.display(q_red)

q = pin.neutral(model)

while True:
    pin.forwardKinematics(model_reduced, data_reduced, q_red)
    iMd = data_reduced.oMi[JOINT_ID].actInv(
        oMdes
    )  # active inverse, chnages basis and moves the vector/point to the appropriate position in the new basis
    err = pin.log(
        iMd
    ).vector  # in joint frame  The logarithm map (log()) is a mathematical operation that maps elements from the group SE(3) to its associated Lie algebra, se(3). This operation effectively converts the complex rotational and translational transformation into a simpler vector representation that captures the "difference" or "error" between two poses in SE(3).
    norm_err = norm(err)
    if norm_err < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pin.computeJointJacobian(
        model_reduced, data_reduced, q_red, JOINT_ID
    )  # jacobian computed in e-e frame
    J = -np.dot(pin.Jlog6(iMd.inverse()), J)
    q_dot_red = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q_red = pin.integrate(model_reduced, q_red, q_dot_red * DT)

    # #Collision check
    # pin.updateGeometryPlacements(
    #     model_reduced, data_reduced, collision_model_reduced, geom_data, q_red
    # )  # updates geom_data by reference

    # # an active pair is a pair of bodies in a joint considered for collision
    # collision_detected = pin.computeCollisions(
    #     model_reduced, data_reduced, collision_model_reduced, geom_data, q_red, True
    # )  # this returns data,geom_data. polymorphic function

    # if not collision_detected:
    #     for idx in range(model_reduced.nq):
    #         # Assuming model.lowerPositionLimit and model.upperPositionLimit store the limits
    #         q_red[idx] = max(
    #             model_reduced.lowerPositionLimit[idx],
    #             min(q_red[idx], model_reduced.upperPositionLimit[idx]),
    #         )

    visualizer.display(q_red)
    print(norm_err)
    i += 1


if success:
    print("Convergence achieved!")
else:
    print(
        "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
    )

print("\nresult: %s" % q_red.flatten().tolist())
print("\nfinal error: %s" % err.T)

pin.forwardKinematics(model_reduced, data_reduced, q_red)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model_reduced.names, data_reduced.oMi):
    print(("{:<24} : {: .3f} {: .3f} {: .3f}".format(name, *oMi.translation.T.flat)))
