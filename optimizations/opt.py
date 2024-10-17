import numpy as np
from cvxopt import matrix, solvers

# Define humanoid parameters
DOF = 8  # Degrees of Freedom
horizon = 10  # MPC prediction horizon
dt = 0.1  # Time step (seconds)

# Joint limits
q_min = -np.pi/4  # Minimum joint angle (radians)
q_max = np.pi/4   # Maximum joint angle (radians)
u_min = -0.5  # Minimum control input (torque or change in joint angle)
u_max = 0.5   # Maximum control input (torque or change in joint angle)

# Initial and target joint angles
q0 = np.zeros(DOF)  # Initial joint angles
q_target = np.ones(DOF) * np.pi/8  # Target joint angles (one step)

# Weighting matrices for cost function
Q = np.eye(DOF)  # State tracking weight
R = 0.01 * np.eye(DOF)  # Control effort weight

# Define system dynamics (simple linear dynamics for demonstration)
A = np.eye(DOF)  # State transition matrix (identity for simplicity)
B = dt * np.eye(DOF)  # Control input matrix (scaled by dt for simplicity)

def mpc_control(q_current, q_target, max_iter=20):
    """
    Performs MPC optimization for controlling the humanoid robot over a prediction horizon.
    
    q_current: The current joint angles (DOF,)
    q_target: The desired joint angles (DOF,)
    
    Returns: The optimal control input for the first step in the horizon
    """
    # Define the total number of variables
    num_vars = horizon * DOF

    # Build the cost function matrices
    # P matrix: penalizes deviation from the target joint angles
    P = np.kron(np.eye(horizon), Q)

    # q vector: penalizes control effort
    q_vec = np.zeros(num_vars)
    for t in range(horizon):
        q_vec[t*DOF:(t+1)*DOF] = -Q @ q_target

    # P matrix quadratic form (control effort)
    P_control = np.kron(np.eye(horizon), R)

    # Construct the full P matrix
    P = P + P_control

    # Construct the equality constraint matrices
    # These encode the system dynamics over the prediction horizon
    A_eq = np.zeros((DOF * (horizon+1), DOF * horizon))
    b_eq = np.zeros(DOF * (horizon+1))

    # Set the initial state constraint (q(0) = q_current)
    A_eq[:DOF, :DOF] = np.eye(DOF)
    b_eq[:DOF] = q_current

    # Set the dynamic constraints for q(t+1) = A*q(t) + B*u(t)
    for t in range(horizon):
        A_eq[DOF*(t+1):DOF*(t+2), DOF*t:DOF*(t+1)] = -B
        if t < horizon-1:
            A_eq[DOF*(t+1):DOF*(t+2), DOF*(t+1):DOF*(t+2)] = A

    # Inequality constraints for control inputs (u_min <= u <= u_max)
    G_control = np.vstack([np.eye(num_vars), -np.eye(num_vars)])
    h_control = np.hstack([u_max * np.ones(num_vars), -u_min * np.ones(num_vars)])

    # Joint angle constraints (q_min <= q <= q_max)
    G_state = np.vstack([np.eye(num_vars), -np.eye(num_vars)])
    h_state = np.hstack([q_max * np.ones(num_vars), -q_min * np.ones(num_vars)])

    # Full G and h matrices for inequality constraints
    G = np.vstack([G_control, G_state])
    h = np.hstack([h_control, h_state])

    # Convert matrices to CVXOPT format
    P = matrix(P)
    q = matrix(q_vec)
    G = matrix(G)
    h = matrix(h)
    A_eq = matrix(A_eq)
    b_eq = matrix(b_eq)

    # Solve the quadratic program using CVXOPT
    solvers.options['show_progress'] = False
    solution = solvers.qp(P, q, G, h, A_eq, b_eq)

    # Extract the control input for the first step
    u_optimal = np.array(solution['x']).reshape(horizon, DOF)[0]

    return u_optimal

# Simulate the MPC control loop
q_current = q0.copy()  # Start at the initial joint angles
q_trajectory = [q_current]  # To store the trajectory

for step in range(50):  # Simulate for 50 steps
    # Compute the optimal control input using MPC
    u_optimal = mpc_control(q_current, q_target)

    # Update the state (apply the control input)
    q_next = A @ q_current + B @ u_optimal
    q_trajectory.append(q_next)

    # Move to the next state
    q_current = q_next

# Convert trajectory list to array for easier analysis
q_trajectory = np.array(q_trajectory)

# Print the final joint positions
print("Final Joint Positions after MPC:")
print(q_trajectory[-1])

# Plotting (optional)
import matplotlib.pyplot as plt

plt.plot(q_trajectory)
plt.title("Joint Angles Over Time with MPC")
plt.xlabel("Time Step")
plt.ylabel("Joint Angle (rad)")
plt.show()
