import numpy as np
from cvxopt import matrix, solvers
solvers.options['show_progress'] = False  # Optional: suppress solver output

class HumanoidMPC:
    def __init__(self, dt, N):
        self.dt = dt  # Time step
        self.N = N    # Prediction horizon

        # Define robot parameters
        self.num_states = 16  # 8 joints * 2 (position and velocity)
        self.num_controls = 6  # 3 for linear acceleration, 3 for angular velocity

        # Weights for the cost function
        self.Q = np.eye(self.num_states)  # State cost
        self.R = np.eye(self.num_controls)  # Control cost

        # Initialize system matrices
        self.A, self.B = self.initialize_system_matrices()

    def initialize_system_matrices(self):
        # Initialize A and B matrices for the linearized system
        A = np.eye(self.num_states)
        A[0:8, 8:16] = np.eye(8) * self.dt  # Update positions based on velocities
        
        B = np.zeros((self.num_states, self.num_controls))
        B[8:11, 0:3] = np.eye(3) * self.dt  # First 3 joints affected by control
        B[11:14, 3:6] = np.eye(3) * self.dt  # Angular velocities affect the next 3 joints
        
        return A, B

    def formulate_qp(self, x0, x_ref):
        # Formulate the Quadratic Program for MPC
        
        # Compute matrices for the QP problem
        P = np.kron(np.eye(self.N), self.R)
        P += np.kron(np.eye(self.N), self.B.T @ self.Q @ self.B)
        P = matrix(P)

        q = np.zeros((self.N * self.num_controls, 1))
        for i in range(self.N):
            q[i*self.num_controls:(i+1)*self.num_controls] = self.B.T @ self.Q @ (self.A @ x_ref[:, i] - x_ref[:, i+1])
        q = matrix(q)

        # Equality constraints
        Aeq = np.zeros((self.num_states, self.N * self.num_controls))
        for i in range(self.N):
            Aeq[:, i*self.num_controls:(i+1)*self.num_controls] = np.linalg.matrix_power(self.A, i) @ self.B

        beq = x_ref[:, -1] - np.linalg.matrix_power(self.A, self.N) @ x0

        # Convert to cvxopt matrices
        Aeq = matrix(Aeq)
        beq = matrix(beq)

        # Inequality constraints (input bounds)
        G = np.vstack([np.eye(self.N * self.num_controls), -np.eye(self.N * self.num_controls)])
        h = np.vstack([np.ones((self.N * self.num_controls, 1)) * 10,  # Upper bound
                       np.ones((self.N * self.num_controls, 1)) * 10]) # Lower bound
        G = matrix(G)
        h = matrix(h)

        return P, q, G, h, Aeq, beq

    def solve(self, x0, x_ref):
        # Solve the MPC problem
        P, q, G, h, Aeq, beq = self.formulate_qp(x0, x_ref)

        # Solve the QP problem
        sol = solvers.qp(P, q, G, h, A=Aeq, b=beq)

        if sol['status'] != 'optimal':
            raise ValueError("Optimal solution not found!")

        # Extract the optimal control sequence
        u_opt = np.array(sol['x']).reshape(self.N, self.num_controls)

        return u_opt

# Example usage
dt = 0.1  # Time step
N = 10    # Prediction horizon

mpc = HumanoidMPC(dt, N)

# Example initial state and reference trajectory (you'll need to provide real data)
x0 = np.zeros(16)
x_ref = np.zeros((16, N+1))  # Reference trajectory
# Add some non-zero values to x_ref to make the problem non-trivial
x_ref[0, :] = np.linspace(0, 1, N+1)  # Example: linear trajectory for first joint

# Solve MPC
# try:
optimal_controls = mpc.solve(x0, x_ref)
print("Optimal control sequence:")
print(optimal_controls)
# except ValueError as e:
#     print(f"Error: {e}")