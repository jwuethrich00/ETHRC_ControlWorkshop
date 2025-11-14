import cvxpy as cp
import numpy as np

# Define problem data
P = np.array([[4, 1],
              [1, 2]])   # Quadratic cost matrix (must be symmetric positive semidefinite)
q = np.array([1, 1])     # Linear cost vector

# Define variables
x = cp.Variable(2)

# Define constraints
constraints = [x >= 0,
               x[0] + x[1] == 1]

# Define objective
objective = cp.Minimize(0.5 * cp.quad_form(x, P) + q.T @ x)

# Define and solve problem
problem = cp.Problem(objective, constraints)
problem.solve()

# Display results
print("Status:", problem.status)
print("Optimal value:", problem.value)
print("Optimal x:", x.value)