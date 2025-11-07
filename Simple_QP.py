import cvxpy as cp
import numpy as np

# TO DO
# Define problem data
P =   # Quadratic cost matrix (must be symmetric positive semidefinite)
q =   # Linear cost vector

# TO DO
# Define variables
x = 

# TO DO
# Define constraints
constraints = 

# TO DO
# Define objective
objective = 

# TO DO
# Define and solve problem
problem = 
problem.solve()

# Display results
print("Status:", problem.status)
print("Optimal value:", problem.value)
print("Optimal x:", x.value)