import casadi as ca

# Define optimization variables
x = ca.MX.sym('x')
y = ca.MX.sym('y')
vars = ca.vertcat(x, y)

# Define objective function
obj = (x - 1)**2 + (y - 2.5)**2

# Define constraints
g1 = x**2 * y - 1
g2 = x + y**2 - 4
g = ca.vertcat(g1, g2)

# Define lower and upper bounds for constraints
lbg = [0, 0]   # g1 >= 0, g2 == 0
ubg = [ca.inf, 0]

# Define lower and upper bounds for variables
lb_vars = [-ca.inf, -ca.inf]
ub_vars = [ca.inf, ca.inf]

# Create NLP dictionary
nlp = {'x': vars, 'f': obj, 'g': g}

# Create solver instance
solver = ca.nlpsol('solver', 'ipopt', nlp)

# Provide initial guess
x0 = [2, 2]

# Solve the problem
sol = solver(x0=x0, lbg=lbg, ubg=ubg, lbx=lb_vars, ubx=ub_vars)

# Extract and display results
x_opt = sol['x']
print("Status:", solver.stats()['return_status'])
print("Optimal solution:", x_opt)
print("Objective value:", float(sol['f']))
