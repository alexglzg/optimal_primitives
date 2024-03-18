from rockit import *
from casadi import *

import numpy as np


nx = 5
nu = 2
Nhor = 20

starting_angle = 0.0
start_x = 0.0
start_y = 0.0
start_v = 0.0
start_w = 0.0

end_angle = 0.0
end_x = 1.0
end_y = 0.0

current_X = vertcat(start_x,start_y,starting_angle,start_v,start_w)

T0 = 0.0

ocp = Ocp(T=FreeTime(T0))

x = ocp.state()
y = ocp.state()
theta = ocp.state()
v = ocp.state()
w = ocp.state()

u1 = ocp.control()
u2 = ocp.control()

X_0 = ocp.parameter(nx)
final_x = ocp.parameter()
final_y = ocp.parameter()
final_angle = ocp.parameter()

ocp.set_der(x, (v*cos(theta)))
ocp.set_der(y, (v*sin(theta)))
ocp.set_der(theta, w)
ocp.set_der(v, u1)
ocp.set_der(w, u2)

ocp.add_objective(ocp.T)
ocp.subject_to(ocp.at_tf(x)==final_x)
ocp.subject_to(ocp.at_tf(y)==final_y)
ocp.subject_to(ocp.at_tf(theta)==final_angle)

ocp.subject_to( (-0.1 <= u1) <= 0.1 )
ocp.subject_to( (-0.1 <= u2) <= 0.1 )
ocp.subject_to( (0 <= v) <= 1 )
ocp.subject_to( (-1 <= w) <= 1 )

X = vertcat(x, y, theta, v, w)
ocp.subject_to(ocp.at_t0(X)==X_0)

# Pick a solution method
options = {"ipopt": {"print_level": 0}}
options["expand"] = True
options["print_time"] = False
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))

ocp.set_value(X_0, current_X)
ocp.set_value(final_x, end_x)
ocp.set_value(final_y, end_y)
ocp.set_value(final_angle, end_angle)
# Solve
#sol = ocp.solve()

#tsa, xsol = sol.sample(x, grid='control')

def return_poses(xf, yf, thetaf, x0):
    ocp.set_value(X_0, x0)
    ocp.set_value(final_x, xf)
    ocp.set_value(final_y, yf)
    ocp.set_value(final_angle, thetaf)
    sol = ocp.solve()
    tsa, xsol = sol.sample(x, grid='control')
    _, ysol = sol.sample(y, grid='control')
    _, thetasol = sol.sample(theta, grid='control')
    
    d = 0
    for i in range(np.size(xsol)-1):
        d+= np.sqrt((xsol[i+1]-xsol[i])**2 + (ysol[i+1]-ysol[i])**2)

    return xsol, ysol, thetasol, d