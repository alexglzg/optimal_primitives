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

v_dot = ocp.control()
w_dot = ocp.control()

X_0 = ocp.parameter(nx)
final_x = ocp.parameter()
final_y = ocp.parameter()
final_angle = ocp.parameter()
upper_speed = ocp.parameter()
lower_speed = ocp.parameter()
max_rot_speed = ocp.parameter()
max_linear_acc = ocp.parameter()
max_rot_acc = ocp.parameter()

ocp.set_der(x, (v*cos(theta)))
ocp.set_der(y, (v*sin(theta)))
ocp.set_der(theta, w)
ocp.set_der(v, v_dot)
ocp.set_der(w, w_dot)

ocp.add_objective(ocp.T)
ocp.subject_to(ocp.at_tf(x)==final_x)
ocp.subject_to(ocp.at_tf(y)==final_y)
ocp.subject_to(ocp.at_tf(theta)==final_angle)

ocp.subject_to( (-max_linear_acc <= v_dot) <= max_linear_acc )
ocp.subject_to( (-max_rot_acc <= w_dot) <= max_rot_acc )
ocp.subject_to( (lower_speed <= v) <= upper_speed )
ocp.subject_to( (-max_rot_speed <= w) <= max_rot_speed )

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

def return_poses(_xf, _yf, _thetaf, _x0, _max_vel, _min_vel, _max_ang_vel, _max_lin_acc, _max_ang_acc):
    ocp.set_value(X_0, _x0)
    ocp.set_value(final_x, _xf)
    ocp.set_value(final_y, _yf)
    ocp.set_value(final_angle, _thetaf)
    ocp.set_value(upper_speed, _max_vel)
    ocp.set_value(lower_speed, _min_vel)
    ocp.set_value(max_rot_speed, _max_ang_vel)
    ocp.set_value(max_linear_acc, _max_lin_acc)
    ocp.set_value(max_rot_acc, _max_ang_acc)
    sol = ocp.solve()
    tsa, xsol = sol.sample(x, grid='control')
    _, ysol = sol.sample(y, grid='control')
    _, thetasol = sol.sample(theta, grid='control')
    
    d = 0
    for i in range(np.size(xsol)-1):
        d+= np.sqrt((xsol[i+1]-xsol[i])**2 + (ysol[i+1]-ysol[i])**2)

    return xsol, ysol, thetasol, d