import numpy as np
import sys
from casadi import *
import do_mpc
import matplotlib.pyplot as plt


def poolData(yin, nVars, polyorder=3, usesine=0):
    yout = SX.sym('yout',84,1)
    ind = 0
    yout[ind] = 1
    ind = ind+1

    # poly order 1
    for i in range(nVars):
        yout[ind] = yin[i]
        ind = ind+1

    if polyorder >= 2:
        # poly order 2
        for i in range(nVars):
            for j in range(i,nVars):
                yout[ind] = yin[i]*yin[j]
                ind = ind+1

    if polyorder >= 3:
        # poly order 3
        for i in range(nVars):
            for j in range(i,nVars):
                for k in range(j,nVars):
                    yout[ind] = yin[i]*yin[j]*yin[k]
                    ind = ind+1

    return yout

model_type = 'continuous' # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

_x = model.set_variable(var_type='_x', var_name='x', shape=(4,1))
_u = model.set_variable(var_type='_u', var_name='u', shape=(2,1))

Z = SX.sym('Z',6,1)
Z[0:4] = _x 
Z[4:6] = _u

y = poolData(Z, 6) # returns as (84 x 1)
Xi = np.loadtxt('Xi.txt', delimiter=',') # (84 x 6)
dx = np.transpose(Xi) @ y # tbh idek how these matricies are multiplied i just go by size :p
model.set_rhs('x', dx[0:4])

Q = np.diag([100,1,100,1])
#[10,0.1,10,0.1])
#x_targ = np.array([ 0.3, 0, 0.3, 0])
x_targ = np.array([ 0.3, 0, -0.3, 0])
#x_targ = np.array([ -0.3, 0, 0.3, 0])
#x_targ = np.array([ -0.3, 0, -0.3, 0])
x_targ = np.reshape(x_targ, (4,1))
model.set_expression(expr_name='cost', expr=sum1((_x.T-np.transpose(x_targ)) @ Q @ (_x-x_targ)))
model.setup()
# x[0]^2 * Q[0] + x[2]^2 * 
dt = 1/130.

mpc = do_mpc.controller.MPC(model)
setup_mpc = {
    'n_robust': 0,
    'n_horizon': 20,
    't_step': dt,
    'state_discretization': 'collocation',
    'store_full_solution':True,
    # Use MA27 linear solver in ipopt for faster calculations:
    # 'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
}

mpc.set_param(**setup_mpc)

mterm = model.aux['cost'] # terminal cost
lterm = model.aux['cost'] # terminal cost
 # stage cost

mpc.set_objective(mterm=mterm, lterm=lterm)

mpc.set_rterm(u=1e-4) # input penalty
mpc.set_objective(mterm=mterm, lterm=lterm)

mpc.set_rterm(u=1e-4) # input penalty

max_x = np.array([[0.3], [10.0], [0.4], [10.0]])
# lower bounds of the states
mpc.bounds['lower','_x','x'] = -max_x
# upper bounds of the states
mpc.bounds['upper','_x','x'] = max_x
# lower bounds of the input
mpc.bounds['lower','_u','u'] = -0.2
# upper bounds of the input
mpc.bounds['upper','_u','u'] =  0.2

mpc.setup()
estimator = do_mpc.estimator.StateFeedback(model)
simulator = do_mpc.simulator.Simulator(model)

simulator.set_param(t_step = dt)
simulator.setup()

# Seed
np.random.seed(99)

# Initial state
e = np.zeros([model.n_x,1])
x0 = np.random.uniform(-3*e,3*e) # Values between +3 and +3 for all states
mpc.x0 = x0
simulator.x0 = x0
estimator.x0 = x0


# Use initial state to set the initial guess.
mpc.set_initial_guess()
graphics = do_mpc.graphics.Graphics(mpc.data)
# Create figure with arbitrary Matplotlib method
fig, ax = plt.subplots(2, sharex=True)
# Configure plot (pass the previously obtained ax objects):
graphics.add_line(var_type='_x', var_name='x', axis=ax[0])
graphics.add_line(var_type='_u', var_name='u', axis=ax[1])

iterations = 50
for k in range(iterations):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    
    graphics.plot_results()
    graphics.plot_predictions()
    graphics.reset_axes()
    plt.draw()
    plt.sca(ax[0])
    plt.plot(0, x_targ[0], 'c:')
    plt.plot(0, x_targ[2], color ='greenyellow', linestyle =':')
    plt.axhline(x_targ[0], color ='cyan', linestyle =':')
    plt.axhline(x_targ[2], color ='greenyellow', linestyle =':')
    plt.legend(['phi','dphi','psi','dpsi',
                'phi_proj','dphi_proj','psi_proj','dpsi_proj',
                'phi_targ', 'psi_targ'],
               loc=3, fontsize='small')
    plt.pause(0.005)
    

from matplotlib import rcParams
rcParams['axes.grid'] = True
rcParams['font.size'] = 18

import matplotlib.pyplot as plt
fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data, figsize=(16,9))
graphics.plot_results()
plt.sca(ax[0])
plt.axhline(x_targ[0], color ='cyan', linestyle =':')
plt.axhline(x_targ[2], color ='greenyellow', linestyle =':')
plt.legend(['phi','dphi','psi','dpsi',
            'phi_proj','dphi_proj','psi_proj','dpsi_proj',
            'phi_targ', 'psi_targ'],
            loc=3, fontsize='x-small')
graphics.reset_axes()
plt.show()