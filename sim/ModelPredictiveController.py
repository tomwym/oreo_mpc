import numpy as np
import sys
from casadi import *
import do_mpc as dm
import matplotlib.pyplot as plt

class MPController(object):

    def __init__(self, xtarg, xifile='Xi.txt', _dt=1/130.):
        model_type = 'continuous' # either 'discrete' or 'continuous'
        self.model = dm.model.Model(model_type)

        _x = self.model.set_variable(var_type='_x', var_name='x', shape=(4,1))
        _u = self.model.set_variable(var_type='_u', var_name='u', shape=(2,1))

        Z = SX.sym('Z',6,1)
        Z[0:4] = _x 
        Z[4:6] = _u

        y = poolData(Z, 6) # returns as (84 x 1)
        Xi = np.loadtxt(xifile, delimiter=',') # (84 x 6)
        dx = np.transpose(Xi) @ y # tbh idek how these matricies are multiplied i just go by size :p
        self.model.set_rhs('x', dx[0:4])

        Q = np.diag([300,1,300,1])
        #[10,0.1,10,0.1])
        #x_targ = np.array([ 0.3, 0, 0.3, 0])
        self.x_targ = xtarg
        #x_targ = np.array([ -0.3, 0, 0.3, 0])
        #x_targ = np.array([ -0.3, 0, -0.3, 0])
        x_targ = np.reshape(self.x_targ, (4,1))
        self.model.set_expression(expr_name='cost', expr=sum1((_x.T-np.transpose(x_targ)) @ Q @ (_x-x_targ)))
        self.model.setup()
        # x[0]^2 * Q[0] + x[2]^2 * 
        dt = _dt

        self.mpc = dm.controller.MPC(self.model)
        setup_mpc = {
            'n_robust': 0,
            'n_horizon': 40,
            't_step': dt,
            'state_discretization': 'collocation',
            'store_full_solution':True,
            # Use MA27 linear solver in ipopt for faster calculations:
            # 'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
        }

        self.mpc.set_param(**setup_mpc)

        mterm = self.model.aux['cost'] # terminal cost
        lterm = self.model.aux['cost'] # terminal cost
        # stage cost

        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        self.mpc.set_rterm(u=1e-4) # input penalty
        self.mpc.set_objective(mterm=mterm, lterm=lterm)

        self.mpc.set_rterm(u=1e-4) # input penalty

        max_x = np.array([[0.3], [10.0], [0.4], [10.0]])
        # lower bounds of the states
        self.mpc.bounds['lower','_x','x'] = -max_x
        # upper bounds of the states
        self.mpc.bounds['upper','_x','x'] = max_x
        force_bound = 10
        # lower bounds of the input
        self.mpc.bounds['lower','_u','u'] = -force_bound
        # upper bounds of the input
        self.mpc.bounds['upper','_u','u'] =  force_bound

        self.mpc.setup()
        self.estimator = dm.estimator.StateFeedback(self.model)
        self.simulator = dm.simulator.Simulator(self.model)

        self.simulator.set_param(t_step = dt)
        self.simulator.setup()



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

if __name__ == "__main__":
    
    mpc1 = MPController()

    # Seed
    np.random.seed(99)

    # Initial state
    e = np.zeros([mpc1.model.n_x,1])
    x0 = np.random.uniform(-3*e,3*e) # Values between +3 and +3 for all states
    mpc1.mpc.x0 = x0
    mpc1.simulator.x0 = x0
    mpc1.estimator.x0 = x0
    mpc1.mpc.set_initial_guess()

    iterations = 50
    for k in range(iterations):
        u0 = mpc1.mpc.make_step(x0)
        y_next = mpc1.simulator.make_step(u0)
        x0 = mpc1.estimator.make_step(y_next)
            

    from matplotlib import rcParams
    rcParams['axes.grid'] = True
    rcParams['font.size'] = 18

    import matplotlib.pyplot as plt
    fig, ax, graphics = dm.graphics.default_plot(mpc1.mpc.data, figsize=(16,9))
    graphics.plot_results()
    plt.sca(ax[0])
    plt.axhline(mpc1.x_targ[0], color ='cyan', linestyle =':')
    plt.axhline(mpc1.x_targ[2], color ='greenyellow', linestyle =':')
    plt.legend(['phi','dphi','psi','dpsi',
                'phi_proj','dphi_proj','psi_proj','dpsi_proj',
                'phi_targ', 'psi_targ'],
                loc=3, fontsize='x-small')
    graphics.reset_axes()
    plt.show()