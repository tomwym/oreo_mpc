import pybullet as p
import time as t
import pybullet_data
from scipy.spatial.transform import Rotation as R
import logging
import oreo as o
import numpy as np
import matplotlib.pyplot as plt
import PIDController as PID
import EyeGeometry as EG
import Path as Path
from casadi import *
import do_mpc
import ModelPredictiveController as MPC

def ManCtrl(robot) :
    robot.InitManCtrl()
    robot.RegKeyEvent(['c', 'q', 'p'])
    while (1):
        robot.UpdManCtrl()
        keys = robot.GetKeyEvents()
        if 'c' in keys :
            robot.CheckAllCollisions()
        if 'p' in keys :
            robot.GetLinkPosOrn('neck_joint')
        if 'q' in keys :
            # quit
            break


def TorqueCtrl(robot) :
    robot.InitTorqueCtrl()
    robot.RegKeyEvent(['q', 'd', 'p'])
    mpc1 = MPC.MPController()
    phi = getPitch()
    psi = getYaw()
    x0 = np.array([*getPitch(), *getYaw()])

    mpc1.mpc.x0 = x0
    mpc1.mpc.set_initial_guess()

    graphics = do_mpc.graphics.Graphics(mpc1.mpc.data)
    # Create figure with arbitrary Matplotlib method
    fig, ax = plt.subplots(2, sharex=True)
    # Configure plot (pass the previously obtained ax objects):
    graphics.add_line(var_type='_x', var_name='x', axis=ax[0])
    graphics.add_line(var_type='_u', var_name='u', axis=ax[1])


    iterations = 100
    for i in range(iterations):

        x0 = np.array([*getPitch(), *getYaw()])
        u0 = mpc1.mpc.make_step(x0)

        graphics.plot_results()
        graphics.plot_predictions()
        graphics.reset_axes()
        plt.draw()
        plt.sca(ax[0])
        plt.plot(0, mpc1.x_targ[0], 'c:')
        plt.plot(0, mpc1.x_targ[2], color ='greenyellow', linestyle =':')
        plt.axhline(mpc1.x_targ[0], color ='cyan', linestyle =':')
        plt.axhline(mpc1.x_targ[2], color ='greenyellow', linestyle =':')
        plt.legend(['phi','dphi','psi','dpsi',
                    'phi_proj','dphi_proj','psi_proj','dpsi_proj',
                    'phi_targ', 'psi_targ'],
                loc=3, fontsize='small')
        plt.pause(0.005)

        ctrl_dict = {'linear_motor_rod_joint_far_left': u0[0],
                    'linear_motor_rod_joint_mid_left': u0[1]}

        robot.ControlActJoints(ctrl_dict)

        keys = robot.GetKeyEvents()
        if 'q' in keys :
            # quit
            break
        if 'd' in keys :
            robot.PrintConstraintDynamics()
        if 'p' in keys :
            robot.GetLinkPosOrn('right_eye_joint')

    from matplotlib import rcParams
    rcParams['axes.grid'] = True
    rcParams['font.size'] = 18

    fig, ax, graphics = do_mpc.graphics.default_plot(mpc1.mpc.data, figsize=(16,9))
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
    

def getPitch():
    # left_eye_link - returns pitch angle, and angular velocity as a tuple
    return (p.getEulerFromQuaternion(p.getLinkState(0,13)[5])[1], p.getLinkState(0,13,1)[7][1])

def getYaw():
    # left_eye_yolk_link - returns yaw angle, and angular velocity as a tuple
    return (p.getEulerFromQuaternion(p.getLinkState(0,12)[5])[2], p.getLinkState(0,12,1)[7][2])


robot = o.Oreo_Robot(True, True, "~/dev/oreo/oreo_sim/oreo/sim", "assembly.urdf", True)
robot.InitModel()
man_ctrl = False

if man_ctrl :
    ManCtrl(robot)
else :
    TorqueCtrl(robot)

robot.Cleanup()

