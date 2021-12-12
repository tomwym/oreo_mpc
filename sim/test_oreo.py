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
from matplotlib import rcParams
import sys
from fractions import Fraction
import datetime


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

"""
def TorqueCtrl(robot) :
    robot.InitTorqueCtrl()
    robot.RegKeyEvent(['q', 'd', 'p'])

    
    # actuator initial positions (when (pitch,yaw)=(0,0))
    init_actuator_pos_outter = p.getLinkState(0,4)[4] # far_left
    init_actuator_pos_middle = p.getLinkState(0,5)[4] # mid_left

    Xi = np.loadtxt('Xi.txt', delimiter=',')

    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    _x = model.set_variable(var_type='_x', var_name='x', shape=(4,1))
    _u = model.set_variable(var_type='_u', var_name='u', shape=(2,1))

    y = poolData()
    dx = y @ Xi


    for i in range(100):

        current_outter_left_pos = p.getLinkState(0,4)[4]
        current_middle_left_pos = p.getLinkState(0,5)[4]

        ctrl_dict = {'linear_motor_rod_joint_far_left': u1,
                    'linear_motor_rod_joint_mid_left': u2}

        getPitch()[0]
        getPitch()[1]
        getYaw()[0]
        getYaw()[1]

        robot.ControlActJoints(ctrl_dict)

        keys = robot.GetKeyEvents()
        if 'q' in keys :
            # quit
            break
        if 'd' in keys :
            robot.PrintConstraintDynamics()
        if 'p' in keys :
            robot.GetLinkPosOrn('right_eye_joint')
"""



def PID_control(robot) :

    robot.InitTorqueCtrl()
    robot.RegKeyEvent(['q', 'd', 'p'])
        
    # actuator initial positions (when (pitch,yaw)=(0,0))
    init_actuator_pos_outter = p.getLinkState(0,4)[4] # far_left
    init_actuator_pos_middle = p.getLinkState(0,5)[4] # mid_left

    pid_outter_left = PID.ActuatorPIDConroller('pid_far_left', init_actuator_pos_outter)
    pid_middle_left = PID.ActuatorPIDConroller('pid_far_left', init_actuator_pos_middle)

    eg = EG.EyeGeometry()

    run_name = 'train'
    datestring = datetime.datetime.today().strftime('%d-%m-%y_%I-%M-%S')
    do_write_sindyc = 1
    do_plot = 1

    if do_write_sindyc:
        f = open('./dat/'+run_name+'sindyc-'+datestring+'.dat',"w+")

    (yawspace, pitchspace) = Path.encapsulatePaths(save=1)

    #path = Path.Path(run_name+'path.csv', 0)
    #path.appendSaccade(_nsamplestotal=2000)
    #pitchspace = path.pitchlist
    #yawspace = path.yawlist

    #Path.plotPitchYaw(yawspace, pitchspace)

    npoints = pitchspace.shape[0]
    iterations = np.ones((npoints), dtype='int32')
    pid_iterations_constant = 2000

    for i, (pitch, yaw) in enumerate(zip(pitchspace[:-1], yawspace[:-1])):
        distance = np.linalg.norm((pitch-pitchspace[i+1], yaw-yawspace[i+1]))
        iterations[i+1] = int(distance*pid_iterations_constant)

    m = int(np.sum(iterations))

    if do_plot:
        tracking = np.zeros((4, m))

    i = 0
    for j, (pitch, yaw) in enumerate(zip(pitchspace, yawspace)):

        actuator, extension = eg.getExtension(yaw, -pitch - 0.025)
        track_outter_to = extension[0]
        track_middle_to = extension[1]
        
        for k in range(iterations[j]):

            current_outter_left_pos = p.getLinkState(0,4)[4]
            current_middle_left_pos = p.getLinkState(0,5)[4]

            u1 = pid_outter_left.getPID(track_outter_to, 
                    cartesian_2_magnitude(init_actuator_pos_outter, current_outter_left_pos))
                # 0.03275
            u2 = pid_middle_left.getPID(track_middle_to, 
                    cartesian_2_magnitude(init_actuator_pos_middle, current_middle_left_pos))
                # 0.03253
            ctrl_dict = {'linear_motor_rod_joint_far_left': u1,
                        'linear_motor_rod_joint_mid_left': u2}

            if do_plot:
                tracking[0][i] = getPitch()[0]
                tracking[1][i] = getYaw()[0]
                tracking[2][i] = u1
                tracking[3][i] = u2

            if do_write_sindyc:
                f.write(str(getPitch()[0]) + '\t')
                f.write(str(getPitch()[1]) + '\t')
                f.write(str(getYaw()[0]) + '\t')
                f.write(str(getYaw()[1]) + '\t')
                f.write(str(u1) + '\t')
                f.write(str(u2) + '\n')

            robot.ControlActJoints(ctrl_dict)

            keys = robot.GetKeyEvents()
            if 'q' in keys :
                # quit
                break
            if 'd' in keys :
                robot.PrintConstraintDynamics()
            if 'p' in keys :
                robot.GetLinkPosOrn('right_eye_joint')

            i = i+1

    if do_write_sindyc:
        f.close()


    if do_plot:
        use_iterations = 0
        if use_iterations:
            x = np.linspace(0, m, m) # on iterations
        else:
            x = np.arange(0, m*robot.TIME_STEP, robot.TIME_STEP) # on time
        plt.close('all') 
        
        plt.subplot(2,1,1)
        plt.plot(x, tracking[0], 'r', label='pitch')

        i = 0
        plt.plot(0,0,'y', label='pitch_target')
        for j, pitch in enumerate(pitchspace):
            iterrange = iterations[j]* (1 if use_iterations else robot.TIME_STEP)
            plt.hlines(pitch, i, i+iterrange, 'y', label='')
            i = i+iterrange
        plt.plot(x, tracking[1], 'b', label='yaw')

        i = 0
        plt.plot(0,0,'c', label='yaw_target')
        for j, yaw in enumerate(yawspace):
            iterrange = iterations[j]* (1 if use_iterations else robot.TIME_STEP)
            plt.hlines(yaw, i, i+iterrange, 'c', label='')
            i = i+iterrange
        plt.legend(loc="best")
        plt.grid() # show grid

        plt.subplot(2,1,2)
        plt.plot(x, tracking[2], 'g', label='u1')
        plt.plot(x, tracking[3], 'm', label='u2')
        plt.legend(loc="best")
        plt.grid() # show grid

        plt.show()



def MPC_control(robot, _dt=1/130.) :
    robot.InitTorqueCtrl()
    robot.RegKeyEvent(['q', 'd', 'p'])
    mpc1 = MPC.MPController(xtarg=np.array([-0.3, 0, 0.3, 0]), xifile='./dat/Xi-2021-7-7_11-16-14.txt', _dt=_dt)
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

    rcParams['axes.grid'] = True
    rcParams['font.size'] = 18

    fig, ax, graphics = do_mpc.graphics.default_plot(mpc1.mpc.data, figsize=(12,7.5))
    graphics.plot_results()
    plt.sca(ax[0])
    plt.axhline(mpc1.x_targ[0], color ='cyan', linestyle =':')
    plt.axhline(mpc1.x_targ[2], color ='greenyellow', linestyle =':')
    plt.legend(['phi','dphi','psi','dpsi',
                'phi_proj','dphi_proj','psi_proj','dpsi_proj',
                'phi_targ', 'psi_targ'],
                loc=3, fontsize='xx-small')
    graphics.reset_axes()
    plt.show()
    

def getPitch():
    # left_eye_link - returns pitch angle, and angular velocity as a tuple
    return (p.getEulerFromQuaternion(p.getLinkState(0,13)[5])[1], p.getLinkState(0,13,1)[7][1])

def getYaw():
    # left_eye_yolk_link - returns yaw angle, and angular velocity as a tuple
    return (p.getEulerFromQuaternion(p.getLinkState(0,12)[5])[2], p.getLinkState(0,12,1)[7][2])

def writeInit():
    f= open('init.dat',"w+")
    f.write('left eye yolk angle' + str(p.getEulerFromQuaternion(p.getLinkState(0,12)[5])) + '\n')
    f.write('left eye yolk omega' + str(p.getLinkState(0,12,1)[7]) + '\n')
    f.write('left eye angle' + str(p.getEulerFromQuaternion(p.getLinkState(0,13)[5])) + '\n')
    f.write('left eye omega' + str(p.getLinkState(0,13,1)[7]) + '\n')
    f.write('right eye yolk angle' + str(p.getEulerFromQuaternion(p.getLinkState(0,14)[5])) + '\n')
    f.write('right eye yolk omega' + str(p.getLinkState(0,14,1)[7]) + '\n')
    f.write('right eye angle' + str(p.getEulerFromQuaternion(p.getLinkState(0,15)[5])) + '\n')
    f.write('right eye omega' + str(p.getLinkState(0,15,1)[7]) + '\n')
    f.close()

def cartesian_2_magnitude(origin, position):
    position = np.asarray(position)

    # determine an arbitrary position scheme - increase x is increase target
    linear_position = np.linalg.norm(position - origin)
    # directional information is lost after taking magnitude of difference to bring to single d
    if origin[0] > position[0]:
        linear_position = -linear_position

    return linear_position

if __name__ == '__main__':
    robot = o.Oreo_Robot(True, True, "~/dev/oreo/oreo_sim/oreo/sim", "assembly.urdf", True, 1/360.)
    robot.InitModel()
    if len(sys.argv) > 1:
        control_operation = int(sys.argv[1])
    else:
        control_operation = 3

    if control_operation == 1:
        ManCtrl(robot)
    elif control_operation == 2 :
        PID_control(robot)
    elif control_operation == 3 :
        if len(sys.argv) > 2:
            MPC_control(robot, float(Fraction(sys.argv[2])))
        else:
            MPC_control(robot, 1/160.)

    robot.Cleanup()


"""
# figuring out yolk vs eye link index + orientation vs omega
# left_eye_yolk_link
#print('left eye yolk angle',  p.getEulerFromQuaternion(p.getLinkState(0,12)[5]))
#print('left eye yolk omega',  p.getLinkState(0,12,1)[7])
# left_eye_link
#print('left eye angle',       p.getEulerFromQuaternion(p.getLinkState(0,13)[5]))
#print('left eye omega',       p.getLinkState(0,13,1)[7])

# right_eye_yolk_link
#print('right eye yolk', p.getEulerFromQuaternion(p.getLinkState(0,14)[5]))
# right_eye_link
#print('right eye',      p.getEulerFromQuaternion(p.getLinkState(0,15)[5]))
"""
