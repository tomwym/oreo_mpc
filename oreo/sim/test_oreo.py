import pybullet as p
import time as t
import pybullet_data
from scipy.spatial.transform import Rotation as R
import logging
import oreo as o

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
    cnt = 0
    torque = 0.1
    force = 0
    
    while(1):
        ctrl_dict = {'neck_joint': torque} 
        if(cnt == 10) :
            cnt += 1
            torque *= -1
            force *= -1
            cnt = 0
        
        robot.ControlActJoints(ctrl_dict)
        cnt += 1
        
        keys = robot.GetKeyEvents()
        if 'q' in keys :
            # quit
            break
        if 'd' in keys :
            robot.PrintConstraintDynamics()
        if 'p' in keys :
            robot.GetLinkPosOrn('right_eye_joint')


robot = o.Oreo_Robot(True, True, "/home/oreo/Documents/oreo_sim/oreo/sim", "assembly.urdf", True)
robot.InitModel()
man_ctrl = True

if man_ctrl :
    ManCtrl(robot)
else :
    TorqueCtrl(robot)

robot.Cleanup()







