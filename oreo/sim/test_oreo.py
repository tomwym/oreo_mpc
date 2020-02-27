import pybullet as p
import time as t
import pybullet_data
from scipy.spatial.transform import Rotation as R
import logging
import oreo as o

def ManCtrl(robot) :
    robot.InitManCtrl()
    robot.RegKeyEvent(['c', 'q', 'd'])
    while (1):
        robot.UpdManCtrl()
        keys = robot.GetKeyEvents()
        if 'c' in keys :
            robot.QueryAllCollisions()
        if 'q' in keys :
            # quit
            break
        if 'd' in keys :
            robot.QueryConstraint()

def TorqueCtrl(robot) :
    robot.InitTorqueCtrl()
    robot.RegKeyEvent(['c', 'q', 'd'])
    cnt = 0
    torque = 1
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
        if 'c' in keys :
            robot.QueryJointDynamics()
        if 'q' in keys :
            # quit
            break
        if 'd' in keys :
            robot.QueryConstraint()

robot = o.Oreo_Robot(True, True, "/home/oreo/Documents/oreo_sim/oreo/sim", "assembly.urdf", True)
robot.InitModel()
man_ctrl = False

if man_ctrl :
    ManCtrl(robot)
else :
    TorqueCtrl(robot)

robot.Cleanup()







