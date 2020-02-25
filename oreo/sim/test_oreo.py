import pybullet as p
import time as t
import pybullet_data
from scipy.spatial.transform import Rotation as R
import logging
import oreo as o

def ManCtrl(robot) :
    robot.InitManCtrl()

    while (1):
        robot.UpdManCtrl()
        if robot.QueryKeyEvent('c') :
            # query collision
            if(robot.QueryCollision('left_eye_joint', 'right_eye_joint')) :
                print("left eye and right eye collision detected")
        if robot.QueryKeyEvent('q') :
            # quit
            break

def TorqueCtrl(robot) :
    robot.InitTorqueCtrl()

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
        t.sleep(0.1)
        cnt += 1

robot = o.Oreo_Robot(True, True, "/home/oreo/Documents/oreo_sim/oreo/sim", "assembly.urdf", True)
robot.InitModel()
man_ctrl = False

if man_ctrl :
    ManCtrl(robot)
else :
    TorqueCtrl(robot)





