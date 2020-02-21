import pybullet as p
import time
import pybullet_data
from scipy.spatial.transform import Rotation as R
import logging
import oreo as o

robot = o.Oreo_Robot(True, True, "/home/oreo/Documents/oreo_sim/oreo/sim", "assembly.urdf")
robot.InitModel(True)
robot.InitManCtrl()

while (1):
    robot.UpdManCtrl()