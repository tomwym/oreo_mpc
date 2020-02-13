import pybullet as p
import time
import pybullet_data

# Setup environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/oreo/Documents/oreo_sim/oreo/assembly")
p.setRealTimeSimulation(0)
startPos = [0,0,1]
startOrn = p.getQuaternionFromEuler([0,0,0])
linkage = p.loadURDF("test.urdf", startPos, startOrn, useFixedBase = 1)


#Set joint control here
numJoints = p.getNumJoints(linkage)
jointInfoList = {}
for i in range(numJoints):
    jointInfo = p.getJointInfo(linkage, i)
    jointInfoList[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    print(jointInfo[0])
    p.resetJointState(linkage, i, 0)


# Set some positions
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)
while (1):
    currPos, currOrn = p.getBasePositionAndOrientation(linkage)


p.disconnect()