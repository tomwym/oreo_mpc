import pybullet as p
import time
import pybullet_data

# Setup environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/oreo/Documents/oreo_sim/four_bar_link")
p.setRealTimeSimulation(0)
startPos = [0,0,1]
startOrn = p.getQuaternionFromEuler([0,0,0])
linkage = p.loadURDF("four_bar_link.urdf", startPos, startOrn, useFixedBase = 1)


#Set joint control here
numJoints = p.getNumJoints(linkage)
jointInfoList = {}
for i in range(numJoints):
    jointInfo = p.getJointInfo(linkage, i)
    jointInfoList[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    print(jointInfo[0])

# Set some positions
p.resetJointState(linkage, jointInfoList['joint1'], -1.57)
p.resetJointState(linkage, jointInfoList['joint2'], -1.57)
p.resetJointState(linkage, jointInfoList['joint3'], -1.57)

p.setGravity(0,0,-9.8)

# Constraint to close the kinematic loop
cid = p.createConstraint(linkage, jointInfoList['dummy_joint'], linkage, jointInfoList['joint3'], p.JOINT_POINT2POINT, [0,0,0], [-0.275, 0.005,0],[0.55,-0.005,0])
p.changeConstraint(cid, maxForce=1000000)
print(cid)

# Set joint motors to move at certain velocity
p.setJointMotorControl2(linkage, jointInfoList['joint1'], p.VELOCITY_CONTROL, targetVelocity = -1.57/5, force=1000)
p.setJointMotorControl2(linkage, jointInfoList['joint3'], p.VELOCITY_CONTROL, targetVelocity = -1.57, force=1000)

p.setGravity(0,0,-9.8)

p.setRealTimeSimulation(1)
while (1):
    currPos, currOrn = p.getBasePositionAndOrientation(linkage)


p.disconnect()