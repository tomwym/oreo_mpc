import pybullet as p
import time
import pybullet_data

# Setup environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/oreo/Documents/oreo_sim/oreo/sim")
p.setRealTimeSimulation(0)
startPos = [0,0,1]
startOrn = p.getQuaternionFromEuler([0,0,0])
urdf_flags = p.URDF_USE_INERTIA_FROM_FILE 
linkage = p.loadURDF("assembly.urdf", startPos, startOrn, useFixedBase = 1, flags = urdf_flags)
print("body_id:" + str(linkage))

#See startup info
numJoints = p.getNumJoints(linkage)
print("There are " + str(numJoints) + " links in file")
jointInfoList = {}
for i in range(numJoints):
    jointInfo = p.getJointInfo(linkage, i)
    jointInfoList[jointInfo[1].decode('UTF-8')] = jointInfo[0]

# Close kinematic loops using constraints 
constraint_type = p.JOINT_POINT2POINT
cid_far_left = p.createConstraint(linkage, jointInfoList['dogbone_joint_far_left'], linkage, jointInfoList['left_eye_joint'], )

# Set dynamics of the links (friction etc.)


# Enable joint control (use position control to begin)
ctrl_mode = p.POSITION_CONTROL
maxforce = 100
p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['pitch_piece_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['skull_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_far_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_mid_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_mid_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
#p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_far_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['left_eye_yolk_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['left_eye_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['right_eye_yolk_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['right_eye_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)

# Create debug window to control joints
neck_ctrl = p.addUserDebugParameter("neck_joint", -1.57, 1.57, 0)

# Set some positions
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)
while (1):
    # Update position according to sliders
    neck_posn = p.readUserDebugParameter(neck_ctrl)
    p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = neck_posn, controlMode = ctrl_mode, force = maxforce)


p.disconnect()