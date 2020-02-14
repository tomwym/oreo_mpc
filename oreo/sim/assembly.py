import pybullet as p
import time
import pybullet_data

# Setup environment
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/oreo/Documents/oreo_sim/oreo/sim")
p.setRealTimeSimulation(0)
startPos = [0,0,1]
startOrn = p.getQuaternionFromEuler([0,0,0])
urdf_flags = p.URDF_USE_INERTIA_FROM_FILE 
linkage = p.loadURDF("assembly.urdf", startPos, startOrn, useFixedBase = 1, flags = urdf_flags)
print("body_id:" + str(linkage))

# See startup info
numJoints = p.getNumJoints(linkage)
print("There are " + str(numJoints) + " links in file")
jointInfoList = {}
for i in range(numJoints):
    jointInfo = p.getJointInfo(linkage, i)
    jointInfoList[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    state = p.getLinkState(linkage, jointInfo[0])
    print(jointInfo[1].decode('UTF-8') + str(state[4]) + str(p.getEulerFromQuaternion(state[5])))

# Close kinematic loops using constraints 
constraint_type = p.JOINT_POINT2POINT
cid_far_left = p.createConstraint(linkage, jointInfoList['left_eye_joint'], linkage, jointInfoList['dogbone_joint_far_left'], constraint_type, [0, 0, 0], [16.47e-3, 12.81e-3,19.77e-3], [30.25e-3, 0, 0])
p.changeConstraint(cid_far_left, maxForce = 100000)

# Set dynamics of the links (friction etc.)


# Enable joint control (use position control to begin)
ctrl_mode = p.POSITION_CONTROL
maxforce = 100
p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['pitch_piece_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['skull_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_far_left'], targetPosition = 0, controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_mid_left'], targetPosition = 0, controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_mid_right'], targetPosition = 0, controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['dogbone_joint_far_right'], targetPosition = 0, controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['left_eye_yolk_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['left_eye_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['right_eye_yolk_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['right_eye_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)

# Create debug window to control joints
neck_ctrl = p.addUserDebugParameter("neck_joint", -1.57, 1.57, 0)
linear_motor_ctrl = p.addUserDebugParameter("linear_motor_rod_joint_far_left", -0.1, 0.1, 0)

# Set some positions
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)
while (1):
    # Update position according to sliders
    neck_posn = p.readUserDebugParameter(neck_ctrl)
    linear_motor_posn = p.readUserDebugParameter(linear_motor_ctrl)
    p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = neck_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = linear_motor_posn, controlMode = ctrl_mode, force=maxforce)


p.disconnect()