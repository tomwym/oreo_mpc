import pybullet as p
import time
import pybullet_data
from scipy.spatial.transform import Rotation as R

# Setup environment
physicsClient = p.connect(p.GUI)
#physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#planeId = p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/oreo/Documents/oreo_sim/oreo/sim")
p.setRealTimeSimulation(0)
startPos = [0,0,0]
startOrn = p.getQuaternionFromEuler([0,0,0])
urdf_flags = p.URDF_USE_INERTIA_FROM_FILE
linkage = p.loadURDF("assembly.urdf", startPos, startOrn, useFixedBase = 1, flags = urdf_flags)

# See startup info
numJoints = p.getNumJoints(linkage)
print("There are " + str(numJoints) + " links in file")
jointInfoList = {}
for i in range(numJoints):
    jointInfo = p.getJointInfo(linkage, i)
    jointInfoList[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    state = p.getLinkState(linkage, jointInfo[0])
    coll_data = p.getCollisionShapeData(linkage, jointInfo[0])
    print(jointInfo[1].decode('UTF-8'))
    print("com frame (posn & orn)" +  str(state[0]) + str(p.getEulerFromQuaternion(state[1])))
    print("inertial offset (posn & orn)" + str(state[2]) + str(p.getEulerFromQuaternion(state[3])))
    print("link frame (posn & orn)" + str(state[4]) + str(p.getEulerFromQuaternion(state[5])))
    print("type " + str(jointInfo[2]))
    print("collision (path, posn, orn)" + str(coll_data))
    print()
    print()

# Set collision characteristics
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_far_left'], jointInfoList['left_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_mid_left'], jointInfoList['left_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_mid_right'], jointInfoList['right_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_far_right'], jointInfoList['right_eye_joint'], 1)

# Set dynamics of the links (friction etc.)

# Go to home position
p.resetJointState(linkage, jointInfoList['neck_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['pitch_piece_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['left_eye_yolk_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['left_eye_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['right_eye_yolk_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['right_eye_joint'], targetValue = 0)

p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_far_left'], targetValue = [0, 0, 0.00427])
p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_mid_left'], targetValue = [0, 0, 0.00427])
p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_mid_right'], targetValue = [0, 0, -0.00427])
p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_far_right'], targetValue = [0, 0, -0.00427])

# Close kinematic loops using constraints 
# Need to transform coordinates to the principal axes of inertia
# [16.47e-3, 12.81e-3, 19.77e-3] [30.25e-3, 0, 0] far left
# [16.47e-3, -15.69e-3, 19.77e-3] [30.25e-3, 0, 0] mid left ****
# [16.47e-3, -14.03e-3, 19.77e-3] [30.25e-3, 0, 0] far right
# [16.47e-3, 14.47e-3, 19.77e-3] [30.25e-3, 0, 0] mid right 
constraint_type = p.JOINT_POINT2POINT
eyeState = p.getLinkState(linkage, jointInfoList['right_eye_joint'])
print("eye state")
print("com frame (posn & orn)" +  str(eyeState[0]) + str(eyeState[1]))
print("inertial offset (posn & orn)" + str(eyeState[2]) + str(eyeState[3]))
print("link frame (posn & orn)" + str(eyeState[4]) + str(eyeState[5]))
print()
print()

eyeRot = R.from_quat(eyeState[1])
eyePos = eyeRot.apply([16.47e-3, 12.81e-3, 19.77e-3]).tolist()
print("eye constraint: " + str(eyePos))
dogbonePos = [30.25e-3, 0, 0]


cid_far_left = p.createConstraint(parentBodyUniqueId = linkage, parentLinkIndex = jointInfoList['left_eye_joint'], childBodyUniqueId=linkage, childLinkIndex = jointInfoList['dogbone_joint_far_left'], jointType = constraint_type, jointAxis = [0, 0, 0], parentFramePosition = [0.015, -0.0016, 0.0245], childFramePosition = dogbonePos)
p.changeConstraint(cid_far_left, maxForce = 100)
cid_mid_left = p.createConstraint(parentBodyUniqueId = linkage, parentLinkIndex = jointInfoList['left_eye_joint'], childBodyUniqueId = linkage, childLinkIndex = jointInfoList['dogbone_joint_mid_left'], jointType = constraint_type, jointAxis = [0, 0, 0], parentFramePosition = [0.0126, -0.0257, 0.0095], childFramePosition = dogbonePos)
p.changeConstraint(cid_mid_left, maxForce = 100)
cid_far_right = p.createConstraint(parentBodyUniqueId = linkage, parentLinkIndex = jointInfoList['right_eye_joint'], childBodyUniqueId = linkage, childLinkIndex = jointInfoList['dogbone_joint_far_right'], jointType = constraint_type, jointAxis = [0, 0, 0], parentFramePosition = [0.0143, -0.0224, 0.0124], childFramePosition = dogbonePos)
p.changeConstraint(cid_far_right, maxForce = 100)
cid_mid_right = p.createConstraint(parentBodyUniqueId = linkage, parentLinkIndex = jointInfoList['right_eye_joint'], childBodyUniqueId = linkage, childLinkIndex = jointInfoList['dogbone_joint_mid_right'], jointType = constraint_type, jointAxis = [0, 0, 0], parentFramePosition = [0.0134, 0.0025, 0.0262], childFramePosition = dogbonePos)
p.changeConstraint(cid_mid_right, maxForce = 100)

# Enable joint control (actuating joints are position control, rest are velo control)
ctrl_mode = p.POSITION_CONTROL
maxforce = 100
p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['pitch_piece_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['left_eye_yolk_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['left_eye_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['right_eye_yolk_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)
p.setJointMotorControl2(linkage, jointInfoList['right_eye_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)

# Spherical joint control
p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_far_left'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])
p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_mid_left'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])
p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_mid_right'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])
p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_far_right'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])

p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_far_left'], targetValue = [0, 0, 0.00427])
p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_mid_left'], targetValue = [0, 0, 0.00427])
p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_mid_right'], targetValue = [0, 0, -0.00427])
p.resetJointStateMultiDof(linkage, jointInfoList['dogbone_joint_far_right'], targetValue = [0, 0, -0.00427])

# Create debug window to control joints
joint_ctrl = {}
neck_ctrl = p.addUserDebugParameter("neck_joint", -1.57, 1.57, 0)
pitch_piece_ctrl = p.addUserDebugParameter("pitch_piece_joint", -1.57, 1.57, 0)
linear_motor_far_left_ctrl = p.addUserDebugParameter("motor_rod_joint_far_left", -0.1, 0.1, 0)
linear_motor_mid_left_ctrl = p.addUserDebugParameter("motor_rod_joint_mid_left", -0.1, 0.1, 0)
linear_motor_far_right_ctrl = p.addUserDebugParameter("motor_rod_joint_far_right", -0.1, 0.1, 0)
linear_motor_mid_right_ctrl = p.addUserDebugParameter("motor_rod_joint_mid_right", -0.1, 0.1, 0)

# Manual Control
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(1)
while (1):
    # Get input from sliders
    neck_posn = p.readUserDebugParameter(neck_ctrl)
    pitch_piece_posn = p.readUserDebugParameter(pitch_piece_ctrl)
    linear_motor_far_right_posn = p.readUserDebugParameter(linear_motor_far_right_ctrl)
    linear_motor_far_left_posn = p.readUserDebugParameter(linear_motor_far_left_ctrl)
    linear_motor_mid_right_posn = p.readUserDebugParameter(linear_motor_mid_right_ctrl)
    linear_motor_mid_left_posn = p.readUserDebugParameter(linear_motor_mid_left_ctrl)

    # Set joints to user-defined positions
    p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = neck_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['pitch_piece_joint'], targetPosition = pitch_piece_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = linear_motor_far_left_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetPosition = linear_motor_mid_left_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetPosition = linear_motor_mid_right_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetPosition = linear_motor_far_right_posn, controlMode = ctrl_mode, force = maxforce)
    


p.disconnect()