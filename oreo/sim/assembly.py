import pybullet as p
import time
import pybullet_data
from scipy.spatial.transform import Rotation as R
import logging

#Setup debug prints
useDebug = True
if (useDebug):
    logging.basicConfig(level=logging.DEBUG, format='%(levelname)s:%(message)s')
    logging.info('LOG_LEVEL: DEBUG')
else:
    logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(message)s')
    logging.info('LOG_LEVEL: INFO')

# Setup environment
#physicsClient = p.connect(p.GUI)
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath("~/dev/oreo/oreo_sim/oreo/sim")
p.setRealTimeSimulation(0)
startPos = [0,0,0]
startOrn = p.getQuaternionFromEuler([0,0,0])
urdf_flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER
linkage = p.loadURDF("assembly.urdf", startPos, startOrn, useFixedBase = 1, flags = urdf_flags)

# See startup info
numJoints = p.getNumJoints(linkage)
logging.info("There are %d links in file\n", numJoints)
jointInfoList = {}
for i in range(numJoints):
    jointInfo = p.getJointInfo(linkage, i)
    jointInfoList[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    state = p.getLinkState(linkage, jointInfo[0])
    coll_data = p.getCollisionShapeData(linkage, jointInfo[0])
    logging.debug("%s %d", jointInfo[1].decode('UTF-8'), i)
    logging.debug("links: %s %d", jointInfo[12], jointInfo[16])
    logging.debug("com frame (posn & orn) %s %s", str(state[0]), str(p.getEulerFromQuaternion(state[1])))
    logging.debug("inertial offset (posn & orn) %s %s", str(state[2]), str(p.getEulerFromQuaternion(state[3])))
    logging.debug("link frame (posn & orn) %s %s", str(state[4]), str(p.getEulerFromQuaternion(state[5])))
    logging.debug("type %s", str(jointInfo[2]))
    logging.debug("collision (path, posn, orn) %s \n\n", str(coll_data))

# Set collision characteristics
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_far_left'], jointInfoList['left_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_mid_left'], jointInfoList['left_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_mid_right'], jointInfoList['right_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_far_right'], jointInfoList['right_eye_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_far_right'], jointInfoList['pitch_piece_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['dogbone_joint_far_left'], jointInfoList['pitch_piece_joint'], 1)
p.setCollisionFilterPair(linkage, linkage, jointInfoList['skull_joint'], jointInfoList['pitch_piece_joint'], 1)

# Set dynamics of the links (friction etc.)

# Go to home position
p.resetJointState(linkage, jointInfoList['neck_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['pitch_piece_joint'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetValue = 0)
p.resetJointState(linkage, jointInfoList['skull_joint'], targetValue = 0)
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
eyeRot = R.from_quat(eyeState[1])
eyePos = eyeRot.apply([16.47e-3, 12.81e-3, 19.77e-3]).tolist()
logging.debug("eye constraint: %s\n", str(eyePos))
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
# p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
# p.setJointMotorControl2(linkage, jointInfoList['pitch_piece_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
# p.setJointMotorControl2(linkage, jointInfoList['skull_joint'], targetPosition = 0, controlMode = ctrl_mode, force = maxforce)
# p.setJointMotorControl2(linkage, jointInfoList['left_eye_yolk_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)
# p.setJointMotorControl2(linkage, jointInfoList['left_eye_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)
# p.setJointMotorControl2(linkage, jointInfoList['right_eye_yolk_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)
# p.setJointMotorControl2(linkage, jointInfoList['right_eye_joint'], controlMode = p.VELOCITY_CONTROL, force = 0)

# Spherical joint control
# p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_far_left'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])
# p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_mid_left'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])
# p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_mid_right'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])
# p.setJointMotorControlMultiDof(linkage, jointInfoList['dogbone_joint_far_right'], controlMode = p.POSITION_CONTROL, targetPosition = [0], targetVelocity = [0, 0, 0], positionGain = 0, velocityGain = 1, force = [0,0,0])


# Create debug window to control joints
# neck_ctrl = p.addUserDebugParameter("neck_joint", -1.57, 1.57, 0)
# pitch_piece_ctrl = p.addUserDebugParameter("pitch_piece_joint", -1.57, 1.57, 0)
# skull_ctrl = p.addUserDebugParameter("skull_joint", -1.57, 1.57, 0)
linear_motor_far_left_ctrl = p.addUserDebugParameter("motor_rod_joint_far_left", -0.1, 0.1, 0)
linear_motor_mid_left_ctrl = p.addUserDebugParameter("motor_rod_joint_mid_left", -0.1, 0.1, 0)
linear_motor_far_right_ctrl = p.addUserDebugParameter("motor_rod_joint_far_right", -0.1, 0.1, 0)
linear_motor_mid_right_ctrl = p.addUserDebugParameter("motor_rod_joint_mid_right", -0.1, 0.1, 0)

# Print contact

# Manual Control
p.setGravity(0,0,-9.8)

useRealTime = False
if useRealTime :
    p.setRealTimeSimulation(1)

# Logging
log_id = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "oreo_log.txt")

while (1):
    # Get input from sliders
    neck_posn = p.readUserDebugParameter(neck_ctrl)
    pitch_piece_posn = p.readUserDebugParameter(pitch_piece_ctrl)
    linear_motor_far_right_posn = p.readUserDebugParameter(linear_motor_far_right_ctrl)
    linear_motor_far_left_posn = p.readUserDebugParameter(linear_motor_far_left_ctrl)
    linear_motor_mid_right_posn = p.readUserDebugParameter(linear_motor_mid_right_ctrl)
    linear_motor_mid_left_posn = p.readUserDebugParameter(linear_motor_mid_left_ctrl)
    skull_posn = p.readUserDebugParameter(skull_ctrl)

    # Set joints to user-defined positions
    # p.setJointMotorControl2(linkage, jointInfoList['neck_joint'], targetPosition = neck_posn, controlMode = ctrl_mode, force = maxforce)
    # p.setJointMotorControl2(linkage, jointInfoList['pitch_piece_joint'], targetPosition = pitch_piece_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_left'], targetPosition = linear_motor_far_left_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_left'], targetPosition = linear_motor_mid_left_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_mid_right'], targetPosition = linear_motor_mid_right_posn, controlMode = ctrl_mode, force = maxforce)
    p.setJointMotorControl2(linkage, jointInfoList['linear_motor_rod_joint_far_right'], targetPosition = linear_motor_far_right_posn, controlMode = ctrl_mode, force = maxforce)
    # p.setJointMotorControl2(linkage, jointInfoList['skull_joint'], targetPosition = skull_posn, controlMode = ctrl_mode, force = maxforce)
    
    if useRealTime == False :
        #events = p.getMouseEvents()
        #for e in events :
            #if (e[0] == p.MOUSE_BUTTON_EVENT) && ()
        p.stepSimulation()

p.stopStateLogging(log_id)
p.disconnect()