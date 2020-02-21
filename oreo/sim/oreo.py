import pybullet as p
import time
from scipy.spatial.transform import Rotation as R
import logging

class Oreo_Robot(object):
    # Class which controls the oreo robot
    # All directions (far_left etc. are from robot's perspective (looking in +ve x-dir))
    # Member Vars
    linkage = 0
    physicsClient = 0
    numJoints = 0
    jointDict = {}

    numConstraints = 4
    constraintList = []
    constraintLinks = [
                        ['left_eye_joint', 'dogbone_joint_far_left'],
                        ['left_eye_joint', 'dogbone_joint_mid_left'],
                        ['right_eye_joint', 'dogbone_joint_mid_right'],
                        ['right_eye_joint', 'dogbone_joint_far_right'], 
                      ]
    PARENT_IDX = 0
    CHILD_IDX = 1
    constraintParentPos = [[0.015, -0.0016, 0.0245], [0.0126, -0.0257, 0.0095], [0.0134, 0.0025, 0.0262], [0.0143, -0.0224, 0.0124]] # pos on eye
    constraintChildPos = [30.25e-3, 0, 0]   # pos on dogbone
    constraintAxis = [0,0,0]
    constraintType = p.JOINT_POINT2POINT

    actJointNames = ["neck_joint", "pitch_piece_joint", "skull_joint", "linear_motor_rod_joint_far_left", "linear_motor_rod_joint_mid_left", \
                "linear_motor_rod_joint_mid_right", "linear_motor_rod_joint_far_right"]
    actJointIds = []
    actJointPos = []
    actJointNum = 0
    actJointHome = 0
    prismaticLim = [-0.1, 0.1]
    revoluteLim = [-1.57, 1.57]
    LIM_MIN_IDX = 0
    LIM_MAX_IDX = 1
    manCtrl = []
    actJointControl = p.POSITION_CONTROL

    dumbJointNames = ["left_eye_yolk_joint", "left_eye_joint", "right_eye_yolk_joint", "right_eye_joint"]
    dumbJointHome = 0
    dumbJointIds = 0
    dumbJointNum = 0
    dumbJointVelo = 0
    dumbJointControl = p.VELOCITY_CONTROL
    dumbJointForce = 0

    spherJointNames = ["dogbone_joint_far_left", "dogbone_joint_mid_left", "dogbone_joint_mid_right", "dogbone_joint_far_right"]
    spherJointHome = [[0, 0, 0.00427], [0, 0, 0.00427], [0, 0, 0.00427], [0, 0, -0.00427], [0, 0, -0.00427]]
    spherJointIds = []
    spherJointNum = 0
    spherJointPos = [0]
    spherJointVelo = [0,0,0]
    spherJointControl = p.POSITION_CONTROL
    spherJointKp = 0
    spherJointKv = 1
    spherJointForce = [0,0,0]

    

    collisionLinks = [
                    ['dogbone_joint_far_left', 'left_eye_joint'],
                    ['dogbone_joint_mid_left', 'left_eye_joint'],
                    ['dogbone_joint_mid_right', 'right_eye_joint'],
                    ['dogbone_joint_far_right', 'right_eye_joint'], 
    ]

    # Constants
    INIT_POS = [0,0,0]
    INIT_ORN = [0,0,0]
    CONSTRAINT_MAX_FORCE = 100
    JOINT_MAX_FORCE = 100


    # Constructor
    def __init__(self, enableDebug, enableGUI, urdfPath, urdfName) :
        # Init logging
        if(enableDebug) :
            logging.basicConfig(level=logging.DEBUG, format='%(levelname)s:%(message)s')
            logging.info('LOG_LEVEL: DEBUG')
        else :
            logging.basicConfig(level=logging.INFO, format='%(levelname)s:%(message)s')
            logging.info('LOG_LEVEL: INFO')
    
        # Setup environment
        if(enableGUI) :
            self.physicsClient = p.connect(p.GUI)
        else :
            self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(urdfPath)
        urdf_flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER
        self.linkage = p.loadURDF(urdfName, self.INIT_POS, p.getQuaternionFromEuler(self.INIT_ORN), useFixedBase = 1, flags = urdf_flags)

    # Go to home position
    def HomePos(self) :
        p.setRealTimeSimulation(0)
        # Reset dumb joint positions
        for dumb_idx in range(self.dumbJointNum) :
            p.resetJointState(self.linkage, self.dumbJointIds[dumb_idx], targetValue = self.dumbJointHome)
        
        # Reset actuated joint positions
        for act_idx in range(self.actJointNum) :
            p.resetJointState(self.linkage, self.actJointIds[act_idx], targetValue = self.actJointHome)
        
        # Update position list
        self.actJointPos = [self.actJointHome]*self.actJointNum

        # Reset spherical joints        
        for spher_idx in range(self.spherJointNum) :
            p.resetJointStateMultiDof(self.linkage, self.spherJointIds[spher_idx], targetValue = self.spherJointHome[spher_idx])

    # Enable/Disable collision
    def ToggleCollision(self, enable):
        state = 0
        if enable:
            state = 1
        for i in range(len(self.collisionLinks)) :
            p.setCollisionFilterPair(self.linkage, self.linkage, self.jointDict[self.collisionLinks[i][0]], self.jointDict[self.collisionLinks[i][1]], state)

    # Set joint control for actuators
    def ControlActJoints(self) :
        p.setJointMotorControlArray(self.linkage, self.actJointIds, self.actJointControl, targetPositions = self.actJointPos, forces = [self.JOINT_MAX_FORCE]*self.actJointNum)

    # Set joint control for dumb joints
    def ControlDumbJoints(self) :
        p.setJointMotorControlArray(self.linkage, self.dumbJointIds, self.dumbJointControl, targetVelocities = [self.dumbJointVelo]*self.dumbJointNum, forces = [self.dumbJointForce]*self.dumbJointNum)

    def ControlSpherJoints(self) :
        p.setJointMotorControlMultiDofArray(self.linkage, self.spherJointIds, self.spherJointControl, targetPositions = [self.spherJointPos]*self.spherJointNum, targetVelocities = [self.spherJointVelo]*self.spherJointNum, forces = [self.spherJointForce]*self.spherJointNum, positionGains = [self.spherJointKp]*self.spherJointNum, velocityGains = [self.spherJointKv]*self.spherJointNum)

    # Init manual control
    def InitManCtrl(self) :
        for idx in range(self.actJointNum):
            name = self.actJointNames[idx]
            if "linear" in name:
                self.manCtrl.append(p.addUserDebugParameter(name, self.prismaticLim[self.LIM_MIN_IDX], self.prismaticLim[self.LIM_MAX_IDX]))
            else :
                self.manCtrl.append(p.addUserDebugParameter(name, self.revoluteLim[self.LIM_MIN_IDX], self.revoluteLim[self.LIM_MAX_IDX]))
        p.setRealTimeSimulation(1)

    # Update manual control
    def UpdManCtrl(self) :
        for idx in range(len(self.manCtrl)) :
            self.actJointPos[idx] = p.readUserDebugParameter(self.manCtrl[idx])
        self.ControlActJoints()

    # Initialize robot model
    def InitModel(self, enableCollision) :
        # Map names to id
        self.numJoints = p.getNumJoints(self.linkage)
        logging.info("There are %d links in file\n", self.numJoints)
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.linkage, i)
            self.jointDict[jointInfo[1].decode('UTF-8')] = jointInfo[0]        
            state = p.getLinkState(self.linkage, jointInfo[0])
            coll_data = p.getCollisionShapeData(self.linkage, jointInfo[0])
            logging.debug("%s %d", jointInfo[1].decode('UTF-8'), i)
            logging.debug("com frame (posn & orn) %s %s", str(state[0]), str(p.getEulerFromQuaternion(state[1])))
            logging.debug("inertial offset (posn & orn) %s %s", str(state[2]), str(p.getEulerFromQuaternion(state[3])))
            logging.debug("link frame (posn & orn) %s %s", str(state[4]), str(p.getEulerFromQuaternion(state[5])))
            logging.debug("type %s", str(jointInfo[2]))
            logging.debug("collision (path, posn, orn) %s \n\n", str(coll_data))
    
        # Create list of actuated joints
        self.actJointIds = [self.jointDict[act_name] for act_name in self.actJointNames]
        self.actJointNum = len(self.actJointNames)
        
        # Create list of spherical joints
        self.spherJointIds = [self.jointDict[spher_name] for spher_name in self.spherJointNames]
        self.spherJointNum = len(self.spherJointNames)

        # Create list of dumb joints
        self.dumbJointIds = [self.jointDict[dumb_name] for dumb_name in self.dumbJointNames]
        self.dumbJointNum = len(self.dumbJointNames)

        # Set collision characteristics
        self.ToggleCollision(enableCollision)

        # Go to home position
        self.HomePos()

        # Constraints
        for i in range(self.numConstraints):
            parent_id = self.jointDict[self.constraintLinks[i][self.PARENT_IDX]]
            child_id = self.jointDict[self.constraintLinks[i][self.CHILD_IDX]]
            self.constraintList.append(p.createConstraint(self.linkage, parent_id, self.linkage, child_id, self.constraintType, self.constraintAxis, self.constraintParentPos[i],  self.constraintChildPos))
            p.changeConstraint(self.constraintList[i], maxForce = self.CONSTRAINT_MAX_FORCE)

        # Set joint control
        self.ControlActJoints()
        self.ControlDumbJoints()
        self.ControlSpherJoints()

        # Gravity
        p.setGravity(0,0,-9.8)      