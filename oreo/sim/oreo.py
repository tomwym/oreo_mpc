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
    useRealTime = False

    numConstraints = 4
    constraintDict = {}
    constraintLinks = [
                        ['left_eye_joint', 'dogbone_joint_far_left', 'constraint_far_left'],
                        ['left_eye_joint', 'dogbone_joint_mid_left', 'constraint_mid_left'],
                        ['right_eye_joint', 'dogbone_joint_mid_right', 'constraint_mid_right'],
                        ['right_eye_joint', 'dogbone_joint_far_right', 'constraint_far_right'], 
                      ]
    PARENT_IDX = 0
    CHILD_IDX = 1
    NAME_IDX = 2
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

    disCollisionLinks = [
        ['dogbone_joint_far_left', 'left_eye_joint'],
        ['dogbone_joint_mid_left', 'left_eye_joint'],
        ['dogbone_joint_mid_right', 'right_eye_joint'],
        ['dogbone_joint_far_right', 'right_eye_joint'],
        ['dogbone_joint_far_left', 'left_eye_yolk_joint'],
        ['dogbone_joint_mid_left', 'left_eye_yolk_joint'],
        ['dogbone_joint_mid_right', 'right_eye_yolk_joint'],
        ['dogbone_joint_far_right', 'right_eye_yolk_joint'],
    ]

    toggCollisionLinks = [
        ['left_eye_joint', 'right_eye_joint'],
        ['left_eye_yolk_joint', 'right_eye_yolk_joint'],
        ['left_eye_yolk_joint', 'right_eye_joint'],
        ['right_eye_yolk_joint', 'left_eye_joint'],
        ['skull_joint', 'pitch_piece_joint'],
        ['dogbone_joint_far_right', 'pitch_piece_joint'],
        ['dogbone_joint_far_left', 'pitch_piece_joint']

    ]
    
    keys = []

    # Constants
    INIT_POS = [0,0,0]
    INIT_ORN = [0,0,0]
    CONSTRAINT_MAX_FORCE = 10000000
    JOINT_MAX_FORCE = 100
    TORQUE_CONTROL = 0
    POSITION_CONTROL = 1
    TIME_STEP = 1/240
    DYN_STATE_SIZE = 6


    # Constructor
    def __init__(self, enableDebug, enableGUI, urdfPath, urdfName, enableRealTime) :
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
        urdf_flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER | p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES | p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.linkage = p.loadURDF(urdfName, self.INIT_POS, p.getQuaternionFromEuler(self.INIT_ORN), useFixedBase = 1, flags = urdf_flags)
        self.useRealTime = enableRealTime

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
        # Turn off some collisions
        for j in range(len(self.disCollisionLinks)) :
            p.setCollisionFilterPair(self.linkage, self.linkage, self.jointDict[self.disCollisionLinks[j][0]], self.jointDict[self.disCollisionLinks[j][1]], 0)
        
        if enable > 0:
            logging.debug("Collision enabled")
            enable = 1
        else :
            logging.debug("Collision disabled")
            enable = 0
        for i in range(len(self.toggCollisionLinks)) :
            p.setCollisionFilterPair(bodyUniqueIdA = self.linkage, bodyUniqueIdB = self.linkage, linkIndexA = self.jointDict[self.toggCollisionLinks[i][0]], linkIndexB = self.jointDict[self.toggCollisionLinks[i][1]], enableCollision = 1)
            if enable > 0:
                logging.debug("Collision pair: %s %s", self.toggCollisionLinks[i][0], self.toggCollisionLinks[i][1])
        print()
        print()
    
    # Toggle Sim type
    def SetSimType(self, realTime) :
        # Set sim type
        if realTime:
            self.useRealTime = True
            p.setRealTimeSimulation(1)
            logging.debug("Setting real time sim")
        else :
            self.useRealTime = False
            p.setRealTimeSimulation(0)
            p.setTimeStep(self.TIME_STEP)
            logging.debug("Setting step sim with timestep %f", self.TIME_STEP)
    
    # Reset actuated joint control
    def ResetActJointControl(self) :
        # Must set to velocity control with 0 velocity and 0 force to "reset"
        p.setJointMotorControlArray(self.linkage, self.actJointIds, p.VELOCITY_CONTROL, forces = [0]*self.actJointNum)

    # Initialize robot for position control
    def InitPosnCtrl(self) :
        self.SetActJointControlType(self.POSITION_CONTROL)
        self.ResetActJointControl()
        self.ControlActJoints([0]*self.actJointNum)

    #Initialize robot for torque control
    def InitTorqueCtrl(self) :
        self.SetActJointControlType(self.TORQUE_CONTROL)
        self.ResetActJointControl()
        self.ControlActJoints([0]*self.actJointNum)        

    # Set actuated joint control type
    def SetActJointControlType(self, ctrl_type) :
        if ctrl_type == self.TORQUE_CONTROL :
            self.actJointControl = p.TORQUE_CONTROL
            logging.debug("Set torque control")
            # Must be step sim for torque control
            self.SetSimType(False)
        else :
            self.actJointControl = p.POSITION_CONTROL
            logging.debug("Set position control")

    # Set joint control for actuators
    def ControlActJoints(self, control) :
        target = []
        links = []
        if isinstance(control, list) :
            for idx in range(len(control)) :
                target.append(control[idx])
            links = self.actJointIds
        elif isinstance(control, dict) :
            for name in control :
                if name in self.actJointNames:
                    target.append(control[name])
                    links.append(self.jointDict[name])
                else :
                    logging.warning("Unknown dict key in control input for ControlActJoints")
        else :
            logging.error("Unknown control input type")

        if self.actJointControl == p.POSITION_CONTROL :
            p.setJointMotorControlArray(self.linkage, links, self.actJointControl, targetPositions = target, forces = [self.JOINT_MAX_FORCE]*len(target))
        elif self.actJointControl == p.TORQUE_CONTROL :
            p.setJointMotorControlArray(self.linkage, links, self.actJointControl, forces = target)

        # Update simulation
        if self.useRealTime == False :
            p.stepSimulation()

        return 0

    # Set joint control for dumb joints
    def ControlDumbJoints(self) :
        p.setJointMotorControlArray(self.linkage, self.dumbJointIds, self.dumbJointControl, targetVelocities = [self.dumbJointVelo]*self.dumbJointNum, forces = [self.dumbJointForce]*self.dumbJointNum)

    # Set joint control for spherical joints
    def ControlSpherJoints(self) :
        p.setJointMotorControlMultiDofArray(self.linkage, self.spherJointIds, self.spherJointControl, targetPositions = [self.spherJointPos]*self.spherJointNum, targetVelocities = [self.spherJointVelo]*self.spherJointNum, forces = [self.spherJointForce]*self.spherJointNum, positionGains = [self.spherJointKp]*self.spherJointNum, velocityGains = [self.spherJointKv]*self.spherJointNum)

    # Init manual control
    def InitManCtrl(self) :
        self.InitPosnCtrl()
        for idx in range(self.actJointNum) :
            name = self.actJointNames[idx]
            if "linear" in name:
                self.manCtrl.append(p.addUserDebugParameter(name, self.prismaticLim[self.LIM_MIN_IDX], self.prismaticLim[self.LIM_MAX_IDX]))
            else :
                self.manCtrl.append(p.addUserDebugParameter(name, self.revoluteLim[self.LIM_MIN_IDX], self.revoluteLim[self.LIM_MAX_IDX]))

    # Update manual control
    def UpdManCtrl(self) :
        pos = [0]*self.actJointNum
        for idx in range(len(self.manCtrl)) :
            pos[idx] = p.readUserDebugParameter(self.manCtrl[idx])
        self.ControlActJoints(pos)
        self.actJointPos = pos

    # Initialize robot model
    def InitModel(self) :
        # Map names to id
        self.numJoints = p.getNumJoints(self.linkage)
        logging.info("There are %d links in file\n", self.numJoints)
        vis_data = p.getVisualShapeData(self.linkage)
        for i in range(self.numJoints):
            jointInfo = p.getJointInfo(self.linkage, i)
            self.jointDict[jointInfo[1].decode('UTF-8')] = i
            p.enableJointForceTorqueSensor(self.linkage, i, enableSensor = True)
            state = p.getLinkState(self.linkage, jointInfo[0])
            coll_data = p.getCollisionShapeData(self.linkage, i)
            logging.debug("%s %d", jointInfo[1].decode('UTF-8'), i)
            logging.debug("com frame (posn (global) & orn (global)) %s %s", str(state[0]), str(p.getEulerFromQuaternion(state[1])))
            logging.debug("inertial offset (posn (link frame), orn (link frame)) %s %s", str(state[2]), str(p.getEulerFromQuaternion(state[3])))
            logging.debug("link frame (posn (global), orn (global)) %s %s", str(state[4]), str(p.getEulerFromQuaternion(state[5])))
            logging.debug("type %s", str(jointInfo[2]))
            logging.debug("collision (posn (COM frame), orn(COM frame)) %s %s", str(coll_data[0][5]), str(p.getEulerFromQuaternion(coll_data[0][6])))
            logging.debug("visual (posn (link frame), orn(link frame)) %s %s\n\n", str(vis_data[i][5]), str(p.getEulerFromQuaternion(vis_data[i][6])))
    
        # Create list of actuated joints
        self.actJointIds = [self.jointDict[act_name] for act_name in self.actJointNames]
        self.actJointNum = len(self.actJointNames)
        
        
        # Create list of spherical joints
        self.spherJointIds = [self.jointDict[spher_name] for spher_name in self.spherJointNames]
        self.spherJointNum = len(self.spherJointNames)

        # Create list of dumb joints
        self.dumbJointIds = [self.jointDict[dumb_name] for dumb_name in self.dumbJointNames]
        self.dumbJointNum = len(self.dumbJointNames)

        # Enable collision by default
        self.ToggleCollision(1)

        # Go to home position
        self.HomePos()

        # Constraints
        for i in range(self.numConstraints):
            parent_id = self.jointDict[self.constraintLinks[i][self.PARENT_IDX]]
            child_id = self.jointDict[self.constraintLinks[i][self.CHILD_IDX]]
            self.constraintDict[self.constraintLinks[i][self.NAME_IDX]] =  p.createConstraint(self.linkage, parent_id, self.linkage, child_id, self.constraintType, self.constraintAxis, self.constraintParentPos[i],  self.constraintChildPos)
            p.changeConstraint(self.constraintDict[self.constraintLinks[i][self.NAME_IDX]], maxForce = self.CONSTRAINT_MAX_FORCE)

        # Set joint control
        self.SetActJointControlType(self.POSITION_CONTROL)
        self.ResetActJointControl()
        self.ControlDumbJoints()
        self.ControlSpherJoints()

        # Gravity
        p.setGravity(0,0,-9.8)

        # Simulation type
        self.SetSimType(self.useRealTime)
    
    # Check if links have collided
    def QueryCollision(self, linkA, linkB) :
        points = p.getContactPoints(bodyA = self.linkage, bodyB = self.linkage, linkIndexA = self.jointDict[linkA], linkIndexB = self.jointDict[linkB])
        if not points :
            logging.debug("No collision points between %s & %s detected\n", linkA, linkB)
            return False
        else :
            logging.debug("Number of collision points between %s & %s detected: %d\n", linkA, linkB, len(points))
            return True
        print()
        print()
    
    # Check all contact points
    def QueryAllCollisions(self) :
        points=p.getContactPoints()
        logging.debug("Contact Points")
        for temp in points :
            logging.debug("a & b = %d & %d", temp[3], temp[4])
        print()
        print()
    
    # Print constraint state
    def QueryConstraint(self) :
        logging.debug("Constraint States")
        for id in self.constraintDict :
            logging.debug(p.getConstraintState(self.constraintDict[id]))
        print()
        print()
    
    # Print joint states
    def QueryJointDynamics(self) :
        logging.info("Joint States")
        id_list = list(range(p.getNumJoints(self.linkage)))
        ret = p.getJointStates(self.linkage, id_list)
        idx = 0
        for state in ret :
            if(len(state) == 4) :
                logging.debug("Id=%d velo=%s rxn=%s applied=%s", idx, str(state[1]), str(state[2]), str(state[3]))
            elif(len(state) == 3) :
                logging.debug("Id=%d velo=%s rxn=%s", idx, str(state[1]), str(state[2]))
            else :
                logging.error("Unexpected return list length %d", len(state))
            idx += 1
        print()
        print()

    # Get list of keys pressed
    def GetKeyEvents(self) :
        pressed = []
        keyEvents = p.getKeyboardEvents()
        for char in self.keys :
            key = ord(char)
            if key in keyEvents and keyEvents[key]&p.KEY_WAS_TRIGGERED :
                pressed.append(char)
        return pressed  

    # Add key to listen
    def RegKeyEvent(self, userIn) :
        if isinstance(userIn, str) :
            if userIn not in self.keys :
                self.keys.append(userIn)
        elif isinstance(userIn, list) :
            for char in userIn :
                if char not in self.keys :
                    self.keys.append(char)

    # Cleanup stuff
    def Cleanup(self) :
        p.disconnect()
    
    # Reset robot
    def Reset(self) :
        p.resetSimulation()
        self.InitModel()

    # Get joint and constraint dynamics
    # returns [Fx,Fy,Fz,Mx,My,Mz]
    def GetDynamics(self, name) :
        # Determine if constraint or joint
        state = []
        if "joint" in name :
            if self.jointDict[name] in self.actJointIds and self.actJointControl == p.TORQUE_CONTROL:
                # use combination of reaction forces and applied motor torques
                ret = p.getJointState(self.jointDict[name])
                state = ret[2]
            else :
                # use combination of joint reaction forces and 
                ret = p.getJointState(self.jointDict[name])
                state = ret[2]
        elif "constraint" in name :
            state = p.getConstraintState(self.constraintDict[name]).tolist()
            state.extend([0]*(self.DYN_STATE_SIZE-len(state)))
        else: 
            logging.error("Unknown name input to GetDynamics")
        
        return state
    
    # Get all joint and constraint dynamics

        
        