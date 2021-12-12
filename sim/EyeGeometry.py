import numpy as np
import scipy.optimize as so

class EyeGeometry(object):

    def __init__(self):
                                
        self.linkLength = .0605
        self.eyePitchAxisOffset = -0.002 # pitch centre of rotation in z direction (from camera origin)
        self.actuatorAngle = np.radians(17.37)
        self.encoderYaw = 0
        self.encoderPitch = 0
    
        self.eyeCentre = np.array([[.029], [.183864], [.080768]])
        self.actuatorOriginLeft = np.array([[0.0145], [0.218937], [0.042097]])
        self.actuatorOriginRight = np.array([[0.0435], [0.218937], [0.042097]])

        # ball joint position in camera coordinates
        self.ballLeft = np.array([[-0.014250], [0.019538], [0.019809]])
        self.ballRight = np.array([[0.014250], [0.019538], [0.019809]])


    def getHeadCoords(self, eyeCoords, yaw, pitch):
        # Calculates positions of points in eye coordinates subject to
        # yaw and pitch angles. 
        # 
        # eyeCoords: (3 x n) points in eye coordinates
        # yaw: yaw angle of eye (rad)
        # pitch: pitch angle of eye (rad)

        translateForward = self.augmentMatrix(np.eye(3), -np.array([[0], [0], [self.eyePitchAxisOffset]]))
        translateBackward = self.augmentMatrix(np.eye(3), np.array([[0], [0], [self.eyePitchAxisOffset]]))

        rotateYaw = self.augmentMatrix(np.array([[np.cos(yaw), 0, -np.sin(yaw)],
                                                 [0, 1, 0],
                                                 [np.sin(yaw), 0, np.cos(yaw)]]),
                                       np.zeros(3))
        rotatePitch = self.augmentMatrix(np.array([[1, 0, 0],
                                                   [0, np.cos(pitch), np.sin(pitch)],
                                                   [0, -np.sin(pitch), np.cos(pitch)]]),
                                         np.zeros(3))                             
        translateEye = self.augmentMatrix(np.eye(3), self.eyeCentre)

        A = translateEye @ rotateYaw @ translateBackward @ rotatePitch @ translateForward

        result = A @ self.augmentVectors(eyeCoords)
        result = result[0:3,:]

        return result


    def getExtension(self, yaw, pitch):
        # Find how far each linear actuator is extended from yaw = pitch = 0 pos.
        # 
        # yaw: yaw angle of camera
        # pitch: pitch angle of camera
        # 
        # actuator: (3 x 2) positions of actuator endpoints [left right]
        # extension: (1 x 2) extension of each actuator [left right]
        
        actuator = np.zeros((3,2))
        extension = np.zeros(2)

        for i in range(2):
            if i == 0:
                ball = self.getHeadCoords(self.ballLeft, yaw, pitch)
                actuatorOrigin = self.actuatorOriginLeft
            else:
                ball = self.getHeadCoords(self.ballRight, yaw, pitch)
                actuatorOrigin = self.actuatorOriginRight
            
            fun = lambda extension: (self.linkLength - getLinkLength(actuatorOrigin, self.actuatorAngle, extension, ball))**2
            #fun = @(extension) (eg.linkLength - getLinkLength(actuatorOrigin, eg.actuatorAngle, extension, ball))^2
            extension[i] = so.fminbound(fun, -.06, .06)
            actuator[:,i] = getActuator(actuatorOrigin, self.actuatorAngle, extension[i])
        
        return (actuator, extension)

    def augmentMatrix(self, rot, trans):
        result = np.eye(4)
        result[0:3,0:3] = rot
        result[0:3,3] = np.ravel(trans)
        return result

    def augmentVectors(self, vectors):
        # appends a row of ones to bottom of vectors
        return np.vstack((vectors, np.ones(np.shape(vectors)[1])))


def getActuator(actuatorOrigin, actuatorAngle, extension):
    # finds actuator position (of rod end center) given the origin, angle, and 
    # linear extension distance beyond neural (pitch,yaw) = (0,0) 
    return np.ravel(np.add(actuatorOrigin, extension * \
            np.array([[0], [np.sin(actuatorAngle)], [np.cos(actuatorAngle)]])))

def getLinkLength(actuatorOrigin, actuatorAngle, extension, ball):
    # finds Euclidian norm of transformed position of ball joint and actuator
    actuator = getActuator(actuatorOrigin, actuatorAngle, extension)
    return np.linalg.norm(np.ravel(ball) - actuator, 2)

def letsplay():
    yaw = 0
    pitch = 0.1

    eg = EyeGeometry()
    actuator, extension = eg.getExtension(yaw, pitch)

    return eg


if __name__ == '__main__':
    eg = letsplay()