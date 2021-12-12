import numpy as np
import matplotlib.pyplot as plt

class Path2(object):

    def __init__(self, order, 
                       pitchrange = (-0.4, 0.4), yawrange = (-0.4, 0.4), 
                       npitch = 5, nyaw = 5):
        self.pitchrange = pitchrange
        self.yawrange = yawrange

        self.npitch = npitch
        self.nyaw = nyaw

        if order == 'yaw':
            self.defineYawMajorGridSpace()
        if order == 'pitch':
            self.definePitchMajorGridSpace()
        if order == 'diag':
            self.defineDiagPathSpace()

        # plotPitchYaw(self.yawspace, self.pitchspace)

    def defineYawMajorGridSpace(self):
        # yawlist describes the pitch angle discretized
        self.pitchlist = np.linspace(self.pitchrange[0], self.pitchrange[1], self.npitch)
        # yawlist describes the yaw angle discretized
        self.yawlist = np.linspace(self.yawrange[0], self.yawrange[1], self.nyaw)

        self.pitchspace = np.zeros((self.pitchlist.shape[0], self.yawlist.shape[0]))
        self.yawspace = np.zeros((self.pitchlist.shape[0], self.yawlist.shape[0]))

        for i in range(self.pitchlist.shape[0]):
            self.pitchspace[i] = np.ones(self.yawlist.shape[0]) * self.pitchlist[i]
            flipyaw = lambda i, yawlist : yawlist if (i % 2 == 0) else np.flip(yawlist)
            self.yawspace[i] = flipyaw(i,self.yawlist) #yawlist[::-1]

        self.pitchspace = self.pitchspace.flatten('C')
        self.yawspace = self.yawspace.flatten('C')

        self.length = self.pitchspace.shape[0]

        return (self.pitchspace, self.yawspace)

    def definePitchMajorGridSpace(self):
        # yawlist describes the pitch angle discretized
        self.pitchlist = np.linspace(self.pitchrange[0], self.pitchrange[1], self.npitch)
        # yawlist describes the yaw angle discretized
        self.yawlist = np.linspace(self.yawrange[0], self.yawrange[1], self.nyaw)

        self.pitchspace = np.zeros((self.pitchlist.shape[0], self.yawlist.shape[0]))
        self.yawspace = np.zeros((self.pitchlist.shape[0], self.yawlist.shape[0]))

        for i in range(self.yawlist.shape[0]):
            self.yawspace[i] = np.ones(self.pitchlist.shape[0]) * self.yawlist[i]
            flippitch = lambda i, pitchlist : pitchlist if (i % 2 == 0) else np.flip(pitchlist)
            self.pitchspace[i] = flippitch(i,self.pitchlist) #yawlist[::-1]

        self.pitchspace = self.pitchspace.flatten('C')
        self.yawspace = self.yawspace.flatten('C')

        self.length = self.pitchspace.shape[0]

        return (self.pitchspace, self.yawspace)


    def defineDiagPathSpace(self):
        if self.npitch != self.nyaw:
            print('pitch discretization =/= yaw discretization')
            self.npitch = self.nyaw

        # yawlist describes the pitch angle discretized
        self.pitchlist = np.linspace(self.pitchrange[0], self.pitchrange[1], self.npitch)
        # yawlist describes the yaw angle discretized
        self.yawlist = np.linspace(self.yawrange[0], self.yawrange[1], self.nyaw)

        self.pitchspace = np.copy(self.pitchlist)
        self.yawspace = np.copy(self.yawlist)

        self.length = self.pitchspace.shape[0]

        return (self.pitchspace, self.yawspace)


    def printPitchYaw(self):
        # prints yaw and pitch vectors
        print(self.pitchspace)
        print(self.yawspace)

def plotPitchYaw(yawspace, pitchspace):
    plt.close('all') 
    fig = plt.figure()
    axes = fig.add_axes([0.1,0.1,0.8,0.8]) 
    axes.plot(yawspace[0], pitchspace[0], 'ro')
    axes.plot(yawspace, pitchspace, 'b') 
    axes.plot(yawspace[-1], pitchspace[-1], 'rx')
    axes.set_xlabel('Yaw angle')
    axes.set_ylabel('Pitch angle')
    axes.set_title('Eye tracking goal')
    plt.show()

def catPaths(pathlist):
    pitchspace = np.array([])
    yawspace = np.array([])
    for path in pathlist:
        pitchspace = np.concatenate([pitchspace, path.pitchspace])
        yawspace = np.concatenate([yawspace, path.yawspace])
    
    return (pitchspace, yawspace)

if __name__ == '__main__':
    path1 = Path(order = 'yaw', 
                 pitchrange = (-0.4, 0.4), 
                 yawrange = (-0.4, 0.4), 
                 npitch = 5, 
                 nyaw = 5)
    path2 = Path(order = 'pitch', 
                 pitchrange = (0.4, -0.4), 
                 yawrange = (0.4, -0.4), 
                 npitch = 5, 
                 nyaw = 5)
    
    pitchspace, yawspace = catPaths(path1, path2)
    path1.plotPitchYaw(yawspace, pitchspace)

    path1 = Path(order = 'pitch', 
                 pitchrange = (-0.4, 0.4), 
                 yawrange = (-0.4, 0.4), 
                 npitch = 5, 
                 nyaw = 5)
    path2 = Path(order = 'yaw', 
                 pitchrange = (0.4, -0.4), 
                 yawrange = (0.4, -0.4), 
                 npitch = 5, 
                 nyaw = 5)

    pitchspace, yawspace = catPaths(path1, path2)
    path1.plotPitchYaw(yawspace, pitchspace)