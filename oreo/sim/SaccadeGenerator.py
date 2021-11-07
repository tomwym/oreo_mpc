import numpy as np
import matplotlib.pyplot as plt


class SaccadeGenerator(object):

    def __init__(self, _nsamplestotal=1000):

        self.nsamplestotal = _nsamplestotal
        self.nsamples = 0

        self.xbound = 25
        self.ybound = 15

        self.xsample = np.array([])
        self.ysample = np.array([])

        self.lastpitch = 0
        self.lastyaw = 0

        self.pitchlist = np.array([self.lastpitch])
        self.yawlist = np.array([self.lastyaw])


    def randGen(self, n, mag):
        return mag*np.random.rand(n)

    def MonteCarlo(self, limitFunction):
        xsample = self.randGen(self.nsamplestotal, self.xbound)
        ysample = self.randGen(self.nsamplestotal, self.ybound)

        functionlimitmask = np.zeros((self.nsamplestotal))
        for i in range(self.nsamplestotal):
            functionlimitmask[i] = ysample[i] > limitFunction(xsample[i])

        lowerlimitmask = xsample < 1

        mask = np.logical_or(functionlimitmask, lowerlimitmask)
        xsample = np.ma.MaskedArray(xsample, mask).compressed()
        xsample = np.deg2rad(xsample)
        ysample = np.ma.MaskedArray(ysample, mask).compressed()
        self.nsamples = xsample.shape[0]

        xline = np.linspace(0, self.xbound, self.nsamples)
        yline = limitFunction(xline)

        # plt.scatter(xsample, ysample)
        # plt.plot(xline, yline)
        # plt.show()

        self.xsample = np.round(xsample, decimals=3)
        self.ysample = np.round(ysample, decimals=1)
    
    def makeTargets(self):

        mean = [0.05, 0]
        cov = [[0.018, 0], [0, 0.02]]  # diagonal covariance

        self.xypairs = np.random.multivariate_normal(mean, cov, self.nsamples)
        print(self.xypairs.shape)
        
        plt.figure(1)
        plt.plot(self.xypairs[:,0], self.xypairs[:,1], 'x')
        plt.axis('equal')
        plt.show()

    def makePath(self):
        self.pitchlist = np.zeros((self.nsamples))
        self.yawlist = np.zeros((self.nsamples))

        v0 = np.zeros((2))

        for i in range(self.nsamples):
            v = self.xypairs[i] - v0
            v_hat = v / np.linalg.norm(v)
            nextpoint = v_hat*self.xsample[i]
            self.pitchlist[i], self.yawlist[i] = nextpoint

        # plt.figure(2)
        # plt.plot(self.pitchlist, self.yawlist, '-*')
        # plt.axis('equal')
        # plt.show()

def limitFunction(x):
    return 15*np.exp(-x/7.6)


if __name__ == "__main__":
    proj = SaccadeGenerator()
    proj.MonteCarlo(limitFunction)
    proj.makeTargets()
    proj.makePath()