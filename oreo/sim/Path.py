import numpy as np
import matplotlib.pyplot as plt
import csv
import SaccadeGenerator as SG
import datetime
import shutil

class Path(object):

    def __init__(self, readPathName='', show_plot=0):
        self.paths = []


        self.lastpitch = 0
        self.lastyaw = 0

        self.pitchlist = np.array([self.lastpitch])
        self.yawlist = np.array([self.lastyaw])

        if readPathName == '':
            print('Probably no file provided...')
        else:
            if self.readPathText(readPathName):
                self.main()

        if show_plot:
            plotPitchYaw(self.yawlist, self.pitchlist)

    def readPathText(self, readPathName):    
        with open(readPathName, 'r') as file:
            reader = csv.reader(file)
            self.paths = list(reader)
        return 1

    def main(self):
        for row in self.paths:
            key = row[0]
            if key == 'straight':
                self.addStraight(float(row[1]), float(row[2]))
            elif key == 'spiral': # spiral,theta1,theta2,r1,r2,npoints
                self.addSpiral((float(row[1]), float(row[2])), 
                               (float(row[3]), float(row[4])),
                                int(row[5]))
            elif key == 'yzz': # yzz,pitch1,yaw1,pitch2,yaw2
                self.addZigZag(list1range = (float(row[1]), float(row[3])), 
                               list2range = (float(row[2]), float(row[4])),
                               npoints = int(row[5]),
                               option='y')
            elif key == 'pzz': # pzz,pitch1,yaw1,pitch2,yaw2
                self.addZigZag(list1range = (float(row[2]), float(row[4])),
                               list2range = (float(row[1]), float(row[3])), 
                               npoints = int(row[5]), option='p')                                      
            elif key == 'ymgs':
                self.addGridSpace(list1range = (-0.4, 0.4), 
                                  list2range = (-0.35, 0.4), 
                                  nseparations = int(row[1]),
                                  option='y')
            elif key == 'pmgs':
                self.addGridSpace(list1range = (0.4, -0.35), 
                                  list2range = (0.4, -0.4), 
                                  nseparations = int(row[1]),
                                   option='p')


    def addStraight(self, nextyaw, nextpitch):

        self.lastpitch = nextpitch
        self.lastyaw = nextyaw

        self.concatLists(np.array([nextpitch]), np.array([nextyaw]), option='y')


    def addSpiral(self, anglerange, radiusrange, n):

        anglelist = np.linspace(anglerange[0], anglerange[1], n, endpoint=True)
        anglelist = np.deg2rad(anglelist)

        radiuslist = np.linspace(radiusrange[0], radiusrange[1], n, endpoint=True)

        pitchlist = np.multiply(radiuslist, np.sin(anglelist))
        yawlist = np.multiply(radiuslist, np.cos(anglelist))

        self.lastpitch = pitchlist[-1]
        self.lastyaw = yawlist[-1]

        self.concatLists(pitchlist, yawlist, option='y')


    def addZigZag(self, list1range, list2range, npoints, option):

        # pitchlist describes the pitch angle discretized
        list1 = np.ones((npoints))
        list1[::2] = list1range[0]
        list1[1::2] = list1range[1]

        # yawlist describes the yaw angle discretized
        list2 = np.linspace(list2range[0], list2range[1], npoints, endpoint=True)

        self.lastpitch = list1[-1]
        self.lastyaw = list2[-1]

        self.concatLists(list1, list2, option)

    def addMakePitchZigZag(self, pitchrange, yawrange, npoints):

        # yawlist describes something
        yawlist = np.ones((npoints))
        yawlist[::2] = yawrange[0]
        yawlist[1::2] = yawrange[1]

        # pitchlist describes the yaw angle discretized
        pitchlist = np.linspace(pitchrange[0], pitchrange[1], npoints, endpoint=True)

        self.addLast(pitchlist[-1], yawlist[-1])
        self.concatLists(pitchlist, yawlist)

    def addGridSpace(self, list1range, list2range, nseparations, option):

        # list1 describes the axis discretized
        list1_linear = np.linspace(list1range[0], list1range[1], nseparations, endpoint=True)
        list1 = np.zeros((2*nseparations))
        list1[::2] = list1_linear
        list1[1::2] = list1_linear

        # yawlist describes the yaw angle discretized
        list2 = np.zeros((2*nseparations))
        for i in range(nseparations):
            list2[2*i], list2[2*i+1] = list2range if (i % 2) == 0 else list2range[::-1]

        self.addLast(list1[-1], list2[-1])
        self.concatLists(list1, list2, option)


    def addPitchMajorGridSpace(self, pitchrange, yawrange, npitch, nyaw):
        # pitchlist describes the pitch angle discretized
        pitchlist = np.linspace(pitchrange[0], pitchrange[1], npitch)
        # yawlist describes the yaw angle discretized
        yawlist = np.linspace(yawrange[0], yawrange[1], nyaw)

        pitchspace = np.zeros((pitchlist.shape[0], yawlist.shape[0]))
        yawspace = np.zeros((pitchlist.shape[0], yawlist.shape[0]))

        for i in range(yawlist.shape[0]):
            yawspace[i] = np.ones(pitchlist.shape[0]) * yawlist[i]
            flippitch = lambda i, pitchlist : pitchlist if (i % 2 == 0) else np.flip(pitchlist)
            pitchspace[i] = flippitch(i,pitchlist) #yawlist[::-1]

        pitchspace = pitchspace.flatten('C')
        yawspace = yawspace.flatten('C')

        self.addLast(pitchspace[-1], yawspace[-1])
        self.concatLists(pitchspace, yawspace)


    def concatLists(self, list1, list2, option):
        if option == 'y':
            self.pitchlist = np.concatenate((self.pitchlist, list1))
            self.yawlist = np.concatenate((self.yawlist, list2))
        elif option == 'p':
            self.pitchlist = np.concatenate((self.pitchlist, list2))
            self.yawlist = np.concatenate((self.yawlist, list1))

    def addLast(self, pitch, yaw):
        self.lastpitch = pitch
        self.lastyaw = yaw

    def appendSaccade(self, _nsamplestotal=1000):
        proj = SG.SaccadeGenerator(_nsamplestotal)
        proj.MonteCarlo(SG.limitFunction)
        proj.makeTargets()
        proj.makePath()
        self.concatLists(proj.pitchlist, proj.yawlist, 'y')


def plotPitchYaw(yawspace, pitchspace):
    plt.close('all') 
    fig = plt.figure()
    axes = fig.add_axes([0.1,0.1,0.8,0.8]) 
    axes.plot(yawspace[0], pitchspace[0], 'ro')
    axes.plot(yawspace, pitchspace, 'b-o') 
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

#def plotFromPointsList():

    # asdf = np.load('./dat/trainpathpoints-07-07-21_11:05:36.pointslist')
    # print(np.shape(asdf))
    # yawspace = asdf[:,0]
    # pitchspace = asdf[:,1]
    # plotPitchYaw(yawspace, pitchspace)

def encapsulatePaths(save=0):
    readsourcename = 'trainpath.csv'
    datestring = datetime.datetime.today().strftime('%d-%m-%y_%I:%M:%S')
    newsourcename = './dat/trainpath-'+datestring+'.csv'
    if save:
        shutil.copy(readsourcename,newsourcename)

    path2 = Path(readsourcename)
    path2.appendSaccade(_nsamplestotal=1000)
    plotPitchYaw(path2.yawlist, path2.pitchlist)

    newpointsname = './dat/trainpathpoints-'+datestring+'.pointslist'
    if save:
        asdf = np.concatenate((path2.yawlist, path2.yawlist))
        asdf.dump(newpointsname)

    return (path2.yawlist, path2.pitchlist)

    # plotPitchYaw(path2.yawlist, path2.pitchlist)


if __name__ == '__main__':

    encapsulatePaths(save=0)
    #plotFromPointsList()