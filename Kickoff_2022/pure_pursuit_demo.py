import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass

def rotationMatrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def wrapAngle(theta): # Wraps angle between -pi and pi
    while(theta > np.pi):
        theta -= 2*np.pi

    while(theta < -np.pi):
        theta += 2*np.pi
    return theta

@dataclass
class PathPoint:
    x: np.ndarray   # Position Vector
    v: float        # Velocity magnitude
    t: float        # time (may not be necessary)
    #theta: float    # Heading angle

class RobotAnim:
    def __init__(self, l, w, x0=np.array([2, 0]), theta0=0):
        self.l = l
        self.w = w
        self.x = x0
        self.theta = theta0
        self.points = np.array([x0, [4,4], [5, 2], [6,0], [7,1]])
        self.pointsIdx = 1

        self.replayBuffer = np.array(x0.reshape((1,2))) 

        self.lookaheadDistance = 0.8

    def diff2twist(self, vl, vr):
        v = (vl+vr)/2
        w = (vr-vl)/self.w
        return (v, w)

    def update(self, vl, vr):
        theta = (vr-vl)/self.w
        if(theta != 0):
            r = (vr + vl)/(2*theta)
            y = r*(1-np.cos(theta))
            x = r*np.sin(theta)
        else:
            v = (vr + vl)/2
            x = v*np.cos(self.theta)
            y = v*np.sin(self.theta)

        self.x = (self.x.reshape((2,1)) + np.matmul(rotationMatrix(self.theta) , np.array([[x], [y]]))).reshape((2,))
        self.theta = wrapAngle(self.theta + theta)
        print(self.x)
        self.replayBuffer = np.append(self.replayBuffer, self.x.reshape((1,2)), axis=0)
        print(self.replayBuffer)

    def getTransformed(self):
        x = [-self.l/2, self.l/2, self.l/2, -self.l/2, -self.l/2]
        y = [self.w/2, self.w/2, -self.w/2, -self.w/2, self.w/2]

        x = np.vstack((x, y))
        return (np.matmul(rotationMatrix(self.theta), x) + self.x.reshape((2,1)))

    def draw(self, line):
        x = self.getTransformed()
        line.set_data(x[0,:], x[1,:])
    
    def computeControl(self):
        distanceToEnd = np.linalg.norm(self.x - self.points[self.pointsIdx, :])

        toNext = self.points[self.pointsIdx, :] - self.points[self.pointsIdx-1, :]
        toNext = toNext/np.linalg.norm(toNext)

        # In coordinates based off starting point
        vToRobot =  self.x - self.points[self.pointsIdx-1,:]
        distanceToLine = np.linalg.norm(vToRobot)**2 - np.dot(vToRobot, toNext)**2
        lookaheadPt = toNext * (np.dot(vToRobot, toNext) + np.sqrt(self.lookaheadDistance**2 - distanceToLine**2))
        lookaheadPt = lookaheadPt + self.points[self.pointsIdx-1, :] # In Global Coordinates

        vRobot = 0.1
        if(distanceToEnd < self.lookaheadDistance):
            if(self.pointsIdx != np.shape(self.points)[0]-1):
                self.pointsIdx += 1
            else:
                lookaheadPt = self.points[self.pointsIdx, :]
                if(np.linalg.norm(self.x - lookaheadPt) > 0.1):
                    vRobot = 0.1*np.linalg.norm(self.x - lookaheadPt)
                else:
                    vRobot = 0


        robotToLookahead = lookaheadPt - self.x
        angleError = np.arctan2(robotToLookahead[1], robotToLookahead[0]) - self.theta
        kappa = 2*np.sin(angleError)/self.lookaheadDistance


        delta = vRobot * self.l * kappa
        vr = vRobot + delta/2
        vl = vRobot - delta/2
        return (vl, vr)




if(__name__ == "__main__"):
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 8), ylim=(-2, 6))
    line, = ax.plot([], [], lw = 3)
    path, = ax.plot([], [], 'b-', lw = 1)
    robot = RobotAnim(0.6,0.4, x0 = np.array([2, 0]))
    ax.plot(robot.points[:,0], robot.points[:,1], 'r--')
    ax.plot([0], [0])
    dt = 0.02 # s

    def init():
        line.set_data([], [])
        path.set_data([], [])
        return line,path,
    def animate(i):
        vl, vr = robot.computeControl()
        robot.update(vl, vr)
        robot.draw(line)
        x_replay = robot.replayBuffer
        path.set_data(x_replay[:,0], x_replay[:,1])
        return line,path,

    anim = FuncAnimation(fig, animate, init_func=init,
                                frames=200, interval=dt*1000, blit=False)
    plt.show()