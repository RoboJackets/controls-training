import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass

@dataclass
class PathPoint:
    x: np.ndarray   # Position Vector
    v: float        # Velocity magnitude
    t: float        # time (may not be necessary)
    #theta: float    # Heading angle

class RobotAnim:
    def __init__(self, l, w, x0=np.array([[0], [0]]), theta0=0):
        self.l = l
        self.w = w
        self.x = x0
        self.theta = theta0

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
            r = (vr + vl)/2
            y = r*np.cos(self.theta)
            x = r*np.sin(self.theta)

        self.x = self.x + np.matmul(self.rotationMatrix(self.theta) , np.array([[x], [y]]))
        print(self.x)
        self.theta = self.theta + theta
        print(self.theta)

    @staticmethod
    def rotationMatrix(theta):
        return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def getTransformed(self):
        x = [-self.l/2, self.l/2, self.l/2, -self.l/2, -self.l/2]
        y = [-self.w/2, -self.w/2, self.w/2, self.w/2, -self.w/2]

        x = np.vstack((x, y))
        return np.matmul(self.rotationMatrix(self.theta), x) + self.x



if(__name__ == "__main__"):
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 4), ylim=(-2, 2))
    line, = ax.plot([], [], lw=3)
    robot = RobotAnim(0.6,0.4, x0 = np.array([[2], [0]]))
    dt = 0.02 # s

    path = [[0, 0.5, 1, 1.5, 2, 2,5],
     [0, 1, -1, 0, 1, -1]]
    def init():
        line.set_data([], [])
        return line,
    def animate(i):
        robot.update(0.01, 0.02)
        x = robot.getTransformed()
        line.set_data(x[0,:], x[1,:])
        return line,

    anim = FuncAnimation(fig, animate, init_func=init,
                                frames=200, interval=dt*1000, blit=True)
    plt.show()