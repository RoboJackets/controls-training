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

    def getTransformed(self):
        x = [-self.l/2, self.l/2, self.l/2, -self.l/2, -self.l/2]
        y = [-self.w/2, -self.w/2, self.w/2, self.w/2, -self.w/2]

        x = np.vstack((x, y))
        rot = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        return np.matmul(rot, x) + self.x



if(__name__ == "__main__"):
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 4), ylim=(-2, 2))
    line, = ax.plot([], [], lw=3)
    robot = RobotAnim(1,0.4)

    path = [[0, 0.5, 1, 1.5, 2, 2,5],
     [0, 1, -1, 0, 1, -1]]
    def init():
        line.set_data([], [])
        return line,
    def animate(i):
        #robot.computePosition()
        x = robot.getTransformed()
        print(x)
        line.set_data(x[0,:], x[1,:])
        return line,

    anim = FuncAnimation(fig, animate, init_func=init,
                                frames=200, interval=20, blit=True)
    plt.show()