import numpy as np
# from numpy import ones_like, pi, cos, sin, sqrt, arctan2
from numpy.linalg import inv


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib

# plt.ion
# matplotlib.use(
#     "Qt5Agg"
# ) 

import argparse


class node():
    def __init__(self, position):
        self.orig_pos = position
        self.pos = position.copy()
        self.neighbours = []


    # def get_neighbour_dist():
        

class frame_mesh():
    def __init__(self):
        self.spring_const = 2
        self.nat_length = 1

        self.xx, self.yy = np.meshgrid(np.arange(0, 2), np.arange(0, 2))
        self.nodes = []
        for i in range(2):
            for j in range(2):
                self.nodes.append(node( np.array((self.xx[i, j], self.yy[i, j])).astype('float64') ))

        for n1 in self.nodes:
            for n2 in self.nodes:
                if np.abs(n1.orig_pos[0] - n2.orig_pos[0])==1 and np.abs(n1.orig_pos[1] - n2.orig_pos[1])==1:
                    continue
                elif np.abs(n1.orig_pos[0] - n2.orig_pos[0])==1 and np.abs(n1.orig_pos[1] - n2.orig_pos[1])==0:
                    n1.neighbours.append(n2)
                elif np.abs(n1.orig_pos[0] - n2.orig_pos[0])==0 and np.abs(n1.orig_pos[1] - n2.orig_pos[1])==1:
                    n1.neighbours.append(n2)

    def plot_mesh(self, ax):
        for n in self.nodes:
            ax.scatter(n.orig_pos[0], n.orig_pos[1], c='k', s=20, marker='.')
            ax.scatter(n.pos[0], n.pos[1], c='r', s=20, marker='x')
            for neig in n.neighbours:
                ax.plot((n.pos[0], neig.pos[0]), (n.pos[1], neig.pos[1]), color="blue", linestyle="--", linewidth=0.2)

    def norm_2(self, x1, x2, y1, y2):
        return ((x2-x1)**2 + (y2-y1)**2)**(1/2)

    def diff_xy(self, x1, x2, y1, y2):
        return (x2-x1), (y2-y1)

    def bearing(self, x1, x2, y1, y2):
        return np.arctan2(y2-y1, x2-x1)

    def warp_mesh(self, obs_pos):
        min_rad = 0.2
        max_rad = 3
        min_d_F = 0.5
        max_d_F = 0
        m = (max_d_F - min_d_F)/(max_rad - min_rad)
        c = min_d_F

        for i, n in enumerate(self.nodes):
            dist = self.norm_2(n.orig_pos[0], obs_pos[0], n.orig_pos[1], obs_pos[1])
            diff = max_rad - dist
            dx, dy = self.diff_xy(n.orig_pos[0], obs_pos[0], n.orig_pos[1], obs_pos[1])
            ang = self.bearing(n.orig_pos[0], obs_pos[0], n.orig_pos[1], obs_pos[1])
            repel_ang = np.pi + ang
            new_Px = 0
            new_Py = 0
            if dist <= max_rad:
                # F = np.tanh(m*(dist-min_rad)+c)*min_d_F
                F = m*(dist-min_rad)+c
                Fx = F*np.cos(repel_ang)
                Fy = F*np.sin(repel_ang)
                new_Px = Fx/self.spring_const
                new_Py = Fy/self.spring_const
            n.pos = n.orig_pos + np.array((new_Px, new_Py))


class Anim():
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(5,5))
        self.obs_pos = np.array((0.7, 1.5))
        self.meshy = frame_mesh()
        self.ani = FuncAnimation(self.fig, 
                                self.animate, 
                                interval=100,
                                frames=50,
                                )

    def animate(self, i):
        self.ax.cla()
        self.ax.set_xlim(-1, 3)
        self.ax.set_ylim(-1, 3)
        self.meshy.warp_mesh(self.obs_pos)
        plt.style.use("ggplot")   
        self.meshy.plot_mesh(self.ax)
        self.ax.scatter(self.obs_pos[0], self.obs_pos[1], c='m', s=20, marker='o')
        self.obs_pos += np.array((0, -0.1))
        # if self.obs_pos[1] < -1:
        #     self.ani.event_source.stop()

def main():
    an = Anim()
    # an.ani.save('video.mp4', writer='ffmpeg')
    plt.show()
    




if __name__ == "__main__":
    main()
























