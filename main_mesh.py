import numpy as np
# from numpy import ones_like, pi, cos, sin, sqrt, arctan2
from numpy.linalg import inv


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib

import argparse

class node():
    def __init__(self, position):
        self.orig_pos = position
        self.neighbours = []
        self.acc = np.array((0.0, 0.0))
        self.vel = np.array((0.0, 0.0))
        self.pos = position.copy()
        

class frame_mesh():
    def __init__(self):
        self.spring_const = 5
        self.dampening_const = 5
        self.nat_length = 1
        self.n_nodes = 5

        self.xx, self.yy = np.meshgrid(np.arange(0, self.n_nodes), np.arange(0, self.n_nodes))
        self.nodes = []
        for i in range(self.n_nodes):
            for j in range(self.n_nodes):
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


    def mesh_acc(self, obs_pos):
        min_rad = 0.2
        max_rad = 1
        min_d_F = 10
        max_d_F = 0
        m = (max_d_F - min_d_F)/(max_rad - min_rad)
        c = min_d_F

        for i, n in enumerate(self.nodes):
            dx, dy = self.diff_xy(n.orig_pos[0], n.pos[0], n.orig_pos[1], n.pos[1])
            dxy = np.array((dx, dy))

            Fk = -self.spring_const*dxy

            Fd = -self.dampening_const*n.vel

            dxc, dyc = self.diff_xy(n.pos[0], obs_pos[0], n.pos[1], obs_pos[1])
            dxyc = np.array((dxc, dyc))
            dist = np.sqrt(dxyc[0]**2 + dxyc[1]**2)
            ang = self.bearing(n.orig_pos[0], obs_pos[0], n.orig_pos[1], obs_pos[1])
            repel_ang = np.pi + ang

            Fc = np.array((0.0, 0.0))
            if dist < max_rad:
                # Fc = m*(dxyc-min_rad)+c
                F = m*(dist-min_rad)+c
                Fx = F*np.cos(repel_ang)
                Fy = F*np.sin(repel_ang)
                Fc = np.array((Fx, Fy))

            n.acc += Fk + Fc + Fd

    def neighbour_condition(self):
        min_sep = 1.1
        for i, n in enumerate(self.nodes):
            for i, neigh in enumerate(n.neighbours):
                dist = self.norm_2(n.pos[0], neigh.pos[0], n.pos[1], neigh.pos[1])
                if dist < min_sep:
                    neigh.acc += n.acc

    def mesh_update(self):
        dt = 0.05
        min_sep = 0.8
        for i, n in enumerate(self.nodes):
            n.vel += n.acc*dt
            n.pos += n.vel*dt
            for i, neigh in enumerate(n.neighbours):
                dxc, dyc = self.diff_xy(n.pos[0], neigh.pos[0], n.pos[1], neigh.pos[1])
                dxyc = np.array((dxc, dyc))
                dist = np.sqrt(dxyc[0]**2 + dxyc[1]**2)
                ang = self.bearing(n.pos[0], neigh.pos[0], n.pos[1], neigh.pos[1])
                if dist < min_sep:
                    neigh.pos +=  n.vel*dt

                    
                    # diff = min_sep - dist
                    # diffx = diff*np.cos(ang)
                    # diffy = diff*np.sin(ang)
                    # neigh.pos += np.array((diffx, diffy))

class Anim():
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(5,5))
        self.obs_pos = np.array((0.7, 5.5))
        self.meshy = frame_mesh()
        self.ani = FuncAnimation(self.fig, 
                                self.animate, 
                                interval=100,
                                frames=200,
                                )
        self.max_xlim = 6
        self.min_xlim = -1
        self.max_ylim = 6
        self.min_ylim = -1

        self.obsticles = np.array(((0.7, 5.5), (3.2, 4.5)))

    def animate(self, i):
        min_rad = 0.2
        max_rad = 1
        theta = np.linspace(0, 2*np.pi, 20)


        self.ax.cla()
        self.ax.set_xlim(self.min_xlim, self.max_xlim)
        self.ax.set_ylim(self.min_ylim, self.max_ylim)

        # self.meshy.warp_mesh(self.obs_pos)
        for i, n in enumerate(self.meshy.nodes):
            n.acc = 0

        for obs in self.obsticles:
            self.meshy.mesh_acc(obs)
        # self.meshy.mesh_acc(self.obs_pos)

        # self.meshy.neighbour_condition()
        self.meshy.mesh_update()
        plt.style.use("ggplot")   
        self.meshy.plot_mesh(self.ax)

        for obs in self.obsticles:
            self.ax.scatter(obs[0], obs[1], c='m', s=20, marker='o')
            self.ax.plot(min_rad*np.cos(theta)+obs[0], min_rad*np.sin(theta)+obs[1], c='r')
            self.ax.plot(max_rad*np.cos(theta)+obs[0], max_rad*np.sin(theta)+obs[1], c='g')
        # self.ax.scatter(self.obs_pos[0], self.obs_pos[1], c='m', s=20, marker='o')
        # self.ax.plot(min_rad*np.cos(theta)+self.obs_pos[0], min_rad*np.sin(theta)+self.obs_pos[1], c='r')
        # self.ax.plot(max_rad*np.cos(theta)+self.obs_pos[0], max_rad*np.sin(theta)+self.obs_pos[1], c='g')

        # for i, obs in enumerate(self.obsticles):
        #     self.obsticles[i, :] += np.array((0, -0.05))
        self.obsticles[0, :] += np.array((0, -0.025))
        self.obsticles[1, :] += np.array((-0.025, -0.025))

        # self.obs_pos += np.array((0, -0.05))

        # if self.obs_pos[1] < -1:
        #     self.ani.event_source.stop()

def main():
    an = Anim()
    an.ani.save('video.mp4', writer='ffmpeg', fps=60)
    plt.show()
    




if __name__ == "__main__":
    main()
























