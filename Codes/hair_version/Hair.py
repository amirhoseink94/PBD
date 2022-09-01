import numpy as np
import re
from Particle import *
import cv2

class Hair(object):
    def __init__(self):
        self.hair_particles, self.hair_M, self.hair_W = self.read_hair_configuration()
        self.n_hair_particles = len(self.hair_particles)
        self.hair_particles[0].isMoving = False
        #self.pinned_position = self.hair_particles[0].pos
    ### ---end __init__---###


    def read_hair_configuration(self):
        f = open("Body\Hair.txt", 'r')
        particles = []
        f_line = f.readline()
        n_particles = int(f.readline())
        print(f_line)
        M = np.ndarray(shape=(n_particles, n_particles), dtype=float)
        W = np.ndarray(shape=(n_particles, n_particles), dtype=float)
        M.fill(0)
        W.fill(0)
        # line = f.readline()
        for i in range(n_particles):
            line = f.readline()
            print("reading:", line)
            data = [x.strip() for x in re.split(',|#|\n|:|\)', line)]

            p = Particle(x=float(data[1]), y=float(data[2]), v_x=float(data[3]), v_y=float(data[4]),
                         mass=float(data[5]), mu_fraction=float(data[6]), name=int(data[0]))
            print(data)
            for j in range(8, 8 + int(data[7])):
                M[i, int(data[j])] = 1
                M[int(data[j]), i] = 1

            p.print_particle()
            particles.append(p)

            #print(M)

        ### ---end while---

        # reading the wave part
        w_particles = int(f.readline())
        for l_num in range(w_particles):
            line = f.readline()
            print("reading:", line)
            data = [x.strip() for x in re.split(',|#|\n|:|\)', line)]
            M[int(data[0]), int(data[1])] = 1
            M[int(data[1]), int(data[0])] = 1
            W[int(data[0]), int(data[1])] = 1
            W[int(data[1]), int(data[0])] = 1
        for i in range(n_particles):
            for j in range(n_particles):
                if M[i, j] == 1:
                    M[i, j] = norm2(particles[i].pos - particles[j].pos)
                    if M[i, j] == 0:
                        M[i, j] = 0.0001
                        print("what the func just happend!")
                        #xx = input()
            ### ---end for j---
        ### ---end for i---
        print(M)
        print(W)

        return particles, M, W
    ### ---end read_hair_configuration---




    def draw_hair_frame(self, frame):
        #frame.fill(255)
        blue_color = (255, 0, 0)
        green_color = (0, 255, 0)

        for i in range(self.n_hair_particles):
            for j in range(i + 1, self.n_hair_particles):
                if self.hair_M[i, j] != 0 and self.hair_W[i, j] == 0:
                    frame = cv2.line(frame, self.hair_particles[i].d_pos(), self.hair_particles[j].d_pos(), green_color, 2)
                    ### ---end if---
            ### ---end for j---
        ### ---end for i---
        #for p in self.hair_particles:
        #   frame = cv2.circle(frame, p.d_pos(), 1, p.color, 1)
        ### ---end for---
    ### ---end draw_frame---
### --- end class Environment