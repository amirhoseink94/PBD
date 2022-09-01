from Particle import Particle
from Environment import Environment
import numpy as np
import imageio
import cv2
import math
import re

from Constraints import *

def read_configuration():
    f = open("Body\shape_configuration.txt",'r')
    particles = []
    f_line =f.readline()
    n_particles = int(f.readline())
    print(f_line)
    M = np.ndarray(shape=(n_particles, n_particles),dtype=float)
    M.fill(0)
    #line = f.readline()
    for i in range(n_particles):
        line = f.readline()
        print("reading:", line)
        data = [x.strip() for x in re.split(',|#|\n|:|\)', line)]
        # [x.strip() for x in my_string.split(',')]
        p = Particle(x=float(data[1]), y=float(data[2]), v_x=float(data[3]), v_y=float(data[4]),
                     mass=float(data[5]), mu_fraction=float(data[6]), name=int(data[0]))
        print(data)
        for j in range(8, 8 + int(data[7])):
            M[i, int(data[j])] = 1
            #M[int(data[j]), i] = float(data[j + 1])

        p.print_particle()
        particles.append(p)
        #line = f.readline()
        print(M)
        #xx=input()
    ### ---end while---
    for i in range(n_particles):
        for j in range(n_particles):
            if M[i,j] == 1:
                M[i,j] = norm2(particles[i].pos - particles[j].pos)
    #M = []
    #for line in f.readline():
    #    print(line)

    print(M)
    #xx =input()
    return particles, M
### ---end read_configuration---


particles, M = read_configuration()


w, h = 300, 505
writer = imageio.get_writer("particle.gif", mode="I")
frame = np.zeros(shape=(h, w, 3))
env = Environment(w, h)
env.set_bounded_box()


fps = 24
fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
writer_vid = cv2.VideoWriter("test.mp4", fourcc, fps, (w, h))

n_frames = 200
color = [(255, 0, 0), (0, 255, 0), (10, 125, 10), (10, 125, 10)]
color_3 = (0, 0, 255)

#read_configuration()

# run the simulation for n_frames
for i in range(n_frames):
    print("frame:", i, "is rendering...")
    # particle step

    # compute all forces
    G = np.array([0, 9.8])
    ind = 0
    for p in particles:
        p.forces[0] = G * p.mass
        print(p.name, p.forces.sum())
        p.compute_next_vel()

        #print("v is", p.vel)
        p.compute_next_pos()

        collision_detect(p, env)
    ### ---end for---

    starts = []
    for p in particles:
        if p.isCollide:
            starts.append(p.name)

    if len(starts) == 0:
        min_particle = 0
        min_position = 0
        for p in particles:
            if p.pos[1] > min_position:
                min_particle = p.name
                min_position = p.pos[1]
        starts.append(0)

    for k in range(100):
        #starts = constraint_projection_Q(particles, M, starts)
        constraint_projection(particles, M)



    for p in particles:
        collision_detect(p, env)

    for p in particles:
        ind += 1
        p.update_pos()
        print(ind)
        p.print_particle()


    draw_frame(frame, particles, M)
    writer.append_data(frame.astype(np.uint8))
    writer_vid.write(frame.astype(np.uint8))

writer_vid.release()
