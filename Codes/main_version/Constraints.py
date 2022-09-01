import numpy as np
import math
from Environment import Environment
#from Particle import Particle
from Vec2D import Vec2D

import cv2

mu_fraction = 0.5       # applies on V_T
damp_fraction = 0.5     # applies on V_N

vec_tresh_hold = 0.01   # applies on any vectors

y_floor = 495


def collision_detect(particle, env):
    (y, x) = np.where(env.border > 0)
    #for particle in particles:
    (x_pos, y_pos) = particle.p_pos()

    if y_pos >= y_floor:
        print("hit!")
        particle.isCollide = True
        # update the velocity after collision
        V_T, V_N = deflect(particle.vel, env.world_N[-1, x_pos])

        V_N = damp_fraction * V_N
        if norm2(V_N) < vec_tresh_hold:
            V_N.fill(0)
        V_T = particle.mu_fraction * V_T

        if norm2(V_T) < vec_tresh_hold:
            V_T.fill(0)


        particle.vel = V_T + V_N
        #particle.forces[1] = -V_T + V_N

        # update position after collision
        particle.pred_pos[0] = x_pos
        particle.pred_pos[1] = (y_floor - (y_pos - y_floor))
        #particle.isCollide = True
        if y_pos == y_floor:
            print("super hit====>", particle.name)
            #particle.vel = particle.mu_fraction * particle.vel
            particle.forces[1] = particle.mass * np.array([0, -9.8])
            unit = 1
            if particle.vel[0] < 0:
                unit = -1
            particle.forces[2] = particle.mass * 9.8 * np.array([-particle.mu_fraction * particle.vel[0], 0])
            print(particle.forces[2]+particle.forces[1]+particle.forces[0],"|", particle.forces.sum())
    #else:
    #    if y_pos +1 == y_floor:
    #        print("on floor!")
    #        #xx = input("hello")
    #        particle.pred_pos[0] = x_pos
    #        particle.pred_pos[1] = y_floor + 1
    #        particle.isCollide = True
    #        particle.forces[1] = particle.mass * np.array([0, -9.8])


    ### ---end for---
### ---end collision_detect---

def constraint_projection(particles, M):
    for i in range(len(particles)):
        for j in range(len(particles)):
            if M[i, j] != 0:
                print(i,j)
                dist = norm2(particles[i].pred_pos - particles[j].pred_pos)
                # print(dist)
                # xx =input()
                dp_1 = 0
                dp_2 = 0
                # print("state:", particles[0].vel + particles[1].vel, dist)
                if dist == 0:
                    dist = 0.00001
                    print("state:", (i, j), particles[i].vel, particles[j].vel, dist)
                    # writer_vid.release()
                    # xx = input()

                mass_sum = particles[i].mass + particles[j].mass
                factor_1 = particles[j].mass / mass_sum
                factor_2 = particles[i].mass / mass_sum
                dp_1 = (-factor_1 * (dist - M[i, j]) / dist) * ((particles[i].pred_pos - particles[j].pred_pos))
                dp_2 = (factor_2 * (dist - M[i, j]) / dist) * ((particles[i].pred_pos - particles[j].pred_pos))
                k_1 = 1
                k_2 = 1
                if particles[i].isCollide:
                    k_1 = 0
                if particles[j].isCollide:
                    k_2 = 0
                particles[i].pred_pos = particles[i].pred_pos + k_1 * dp_1
                particles[j].pred_pos = particles[j].pred_pos + k_2 * dp_2
            ### ---end if---
        ### ---end for j---
    ### ---end for i---
    #xx = input()
### ---end constraints_projection---




def constraint_projection_Q(particles, M, starts_old):
    (n, n) = M.shape
    visited_t = [0] * n
    inqu_t = [0] * n
    visited = np.array(visited_t)
    visited_edge = np.full_like(M, 0)
    inqu = np.array(inqu_t)
    print(visited)
    q = []
    # define starts
    check = False
    starts_new = []
    starts = []
    for p in particles:
        if p.isCollide:
            starts_new.append(p.name)
            print("wow, this is here!")
            check = True

    if len(starts_new) == 0:
        #st = np.random.randint(n)
        #starts.append(st)
        starts = starts_old
    else:
        starts = starts_new

    for item in starts:
        q.append(item)
    inqu[starts] = 1
    while len(q) != 0:
        current = q.pop(0)
        # do whatever you need to do
        print("current is:", current)
        i = current
        neighbours = np.where(M[current, :] != 0)[0]
        for j in neighbours:
            if M[i, j] != 0 :
                #and visited_edge[i, j] == 0
                print(i, j)
                visited_edge[i, j] = 1
                visited_edge[j, i] = 1
                dist = norm2(particles[i].pred_pos - particles[j].pred_pos)
                # print(dist)
                # xx =input()
                dp_1 = 0
                dp_2 = 0
                # print("state:", particles[0].vel + particles[1].vel, dist)
                if dist == 0:
                    dist = 0.00001
                    print("state:", (i, j), particles[i].vel, particles[j].vel, dist)
                    # writer_vid.release()
                    # xx = input()

                mass_sum = particles[i].mass + particles[j].mass
                factor_1 = particles[i].mass / mass_sum
                factor_2 = particles[j].mass / mass_sum
                dp_1 = (-factor_1 * (dist - M[i, j]) / dist) * ((particles[i].pred_pos - particles[j].pred_pos))
                dp_2 = (factor_2 * (dist - M[i, j]) / dist) * ((particles[i].pred_pos - particles[j].pred_pos))
                k_1 = 1
                k_2 = 1
                if particles[i].isCollide:
                    k_1 = 0
                if particles[j].isCollide:
                    k_2 = 0
                particles[i].pred_pos = particles[i].pred_pos + k_1 * dp_1
                particles[j].pred_pos = particles[j].pred_pos + k_2 * dp_2
            # end whatever i want to do
        visited[current] = 1
        # print("nei is:", neighbours)
        for v in neighbours:
            if visited[v] == 0 and inqu[v] == 0:
                q.append(v)
                inqu[v] = 1
            ### ---end while---
    #if check:
    #    xx = input()
    return starts
### ---end constraints_projection_Q---


#def damping(particles):

# tool functions
def norm2(vec):
    return math.sqrt(np.sum(np.power(vec, 2)))
### ---end norm2---

def deflect(v_vec, n_vec):
    dot_res = -np.sum(v_vec * n_vec)
    V_N = dot_res * n_vec
    if norm2(V_N) < vec_tresh_hold:
        V_N.fill(0)
    V_T = v_vec + V_N

    return V_T, V_N
### ---end deflect---

blue_color = (255, 0, 0)
green_color = (0, 255, 0)
black_color = (0, 0, 0)

def draw_frame(frame, particles, M):
    frame.fill(255)
    frame = cv2.line(frame, (0, y_floor), (500, y_floor), black_color, 1)
    for i in range(len(particles)):
        for j in range(i+1, len(particles)):
            if M[i, j] != 0:
                frame = cv2.line(frame, particles[i].d_pos(), particles[j].d_pos(), green_color, 1)
                ### ---end if---
        ### ---end for j---
    ### ---end for i---
    for p in particles:
        frame = cv2.circle(frame, p.d_pos(), 2, p.color, 2)
    ### ---end for---
### ---end draw_frame---




