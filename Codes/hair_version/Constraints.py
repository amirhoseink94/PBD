import numpy as np
import math
from Environment import Environment



import cv2

mu_fraction = 0.5       # applies on V_T
damp_fraction = 0.1     # applies on V_N

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
    ### ---end for---
### ---end collision_detect---


def collision_detect_circle(particle, circle):

    #for particle in particles:
    #(x_pos, y_pos) = particle.p_pos()

    r_1 = circle.pos[0]
    r_2 = circle.pos[1]

    c_pos = np.array([r_1, r_2])

    dist_to_centre = norm2(particle.pos - c_pos)

    if dist_to_centre <= circle.r:
        print("hit the circle!")
        particle.isCollide = True


        # update the velocity after collision
        N = particle.pos - circle.pos
        len_N = norm2(N)
        # compute N
        alph = particle.pred_pos[0]
        beta = particle.pred_pos[1]
        a = particle.pos[0]
        b = particle.pos[1]
        var_A = (alph - a)**2 + (beta - b)**2
        var_B = 2 * ((a - r_1) * (alph - a) + (b - r_2) * (beta - b))
        var_C = (a - r_1)**2 + (b - r_2)**2 - circle.r**2
        print("dual:", var_A, var_B, var_C)
        delta = math.sqrt(var_B**2 - 4 * var_A * var_C)
        t_1 = (delta - var_B)/(2 * var_A)
        t_2 = (-delta - var_B)/(2 * var_A)

        print("what we get out of equations:", t_1, t_2, norm2(particle.pos - circle.pos), norm2(particle.pred_pos - circle.pos), circle.r)

        #if (0 <= t_1) and (t_1 <= 1):
        #    N = np.array([a + t_1 * (alph - a), b + t_1 * (beta - b)]) - c_pos
        #    print("######################################",t_1)
        #if (0 <= t_2) and (t_2 <= 1):
        #    N = np.array([a + t_2 * (alph - a), b + t_2 * (beta - b)]) - c_pos
        #    print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^",t_2)

        #####
        N = (1./len_N) * N
        print(norm2(N))
        V_T, V_N = deflect(particle.vel, N)

        V_N = damp_fraction * V_N

        if norm2(V_N) < vec_tresh_hold:
            V_N.fill(0)
        V_T = damp_fraction * V_T

        if norm2(V_T) < vec_tresh_hold:
            V_T.fill(0)
        #xx = input()


        print("new vel is:", particle.vel, "|", V_T + V_N, "++", N)
        print((-particle.vel + V_T + V_N)/N)
        #particle.forces[1] = -V_T + V_N
        #particle.vel = V_T + V_N
        # update position after collision
        l_N = ((particle.pred_pos-c_pos) * N).sum()
        length = norm2(particle.pred_pos - c_pos)
        #particle.pred_pos = particle.pred_pos + 1.5 * (circle.r - l_N) * N
        #particle.pred_pos = (1./length) * (particle.pred_pos - c_pos) * (circle.r + (circle.r - length)) + c_pos
        particle.pred_pos = particle.pos + N * (0*circle.r + 1*(circle.r - len_N))
        print(particle.pos,"|", particle.pred_pos, "dist is:", norm2(particle.pos - particle.pred_pos))
        #particle.pred_pos[0] = x_pos
        #particle.pred_pos[1] = (y_floor - (y_pos - y_floor))


        #particle.isCollide = True
        if dist_to_centre == circle.r:
            print("super hit====>", particle.name)
            xx = input()
            #particle.vel = particle.mu_fraction * particle.vel
            particle.forces[1] = particle.mass * N

            #particle.forces[2] = particle.mass * 9.8 * np.array([-particle.mu_fraction * particle.vel[0], 0])
            #print(particle.forces[2]+particle.forces[1]+particle.forces[0],"|", particle.forces.sum())
    ### ---end for---
    if circle.pos[0] != c_pos[0] or circle.pos[1] != c_pos[1]:
        print("what the hell!")
        xx =input()
### ---end collision_detect---

def constraint_projection(particles, M, k_damping):
    for i in range(len(particles)):
        for j in range(i+1, len(particles)):
            if M[i, j] != 0:
                #print(i,j)
                dist = norm2(particles[i].pred_pos - particles[j].pred_pos)
                # print(dist)
                # xx =input()
                dp_1 = 0
                dp_2 = 0
                # print("state:", particles[0].vel + particles[1].vel, dist)
                if dist == 0:
                    dist = 0.00001
                    #print("state:", (i, j), particles[i].vel, particles[j].vel, dist)
                    # writer_vid.release()
                    # xx = input()

                if dist > 100:
                    #dist = 0.00001
                    print("it seems that we have a breakage!", i, j, dist)
                    #writer_vid.release()
                    #xx = input()

                mass_sum = particles[i].mass + particles[j].mass
                factor_1 = particles[j].mass / mass_sum
                factor_2 = particles[i].mass / mass_sum
                dp_1 = (-factor_1 * (dist - M[i, j]) / dist) * ((particles[i].pred_pos - particles[j].pred_pos))
                dp_2 = (factor_2 * (dist - M[i, j]) / dist) * ((particles[i].pred_pos - particles[j].pred_pos))
                k_1 = k_damping
                k_2 = k_damping
                #if particles[i].isCollide:
                #    k_1 = 0
                #if particles[j].isCollide:
                #    k_2 = 0
                if particles[i].isMoving:
                    particles[i].pred_pos = particles[i].pred_pos + k_1 * dp_1
                    particles[j].pred_pos = particles[j].pred_pos + k_2 * dp_2
                else:
                    particles[j].pred_pos = particles[j].pred_pos + 2 * k_2 * dp_2
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
    #print(visited)
    q = []
    # define starts
    check = False
    starts_new = []
    starts = []
    for p in particles:
        if p.isCollide:
            starts_new.append(p.name)
            #print("wow, this is here!")
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
        #print("current is:", current)
        i = current
        neighbours = np.where(M[current, :] != 0)[0]
        for j in neighbours:
            if M[i, j] != 0 :
                #and visited_edge[i, j] == 0
                #print(i, j)
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
                    #print("state:", (i, j), particles[i].vel, particles[j].vel, dist)
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
                if particles[i].isMoving:
                    particles[i].pred_pos = particles[i].pred_pos + k_1 * dp_1
                    particles[j].pred_pos = particles[j].pred_pos + k_2 * dp_2
                else:
                    particles[j].pred_pos = particles[j].pred_pos + 2 * k_2 * dp_2
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
red_color = (0, 255, 0)

def draw_frame(frame, particles, M):
    frame.fill(0)
    for i in range(len(particles)):
        for j in range(i+1, len(particles)):
            if M[i, j] != 0:
                frame = cv2.line(frame, particles[i].d_pos(), particles[j].d_pos(), red_color, 1)
                ### ---end if---
        ### ---end for j---
    ### ---end for i---
    for p in particles:
        frame = cv2.circle(frame, p.d_pos(), 2, p.color, 2)
    ### ---end for---
### ---end draw_frame---




