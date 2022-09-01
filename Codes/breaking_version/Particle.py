import numpy as np
import math


class Particle(object):
    def __init__(self, x, y, v_x, v_y, mass, a_x=0, a_y=0,
                 dt=1./24., isMoving=True, mu_fraction=0.5, color=(255, 0, 0), name=0):
        self.pos_x = x
        self.pos_y = y
        self.pos = np.array([self.pos_x, self.pos_y])
        self.pred_pos = self.pos

        self.vel_x = v_x
        self.vel_y = v_y
        self.vel = np.array([self.vel_x, self.vel_y])

        self.acc_x = a_x
        self.acc_y = a_y
        self.acc = np.array([self.acc_x, self.acc_y])

        self.mass = mass

        self.dt = dt
        self.isMoving = isMoving
        self.isCollide = False
        self.mu_fraction = mu_fraction
        if self.mass > 60:
            self.color = (0, 0, 255)
        if (self.mass <= 60) and (self.mass > 30):
            self.color = (0, 153, 255)
        if self.mass <= 30:
            self.color = (255, 0, 0)


        self.forces = np.ndarray(shape=(3,), dtype=np.ndarray)
        self.forces[0] = np.array([0, 0])
        self.forces[1] = np.array([0, 0])
        self.forces[2] = np.array([0, 0])

        self.name = name
        self.breaking_point = 5
    ### ---end __init__---

    def compute_next_vel(self):
        if self.isMoving:
            force = self.forces.sum()
            #print("the overal force is:", force)
            self.vel = self.vel + (self.dt * force) / self.mass
            self.vel_x = self.vel[0]
            self.vel_y = self.vel[1]
            self.forces[0].fill(0)
            self.forces[1].fill(0)
            self.forces[2].fill(0)

    ### ---end compute_next_vel---

    def compute_next_pos(self):
        if self.isMoving:
            self.pred_pos = self.pos + self.vel
        else:
            self.pred_pos = self.pos
        # self.pos_x = self.pos[0]
        # self.pos_y = self.pos[1]
        # self.pos = (self.pos_x, self.pos_y)
        #return pos_predict
    ### ---end compute_pos_next---


    def compute_acc(self, force):
        self.acc = force / self.mass
        self.acc_x, self.acc_y = (self.acc[0], self.acc[1])
    ### ---end compute_acc---



    def compute_vel(self):
        self.vel = self.vel + self.dt * self.acc
        self.vel_x = self.vel[0]
        self.vel_y = self.vel[1]
    ### ---end compute_vel---

    def compute_pos(self):
        pos_predict = self.pos + self.dt + self.vel
        #self.pos_x = self.pos[0]
        #self.pos_y = self.pos[1]
        # self.pos = (self.pos_x, self.pos_y)
        return pos_predict
    ### ---end compute_vel---

    def set_pos(self, pos):
        self.pos = pos
    ### ---end set_pos---

    def set_pos(self, x, y):
        self.pos_x = x
        self.pos_y = y
        self.pos[0] = x
        self.pos[1] = y
    ### ---end set_pos---

    def set_vel(self, v_x, v_y):
        self.vel[0] = v_x
        self.vel[1] = v_y
        self.vel_x = v_x
        self.vel_y = v_y
    ### ---end set_vel---

    def d_pos(self):
        return (math.floor(self.pos[0]), math.floor(self.pos[1]))
    ### ---end d_pos---

    def p_pos(self):
        return (math.floor(self.pred_pos[0]), math.floor(self.pred_pos[1]))
    ### ---end p_pos---

    def M_pos(self):
        return (math.floor(self.pos[1]), math.floor(self.pos[0]))
    ### ---end M_pos---

    def update_pos(self):
        #print("compare:", self.pred_pos, self.pos)
        if not self.isCollide:
            self.vel = (self.pred_pos - self.pos)#/self.dt
            self.pos = self.pred_pos
            self.pos_x = self.pos[0]
            self.pos_y = self.pos[1]

        else:
            print("we are here!", self.name)
            self.pos = self.pred_pos
            self.pos_x = self.pos[0]
            self.pos_y = self.pos[1]
            self.isCollide = False

        #if self.isCollide:
        #    self.vel = self.vel * self.mu_fraction
        #    self.isCollide = False
        #if norm2(self.vel) < 0.1:
        #    self.vel[0] = 0
        #    self.vel[1] = 0
        #    self.isMoving =False
        #    xx = input("what is happening?")


    ### ---end update_pos---


    def print_particle(self):
        print("############################")
        print(self.name)
        print("the position is", self.pos)
        print("the velocity is:", self.vel)
        print("the collision status is:", self.isCollide)
        print("the mass is", self.mass)
        print("the mu fraction is:", self.mu_fraction)
        print("############################")




### ---end class Partcile---
