from Particle import *
import cv2

class Circle(Particle):
    def __init__(self, x, y, v_x, v_y, r):
        Particle.__init__(self, x, y, v_x, v_y, 1, color=(0, 0, 255))
        self.r = r
    ### ---end __init__---

    def draw_circle(self, frame):
        frame = cv2.circle(frame, self.d_pos(), self.r, self.color, -1)
