import numpy as np
import math

class Environment:
    def __init__(self, x, y):
        self.size_x = x
        self.size_y = y
        self.world_bounds = np.zeros(shape=(y, x), dtype=int)
        self.border = np.zeros(shape=(y, x), dtype=int)
        self.world_N = np.ndarray(shape=(y, x), dtype=np.ndarray)
        # self.world_N_Y = np.zeros(shape=(y, x), dtype=int)
    ### ---end __init__---###

    def set_bounded_box(self):
        self.world_bounds[0, :] = 1
        self.world_bounds[-1, :] = 1
        self.world_bounds[:, 0] = 1
        self.world_bounds[:, -1] = 1
        ######
        self.border[0, :] = 1
        self.border[-1, :] = 1
        self.border[:, 0] = 1
        self.border[:, -1] = 1
        ######
        self.border[1, :] = 1
        self.border[-2, :] = 1
        self.border[:, 1] = 1
        self.border[:, -2] = 1
        ######
        self.world_N.fill(np.zeros(shape=(1, 2)))

        for i in range(self.size_y):
            self.world_N[i, 0] = np.array([1, 0])
            self.world_N[i, -1] = np.array([-1, 0])
        ### ---end for---

        for i in range(self.size_x):
            self.world_N[0, i] = np.array([0, 1])
            self.world_N[-1, i] = np.array([0, -1])
    ### ---end set_bounded_box---

### --- end class Environment