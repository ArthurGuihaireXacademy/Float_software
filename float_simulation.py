from depth_hold import PID_controller
from graph_data import graph_data
import numpy as np
import matplotlib.pyplot as plt

class DepthController():
    def __init__(self):
        pass
    def update_depth_hold(self, a=None, b=None):
        if velocity > 0.05:
            return 1
        if velocity < -0.05:
            return -1
        if depth < 2:
            return 1
        if depth > 3:
            return -1
        if velocity > 0.02:
            return 1
        if velocity < -0.02:
            return -1
        if depth > 2.5 and velocity < 0:
            return -1
        if depth < 2.5 and velocity > 0:
            return 1
        return 0

if __name__ == "__main__":
    dt = 0.1
    FRICTION = 1 - 0.5 * dt
    MASS = 5.0

    depth = 0.0
    velocity = 0.0
    buoyancy = 0.1
    #controller = DepthController()
    #pid = PID_controller(15, 0, -100, dt)
    pid = PID_controller(6, 0, -40, dt)
    pid.start_depth_hold(depth, 3.5)
    threshold = 0.05
    iterations = int(60/dt)

    def physics(pid):
        global depth, velocity, buoyancy
        action = pid.update_depth_hold(depth)
        #print(action)
        if action > threshold:
            buoyancy -= 0.1 * dt
        elif action < -threshold:
            buoyancy += 0.1 * dt
        buoyancy = max(-0.4, min(0.4, buoyancy))
        velocity += dt * buoyancy / MASS
        velocity *= FRICTION
        depth -= velocity
        if depth < 0:
            depth = 0.0
            velocity = 0.0

    time_array = np.empty(iterations+1)
    time_array[0] = 0
    depth_array = np.empty(iterations+1)
    depth_array[0] = 0

    for i in range(iterations):
        physics(pid)
        time_array[i+1] = i * dt
        depth_array[i+1] = depth
    
    '''scores = np.zeros((40, 20), dtype=int)

    for Kp in range(1, 41):
        iteration = 0
        for Kd in range(-0, -200, -10):
            depth = 0.0
            velocity = 0.0
            buoyancy = 0.1
            pid = PID_controller(Kp, 0, Kd, dt)
            pid.start_depth_hold(0, 2.5)
            #print(iterations)
            for i in range(iterations):
                physics(pid)
                #time_array[i+1] = i * dt
                #depth_array[i+1] = depth
                if 2 < depth < 3:
                    scores[Kp-1][iteration] += 1
            iteration += 1

    for array in scores:
        for value in array:
            print(value, end=" ")
        print()'''
    graph_data(time_array, depth_array)