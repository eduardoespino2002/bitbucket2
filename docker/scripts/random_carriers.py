

import random
import numpy as np


xmin = -1.2
xmax = 0.0
ymin = -1.2
ymax = 2.3

min_dist = 0.45


class Collision(Exception):
    pass

def run(n):

    locations = np.zeros((n, 3))
    for trials in range(1000):
        for i in range(n):
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            th = random.uniform(-np.pi, np.pi)
            locations[i,:] = (x, y, th)
        
        try:
            for i in range(n - 1):
                for j in range(i + 1, n):
                    dist = np.linalg.norm(locations[i, 0:2] - locations[j, 0:2])
                    if dist < min_dist:
                        #print(locations[i, 0:2], locations[j, 0:2], dist)
                        raise Collision
        except Collision:
            #print('found collision')
            continue
        else:
            print('found solution')
            for i in range(n):
                print('    <include><uri>model://carrier1%02d</uri><name>carrier1%02d</name><pose>%f %f 0 0 0 %f</pose></include>' % 
                            (i, i, locations[i, 0], locations[i, 1], locations[i, 2]))
            break






if __name__ == '__main__':
    run(10)