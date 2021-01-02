from tqdm import tqdm
import os.path
import random
import math

min_particles, max_particles = 10, 1000

def get_dist(x, y):
    return math.sqrt(x**2 + y**2)

def get_nbrs(x, y):
    return [[x+1, y], [x, y+1], 
            [x-1, y], [x, y-1], 
            [x+1, y-1], [x-1, y+1]]

if __name__ == '__main__':
    for i in tqdm(range(0, 10)):
        filename = f"ellipse_{i}.txt"
        if os.path.isfile(filename):
            continue
        size = random.randint(min_particles, max_particles)
        particles = [[0, 0]]

        while len(particles) < size:
            min_dist = get_dist(max_particles, max_particles)
            p_min = None
            for p in particles:
                if get_dist(p[0], p[1]) < min_dist:
                    nbrs = get_nbrs(p[0], p[1])
                    if any(not nbr in particles for nbr in nbrs):
                        min_dist = get_dist(p[0], p[1])
                        p_min = p
            nbrs = get_nbrs(p_min[0], p_min[1])
            for nbr in nbrs:
                if not nbr in particles:
                    particles.append(nbr)
                    break
        
        file = open(filename, "w")
        for p in particles:
            file.write(f"{p[0]},{p[1]}\n")
        file.close()