from tqdm import tqdm
import os.path
import random
import math
import copy

min_particles, max_particles = 10, 1000

def get_nbrs(x, y):
    return [[x+1, y], [x, y+1], 
            [x-1, y], [x, y-1], 
            [x+1, y-1], [x-1, y+1]]

def is_connected(p, q, particles):
    if p == q:
        return True
    connected = False
    for nbr in get_nbrs(p[0], p[1]):
        if nbr in particles:
            particles_minus_p = copy.deepcopy(particles)
            particles_minus_p.remove(p)
            if is_connected(nbr, q, particles_minus_p):
                connected = True
                break
    return connected

if __name__ == '__main__':
    for i in tqdm(range(0, 100)):
        filename = f"random_{i}.txt"
        if os.path.isfile(filename):
            continue
        size = min_particles + i * 10
        particles = [[0, 0]]

        while len(particles) < size:
            p = random.choice(particles)
            nbrs = get_nbrs(p[0], p[1])
            for nbr in nbrs:
                if not nbr in particles:
                    nbr_nbrs_minus_p = get_nbrs(nbr[0], nbr[1])
                    for x in nbr_nbrs_minus_p:
                        if not x in particles:
                            nbr_nbrs_minus_p.remove(x)
                    nbr_nbrs = copy.deepcopy(nbr_nbrs_minus_p)
                    nbr_nbrs.append(p)
                    legal = True
                    for nbr_nbr in nbr_nbrs_minus_p:
                        if nbr_nbr in particles:
                            if not nbr_nbr in nbrs:
                                if not is_connected(p, nbr_nbr, nbr_nbrs):
                                    legal = False
                                    break
                    if legal:
                        particles.append(nbr)
                        break
        
        file = open(filename, "w")
        for p in particles:
            file.write(f"{p[0]},{p[1]}\n")
        file.close()