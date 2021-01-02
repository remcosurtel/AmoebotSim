from tqdm import tqdm
import os.path
import random
import math

min_particles, max_particles = 10, 1000

def get_nbrs(x, y):
    return [[x+1, y], [x, y+1], 
            [x-1, y], [x, y-1], 
            [x+1, y-1], [x-1, y+1]]

if __name__ == '__main__':
    for i in tqdm(range(0, 10)):
        filename = f"tree_{i}.txt"
        if os.path.isfile(filename):
            continue
        size = random.randint(min_particles, max_particles)
        particles = [[0, 0]]

        while len(particles) < size:
            p = random.choice(particles)
            nbrs = get_nbrs(p[0], p[1])
            for nbr in nbrs:
                if not nbr in particles:
                    nbr_nbrs = get_nbrs(nbr[0], nbr[1])
                    nbr_nbrs.remove(p)
                    legal = True
                    for nbr_nbr in nbr_nbrs:
                        if nbr_nbr in particles:
                            if not nbr_nbr in nbrs:
                                legal = False
                                break
                    if legal:
                        particles.append(nbr)
                        break
        
        file = open(filename, "w")
        for p in particles:
            file.write(f"{p[0]},{p[1]}\n")
        file.close()