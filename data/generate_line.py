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
    for i in tqdm(range(0, 21)):
        filename = f"line_{i}.txt"
        if os.path.isfile(filename):
            continue
        size = min_particles if i == 0 else i * 25
        particles = [[0, 0]]

        while len(particles) < size:
            p = particles[len(particles)-1]
            particles.append([p[0]+1, p[1]])
        
        file = open(filename, "w")
        for p in particles:
            file.write(f"{p[0]},{p[1]}\n")
        file.close()