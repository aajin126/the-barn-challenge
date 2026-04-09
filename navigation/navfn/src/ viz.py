import numpy as np
import matplotlib.pyplot as plt
import time

# Constants
POT_HIGH = float('inf')

def load_data(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        ns, nx, ny = map(int, lines[0].strip().split())
        potentials = []
        queues = []

        for i in range(1, len(lines), 2):
            potentials.append(np.array(list(map(float, lines[i].strip().split()))).reshape((ny, nx)))
            queues.append(list(map(int, lines[i + 1].strip().split())))

    return nx, ny, potentials, queues

def visualize_map(nx, ny, potentials, queues, interval=0.5):
    plt.figure(figsize=(8, 8))

    for potential, queue in zip(potentials, queues):
        plt.imshow(potential, cmap='hot', origin='upper', extent=(0, nx, 0, ny))
        for q in queue:
            x, y = q % nx, q // nx
            plt.scatter(x, y, color='blue', s=10)  # Queue points

        plt.pause(interval)
        plt.clf()

    plt.show()

if __name__ == "__main__": 
    file_path = "astar_data.txt"
    nx, ny, potentials, queues = load_data(file_path)
    visualize_map(nx, ny, potentials, queues)