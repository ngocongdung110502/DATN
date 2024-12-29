import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from GA_EEGNBR.src.sensor import distance
from GA_EEGNBR.utils.plot import plot_nodes
from GA_EEGNBR.utils.plot import plot_best_path_generation
from GA_EEGNBR.utils.init_population import is_valid_path

#Hàm thuật toán di truyền
def genetic_algorithm(population, all_nodes, sinks, sources, num_generation, population_size, mutation_rate, radius):
    best_paths = []

    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.ion()
    plot_nodes(ax, all_nodes)
    '''

    for generation in range(num_generation):
        population = sorted(population, key = fitness, reverse=True)
        best_path = population[0]
        best_paths.append(best_path)
        total_distance = sum(distance(best_path[i], best_path[i+1]) for i in range(len(best_path) - 1))
        #print(f"Generation {generation + 1}: Best Path Distance = {total_distance}")

        '''
        ax.clear()
        plot_nodes(ax, all_nodes)
        plot_best_path_generation(ax, best_path, generation)
        plt.pause(0.1)
        '''

        new_population = population[:int(0.3 * population_size)]
        while len(new_population) < population_size:
            parent1, parent2 = random.choice(population[:15]), random.choice(population[:15])

            #child1, child2 = two_point_crossover(parent1, parent2)
            child1, child2 = crossover(parent1, parent2)

            if is_valid_path(child1, sinks, radius):
                child1 = mutate(child1, mutation_rate, all_nodes, sources, sinks, radius)
                if child1:
                    new_population.append(child1)

            if is_valid_path(child2, sinks, radius):
                child2 = mutate(child2, mutation_rate, all_nodes, sources, sinks, radius)
                if child2:
                    new_population.append(child2)

        population = new_population

        '''
        new_population = []
        for i in range(population_size//2):
            parent1, parent2 = random.choice(population[:50]), random.choice(population[:50])
            #parent1, parent2 = random.sample(population, 2)
            while True:
                child1, child2 = crossover(parent1, parent2)
                if is_valid_path(child1, sinks, radius) and is_valid_path(child2, sinks, radius):
                    new_population.append(child1)
                    new_population.append(child2)
                    break

        muta = random.sample(new_population, int(mutation_rate*population_size))
        for i in muta:
            mutate(i, mutation_rate, all_nodes, sources, sinks, radius)

        population = population + new_population
        population = sorted(population, key = fitness, reverse=True)
        _population_size = 2 * population_size
        t2 = int(population_size * 2 / 5)
        t3 = int(population_size / 5)
        t1 = population_size - t2 - t3
        population = population[:t1] + population[_population_size//3:_population_size//3+t2] + population[-t3:]
        '''
    #plt.ioff()
    #plt.show()

    best_path = sorted(population, key=fitness, reverse=True)[0]
    return best_path

#Hàm đánh giá độ thích nghi
def fitness(path, energy_threshold = 0.5):
    if path is None:
        return 0
    total_energy_residual = sum(node.energy for node in path)
    return (total_energy_residual-energy_threshold)/sum(distance(path[i], path[i+1]) for i in range(len(path) - 1))

#Hàm loại bỏ các node trùng lặp trong đường đi
def remove_duplicates(path):
    seen = set()
    new_path = []
    for node in path:
        if node not in seen:
            new_path.append(node)
            seen.add(node)
    return new_path

#Hàm lai ghép
def crossover1(path1, path2):
    if path1 is None or path2 is None:
        print(f"Parent is None")

    common_nodes = set(path1[1:-1]) & set(path2[1:-1])
    if not common_nodes:
        print("No common node found, performing normal crossover.")
        # Nếu không có node chung, quay lại phương pháp lai ghép bình thường
        split_point = random.randint(1, min(len(path1), len(path2)) - 2)
        child1 = path1[:split_point] + path2[split_point:]
        child2 = path2[:split_point] + path1[split_point:]
        return remove_duplicates(child1), remove_duplicates(child2)

    common_node = random.choice(list(common_nodes))

    split_point1 = path1.index(common_node)
    split_point2 = path2.index(common_node)

    # Lai ghép tại điểm chung
    child1 = path1[:split_point1 + 1] + path2[split_point2 + 1:]
    child2 = path2[:split_point2 + 1] + path1[split_point1 + 1:]

    # Loại bỏ các node trùng lặp (nếu có)
    return remove_duplicates(child1), remove_duplicates(child2)

def crossover(path1, path2):
    split_point = random.randint(1, min(len(path1), len(path2)) - 2)
    child1 = path1[:split_point] + path2[split_point:]
    child2 = path2[:split_point] + path1[split_point:]
    return remove_duplicates(child1), remove_duplicates(child2)

#Hàm lại ghép cắt 2 điểm
def two_point_crossover(parent1, parent2):
    if parent1 is None or parent2 is None:
        return None, None

    size = min(len(parent1), len(parent2))
    point1 = random.randint(1, size - 3)
    point2 = random.randint(point1 + 1, size - 2)
    child1 = parent1[:point1] + parent2[point1:point2] + parent1[point2:]
    child2 = parent2[:point1] + parent1[point1:point2] + parent2[point2:]

    return remove_duplicates(child1), remove_duplicates(child2)

#Hàm đột biến
def mutate(path, mutation_rate, all_nodes, sources, sinks, radius):
    index = random.randint(1, len(path) - 1)
    current_node = path[index]
    previous_node = path[index-1]
    #Cần kiểm tra xem trong các node đột biến có khoảng các với path[index+1] có thỏa mãn ràng buộc không nữa
    #valid_nodes = [node for node in all_nodes if node not in path and node not in sources and distance(path[index-1], node) <= radius and distance(path[index+1], node) <= radius]
    valid_nodes = [node for node in previous_node.neighbors if node != current_node and node.hc == current_node.hc]
    # new_node = random.choice([node for node in all_nodes if node not in path and node not in sources and distance(path[index-1], node) < radius ])
    if random.random() < mutation_rate:
        for new_node in valid_nodes:
            path[index] = new_node
            if is_valid_path(path, sinks, radius):
                return path
    return None


