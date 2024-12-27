import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from GA_EEGNBR.config.config_parser import parser
from GA_EEGNBR.src.genetic_algorithm import genetic_algorithm
from GA_EEGNBR.src.sensor import initialize_nodes, calculate_neighbors
from GA_EEGNBR.utils.init_population import initialize_population, setup_network
from GA_EEGNBR.src.sensor import distance
from GA_EEGNBR.utils.plot import plot_best_path, plot_nodes

space_dim = int(parser['Sensor']['space_dim'])
depth_range = int(parser['Sensor']['depth_range'])
num_sinks = int(parser['Sensor']['num_sinks'])
num_sources = int(parser['Sensor']['num_sources']) 
num_nodes = int(parser['Sensor']['num_nodes'])
radius = int(parser['Constraints']['radius'])
energy_threshold = float(parser['Sensor']['energy_threshold'])

num_generations = int(parser['Genetic Algorithm']['num_generations'])
population_size = int(parser['Genetic Algorithm']['population_size'])
mutation_rate = float(parser['Genetic Algorithm']['mutation_rate'])

nodes, sinks, sources= initialize_nodes(num_nodes, num_sinks, num_sources, space_dim, [depth_range, space_dim])
all_nodes = sinks + sources + nodes
calculate_neighbors(nodes, sinks, sources, radius)
paths = setup_network(sinks)

for node in all_nodes:
    print(node)

populations = initialize_population(all_nodes, sources, sinks, radius, population_size)
for population in populations:
    for i in range(1, 101):
        best_path = genetic_algorithm(population, all_nodes, sinks, sources, num_generations, population_size, mutation_rate, radius)
        total_distance = sum(distance(best_path[i], best_path[i+1]) for i in range(len(best_path) - 1))
        total_energy = sum(best_path[i].energy for i in range(len(best_path)))
        print(f"Best path: {[node.node_id for node in best_path]}")
        print(f"Total distance of best path: {total_distance}")
        print(f'Total energy remain: {total_energy}')

        print("\nDistances between nodes in best path:")
        for j in range(len(best_path) - 1):
            d = distance(best_path[j], best_path[j + 1])
            print(f"  Node {best_path[j].node_id} -> Node {best_path[j + 1].node_id}: {d:.2f}")

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        plot_nodes(ax, all_nodes)
        plot_best_path(ax, best_path)

        plt.show()