from GA_EEGNBR.config.config_parser import parser
from GA_EEGNBR.src.genetic_algorithm import genetic_algorithm
from GA_EEGNBR.src.sensor import initialize_node
from GA_EEGNBR.utils.init_population import initialize_population
from GA_EEGNBR.src.sensor import distance

space_dim = int(parser['Sensor']['space_dim'])
depth_range = int(parser['Sensor']['depth_range'])
num_sinks = int(parser['Sensor']['num_sinks'])
num_sources = int(parser['Sensor']['num_sources']) 
num_nodes = int(parser['Sensor']['num_nodes'])
radius = int(parser['Constraints']['radius'])

num_generations = int(parser['Genetic Algorithm']['num_generations'])
population_size = int(parser['Genetic Algorithm']['population_size'])
mutation_rate = float(parser['Genetic Algorithm']['mutation_rate'])

sinks, sources, nodes = initialize_node(num_nodes, num_sinks, num_sources, space_dim, depth_range)
all_nodes = sinks + sources + nodes

for node in all_nodes:
    print(node)

population = initialize_population(all_nodes, sources, sinks, radius, population_size)
best_path = genetic_algorithm(population, all_nodes, sinks, sources, num_generations, population_size, mutation_rate, radius)

total_distance = sum(distance(best_path[i], best_path[i+1]) for i in range(len(best_path) - 1))
print(f"Best path: {[node.node_id for node in best_path]}")
print(f"Total distance of best path: {total_distance}")