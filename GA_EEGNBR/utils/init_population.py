import random
from GA_EEGNBR.src.sensor import distance

#Hàm kiểm tra xem đường đi có thỏa mãn không
def is_valid_path(path, sinks):
    if path is None:
        return False
    return path[-1] in sinks

#Hàm sắp xếp các node sink theo thức tự gần node nguồn nhất
def sorted_sinks_by_distance(source_node, sinks):
    sorted_sinks = sorted(sinks, key=lambda sink: distance(source_node, sink))

#Hàm khởi tạo đường ngẫu nhiên cho quần thể
def create_random_path(all_nodes, source_node, sources, sinks, radius, max_attempts = 100):
    for _ in range(max_attempts):
        path = []
        current_node = source_node
        while current_node not in sinks:
            next_node = [node for node in all_nodes if node not in path and node not in sources and distance(current_node, node) < radius]
            if not next_node:
                break

            sink_in_next_node = [node for node in next_node if node in sinks]
            if sink_in_next_node:
                current_node = sink_in_next_node
            else:
                current_node = random.choice(next_node)

            path.append(current_node)

        if current_node in sinks:
            return path

    return random.choice(all_nodes)

#Hàm khỏi tạo quần thể ban đầu
def initialize_population(all_nodes, sources, sinks, radius, population_size):
    population = []
    for source_node in sources:
        sorted_sinks = sorted_sinks_by_distance(source_node, sinks)
        closet_sinks = sorted_sinks[:2]
        for _ in range(population_size):
            while True:
                path = create_random_path(all_nodes, source_node, sources, closet_sinks, radius)
                if is_valid_path(path, sinks) and path not in population:
                    population.append(path)
                    break

    return population