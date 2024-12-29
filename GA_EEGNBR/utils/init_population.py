import random
import numpy as np

from GA_EEGNBR.src.sensor import distance, GuidePacket
from GA_EEGNBR.utils.plot import plot_best_path, plot_nodes
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

random.seed(42)
np.random.seed(42)

#Hàm kiểm tra xem đường đi có thỏa mãn không
def is_valid_path(path, sinks, radius, energy_threshold=0.5):
    if path is None or path[-1] not in sinks:
        return False

    for i in range(len(path)-1):
        if distance(path[i], path[i+1]) > radius:
            return False
    for i in range(len(path)):
        if path[i].energy > energy_threshold:
            return True

    return True

#Hàm sắp xếp các node sink theo thức tự gần node nguồn nhất
def sorted_sinks_by_distance(source_node, sinks):
    sorted_sinks = sorted(sinks, key=lambda sink: distance(source_node, sink))
    return sorted_sinks

def setup_network(sinks, guide_packet_size):
    paths = []
    for sink in sinks:
        packet = GuidePacket(sink.gpsn, sink.hc)
        updated_nodes = sink.broadcast_packet(packet, guide_packet_size)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet, guide_packet_size))
                for neighbor in node.neighbors:
                    if node.hc != neighbor.hc:
                        paths.append((node, neighbor))  # Lưu lại các đường đi
                updated_nodes = next_updated_nodes

    return paths

#Hàm khởi tạo đường ngẫu nhiên cho quần thể
def create_random_path(all_nodes, source_node, sources, sinks, closet_sinks):
    #for _ in range(max_attempts):
    path = [source_node]
    current_node = source_node
    while current_node not in sinks:
        next_node = [node for node in current_node.neighbors if node not in path and (node.hc == current_node.hc - 1)]
        if not next_node:
            break

        sink_in_next_node = [node for node in next_node if node in sinks]
        if sink_in_next_node:
            current_node = random.choice(sink_in_next_node)
        else:
            current_node = random.choice(next_node)

        path.append(current_node)

    if current_node in sinks:
            return path

    #return random.choice(all_nodes)

#Hàm khởi tạo quần thể ban đầu
def initialize_population(all_nodes, sources, sinks, radius, population_size):
    populations = []
    for source_node in sources:
        population = []
        sorted_sinks = sorted_sinks_by_distance(source_node, sinks)
        closet_sinks = sorted_sinks[:2]
        for i in range(population_size):
            while True:
                path = create_random_path(all_nodes, source_node, sources, sinks, closet_sinks)
                if path not in population:
                    population.append(path)
                    print(f'Path{i}: {[node.node_id for node in path]}')
                    total_distance = sum(distance(path[i], path[i + 1]) for i in range(len(path) - 1))
                    print(f'Total distance: {total_distance}')
                    break
        populations.append(population)

    return populations