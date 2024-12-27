import random
import numpy as np

random.seed(42)
np.random.seed(42)

NINF = float('-inf')

#Định nghĩa các cảm biến sensor, sink và source
class Node:
    def __init__(self, node_id, position, is_sink = False, is_source=False):
        self.node_id = node_id
        self.position = position
        self.hc = NINF
        self.gpsn = 0
        self.neighbors = []
        self.is_sink = is_sink
        self.is_source = is_source
        self.energy = random.randint(25, 30)

    def __repr__(self):
        return f"Node(ID={self.node_id}, Position={self.position}, Hop-count={self.hc}, IsSink={self.is_sink}, IsSource={self.is_source})"

    def update_with_packet(self, packet):
        if packet.gpsn > self.gpsn:
            self.gpsn = packet.gpsn
            self.hc = packet.hc + 1
            return True
        elif packet.gpsn == self.gpsn and self.hc > packet.hc + 1:
            self.hc = packet.hc + 1
            return True
        return False

    # broadcast guide packet for neighbor nodes
    def broadcast_packet(self, packet):
        updated_nodes = []

        for neighbor in self.neighbors:
            if neighbor.update_with_packet(packet):
                new_packet = GuidePacket(packet.gpsn, neighbor.hc)
                updated_nodes.append((neighbor, new_packet))
        return updated_nodes

class GuidePacket:
    def __init__(self, gpsn, hc):
        self.gpsn = gpsn
        self.hc = hc

def calculate_neighbors(nodes, sinks, sources, communication_radius):
    all_nodes = nodes + sinks + sources
    for node in all_nodes:
        node.neighbors = [n for n in all_nodes if
                          np.linalg.norm(node.position - n.position) <= communication_radius and n != node]


'''
#Hàm sinh tọa độ ngẫu nhiên
def generate_random_coordinates(x_range, y_range, z_range):
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    z = random.uniform(*z_range)

    return x, y, z

#Hàm tạo node
def create_node(num, label, x_range, y_range, z_range, is_sink = False, is_source = False):
    return [Node(node_id=f'{label}{i+1}', position=generate_random_coordinates(x_range, y_range, z_range),
                 is_sink = is_sink, is_source=is_source) for i in range(num)]

#Hàm khởi tạo các node
def initialize_node(num_node, num_sink, num_source, space_dim, depth_range):
    sinks = create_node(num_sink, "Si", (0, space_dim), (0, space_dim),
                        (0,0), is_sink=True)
    sources = create_node(num_source, "So", (0, space_dim), (0, space_dim),
                          (-space_dim, -depth_range), is_source=True)

    nodes = create_node(num_node, "N", (0, space_dim), (0, space_dim),
                        (-space_dim, 0))
    return sinks, sources, nodes
'''

def initialize_nodes(num_nodes, num_sinks, num_source_nodes, space_dim, depth_range):
    nodes = []
    sinks = []
    sources = []

    for i in range(num_nodes):
        position = np.random.rand(3) * space_dim
        position[2] = np.random.uniform(0, space_dim) * -1
        nodes.append(Node(f'N{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[2] = 0
        sink = Node(f'Si{i}', position, is_sink=True)
        sink.hc = 0
        sink.gpsn = 1
        sinks.append(sink)

    for i in range(num_source_nodes):
        position = np.random.rand(3) * space_dim
        position[2] = np.random.uniform(depth_range[0], depth_range[1]) * -1
        source = Node(f'So{i}', position, is_source=True)
        sources.append(source)

    return nodes, sinks, sources

#Hàm tính khoảng cách giữa 2 node
def distance(node1, node2):
    return np.linalg.norm(np.array(node1.position) - np.array(node2.position))




