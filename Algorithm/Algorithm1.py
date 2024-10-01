import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
NINF = float('inf')

#Dinh nghia goi GUIDE gom gpsn and hc
class GUIDEPacket:
    def __init__(self, gpsn, hc):
        self.gpsn = gpsn
        self.hc = hc

# Dinh nghia nut trong mang gom Sensor and Sink
class SensorNode:
    def __init__(self, id, position, is_sink=False):
        self.id = id
        self.position = position
        self.gpsn = 0
        self.hc = NINF
        self.neighbors = []
        self.is_sink = is_sink

    def update_with_packet(self, packet): # Đinh nghia ham cap nhat gpsn va hc bang packet
        if packet.gpsn > self.gpsn:
            self.gpsn = packet.gpsn
            self.hc = packet.hc + 1
            #packet.hc += 1
            return True
        elif packet.gpsn == self.gpsn and self.hc > packet.hc + 1:
            self.hc = packet.hc + 1
            #packet.hc += 1
            return True
        return False

    def broadcast_packet(self, packet): # Ham truyen packet cho cac nut lan can
        updated_nodes = []
        for neighbor in self.neighbors:
            if neighbor.update_with_packet(packet):
                new_packet = GUIDEPacket(packet.gpsn, neighbor.hc)
                updated_nodes.append((neighbor, new_packet))
        return updated_nodes

# Khoi tao cac nut sensor va sinks
def initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers, xy_limit):
    nodes = []
    sinks = []

    nodes_per_layer = num_nodes // depth_layers

    for layer in range(depth_layers):
        for i in range(nodes_per_layer):
            position = np.random.rand(3) * space_dim
            position[0] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn tọa độ x
            position[1] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn tọa độ y
            depth_start = -layer * (max_depth / depth_layers)
            depth_end = -(layer + 1) * (max_depth / depth_layers)
            position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
            nodes.append(SensorNode(f'node_{layer}_{i}', position))

    remaining_nodes = num_nodes % depth_layers
    for i in range(remaining_nodes):
        position = np.random.rand(3) * space_dim
        position[0] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn tọa độ x
        position[1] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn tọa độ y
        layer = np.random.randint(0, depth_layers)
        depth_start = -layer * (max_depth / depth_layers)
        depth_end = -(layer + 1) * (max_depth / depth_layers)
        position[2] = depth_start - np.random.rand() * (depth_start - depth_end)  
        nodes.append(SensorNode(f'node_remaining_{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[0] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn tọa độ x
        position[1] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn tọa độ y
        position[2] = 0
        sink = SensorNode(f'sink_{i}', position, is_sink=True)
        sink.gpsn = 1
        sink.hc = 0
        sinks.append(sink)

    return nodes, sinks

#Tinh toan khoacg cach giua cac node va tim cac nut lan can (neighbors)
def calculate_neighbors(nodes, sinks, communication_radius):
    all_nodes = nodes + sinks
    for node in all_nodes:
        node.neighbors = [n for n in all_nodes if np.linalg.norm(node.position - n.position) <= communication_radius and n != node]

#Ham xay dung mang huong dan
def simulate_guiding_network(num_nodes, num_sinks, space_dim, max_depth, depth_layers, communication_radius, xy_limit):
    nodes, sinks = initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers, xy_limit)
    calculate_neighbors(nodes, sinks, communication_radius)

    for sink in sinks:
        packet = GUIDEPacket(sink.gpsn, sink.hc)
        updated_nodes = sink.broadcast_packet(packet)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet))
            updated_nodes = next_updated_nodes

    return nodes, sinks

#Hien thi mang huong dan trong khong gian 3D
def visualize_network(nodes, sinks):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')
        for neighbor in node.neighbors:
            if node.hc != neighbor.hc:
                ax.plot([node.position[0], neighbor.position[0]], [node.position[1], neighbor.position[1]], [node.position[2], neighbor.position[2]], 'g-', lw=0.5)

    for sink in sinks:
        ax.scatter(sink.position[0], sink.position[1], sink.position[2], c='r', marker='s')
        ax.text(sink.position[0], sink.position[1], sink.position[2], '0', color='red')  # Sink hop count is always 0

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# Parameters
NUM_NODES = 40
NUM_SINKS = 5
SPACE_DIM = 100
MAX_DEPTH = 100  
DEPTH_LAYERS = 10
COMMUNICATION_RADIUS = 19
XY_LIMIT = 20

nodes, sinks = simulate_guiding_network(NUM_NODES, NUM_SINKS, SPACE_DIM, MAX_DEPTH, DEPTH_LAYERS, COMMUNICATION_RADIUS, XY_LIMIT)
visualize_network(nodes, sinks)
