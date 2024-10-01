import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class GUIDEPacket:
    def __init__(self, gpsn, hc):
        self.gpsn = gpsn
        self.hc = hc

class SensorNode:
    def __init__(self, node_id, position):
        self.node_id = node_id
        self.position = position
        self.gpsn = 0
        self.hc = float('inf')
        self.neighbors = []

    def broadcast_packet(self, packet):
        updated_nodes = []
        for neighbor in self.neighbors:
            if packet.gpsn > neighbor.gpsn:
                neighbor.gpsn = packet.gpsn
                neighbor.hc = packet.hc + 1
                updated_nodes.append(neighbor)
            elif packet.gpsn == neighbor.gpsn and packet.hc + 1 < neighbor.hc:
                neighbor.hc = packet.hc + 1
                updated_nodes.append(neighbor)
        return updated_nodes

class SinkNode(SensorNode):
    def __init__(self, node_id, position):
        super().__init__(node_id, position)
        self.gpsn = 1
        self.hc = 0

def calculate_neighbors(nodes, sinks, communication_radius):
    all_nodes = nodes + sinks
    for node in all_nodes:
        node.neighbors = [n for n in all_nodes if np.linalg.norm(node.position - n.position) <= communication_radius and n != node]

def initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers):
    nodes = []
    sinks = []

    nodes_per_layer = num_nodes // depth_layers
    for layer in range(depth_layers):
        for i in range(nodes_per_layer):
            position = np.random.rand(3) * space_dim
            depth_start = -layer * (max_depth / depth_layers)
            depth_end = -(layer + 1) * (max_depth / depth_layers)
            position[2] = depth_start - np.random.rand() * (depth_start - depth_end)  # Depth (Z-axis) within the layer
            nodes.append(SensorNode(f'node_{layer}_{i}', position))

    # Add any remaining nodes to random layers
    remaining_nodes = num_nodes % depth_layers
    for i in range(remaining_nodes):
        position = np.random.rand(3) * space_dim
        layer = np.random.randint(0, depth_layers)
        depth_start = -layer * (max_depth / depth_layers)
        depth_end = -(layer + 1) * (max_depth / depth_layers)
        position[2] = depth_start - np.random.rand() * (depth_start - depth_end)  # Depth (Z-axis) within the layer
        nodes.append(SensorNode(f'node_remaining_{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[2] = 0  # Sink nodes are on the surface
        sinks.append(SinkNode(f'sink_{i}', position))

    return nodes, sinks

def simulate_guiding_network(num_nodes, num_sinks, space_dim, max_depth, depth_layers, communication_radius):
    nodes, sinks = initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers)
    calculate_neighbors(nodes, sinks, communication_radius)

    for sink in sinks:
        packet = GUIDEPacket(sink.gpsn, sink.hc)
        updated_nodes = sink.broadcast_packet(packet)

        while updated_nodes:
            next_updated_nodes = []
            for node in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(GUIDEPacket(node.gpsn, node.hc)))
            updated_nodes = next_updated_nodes

    return nodes, sinks

def visualize_network(nodes, sinks):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')
        for neighbor in node.neighbors:
            ax.plot([node.position[0], neighbor.position[0]], [node.position[1], neighbor.position[1]], [node.position[2], neighbor.position[2]], 'k-', lw=0.5)

    for sink in sinks:
        ax.scatter(sink.position[0], sink.position[1], sink.position[2], c='r', marker='^')
        ax.text(sink.position[0], sink.position[1], sink.position[2], '0', color='red')  # Sink hop count is always 0

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# Parameters
NUM_NODES = 20
NUM_SINKS = 2
SPACE_DIM = 60  # Determines the size of the 3D space (e.g., 100x100x100 cube)
MAX_DEPTH = 60 # Maximum depth of the sea
DEPTH_LAYERS = 3  # Number of depth layers
COMMUNICATION_RADIUS = 25

nodes, sinks = simulate_guiding_network(NUM_NODES, NUM_SINKS, SPACE_DIM, MAX_DEPTH, DEPTH_LAYERS, COMMUNICATION_RADIUS)
visualize_network(nodes, sinks)
