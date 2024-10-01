import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
NINF = float('inf')

class GUIDEPacket:
    def __init__(self, gpsn, hc):
        self.gpsn = gpsn
        self.hc = hc

class QueryPacket:
    def __init__(self, sender_id, source_id, dpsn, hc):
        self.sender_id = sender_id
        self.source_id = source_id
        self.dpsn = dpsn
        self.hc = hc

class AckPacket:
    def __init__(self, receiver_id, sender_id, energy, hc):
        self.receiver_id = receiver_id
        self.sender_id = sender_id
        self.energy = energy
        self.hc = hc

class SensorNode:
    def __init__(self, id, position, is_sink=False):
        self.id = id
        self.position = position
        self.gpsn = 0
        self.hc = NINF
        self.neighbors = []
        self.is_sink = is_sink
        self.memory_queue = set()
        self.energy = np.random.uniform(0.5, 1.0)  # Random initial energy

    def update_with_packet(self, packet):
        if packet.gpsn > self.gpsn:
            self.gpsn = packet.gpsn
            self.hc = packet.hc + 1
            return True
        elif packet.gpsn == self.gpsn and self.hc > packet.hc + 1:
            self.hc = packet.hc + 1
            return True
        return False

    def broadcast_packet(self, packet):
        updated_nodes = []
        for neighbor in self.neighbors:
            if neighbor.update_with_packet(packet):
                new_packet = GUIDEPacket(packet.gpsn, neighbor.hc)
                updated_nodes.append((neighbor, new_packet))
        return updated_nodes

    def send_query_packet(self, query_packet, communication_radius):
        self.memory_queue.add(query_packet.dpsn)
        ack_packets = []
        for neighbor in self.neighbors:
            if np.linalg.norm(self.position - neighbor.position) <= communication_radius:
                ack_packets.append(neighbor.receive_query_packet(query_packet, communication_radius))
        return ack_packets

    def receive_query_packet(self, query_packet, communication_radius):
        if query_packet.hc > self.hc and query_packet.dpsn not in self.memory_queue:
            ack_packet = AckPacket(receiver_id=self.id, sender_id=query_packet.sender_id, energy=self.energy, hc=self.hc)
            self.memory_queue.add(query_packet.dpsn)
            self.send_query_packet(query_packet, communication_radius)
            return ack_packet
        return None

    def select_optimal_relay(self, ack_packets, energy_threshold):
        optimal_relay = None
        min_hc = min(ack_packet.hc for ack_packet in ack_packets if ack_packet)
        candidates = [ack_packet for ack_packet in ack_packets if ack_packet and ack_packet.hc == min_hc]
        
        if candidates:
            optimal_relay = max(candidates, key=lambda packet: (packet.energy - energy_threshold) / np.linalg.norm(self.position - packet.sender_id))
        
        return optimal_relay

def initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers, xy_limit):
    nodes = []
    sinks = []

    nodes_per_layer = num_nodes // depth_layers

    for layer in range(depth_layers):
        for i in range(nodes_per_layer):
            position = np.random.rand(3) * space_dim
            position[0] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn x
            position[1] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn y
            depth_start = -layer * (max_depth / depth_layers)
            depth_end = -(layer + 1) * (max_depth / depth_layers)
            position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
            nodes.append(SensorNode(f'node_{layer}_{i}', position))

    remaining_nodes = num_nodes % depth_layers
    for i in range(remaining_nodes):
        position = np.random.rand(3) * space_dim
        position[0] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn x
        position[1] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn y
        layer = np.random.randint(0, depth_layers)
        depth_start = -layer * (max_depth / depth_layers)
        depth_end = -(layer + 1) * (max_depth / depth_layers)
        position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
        nodes.append(SensorNode(f'node_remaining_{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[0] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn x
        position[1] = np.random.uniform(-xy_limit, xy_limit)  # Giới hạn y
        position[2] = 0
        sink = SensorNode(f'sink_{i}', position, is_sink=True)
        sink.gpsn = 1
        sink.hc = 0
        sinks.append(sink)

    return nodes, sinks

def calculate_neighbors(nodes, sinks, communication_radius):
    all_nodes = nodes + sinks
    for node in all_nodes:
        node.neighbors = [n for n in all_nodes if np.linalg.norm(node.position - n.position) <= communication_radius and n != node]

def simulate_guiding_network(num_nodes, num_sinks, space_dim, max_depth, depth_layers, communication_radius, xy_limit, energy_threshold):
    nodes, sinks = initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers, xy_limit)
    calculate_neighbors(nodes, sinks, communication_radius)

    paths = []  # Danh sách các đường đi được tìm thấy

    for sink in sinks:
        packet = GUIDEPacket(sink.gpsn, sink.hc)
        updated_nodes = sink.broadcast_packet(packet)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet))
                paths.append((node, new_packet))  # Lưu lại các đường đi
            updated_nodes = next_updated_nodes

    # In ra thông tin các nút để kiểm tra
    for node in nodes:
        print(f'Node ID: {node.id}, Position: {node.position}, HC: {node.hc}, GPSN: {node.gpsn}')
    
    for sink in sinks:
        print(f'Sink ID: {sink.id}, Position: {sink.position}, HC: {sink.hc}, GPSN: {sink.gpsn}')
    
    return nodes, sinks, paths

def visualize_network(nodes, sinks, paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')
        for neighbor in node.neighbors:
            ax.plot([node.position[0], neighbor.position[0]], [node.position[1], neighbor.position[1]], [node.position[2], neighbor.position[2]], 'k-', lw=0.5)

    for sink in sinks:
        ax.scatter(sink.position[0], sink.position[1], sink.position[2], c='r', marker='^')
        ax.text(sink.position[0], sink.position[1], sink.position[2], '0', color='red')

    # Vẽ các đường đi tìm được với màu khác
    for node, packet in paths:
        ax.plot([node.position[0], packet.sender_id[0]], [node.position[1], packet.sender_id[1]], [node.position[2], packet.sender_id[2]], 'g-', lw=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# Parameters
NUM_NODES = 40
NUM_SINKS = 3
SPACE_DIM = 100
MAX_DEPTH = 100  
DEPTH_LAYERS = 10
COMMUNICATION_RADIUS = 20
XY_LIMIT = 20  # Giới hạn tọa độ x và y
ENERGY_THRESHOLD = 0.1  # Ngưỡng năng lượng

nodes, sinks, paths = simulate_guiding_network(NUM_NODES, NUM_SINKS, SPACE_DIM, MAX_DEPTH, DEPTH_LAYERS, COMMUNICATION_RADIUS, XY_LIMIT, ENERGY_THRESHOLD)
visualize_network(nodes, sinks, paths)
