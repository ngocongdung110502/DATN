import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
NINF = float('-inf')

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

class DataPacket:
    def __init__(self, sender_id):
        self.sender_id = sender_id

class SensorNode:
    def __init__(self, id, position, is_sink=False):
        self.id = id
        self.position = position
        self.gpsn = 0
        self.hc = NINF
        self.neighbors = []
        self.is_sink = is_sink
        self.memory_queue = set()
        self.energy = np.random.uniform(0.5, 1.0)

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

    def send_query_packet(self, query_packet, communication_radius, timer):
        self.memory_queue.add(query_packet.dpsn)
        ack_packets = []
        for neighbor in self.neighbors:
            if np.linalg.norm(self.position - neighbor.position) <= communication_radius:
                ack_packet = neighbor.receive_query_packet(query_packet, communication_radius)
                if ack_packet:
                    ack_packets.append(ack_packet)
        timer.expire()
        return ack_packets

    def receive_query_packet(self, query_packet, communication_radius):
        if query_packet.hc > self.hc and query_packet.sender_id not in self.memory_queue:
            ack_packet = AckPacket(receiver_id=query_packet.sender_id, sender_id=self.id, energy=self.energy, hc=self.hc)
            self.memory_queue.add(query_packet.sender_id)
            timer = Timer(2 * communication_radius / 1500)
            self.send_query_packet(query_packet, communication_radius, timer)
            return ack_packet
        return None

    def select_optimal_relay(self, ack_packets, energy_threshold):
        optimal_relay = None
        min_hc = min((ack_packet.hc for ack_packet in ack_packets if ack_packet), default=NINF)
        candidates = [ack_packet for ack_packet in ack_packets if ack_packet and ack_packet.hc == min_hc]

        if candidates:
            def compute_priority(packet):
                try:
                    sender_node = next(n for n in self.neighbors if n.id == packet.sender_id)
                    distance = np.linalg.norm(self.position - sender_node.position)
                    energy_component = max(0, packet.energy - energy_threshold)
                    return energy_component / distance
                except StopIteration:
                    return NINF
            
            optimal_relay = max(candidates, key=compute_priority, default=None)

        return optimal_relay

    def generate_data_packet(self):
        return DataPacket(self.id)

class Timer:
    def __init__(self, duration):
        self.duration = duration
        self.remaining_time = duration

    def tick(self, time_passed):
        self.remaining_time -= time_passed

    def is_expired(self):
        return self.remaining_time <= 0

    def expire(self):
        self.remaining_time = 0

def initialize_nodes(num_nodes, num_sinks, space_dim, max_depth, depth_layers, xy_limit):
    nodes = []
    sinks = []

    nodes_per_layer = num_nodes // depth_layers

    for layer in range(depth_layers):
        for i in range(nodes_per_layer):
            position = np.random.rand(3) * space_dim
            position[0] = np.random.uniform(-xy_limit, xy_limit)
            position[1] = np.random.uniform(-xy_limit, xy_limit)
            depth_start = -layer * (max_depth / depth_layers)
            depth_end = -(layer + 1) * (max_depth / depth_layers)
            position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
            nodes.append(SensorNode(f'node_{layer}_{i}', position))

    remaining_nodes = num_nodes % depth_layers
    for i in range(remaining_nodes):
        position = np.random.rand(3) * space_dim
        position[0] = np.random.uniform(-xy_limit, xy_limit)
        position[1] = np.random.uniform(-xy_limit, xy_limit)
        layer = np.random.randint(0, depth_layers)
        depth_start = -layer * (max_depth / depth_layers)
        depth_end = -(layer + 1) * (max_depth / depth_layers)
        position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
        nodes.append(SensorNode(f'node_remaining_{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[0] = np.random.uniform(-xy_limit, xy_limit)
        position[1] = np.random.uniform(-xy_limit, xy_limit)
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

    paths = []
    optimal_paths = []
    all_max_hc_paths = []  # List to store all paths from max hop count nodes to sinks

    for sink in sinks:
        packet = GUIDEPacket(sink.gpsn, sink.hc)
        updated_nodes = sink.broadcast_packet(packet)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet))
                paths.append((node, node.neighbors))  # Lưu lại các đường đi
            updated_nodes = next_updated_nodes

    # Find the maximum hop count
    max_hc = max((node.hc for node in nodes), default=NINF)

    # Find all nodes with the maximum hop count
    max_hc_nodes = [node for node in nodes if node.hc == max_hc]

    total_hop_counts = 0  # Tổng số hop count từ tất cả các nút có hopcount max đến sinks

    for max_hc_node in max_hc_nodes:

        for node in nodes:
            node.memory_queue.clear()

        current_node = max_hc_node #source node 
        hop_count = 0  # Initialize hop count
        path = []  # Path from the current node to a sink
        while not current_node.is_sink:
            ack_packets = []
            for neighbor in current_node.neighbors:
                query_packet = QueryPacket(current_node.id, current_node.id, current_node.gpsn, current_node.hc)
                timer = Timer(2 * communication_radius / 1500)
                ack_packet = neighbor.receive_query_packet(query_packet, communication_radius)
                if ack_packet:
                    ack_packets.append(ack_packet)
            print(f'Node {current_node.id} received ack_packets: {[p.sender_id for p in ack_packets]}')

            optimal_relay = current_node.select_optimal_relay(ack_packets, energy_threshold)

            if optimal_relay:
                print(f'Optimal relay selected by {current_node.id}: {optimal_relay.sender_id}')
                next_node = next(n for n in current_node.neighbors if n.id == optimal_relay.sender_id)
                optimal_paths.append((current_node, next_node))
                path.append((current_node, next_node))  # Append to path
                current_node = next_node
                hop_count += 1  # Increment hop count for each hop
            else:
                print(f"No optimal relay found from node {current_node.id}")
                break

        all_max_hc_paths.append(path)  # Store the path from the current max hc node to a sink

        print(f'Number of nodes from source to sink for {max_hc_node.id}: {hop_count}')
        total_hop_counts += hop_count

    print(f'Total hop counts from all nodes with max hopcount to sinks: {total_hop_counts}')

    for node in nodes:
        print(f'Node ID: {node.id}, Position: {node.position}, HC: {node.hc}, GPSN: {node.gpsn}')
    
    for sink in sinks:
        print(f'Sink ID: {sink.id}, Position: {sink.position}, HC: {sink.hc}, GPSN: {sink.gpsn}')

    return nodes, sinks, paths, optimal_paths, all_max_hc_paths

def visualize_network(nodes, sinks, paths, optimal_paths, all_max_hc_paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(*node.position, c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')

    for sink in sinks:
        ax.scatter(*sink.position, c='r', marker='s')
        ax.text(sink.position[0], sink.position[1], sink.position[2], f'{sink.hc}', color='red')

    for path in paths:
        node, neighbors = path
        for neighbor in neighbors:
            if node.hc != neighbor.hc:
                ax.plot([node.position[0], neighbor.position[0]], 
                    [node.position[1], neighbor.position[1]], 
                    [node.position[2], neighbor.position[2]], 'g-', lw=0.5)

    '''
    for path in optimal_paths:
        node, next_node = path
        ax.plot([node.position[0], next_node.position[0]], [node.position[1], next_node.position[1]], [node.position[2], next_node.position[2]], 'r-', lw=2)
    '''
    # Plot all paths from nodes with max hop count to sinks
    for path in all_max_hc_paths:
        for node, next_node in path:
            ax.plot([node.position[0], next_node.position[0]], [node.position[1], next_node.position[1]], [node.position[2], next_node.position[2]], 'r-', lw=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Network Visualization')

    plt.show()

# Simulation parameters
NUM_NODES = 50
NUM_SINKS = 4
SPACE_DIM = 100
MAX_DEPTH = 100  
DEPTH_LAYERS = 10
COMMUNICATION_RADIUS = 20
XY_LIMIT = 20  # Giới hạn tọa độ x và y
ENERGY_THRESHOLD = 0.1  # Ngưỡng năng lượng

nodes, sinks, paths, path_from_max_hc, all_max_hc_paths = simulate_guiding_network(NUM_NODES, NUM_SINKS, SPACE_DIM, MAX_DEPTH, DEPTH_LAYERS, COMMUNICATION_RADIUS, XY_LIMIT, ENERGY_THRESHOLD)
visualize_network(nodes, sinks, paths, path_from_max_hc, all_max_hc_paths)