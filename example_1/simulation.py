import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nodes import SensorNode, GUIDEPacket, QueryPacket, Timer

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

    for sink in sinks:
        packet = GUIDEPacket(sink.gpsn, sink.hc)
        updated_nodes = sink.broadcast_packet(packet)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet))
                for neighbor in node.neighbors:
                    if node.hc != neighbor.hc:
                        paths.append((node, neighbor))  # Lưu lại các đường đi
            updated_nodes = next_updated_nodes

    # Find the node with the maximum hop count
    max_hc_node = max(nodes, key=lambda n: n.hc, default=None)

    if max_hc_node:
        current_node = max_hc_node
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
                current_node = next_node
            else:
                print(f"No optimal relay found from node {current_node.id}")
                break
    
    for node in nodes:
        print(f'Node ID: {node.id}, Position: {node.position}, HC: {node.hc}, Energy: {node.energy} GPSN: {node.gpsn}')
    
    for sink in sinks:
        print(f'Sink ID: {sink.id}, Position: {sink.position}, HC: {sink.hc}, Energy: {sink.energy}, GPSN: {sink.gpsn}')

    return nodes, sinks, paths, optimal_paths

def visualize_network(nodes, sinks, paths, optimal_paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(*node.position, c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')

    for sink in sinks:
        ax.scatter(*sink.position, c='r', marker='s')
        ax.text(sink.position[0], sink.position[1], sink.position[2], f'{sink.hc}', color='red')

    """
    for path in paths:
        node, neighbors = path
        for neighbor in neighbors:
            if node.hc != neighbor.hc:
                ax.plot([node.position[0], neighbor.position[0]], 
                    [node.position[1], neighbor.position[1]], 
                    [node.position[2], neighbor.position[2]], 'g-', lw=0.5)
    """
    for node, neighbor in paths:  # Lặp qua các cặp (node, neighbor)
        ax.plot([node.position[0], neighbor.position[0]], 
                [node.position[1], neighbor.position[1]], 
                [node.position[2], neighbor.position[2]], 'g-', lw=0.5)

    for path in optimal_paths:
        node, next_node = path
        ax.plot([node.position[0], next_node.position[0]], [node.position[1], next_node.position[1]], [node.position[2], next_node.position[2]], 'r-', lw=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Network Visualization')

    plt.show()