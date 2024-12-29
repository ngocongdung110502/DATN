import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from nodes import SensorNode
from packet import GuidePacket, QueryPacket, DataPacket

NINF = float('-inf')

'''
def initialize_nodes(num_nodes, num_sinks, num_source_node, space_dim, depth_layers, depth_range):
    nodes = []
    sinks = []
    sources = []

    node_per_layer = num_nodes // depth_layers

    for layer in range(depth_layers):
        for i in range(node_per_layer):
            position = np.random.rand(3) * space_dim
            depth_start = -layer * (space_dim / depth_layers)
            depth_end = -(layer + 1) * (space_dim / depth_layers)
            position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
            nodes.append(SensorNode(f'node_{layer}_{i}', position))

    remaining_nodes = num_nodes % depth_layers
    for i in range(remaining_nodes):
        position = np.random.rand(3) * space_dim
        layer = np.random.randint(0, depth_layers)
        depth_start = -layer * (space_dim / depth_layers)
        depth_end = -(layer + 1) * (space_dim / depth_layers)
        position[2] = depth_start - np.random.rand() * (depth_start - depth_end)
        nodes.append(SensorNode(f'node_remaining_{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[2] = 0  # Sinks are on the water surface
        sink = SensorNode(f'sink_{i}', position, is_sink=True)
        sink.gpsn = 1
        sink.hc = 0
        sinks.append(sink)

    for i in range(num_source_node):
        position = np.random.rand(3) * space_dim
        position[2] = np.random.uniform(depth_range[0], depth_range[1]) * -1
        source = SensorNode(f'source_{i}', position, is_source=True)
        sources.append(source)

    return nodes, sinks, sources
'''

def initialize_nodes(num_nodes, num_sinks, num_source_nodes, space_dim, depth_range):
    nodes = []
    sinks = []
    sources = []

    for i in range(num_nodes):
        position = np.random.rand(3) * space_dim
        position[2] = np.random.uniform(0, space_dim) * -1
        nodes.append(SensorNode(f'Node_{i}', position))

    for i in range(num_sinks):
        position = np.random.rand(3) * space_dim
        position[2] = 0
        sink = SensorNode(f'Sink_{i}', position)
        sink.gpsn = 1
        sink.hc = 0
        sinks.append(sink)

    for i in range(num_source_nodes):
        position = np.random.rand(3) * space_dim
        position[2] = np.random.uniform(depth_range[0], depth_range[1]) * -1
        source = SensorNode(f'Source_{i}', position, is_source=True)
        sources.append(source)

    return nodes, sinks, sources

def calculate_neighbors(nodes, sinks, sources, communication_radius):
    all_nodes = nodes + sinks + sources
    for node in all_nodes:
        node.neighbors = [n for n in all_nodes if
                          np.linalg.norm(node.position - n.position) <= communication_radius and n != node]


def simulate_guiding_network(num_nodes, num_sinks, num_source_node, space_dim, depth_layers, depth_range, communication_radius, num_rounds, energy_threshold):
    #nodes, sinks, sources = initialize_nodes(num_nodes, num_sinks, num_source_node, space_dim, depth_layers, depth_range)
    nodes, sinks, sources = initialize_nodes(num_nodes, num_sinks, num_source_node, space_dim, depth_range)
    calculate_neighbors(nodes, sinks, sources, communication_radius)

    paths = []
    optimal_paths = []
    PDR = []
    average_delay = []
    average_residual_energy = []
    alive_nodes_count = []
    successful_deliveries = 0
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Broadcast GUIDEPacket to set up guiding network
    for sink in sinks:
        packet = GuidePacket(sink.gpsn, sink.hc)
        guide_packet_size = 32
        updated_nodes = sink.broadcast_packet(packet, guide_packet_size)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet, guide_packet_size))
                for neighbor in node.neighbors:
                    if node.hc != neighbor.hc:
                        paths.append((node, neighbor))  # Lưu lại các đường đi
                updated_nodes = next_updated_nodes

    for source in sources:
        for dpsn in range(num_rounds):
            print(f'Round {dpsn} with source {source.id}')
            signaling_packet_size = 32

            current_optimal_path = []
            for node in nodes + sinks:
                node.memory_queue.clear()

            current_node = source
            hop_count = 0
            data_packet_paths = [source]
            while not current_node.is_sink:
                query_packet = QueryPacket(current_node.id, current_node.id, dpsn, current_node.hc)
                if current_node.energy > energy_threshold:
                    ack_packets = current_node.send_query_packet(query_packet, communication_radius,
                                                                 signaling_packet_size)
                    print(f'Node {current_node.id} received ack_packets: {[p.sender_id for p in ack_packets]}')
                else:
                    print(f'Node {current_node.id} does not have enough energy')
                    break

                optimal_node = current_node.select_optimal_node(ack_packets, energy_threshold)

                if optimal_node:
                    print(f'Optimal node selected by {current_node.id}: {optimal_node.sender_id}')
                    next_node = next(n for n in current_node.neighbors if n.id == optimal_node.sender_id)
                    data_packet = DataPacket(source.id, next_node.id, dpsn, 400)
                    distance = np.linalg.norm(current_node.position - next_node.position)
                    current_node.send_data_packet(data_packet, next_node, distance)
                    data_packet_paths.append(next_node)
                    current_optimal_path.append((current_node, next_node))

                    current_node = next_node
                    hop_count += 1
                else:
                    print(f'No optimal node found from node {current_node.id}')
                    break

            def is_packet_delivered_to_sink(data_packet_paths, sinks):
                return data_packet_paths[-1] in sinks

            if is_packet_delivered_to_sink(data_packet_paths, sinks):
                successful_deliveries += 1

            print(f'Number of nodes from source to sink for {source.id}: {hop_count}')
            print(f'Data packet {dpsn} pass through nodes: {[node.id for node in data_packet_paths]}')
            print(f'Hop count of node pass through: {[node.hc for node in data_packet_paths]}')
            visualize_path_DataPacket(ax, nodes, sinks, sources, paths, current_optimal_path, dpsn, hop_count)

            optimal_paths.append(current_optimal_path)

    total_energy = sum(node.energy for node in nodes)
    average_residual_energy.append(total_energy / (num_nodes+num_sinks+num_source_node))  # Năng lượng trung bình còn lại
    alive_nodes = sum(1 for node in nodes if node.energy > energy_threshold)  # Số lượng nút còn sống
    alive_nodes_count.append(alive_nodes)

    for node in nodes:
        print(f'Node ID: {node.id}, Position: {node.position}, HC: {node.hc}, Energy: {node.energy}, GPSN: {node.gpsn}')

    for sink in sinks:
        print(f'Sink ID: {sink.id}, Position: {sink.position}, HC: {sink.hc}, Energy: {sink.energy}, GPSN: {sink.gpsn}')

    for source in sources:
        print(
            f'Source ID: {source.id}, Position: {source.position}, HC: {source.hc}, Energy: {source.energy}, GPSN: {source.gpsn}')

    print(
        f'Avg Residual Energy = {average_residual_energy[-1]:.2f} J, Alive Nodes = {alive_nodes}, Success Data Packet = {successful_deliveries}')

    return nodes, sinks, sources, paths, optimal_paths


def visualize_path_DataPacket(ax, nodes, sinks, sources, paths, optimal_path, dpsn, hop_count):
    ax.cla()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Path of Data Packet")

    for node in nodes:
        ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')
        for neighbor in node.neighbors:
            if node.hc != neighbor.hc:
                ax.plot([node.position[0], neighbor.position[0]], [node.position[1], neighbor.position[1]],
                        [node.position[2], neighbor.position[2]], 'g-', lw=1)

    for sink in sinks:
        ax.scatter(sink.position[0], sink.position[1], sink.position[2], c='r', marker='^')
        ax.text(sink.position[0], sink.position[1], sink.position[2], '0', color='red')  # Sink hop count is always 0

    for source in sources:
        ax.scatter(source.position[0], source.position[1], source.position[2], c='g', marker='s')
        ax.text(source.position[0], source.position[1], source.position[2], f'{source.hc}', color='green')

    '''
    for path in paths:
        node, neighbor = path
        ax.plot([node.position[0], neighbor.position[0]],
                [node.position[1], neighbor.position[1]],
                [node.position[2], neighbor.position[2]], 'g-', lw=1)
    '''

    for node, next_node in optimal_path:
        ax.plot([node.position[0], next_node.position[0]], [node.position[1], next_node.position[1]],
                [node.position[2], next_node.position[2]], 'r-', lw=2)

    ax.text2D(0.05, 0.95, f'Packet ID: {dpsn}\nHop Count: {hop_count}', transform=ax.transAxes, fontsize=10,
              color='blue')

    plt.draw()
    plt.pause(0.5)

def visualize_network(nodes, sinks, sources, paths, optimal_paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')

        for neighbor in node.neighbors:
            if node.hc != neighbor.hc:
                ax.plot([node.position[0], neighbor.position[0]], [node.position[1], neighbor.position[1]],
                        [node.position[2], neighbor.position[2]], 'g-', lw=1)

    for sink in sinks:
        ax.scatter(sink.position[0], sink.position[1], sink.position[2], c='r', marker='^')
        ax.text(sink.position[0], sink.position[1], sink.position[2], '0', color='red')  # Sink hop count is always 0

    for source in sources:
        ax.scatter(source.position[0], source.position[1], source.position[2], c='g', marker='^')
        ax.text(source.position[0], source.position[1], source.position[2], f'{source.hc}', color='green')

    '''
    for path in paths:
        node, neighbor = path
        ax.plot([node.position[0], neighbor.position[0]],
                [node.position[1], neighbor.position[1]],
                [node.position[2], neighbor.position[2]], 'g-', lw=1)
    '''

    for path in optimal_paths:
        for node, next_node in path:
            ax.plot([node.position[0], next_node.position[0]], [node.position[1], next_node.position[1]],
                    [node.position[2], next_node.position[2]], 'r-', lw=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Network Visualization')
    plt.show()
