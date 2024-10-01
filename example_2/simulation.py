import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nodes import SensorNode, Timer
from packet import GUIDEPacket, QueryPacket, AckPacket

NINF = float('-inf')

def initialize_nodes(num_nodes, num_sinks, num_source_node, space_dim, depth_layers, depth_range):
    nodes = []
    sinks = []
    sources = []

    node_per_layer = num_nodes//depth_layers

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

def calculate_neighbors(nodes, sinks, sources, communication_radius):
    all_nodes = nodes + sinks + sources
    for node in all_nodes:
        node.neighbors = [n for n in all_nodes if np.linalg.norm(node.position - n.position) <= communication_radius and n != node]


def simulate_guiding_network(num_nodes, num_sinks, num_source_node, space_dim, depth_layers, depth_range, communication_radius, num_rounds):
    nodes, sinks, sources = initialize_nodes(num_nodes, num_sinks, num_source_node, space_dim, depth_layers, depth_range)
    calculate_neighbors(nodes, sinks, sources, communication_radius)

    paths = []
    optimal_paths = []
    PDR = []
    average_delay = []
    average_residual_energy = []
    alive_nodes_count = []

    """
    for round in range(num_rounds):
        # Broadcast GUIDEPacket to set up initial hop counts
        for sink in sinks:
            packet = GUIDEPacket(sink.gpsn, sink.hc)
            updated_nodes = sink.broadcast_packet(packet)

            while updated_nodes:
                next_updated_nodes = []
                for node, new_packet in updated_nodes:
                    next_updated_nodes.extend(node.broadcast_packet(new_packet))
                    for neighbor in node.neighbors:
                        if node.hc != neighbor.hc:
                            paths.append((node, neighbor))
                updated_nodes = next_updated_nodes

        # Select nodes with the maximum hop count as source nodes
        max_hc = max(node.hc for node in nodes)
        source_nodes = [node for node in nodes if node.hc == max_hc]

        # Generate and send data packets from source nodes
        for source_node in source_nodes:
            delay = source_node.generate_and_send_data(packet_id=round, communication_radius=communication_radius, energy_threshold=energy_threshold)
            average_delay.append(delay)
            PDR.append(1)

        total_energy = sum(node.energy for node in nodes)
        average_residual_energy.append(total_energy / num_nodes)
        alive_nodes = sum(1 for node in nodes if node.energy > energy_threshold)
        alive_nodes_count.append(alive_nodes)
    """

    # Broadcast GUIDEPacket to set up guiding network
    for sink in sinks:
        packet = GUIDEPacket(sink.gpsn, sink.hc)
        packet_size = 4
        updated_nodes = sink.broadcast_packet(packet, packet_size)

        while updated_nodes:
            next_updated_nodes = []
            for node, new_packet in updated_nodes:
                next_updated_nodes.extend(node.broadcast_packet(new_packet, packet_size))
                for neighbor in node.neighbors:
                    if node.hc != neighbor.hc:
                        paths.append((node, neighbor))  # Lưu lại các đường đi
            updated_nodes = next_updated_nodes
    
    # Find node have max hopcount
    max_hc = max((node.hc for node in nodes), default=NINF)
    max_hc_nodes = [node for node in nodes if node.hc == max_hc]
    
    total_hop_counts = 0

    for dpsn in range(100):
        for max_hc_node in max_hc_nodes:
            packet_size = 4
            for node in nodes:
                node.memory_queue.clear()
            
            current_node = max_hc_node
            hop_count = 0
            path = []
            while not current_node.is_sink:
                ack_packets = []
                query_packet = QueryPacket(current_node.id, current_node.id, dpsn, current_node.hc)
                ack_packets = current_node.send_query_packet(query_packet, communication_radius, packet_size)
                

                optimal_node = current_node.select_optimal_node

    '''
    for max_hc_node in max_hc_nodes:
        packet_size = 4
        for node in nodes:
            node.memory_queue.clear()

        current_node = max_hc_node
        hop_count = 0
        path = []
        while not current_node.is_sink:
            ack_packets = []
            for neighbor in current_node.neighbors:
                query_packet = QueryPacket(current_node.id, current_node.id, current_node.gpsn, current_node.hc)
                ack_packet = neighbor.receive_query_packet(query_packet, communication_radius)
                if ack_packet:
                    ack_packets.append(ack_packet)

            optimal_relay = current_node.select_optimal_relay(ack_packets, energy_thershold) 

            if optimal_relay:
                print(f'Optimal relay selected by {current_node.id}: {optimal_relay.sender_id}')
                next_node = next(n for n in current_node.neighbors if n.id == optimal_relay.sender_id)
                optimal_paths.append((current_node, next_node))
                path.append((current_node, next_node))
                current_node = next_node
                hop_count += 1
            else:
                print(f'No optimal relay found from node {current_node.id}')
                break
        all_max_hc_  
            

    return nodes, sinks, sources, paths
    
    
    
    
    for node in nodes:
        print(f'Node ID: {node.id}, Position: {node.position}, HC: {node.hc}, Energy: {node.energy}, GPSN: {node.gpsn}')
    
    for sink in sinks:
        print(f'Sink ID: {sink.id}, Position: {sink.position}, HC: {sink.hc}, Energy: {sink.energy}, GPSN: {sink.gpsn}')
    
    
    print(f"PDR: {sum(PDR)/num_rounds}")
    print(f"Average E2E Delay: {sum(average_delay)/len(average_delay)}")
    print(f"Average Residual Energy: {sum(average_residual_energy)/len(average_residual_energy)}")
    print(f"Number of Alive Sensor Nodes: {sum(alive_nodes_count)/len(alive_nodes_count)}")

    return nodes, sinks, paths, optimal_paths
    '''

def visualize_network(nodes, sinks, sources, paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for node in nodes:
        ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='o')
        ax.text(node.position[0], node.position[1], node.position[2], f'{node.hc}', color='blue')
        for neighbor in node.neighbors:
            if node.hc != neighbor.hc:
                ax.plot([node.position[0], neighbor.position[0]], [node.position[1], neighbor.position[1]], [node.position[2], neighbor.position[2]], 'g-', lw=0.5)

    for sink in sinks:
        ax.scatter(sink.position[0], sink.position[1], sink.position[2], c='r', marker='^')
        ax.text(sink.position[0], sink.position[1], sink.position[2], '0', color='red')  # Sink hop count is always 0

    for source in sources:
        ax.scatter(source.position[0], source.position[1], source.position[2], c='g', marker='^')
        ax.text(source.position[0], source.position[1], source.position[2], f'{source.hc}', color='green')

    for path in paths:
        node, neighbor = path
        ax.plot([node.position[0], neighbor.position[0]], 
                [node.position[1], neighbor.position[1]], 
                [node.position[2], neighbor.position[2]], 'k-', lw=0.5)


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()