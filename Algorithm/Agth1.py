import numpy as np
import random
import math
import matplotlib.pyplot as plt

# Simulation parameters
area_size = (1000, 1000, 500)  # 3D space dimensions (x, y, z)
num_nodes = 50  # Number of sensor nodes
num_sinks = 2  # Number of sinks
transmission_range = 250  # Transmission range in meters
initial_energy = 100  # Initial energy in Joules

# Node class definition
class Node:
    def __init__(self, node_id, x, y, z, is_sink=False):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.z = z
        self.is_sink = is_sink
        self.energy = initial_energy if not is_sink else float('inf')
        self.neighbors = []

    def distance_to(self, other_node):
        return math.sqrt((self.x - other_node.x)**2 + (self.y - other_node.y)**2 + (self.z - other_node.z)**2)

# Deploy nodes randomly
nodes = []
for i in range(num_nodes):
    x, y, z = random.uniform(0, area_size[0]), random.uniform(0, area_size[1]), random.uniform(0, area_size[2])
    nodes.append(Node(i, x, y, z))

# Deploy sinks on the surface
sinks = []
for i in range(num_sinks):
    x, y = random.uniform(0, area_size[0]), random.uniform(0, area_size[1])
    sinks.append(Node(num_nodes + i, x, y, 0, is_sink=True))

# Add nodes and sinks to the network
network = nodes + sinks

# Identify neighbors within the transmission range
for node in network:
    for other_node in network:
        if node != other_node and node.distance_to(other_node) <= transmission_range:
            node.neighbors.append(other_node)

# Function to simulate packet transmission
def transmit_packet(source, destination):
    if source.energy <= 0:
        return False
    distance = source.distance_to(destination)
    transmission_energy = 1 + 0.001 * distance  # Simple energy model
    source.energy -= transmission_energy
    if source.energy < 0:
        source.energy = 0
    return source.energy > 0

# Function to perform routing using EEGNBR (simplified)
def eegnbr_routing(source, packet_size=512):
    if source.is_sink:
        return 0  # No delay for sinks
    path = [source]
    current_node = source
    total_delay = 0
    while not current_node.is_sink:
        if not current_node.neighbors:
            return float('inf')  # No path to sink
        next_node = min(current_node.neighbors, key=lambda n: n.distance_to(current_node))
        if not transmit_packet(current_node, next_node):
            return float('inf')  # Packet lost due to energy depletion
        path.append(next_node)
        current_node = next_node
        total_delay += current_node.distance_to(next_node) / 1500  # Assuming speed of sound in water ~ 1500 m/s
    return total_delay

# Simulate packet transmission from each node
delays = []
for node in nodes:
    delay = eegnbr_routing(node)
    delays.append(delay)

# Calculate performance metrics
average_delay = np.mean([d for d in delays if d < float('inf')])
packet_delivery_ratio = sum(1 for d in delays if d < float('inf')) / len(nodes)
total_energy_consumed = sum(initial_energy - node.energy for node in nodes)

# Print results
print(f"Average End-to-End Delay: {average_delay:.2f} seconds")
print(f"Packet Delivery Ratio: {packet_delivery_ratio:.2%}")
print(f"Total Energy Consumed: {total_energy_consumed:.2f} Joules")

# Visualize network deployment
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for node in nodes:
    ax.scatter(node.x, node.y, node.z, c='b')
for sink in sinks:
    ax.scatter(sink.x, sink.y, sink.z, c='r')
plt.show()
