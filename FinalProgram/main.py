import numpy as np
from simulation import simulate_guiding_network, visualize_network


NUM_NODES = 200
NUM_SINKS = 5
NUM_SOURCE_NODE = 1
SPACE_DIM = 500
DEPTH_LAYERS = 10
DEPTH_RANGE = [450, 500]
COMMUNICATION_RADIUS = 100
NUM_ROUNDS = 100
ENERGY_THRESHOLD = 2*(400/10000+ COMMUNICATION_RADIUS/1500) + 0.1*400/10000

nodes, sinks, sources, paths, optimal_paths = simulate_guiding_network(NUM_NODES, NUM_SINKS, NUM_SOURCE_NODE, SPACE_DIM, DEPTH_LAYERS, DEPTH_RANGE, COMMUNICATION_RADIUS, NUM_ROUNDS, ENERGY_THRESHOLD)
visualize_network(nodes, sinks, sources, paths, optimal_paths)