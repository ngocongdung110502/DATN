import numpy as np
from simulation import simulate_guiding_network, visualize_network

NUM_NODES = 50
NUM_SINKS = 5
NUM_SOURCE_NODE = 1
SPACE_DIM = 100
DEPTH_LAYERS = 10
DEPTH_RANGE = [90, 100]
COMMUNICATION_RADIUS = 30
NUM_ROUNDS = 100

nodes, sinks, sources, paths= simulate_guiding_network(NUM_NODES, NUM_SINKS, NUM_SOURCE_NODE, SPACE_DIM, DEPTH_LAYERS, DEPTH_RANGE, COMMUNICATION_RADIUS, NUM_ROUNDS)
visualize_network(nodes, sinks, sources, paths)