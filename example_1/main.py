from simulation import simulate_guiding_network, visualize_network

# Simulation parameters
NUM_NODES = 40
NUM_SINKS = 3
SPACE_DIM = 100
MAX_DEPTH = 100  
DEPTH_LAYERS = 10
COMMUNICATION_RADIUS = 20
XY_LIMIT = 20  # Giới hạn tọa độ x và y
ENERGY_THRESHOLD = 0.2  # Ngưỡng năng lượng

nodes, sinks, paths, path_from_max_hc = simulate_guiding_network(NUM_NODES, NUM_SINKS, SPACE_DIM, MAX_DEPTH, DEPTH_LAYERS, COMMUNICATION_RADIUS, XY_LIMIT, ENERGY_THRESHOLD)
visualize_network(nodes, sinks, paths, path_from_max_hc)