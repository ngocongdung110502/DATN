import random
import numpy as np

#Định nghĩa các cảm biến sensor, sink và source
class Node:
    def __init__(self, node_id, position, is_sink = False, is_source=False, energy = 30):
        self.node_id = node_id
        self.position = position
        self.is_sink = is_sink
        self.is_source = is_source
        self.energy = energy

    def __repr__(self):
        return f"Node(ID={self.node_id}, Position={self.position}, IsSink={self.is_sink}, IsSource={self.is_source})"

#Hàm sinh tọa độ ngẫu nhiên
def generate_random_coordinates(x_range, y_range, z_range):
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    z = random.uniform(*z_range)

    return x, y, z

#Hàm tạo node
def create_node(num, label, x_range, y_range, z_range, is_sink = False, is_source = False):
    return [Node(node_id=f'{label}{i+1}', position=generate_random_coordinates(x_range, y_range, z_range),
                 is_sink = is_sink, is_source=is_source) for i in range(num)]

#Hàm khởi tạo các node
def initialize_node(num_node, num_sink, num_source, space_dim, depth_range):
    sinks = create_node(num_sink, "Si", (0, space_dim), (0, space_dim),
                        (0,0), is_sink=True)
    sources = create_node(num_source, "So", (0, space_dim), (0, space_dim),
                          (-space_dim, -depth_range), is_source=True)

    nodes = create_node(num_node, "N", (0, space_dim), (0, space_dim),
                        (-space_dim, 0))
    return sinks, sources, nodes

#Hàm tính khoảng cách giữa 2 node
def distance(node1, node2):
    return np.linalg.norm(np.array(node1.position) - np.array(node2.position))



