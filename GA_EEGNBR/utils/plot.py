import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Hàm vẽ các node trong không gian 3D
def plot_nodes(ax, all_nodes):
    for node in all_nodes:
        if node.is_sink:
            ax.scatter(node.position[0], node.position[1], node.position[2], c='r', marker='o',
                       label='Sink' if node.node_id == "Si1" else "")
        if node.is_source:
            ax.scatter(node.position[0], node.position[1], node.position[2], c='b', marker='^',
                       label='Source' if node.node_id == "So1" else "")
        if not node.is_sink and not node.is_source:
            ax.scatter(node.position[0], node.position[1], node.position[2], c='g', marker='o',
                       label='Sensor' if node.node_id == "N1" else "")
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')
    ax.legend()

#Hàm vẽ đường đi tốt nhất
def plot_best_path(ax, best_path):
    xs, ys, zs = zip(*[node.position for node in best_path])
    ax.plot(xs, ys, zs, c='k', linewidth=2, label='Best_Path')

def plot_best_path_generation(ax, best_path, generation):
    xs, ys, zs = zip(*[node.position for node in best_path])
    ax.plot(xs, ys, zs, c='black', linewidth=1)
    ax.text(xs[-1], ys[-1], zs[-1], f'Gen {generation+1}', fontsize = 8, color='black')
