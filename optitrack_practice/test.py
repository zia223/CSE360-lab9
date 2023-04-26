import networkx as nx
import matplotlib.pyplot as plt


G = nx.Graph()

world_nodes = {1: [-1.204, -0.142],
               2: [-0.264, -0.289],
               3: [0.749, -0.200],
               4: [0.818,  1.314],
               5: [-0.165, 1.377],
               6: [-0.287, 0.925],
               7: [-0.177, 0.283],
               8: [-0.794, 0.827],
               9: [-1.388, 0.189],
               10: [0.940, 0.291]}

G.add_nodes_from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])


G.add_edges_from([(1, 2), (2, 3), (3, 7), (7, 10), (7,6), (3, 10), (10, 4), (4,5), (5,6), (6, 8), (8,9), (9,1), (2,7)])
#nx.draw(G, with_labels = True)

#plt.show()

nodes = nx.dijkstra_path(G, 1, 6)
edges = nx.utils.pairwise(nodes)
print(nodes)
print(world_nodes[nodes[2]])


