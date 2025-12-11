import numpy as np
import matplotlib.pyplot as plt

graph = {}
rows = 20
cols = 30

for coord_x in range(rows):
    for coord_y in range(cols):
        neighbors = []
        if coord_x + 1 < rows:
            neighbors.append((coord_x + 1, coord_y))
        if coord_x - 1 >= 0:
            neighbors.append((coord_x - 1, coord_y))
        if coord_y + 1 < cols:
            neighbors.append((coord_x, coord_y + 1))
        if coord_y - 1 >= 0:
            neighbors.append((coord_x, coord_y - 1))
        graph[(coord_x,coord_y)] = neighbors

collision_map = np.random.rand(rows, cols)<0.1

start = (np.random.randint(0,rows), np.random.randint(0,cols))

goal = (np.random.randint(0,rows),np.random.randint(0,cols))


class BFS:
    def __init__(self, collision_map : list[list[float]], graph : dict, start : tuple, goal : tuple):
        self.collision_map = collision_map
        self.graph = graph
        self.start = start
        self.goal = goal

    def getNeighbors(self, coord): #checks all possible neighbors and filters out nodes with a collision
        neighbors = self.graph.get(coord, [])
        safe_neighbors = []
        for neighbor in neighbors:
            if self.collision_map[neighbor[0]][neighbor[1]] == False:
                safe_neighbors.append(neighbor)
        return safe_neighbors

    def path_trace(self, prev_nodes : dict, node : tuple):
        p = node
        path = []

        while p != self.start:
            path.append(p)
            p = prev_nodes.get(p)
        path.append(self.start)

        return path

    def search(self):
        plt.imshow(self.collision_map, cmap='gray_r') #builds map based on collisions
        plt.plot(self.goal[1], self.goal[0], 'y*') #plots goal on map
        plt.ion() #sets it into interactive mode
        plt.show() #displays current map
        queue = [self.start]
        prev_nodes = {}
        visited = set()
        while queue:
            current_node = queue.pop(0)
            plt.plot(current_node[1], current_node[0], 'g*') #plots current nodes as they are visited
            plt.pause(0.00001) #pauses each step so it doesn't go too fast to observe

            if current_node == self.goal:
                plt.plot(self.goal[1], self.goal[0], 'r*', markersize=12) #plots a big goal when found
                plt.pause(1) #pauses for a bit for a moment of appreciation
                print("Goal found!")
                print(current_node)
                path = self.path_trace(prev_nodes, current_node)

                for p in path:
                    plt.plot(p[1], p[0], 'r.')
                    plt.pause(0.01)   # animate tracing the path
                plt.ioff()
                plt.pause(5)

                return current_node
            current_neighbors = self.getNeighbors(current_node)
            visited.add(current_node)
            prev_node = current_node
            for neighbor in current_neighbors:
                if neighbor not in visited and neighbor not in queue:
                    prev_nodes[neighbor] = current_node
                    queue.append(neighbor)
        print("Goal not found.")

if __name__ == "__main__":
    bfs = BFS(collision_map, graph, start, goal)
    result = bfs.search()
