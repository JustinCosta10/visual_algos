import numpy as np


class DFS:
    def __init__(self, collision_map, graph : dict):
        self.collision_map = collision_map
        self.graph = graph
        self.visited = set()
        self.parent = {}

    def getNeighbors(self, coord): #checks all possible neighbors and filters out nodes with a collision
        neighbors = self.graph.get(coord, [])
        safe_neighbors = []
        for neighbor in neighbors:
            if self.collision_map[neighbor[0]][neighbor[1]] == False:
                safe_neighbors.append(neighbor)
        return safe_neighbors

    def path_trace(self, prev_nodes : dict, node : tuple, start : tuple):
        p = node
        path = []

        while p != start:
            path.append(p)
            p = prev_nodes.get(p)
        path.append(start)
        path.reverse() #puts path in order

        return path

    def search(self, start : tuple, goal : tuple):

        found = self.recurse_dfs_helper(start, goal)

        if found == goal:

            print("Goal found!")
            print(found)

            # reconstruct path
            path = self.path_trace(self.parent, found, start)
            print(path)
            return path
        else:
            print("Goal not found")

    def recurse_dfs_helper(self, node, goal):
        if node in self.visited:
            return False

        if node == goal:
            return node

        self.visited.add(node)
        neighbors = self.getNeighbors(node)
        for neighbor in neighbors:
            if neighbor not in self.visited:
                self.parent[neighbor] = node
                result = self.recurse_dfs_helper(neighbor, goal)
                if result:
                    return result
        return False


if __name__ == "__main__":
    #Generating maps, start and goal
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

    #Running DFS
    dfs = DFS(collision_map, graph)
    result = dfs.search(start, goal)
