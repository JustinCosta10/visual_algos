import numpy as np

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
    def __init__(self, collision_map, graph : dict):
        self.collision_map = collision_map
        self.graph = graph


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

        while p != start:
            path.append(p)
            p = prev_nodes.get(p)
        path.append(start)
        path.reverse()

        return path

    def search(self, start, goal):
        queue = [start]
        prev_nodes = {}
        visited = set()
        while queue:
            current_node = queue.pop(0)

            if current_node == goal:
                print("Goal found!")
                print(current_node)
                path = self.path_trace(prev_nodes, current_node)
                print(path)

                return path

            current_neighbors = self.getNeighbors(current_node)
            visited.add(current_node)
            for neighbor in current_neighbors:
                if neighbor not in visited and neighbor not in queue:
                    prev_nodes[neighbor] = current_node
                    queue.append(neighbor)
        print("Goal not found.")

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
    
    #Running BFS
    bfs = BFS(collision_map, graph)
    result = bfs.search(start, goal)
