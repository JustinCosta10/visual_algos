import numpy as np
import matplotlib.pyplot as plt
import math
from heapq import heapify, heappush, heappop
from collections import defaultdict

class Dijkstra:

    def __init__(self, collision_map, graph : dict):
        self.collision_map = collision_map
        self.graph = graph

    def getNeighbors(self, coord):
        neighbors = self.graph.get(coord, [])
        safe_neighbors = []
        for neighbor in neighbors:
            x = neighbor[1][0]
            y = neighbor[1][1]
            if self.collision_map[x][y] == False:
                safe_neighbors.append((neighbor[0], (x,y)))
        return safe_neighbors

    def path_trace(self, prev_nodes : dict, node : tuple, start : tuple):
        p = node
        path = []

        while p != start:
            path.append(p)
            p = prev_nodes.get(p)
        path.append(start)

        path.reverse()

        return path

    def search(self, start : tuple, goal : tuple):
        self.distances = defaultdict(lambda:float("inf"))
        self.distances[start] = 0
        heap = []
        prev_nodes = {}
        visited = set()
        heappush(heap, (0, start))
        print(f"Starting at {start}")
        while heap:

            current_node = heappop(heap)
            #splitting into coordinate xy and cost
            current_xy = current_node[1]
            current_cost = current_node[0]

            if current_xy == goal:
                print(f"Goal found: {current_xy}")
                path = self.path_trace(prev_nodes, current_xy, start)
                print(path)
                return path

            current_neighbors = self.getNeighbors(current_xy)
            visited.add(current_xy)

            for neighbor in current_neighbors:
                #splitting into coordinate xy and cost
                neigh_xy = neighbor[1]
                neigh_cost = neighbor[0]

                new_distance = current_cost + neigh_cost

                if neigh_xy not in visited:
                    if new_distance < self.distances[neigh_xy]:
                        self.distances[neigh_xy] = current_cost + neigh_cost
                        prev_nodes[neigh_xy] = current_xy
                        heappush(heap, (self.distances[neigh_xy], neigh_xy))

        print("Goal not found.")

if __name__ == "__main__":

    #Generating graphs, start and goal with distances for djikstra
    graph = {}
    rows = 20
    cols = 30

    directions = [
        (-1,  0, 1),   # up
        ( 1,  0, 1),   # down
        ( 0, -1, 1),   # left
        ( 0,  1, 1),   # right
        (-1, -1, math.sqrt(2)),  #diagonals
        (-1,  1, math.sqrt(2)),
        ( 1, -1, math.sqrt(2)),
        ( 1,  1, math.sqrt(2)),
    ]

    for x in range(rows):
        for y in range(cols):
            neighbors = []
            for dx, dy, cost in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < rows and 0 <= ny < cols:
                    neighbors.append((cost, (nx,ny))) #(cost, coordinates) so that heap can sort by cost
            graph[(x, y)] = neighbors

    collision_map = np.random.rand(rows, cols)<0.1

    start = (np.random.randint(0,rows), np.random.randint(0,cols))

    goal = (np.random.randint(0,rows),np.random.randint(0,cols))
    
    #Running dijkstra
    dijkstra = Dijkstra(collision_map, graph)
    result = dijkstra.search(start, goal)
