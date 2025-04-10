# DAA-Project
#Project :- City Map Navigator: Shortest Path Using Dijkstra and A*


import heapq
import math
import time

class Node:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y
        self.neighbors = []

    def add_neighbor(self, neighbor, cost):
        self.neighbors.append((neighbor, cost))

    def __lt__(self, other):
        return self.name < other.name  # For priority queue tie-breaking

class CityMap:
    def __init__(self):
        self.nodes = {}

    def add_location(self, name, x, y):
        self.nodes[name] = Node(name, x, y)

    def connect_locations(self, from_name, to_name, cost):
        self.nodes[from_name].add_neighbor(self.nodes[to_name], cost)
        self.nodes[to_name].add_neighbor(self.nodes[from_name], cost)

    def heuristic(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def dijkstra(self, start_name, end_name):
        start = self.nodes[start_name]
        end = self.nodes[end_name]
        distances = {node: float('inf') for node in self.nodes.values()}
        distances[start] = 0
        pq = [(0, start)]
        came_from = {}

        while pq:
            current_dist, current_node = heapq.heappop(pq)

            if current_node == end:
                return self.reconstruct_path(came_from, end), current_dist

            for neighbor, cost in current_node.neighbors:
                new_dist = current_dist + cost
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    came_from[neighbor] = current_node
                    heapq.heappush(pq, (new_dist, neighbor))

        return [], float('inf')

    def a_star(self, start_name, end_name):
        start = self.nodes[start_name]
        end = self.nodes[end_name]
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {node: float('inf') for node in self.nodes.values()}
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.nodes.values()}
        f_score[start] = self.heuristic(start, end)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == end:
                return self.reconstruct_path(came_from, end), g_score[end]

            for neighbor, cost in current.neighbors:
                tentative_g = g_score[current] + cost
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, end)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return [], float('inf')

    def reconstruct_path(self, came_from, current):
        path = [current.name]
        while current in came_from:
            current = came_from[current]
            path.append(current.name)
        return list(reversed(path))


# --- Driver Code ---
if __name__ == "__main__":
    city = CityMap()

    # Add locations (x, y are coordinates for heuristic)
    city.add_location("A", 0, 0)
    city.add_location("B", 2, 4)
    city.add_location("C", 5, 2)
    city.add_location("D", 6, 6)
    city.add_location("E", 8, 3)

    # Connect roads with travel cost (could be distance, time, etc.)
    city.connect_locations("A", "B", 5)
    city.connect_locations("A", "C", 6)
    city.connect_locations("B", "D", 4)
    city.connect_locations("C", "D", 3)
    city.connect_locations("C", "E", 5)
    city.connect_locations("D", "E", 2)

    start_node = "A"
    end_node = "E"

    print(f"Finding shortest path from {start_node} to {end_node}...\n")

    # Dijkstra
    t1 = time.time()
    dijkstra_path, dijkstra_cost = city.dijkstra(start_node, end_node)
    t2 = time.time()

    print("🔷 Dijkstra's Algorithm:")
    print("Path:", " -> ".join(dijkstra_path))
    print("Cost:", dijkstra_cost)
    print("Time:", round((t2 - t1) * 1000, 3), "ms\n")

    # A*
    t3 = time.time()
    astar_path, astar_cost = city.a_star(start_node, end_node)
    t4 = time.time()

    print("⭐ A* Algorithm:")
    print("Path:", " -> ".join(astar_path))
    print("Cost:", astar_cost)
    print("Time:", round((t4 - t3) * 1000, 3), "ms")
