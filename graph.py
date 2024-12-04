# graph.py

from typing import Dict, Optional, List
from heapq import heappush, heappop
import math
from vect2d import Vect2D

def congestion_factor(boid_count):
    return math.exp(boid_count) - 1

class Node:
    def __init__(self, coord, id):
        self.id = id
        self.coord = coord
        self.neighbors = {}  # Store neighbors and their traffic weights

    def connect(self, neighbor, base_cost=1.0):
        """Connect this node to a neighbor with a base cost."""
        if neighbor not in self.neighbors:
            self.neighbors[neighbor] = base_cost  # Initialize with base cost
            neighbor.connect(self, base_cost)

#### edit by chengpeng
    def update_traffic(self, neighbor, boid_count):
        """Update traffic weight for a neighbor based on boid count."""
        if neighbor in self.neighbors:
            weight = max(1.0, congestion_factor(boid_count))
            self.neighbors[neighbor] = weight
            neighbor.neighbors[self] = weight  # Ensure symmetry
#### end

    def get_traffic(self, neighbor):
        """Get the traffic weight for a neighbor."""
        return self.neighbors.get(neighbor, float('inf'))

    def __lt__(self, other):
        """Comparison method for priority queue."""
        return self.id < other.id
    
#### edit by chengpeng
    def __hash__(self):
        return hash(self.id)
    
    def __eq__(self, other):
        return self.id == other.id
#### end

class Graph:
    def __init__(self):
        self.nodes: List[Node] = []
        self.edges = []
#### edit by chengpeng
        self.boids_on_nodes: Dict[Node, int] = {}  # Track boid count on nodes
#### end

    def add_node(self, node: Node):
        self.nodes.append(node)
        self.boids_on_nodes[node] = 0  # Initialize boid count

    def get_node(self, id):
        for node in self.nodes:
            if node.id == id:
                return node

    def get_nearest_node(self, coord: Vect2D) -> Optional[Node]:
        nearest_node = None
        nearest_distance = float('inf')
        for node in self.nodes:
            distance = node.coord.get_distance_to(coord)
            if distance < nearest_distance:
                nearest_node = node
                nearest_distance = distance
        return nearest_node

#### edit by chengpeng
    def compute_shortest_path(self, source: Node, destination: Node) -> List[Node]:
        """Compute the shortest path from source to destination considering traffic weights."""
        distances = {node: float('inf') for node in self.nodes}
        previous_nodes = {node: None for node in self.nodes}
        distances[source] = 0

        priority_queue = [(0, source)]

        while priority_queue:
            current_distance, current_node = heappop(priority_queue)

            if current_node == destination:
                break

            if current_distance > distances[current_node]:
                continue

            for neighbor in current_node.neighbors:
                # Modified cost calculation to include traffic weight
                traffic_weight = current_node.get_traffic(neighbor)
                distance = current_distance + self._distance_between(current_node, neighbor) + traffic_weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heappush(priority_queue, (distance, neighbor))

        # Reconstruct the path from source to destination
        return self._reconstruct_path(previous_nodes, source, destination)

    def _reconstruct_path(self, previous_nodes, start_node, goal_node):
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = previous_nodes[current]
        path.reverse()
        if path[0] == start_node:
            return path
        else:
            return []
#### end

    def _distance_between(self, node1: Node, node2: Node) -> float:
        """Calculate base distance between two nodes."""
        return node1.coord.get_distance_to(node2.coord)

    def save_graph(self, filename):
        with open(filename, 'w') as f:
            for node in self.nodes:
                f.write(f"{node.id} {node.coord.x} {node.coord.y}\n")
            f.write("#\n")
            for node in self.nodes:
                for neighbor in node.neighbors:
                    if node.id < neighbor.id:
                        f.write(f"{node.coord.x} {node.coord.y} {neighbor.coord.x} {neighbor.coord.y}\n")

def generate_graph():
    graph = Graph()

    node1 = Node(Vect2D(10, 10), 1)
    node2 = Node(Vect2D(20, 20), 2)
    node3 = Node(Vect2D(20, 40), 3)
    node4 = Node(Vect2D(40, 40), 4)
    node5 = Node(Vect2D(40, 10), 5)
    node6 = Node(Vect2D(60, 20), 6)
    node7 = Node(Vect2D(60, 45), 7)

    node1.connect(node2)
    node1.connect(node3)
    node2.connect(node3)
    node2.connect(node4)
    node3.connect(node4)
    node2.connect(node5)
    node4.connect(node5)
    node4.connect(node6)
    node5.connect(node6)
    node6.connect(node7)
    node7.connect(node4)

    graph.add_node(node1)
    graph.add_node(node2)
    graph.add_node(node3)
    graph.add_node(node4)
    graph.add_node(node5)
    graph.add_node(node6)
    graph.add_node(node7)

    return graph
