from __future__ import annotations
from typing import Dict, Optional, List
from heapq import heappush, heappop
from vect2d import Vect2D, get_segment_intersection
import random

class Node:
    def __init__(self, coord: Vect2D, id: int):
        self.id = id
        self.coord = coord
        self.neighbors: Dict[Node, float] = {}  # Use a dictionary to store neighbors and traffic weights
        self.routes: Dict[int, Node] = {}

    def connect(self, neighbor: Node, base_cost: float = 1.0):
        """Connect this node to a neighbor with a base cost."""
        if neighbor not in self.neighbors:
            self.neighbors[neighbor] = base_cost  # Initialize with base cost
            neighbor.connect(self, base_cost)

    def update_traffic(self, neighbor: Node, change: float):
        """Update traffic weight for a neighbor."""
        if neighbor in self.neighbors:
            # Update traffic weight and ensure it doesn't drop below base cost
            self.neighbors[neighbor] = max(1.0, self.neighbors[neighbor] + change)

    def get_traffic(self, neighbor: Node) -> float:
        """Get the traffic weight for a neighbor."""
        return self.neighbors.get(neighbor, float('inf'))

    def get_coord(self) -> Vect2D:
        return self.coord

    def get_next_node(self, destination_id: int) -> Optional[Node]:
        """Get the next node toward a destination."""
        return self.routes.get(destination_id)

    def __lt__(self, other: Node):
        """Comparison method for priority queue."""
        return self.id < other.id

class Graph:
    def __init__(self):
        self.nodes: List[Node] = []
        self.edges = []

    @staticmethod
    def generate_random_graph(num_nodes: int, width: int, height: int, padding: float, min_edges_per_node: int, max_edges_per_node: int) -> Graph:
        graph = Graph()
        for i in range(num_nodes):
            coord = Vect2D(random.randint(width * padding, width*(1 - padding)), random.randint(height * padding, height*(1 - padding)))
            node = Node(coord, i)
            graph.add_node(node)
        for node in graph.nodes:
            num_edges = random.randint(min_edges_per_node, max_edges_per_node)
            local_nodes = graph._get_local_nodes(node)
            for local_node in local_nodes:
                print("here")
                intersecting = False
                if local_node not in node.neighbors:
                    for edge in graph.edges:
                        intersection = get_segment_intersection(node.coord, local_node.coord, edge[0].coord, edge[1].coord)
                        if intersection is not None and intersection != node.coord and intersection != local_node.coord:
                            intersecting = True
                            break
                    if not intersecting:
                        node.connect(local_node)
                        graph.edges.append((node, local_node))
                if len(node.neighbors) >= num_edges:
                    break

        graph.edges = graph._get_edges()
        return graph

    def add_node(self, node: Node):
        self.nodes.append(node)

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

    def get_edge(self, id):
        for edge in self.edges:
            if edge.id == id:
                return edge
            
    def run_dijkstra(self):
        for source in self.nodes:
            self._compute_routes_from(source)

    def save_graph(self, filename):
        with open(filename, 'w') as f:
            for node in self.nodes:
                f.write(f"{node.id} {node.coord.x} {node.coord.y}\n")
            f.write("#\n")
            for node in self.nodes:
                for neighbor in node.neighbors:
                    if node.id < neighbor.id:
                        f.write(f"{node.coord.x} {node.coord.y} {neighbor.coord.x} {neighbor.coord.y}\n")

    def _get_edges(self):
        edges = []
        for node in self.nodes:
            for neighbor, _ in node.neighbors.items():
                if node.id < neighbor.id:
                    edges.append((node, neighbor))
        return edges

    def _compute_routes_from(self, source: Node):
        distances = {node: float('inf') for node in self.nodes} 
        previous_nodes = {node: None for node in self.nodes}
        distances[source] = 0

        priority_queue = [(0, source)]

        while priority_queue:
            current_distance, current_node = heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor in current_node.neighbors:
                distance = current_distance + self._distance_between(current_node, neighbor)
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heappush(priority_queue, (distance, neighbor))

        for destination in self.nodes:
            if destination == source:
                continue
            next_hop = self._get_next_hop(destination, previous_nodes)
            if next_hop:
                source.routes[destination.id] = next_hop

    def _get_next_hop(self, destination: Node, previous_nodes: Dict[Node, Optional[Node]]) -> Optional[Node]:
        current = destination
        while previous_nodes[current] and previous_nodes[previous_nodes[current]]:
            if previous_nodes[previous_nodes[current]] is None:
                return current
            current = previous_nodes[current]
        return current

    def _distance_between(self, node1: Node, node2: Node) -> float:
        return node1.coord.get_distance_to(node2.coord)

    def _is_connected(self):
        for node in self.nodes:
            if not node.neighbors:
                return False
        return True

    # return n nearest nodes to node not including node
    def _get_local_nodes(self, node, n=None) -> Optional[Node]:
        if n is None:
            n = len(self.nodes) - 1
        rest = self.nodes.copy()
        rest.remove(node)
        nodes = sorted(rest, key=lambda n: node.coord.get_distance_to(n.coord))
        return nodes[:n]

def generate_graph():
    """ graph = Graph()

    node1 = Node(Vect2D(10, 10), 1)
    node2 = Node(Vect2D(20, 20), 2)
    node3 = Node(Vect2D(20, 40), 3)
    node4 = Node(Vect2D(40, 40), 4)
    node5 = Node(Vect2D(40, 10), 5)
    node6 = Node(Vect2D(60, 20), 6)
    node7 = Node(Vect2D(60, 45), 7)

    node1.connect(node2)
    node2.connect(node3)
    node3.connect(node4)
    node2.connect(node5)
    node4.connect(node5)
    node4.connect(node6)
    node5.connect(node6)
    node6.connect(node7)

    graph.add_node(node1)
    graph.add_node(node2)
    graph.add_node(node3)
    graph.add_node(node4)
    graph.add_node(node5)
    graph.add_node(node6)
    graph.add_node(node7)

    graph.run_dijkstra()
    for node in graph.nodes:
        print(f"Node {node.id} has routes to:")
        for destination_id, next_hop in node.routes.items():
            print(f"  Node {destination_id} via Node {next_hop.id}")

    return graph """
    graph = Graph.generate_random_graph(15, 100, 60, 0.1, 2, 4)
    graph.run_dijkstra()
    return graph

if __name__ == "__main__":
    graph = Graph()

    node1 = Node(Vect2D(0, 0), 1)
    node2 = Node(Vect2D(1, 1), 2)
    node3 = Node(Vect2D(1, 2), 3)
    node4 = Node(Vect2D(2, 2), 4)
    node5 = Node(Vect2D(2, 0), 5)

    node1.connect(node2)
    node2.connect(node3)
    node3.connect(node4)
    node2.connect(node5)
    node4.connect(node5)

    graph.add_node(node1)
    graph.add_node(node2)
    graph.add_node(node3)
    graph.add_node(node4)
    graph.add_node(node5)

    graph.run_dijkstra()

    for node in graph.nodes:
        print(f"Node {node.id} has routes to:")
        for destination_id, next_hop in node.routes.items():
            print(f"  Node {destination_id} via Node {next_hop.id}")