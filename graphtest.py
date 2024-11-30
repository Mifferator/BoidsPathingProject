from __future__ import annotations
from pygame import Vector2
from typing import Dict, Optional, List
from heapq import heappush, heappop


class Node:
    def __init__(self, coord, id):
        self.id = id
        self.coord = coord
        self.neighbors = {}  # Store neighbors and their traffic weights
        self.routes = {}

    def connect(self, neighbor, base_cost=1.0):
        """Connect this node to a neighbor with a base cost."""
        if neighbor not in self.neighbors:
            self.neighbors[neighbor] = base_cost  # Initialize with base cost
            neighbor.connect(self, base_cost)

    def update_traffic(self, neighbor, change):
        """Update traffic weight for a neighbor."""
        if neighbor in self.neighbors:
            self.neighbors[neighbor] = max(1.0, self.neighbors[neighbor] + change)

    def get_traffic(self, neighbor):
        """Get the traffic weight for a neighbor."""
        return self.neighbors.get(neighbor, float('inf'))

    def get_next_node(self, destination_id):
        """Get the next node toward a destination."""
        return self.routes.get(destination_id)

    def __lt__(self, other):
        """Comparison method for priority queue."""
        return self.id < other.id


class Graph:
    def __init__(self):
        self.nodes: List[Node] = []
        self.edges = []

    def add_node(self, node: Node):
        self.nodes.append(node)

    def get_node(self, id):
        for node in self.nodes:
            if node.id == id:
                return node

    def get_nearest_node(self, coord: Vector2) -> Optional[Node]:
        nearest_node = None
        nearest_distance = float('inf')
        for node in self.nodes:
            distance = node.coord.distance_to(coord)
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

    def _compute_routes_from(self, source: Node):
        """Compute routes from a source node to all other nodes using Dijkstra's algorithm."""
        distances = {node: float('inf') for node in self.nodes}
        previous_nodes = {node: None for node in self.nodes}
        distances[source] = 0

        priority_queue = [(0, source)]

        while priority_queue:
            current_distance, current_node = heappop(priority_queue)

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

        # Save the best route from the source to all destinations
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
        """Calculate base distance between two nodes."""
        return node1.coord.distance_to(node2.coord)


def generate_graph():
    graph = Graph()

    node1 = Node(Vector2(10, 10), 1)
    node2 = Node(Vector2(20, 20), 2)
    node3 = Node(Vector2(20, 40), 3)
    node4 = Node(Vector2(40, 40), 4)
    node5 = Node(Vector2(40, 10), 5)
    node6 = Node(Vector2(60, 20), 6)
    node7 = Node(Vector2(60, 45), 7)

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

    return graph


if __name__ == "__main__":
    graph = Graph()

    node1 = Node(Vector2(0, 0), 1)
    node2 = Node(Vector2(1, 1), 2)
    node3 = Node(Vector2(1, 2), 3)
    node4 = Node(Vector2(2, 2), 4)
    node5 = Node(Vector2(2, 0), 5)

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