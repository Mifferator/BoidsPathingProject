import heapq
import math
import numpy as np

class Robot:
    def __init__(self, robot_id, start_position):
        self.robot_id = robot_id
        self.start_position = start_position
        self.visited_nodes = {start_position}  # history of rob visited
        self.path = [start_position]  # the path from start

    def visit_node(self, node):
        self.visited_nodes.add(node)
        self.path.append(node)

class DijkstraWithTraffic:
    def __init__(self, graph):
        self.graph = graph

    def congestion_factor(self, robot_count):# calculate the cost
        return math.exp(robot_count)-1    

    def plan_path(self, start, goal, robot, robots_on_nodes):#Dijkstra
        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0
        previous_nodes = {node: None for node in self.graph}
        priority_queue = [(0, start)]  # (distance, node)

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_node == goal:
                break

            if current_distance > distances[current_node]:
                continue

            for neighbor, weight in self.graph[current_node]:
                if neighbor in robot.visited_nodes:
                    continue

                extra_cost = self.congestion_factor(robots_on_nodes.get(neighbor, 0))
                total_cost = current_distance + weight + extra_cost

                if total_cost < distances[neighbor]:
                    distances[neighbor] = total_cost
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (total_cost, neighbor))

        return self.reconstruct_path(previous_nodes, start, goal)

    def reconstruct_path(self, previous_nodes, start, goal):#get the path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous_nodes[current]
        path.reverse()
        if path[0] == start :# robot start not in goal
            return path
        else: 
            return[]

class MultiRobotSimulator:
    def __init__(self, graph, start_positions, goal):
        self.graph = graph
        self.start_positions = start_positions
        self.goal = goal

        self.total_robots = len(start_positions)
        self.robot_positions = start_positions[:]
        self.robots_on_nodes = {node: 0 for node in graph}
        self.robots = [Robot(robot_id, start) for robot_id, start in enumerate(start_positions)]

        self.path_planner = DijkstraWithTraffic(graph)

    def simulate(self):
        completed = [False] * self.total_robots

        # init number of rob
        for pos in self.robot_positions:
            self.robots_on_nodes[pos] += 1

        while not all(completed):
            #upgrade the path in time
            for robot in self.robots:
                if completed[robot.robot_id]:
                    continue

                current_position = self.robot_positions[robot.robot_id]
                path = self.path_planner.plan_path(current_position, self.goal, robot, self.robots_on_nodes)

                if not path:
                    print(f"Robot {robot.robot_id} cannot reach the goal.")
                    completed[robot.robot_id] = True
                    continue

                # move to next node and upgrade position
                if len(path) > 1:
                    next_step = path[1]
                    robot.visit_node(next_step)
                else:
                    next_step = self.goal

                self.update_robot_position(robot.robot_id, current_position, next_step)

                if next_step == self.goal:
                    completed[robot.robot_id] = True

    def update_robot_position(self, robot_id, current_position, next_position):      
        self.robots_on_nodes[current_position] -= 1        
        self.robots_on_nodes[next_position] += 1   
        self.robot_positions[robot_id] = next_position


if __name__ == "__main__":
    # define the grah：node -> [(neighbor, cost for path)]
    graph = {
        0: [(1, 4), (2, 1)],
        1: [(0, 4), (3, 1)],
        2: [(0, 1), (3, 1), (4, 2)],
        3: [(1, 1), (2, 1), (4, 3), (5, 5)],
        4: [(2, 2), (3, 3), (5, 3)],
        5: [(3, 5), (4, 3)]
    }

    start_node = np.random.randint(0,6,size = 50)
    goal_node = 5

    simulator = MultiRobotSimulator(graph, start_node, goal_node)
    simulator.simulate()

    # print the result
    print("\nAll Robots' Paths:")
    for robot in simulator.robots:
        print(f"Robot {robot.robot_id}: {robot.path}")
