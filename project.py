# Import libraries
import os
import pygame
import sys
import random
import math
from graph import Node, Graph, generate_graph
from boid_statistic import BoidStatistics

# ==============================================================================
# SETTINGS FOR BOIDS     by David Gabriel
# ==============================================================================
# Set global variables
# constants
PI = math.pi

AVOID_PREDICTIVENESS = 1.0
AVOID_WEIGHT = 0.9
SEPARATION_WEIGHT = 0.8
ALIGNMENT_WEIGHT = 0.0
COHESION_WEIGHT = 0.0
TARGET_WEIGHT = 0.9
STEER_FORCE = 0.2
MIN_SPEED = 0.1 # m/s
MAX_SPEED = 10.0 # m/s
MIN_VIEW_DIST = 1 # m
VIEW_DIST = 10 # m
VIEW_ANGLE = 110 # degrees
DESTINATION_THRESHOLD = 0.5 # m
PASSTHROUGH_MULITPLIER = 4
COLLISION_THRESHOLD = 0.2 # m
SPAWN_RADIUS = 2.0 # m

SCALE = 10 # pixels / m
WIDTH=100 # m
HEIGHT=60 # m

BOID_COLOR = pygame.Color(0,0,255) #r,g,b
OBSTACLE_COLOR = pygame.Color(92,1,1) #r,g,b

STATISTICS_DIR = "boid_statistics"

# bool control variables
flocking = True
avoiding  = True
targeting = True

follow = False
debug = True
showNeighborVects = True
showProbe = False
showSeparationVect = True
showAlignmentVect = True
showCohesionVect = True
showTargetVect = True
showHeadings = True
showObstacles = True
showGraph = True


# ==============================================================================
# HELPER FUNCTIONS    by David Gabriel
# ==============================================================================
# Random functions
def get_random_int(min_val, max_val):
    return random.randint(min_val, max_val)

def get_random_float(min_val, max_val):
    return random.uniform(min_val, max_val)

# Function to check if two line segments intersect
def get_segment_intersection(p0, p1, p2, p3):
    a1 = p1.y - p0.y
    b1 = p0.x - p1.x
    c1 = a1 * p0.x + b1 * p0.y
    a2 = p3.y - p2.y
    b2 = p2.x - p3.x
    c2 = a2 * p2.x + b2 * p2.y
    denominator = a1 * b2 - a2 * b1

    if denominator == 0:
        return None  # lines are parallel

    x = (b2 * c1 - b1 * c2) / denominator
    y = (a1 * c2 - a2 * c1) / denominator
    ratio_x0 = (x - p0.x) / (p1.x - p0.x) if (p1.x - p0.x) != 0 else float('inf')
    ratio_y0 = (y - p0.y) / (p1.y - p0.y) if (p1.y - p0.y) != 0 else float('inf')
    ratio_x1 = (x - p2.x) / (p3.x - p2.x) if (p3.x - p2.x) != 0 else float('inf')
    ratio_y1 = (y - p2.y) / (p3.y - p2.y) if (p3.y - p2.y) != 0 else float('inf')

    if ((0 <= ratio_x0 <= 1 or 0 <= ratio_y0 <= 1) and
        (0 <= ratio_x1 <= 1 or 0 <= ratio_y1 <= 1)):
        return Vect2D(x, y)
    else:
        return None

# Function to check if a line segment intersects a circle
def line_circle_intersection(lp0, lp1, cp, r):
    # Check if either endpoint is inside the circle
    if point_circle_intersection(lp0, cp, r) or point_circle_intersection(lp1, cp, r):
        return True

    # Find the closest point on the line segment to the circle center
    length = lp0.get_distance_to(lp1)
    if length == 0:
        raise ValueError("Error: lp0 should not equal lp1:\n",lp0," == ",lp1)
    dot = ((cp.x - lp0.x) * (lp1.x - lp0.x) + (cp.y - lp0.y) * (lp1.y - lp0.y)) / (length ** 2)
    closest = Vect2D(lp0.x + dot * (lp1.x - lp0.x), lp0.y + dot * (lp1.y - lp0.y))

    # Check if the closest point lies on the segment
    if not line_point_intersection(lp0, lp1, closest):
        return False

    # Check if the distance to the closest point is less than the circle's radius
    dist = cp.get_distance_to(closest)
    return dist <= r


# Function to check if a point is inside a circle
def point_circle_intersection(p, cp, r):
    return p.get_distance_to(cp) <= r


# Function to check if a point lies on a line segment
def line_point_intersection(lp0, lp1, p):
    length = lp0.get_distance_to(lp1)
    d0 = p.get_distance_to(lp0)
    d1 = p.get_distance_to(lp1)

    buffer = 0.1  # tolerance for floating-point errors
    return (d0 + d1 >= length - buffer) and (d0 + d1 <= length + buffer)

# sample a point at random in a given radius around a given coordinate
def sample_point_in_circle(center: 'Vect2D', radius: float) -> "Vect2D":
    angle = get_random_float(0, 2 * PI)
    rad = get_random_float(0, radius)
    x = center.x + math.sin(angle) * rad
    y = center.y + math.cos(angle) * rad
    return Vect2D(x, y)

# ==============================================================================
# BOID CLASSES     by David Gabriel
# ==============================================================================
# VECT2D CLASS
class Vect2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def to_vector2(self) -> pygame.Vector2:
        return pygame.Vector2(self.x, self.y)
    
    @staticmethod
    def from_vector2(v: pygame.Vector2) -> 'Vect2D':
        return Vect2D(v.x, v.y)

    def set(self, x, y):
        self.x = x
        self.y = y

    def set_from_angle(self, magnitude, angle):
        self.x = magnitude*math.cos(angle)
        self.y = magnitude*math.sin(angle)

    def set_to_unit_vect(self):
        mag = self.get_magnitude()
        if mag != 0:
            self.x /= mag
            self.y /= mag
    
    def scale_to(self, scalar):
        self.set_to_unit_vect()
        self.x *= scalar
        self.y *= scalar

    def get_copy(self):
        return Vect2D(self.x, self.y)
    
    def get_magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    def get_distance_to(self, v):
        return math.sqrt((self.x - v.x) ** 2 + (self.y - v.y) ** 2)
    
    def get_angle_unit_vect_to(self, v):
        d = Vect2D(v.x - self.x, v.y - self.y)
        return d.get_unit_vect()
    
    def get_unit_vect(self):
        mag = self.get_magnitude()
        if mag != 0:
            return Vect2D(self.x / mag, self.y / mag)
        return Vect2D(0, 0)
    
    def get_draw_format(self):
        return (int(self.x*SCALE), int(self.y*SCALE))
    
    def cross(self, v):
        return self.x * v.y - self.y * v.x

    # Operator Overloading
    def __add__(self, v):
        return Vect2D(self.x + v.x, self.y + v.y)

    def __sub__(self, v):
        return Vect2D(self.x - v.x, self.y - v.y)

    def __mul__(self, other):
        if isinstance(other, Vect2D):
            return (self.x * other.x) + (self.y * other.y) # dot product
        return Vect2D(self.x * other, self.y * other)
    
    def __truediv__(self, scalar):
        if scalar == 0:
            raise ValueError("Cannot divide by zero.")
        return Vect2D(self.x / scalar, self.y / scalar)
    
    def __iadd__(self, v):
        self.x += v.x
        self.y += v.y
        return self
    
    def __isub__(self, v):
        self.x -= v.x
        self.y -= v.y
        return self

    def __imult__(self, scalar):
        self.x *= scalar
        self.y *= scalar
        return self

    def __itruediv__(self, scalar):
        self.x /= scalar
        self.y /= scalar
        return self

    def __neg__(self):
        return Vect2D(-self.x, -self.y)

    def __eq__(self, v):
        return self.x == v.x and self.y == v.y

    def __ne__(self, v):
        return not self.__eq__(v)
    
    def __repr__(self):
        return f"Vect2D({self.x}, {self.y})"

# OBSTACLE CLASSES
class Obstacle:
    def __init__(self, vertices):
        print("A New OBSTACLE Has Been Created.")
        self.vertices = vertices
        self.color = OBSTACLE_COLOR
    def edges(self):
        return [(self.vertices[i], self.vertices[(i+1) % len(self.vertices)]) for i in range(len(self.vertices))]

    def draw(self):
        points = [(v + camera_offset).get_draw_format() for v in self.vertices]
        pygame.draw.polygon(screen, self.color, points)

class Circular_Obstacle:
    def __init__(self, radius, pos):
        print("A New OBSTACLE Has Been Created.")
        self.pos = pos
        self.radius = radius
        self.color = OBSTACLE_COLOR

    def draw(self):
        offset_pos = self.pos + camera_offset
        pygame.draw.circle(screen, self.color, offset_pos.get_draw_format(), int(self.radius*SCALE))

# BOID CLASS
class Boid:
    def __init__(self,  
                    boid_id: int, 
                    destination: Node, 
                    destination_callback: callable = None, 
                    collision_callback: callable = None,
                    pos = Vect2D(get_random_int(0, 100), get_random_int(0, 60)),
                    boid_radius: float = 0.25,
                    selected=False):
        self.id = boid_id
        self.selected = selected
        self.heading = get_random_float(-PI, PI)
        self.speed = get_random_float(MIN_SPEED, MAX_SPEED)
        self.pos = pos
        self.vel = Vect2D(0, 0)
        self.vel.set_from_angle(self.speed, self.heading)
        self.acc = Vect2D(0, 0)
        self.min_speed = MIN_SPEED
        self.max_speed = MAX_SPEED
        self.maxSteerForce = STEER_FORCE*MAX_SPEED
        self.destination = destination
        self.target = None
        self.view_distance = VIEW_DIST
        self.prop_view_distance = max(MIN_VIEW_DIST, VIEW_DIST * (self.speed / MAX_SPEED))
        self.view_angle = math.radians(VIEW_ANGLE)
        self.neighbors = []
        self.radius = boid_radius
        self.color =  pygame.Color(0,0,0) if selected and debug else BOID_COLOR
        self.destination_callback = destination_callback
        self.collision_callback = collision_callback
        self.statistics = BoidStatistics(self)


    def set_target(self, target):
        self.target = target

    def is_on_node(self, node):
        return node.coord.get_distance_to(self.pos) < DESTINATION_THRESHOLD * PASSTHROUGH_MULITPLIER

    def get_steering_force(self, steer):
        # steering force = desired velocity - current velocity
        steer -= self.vel
        # bound steer force
        if steer.get_magnitude() > self.max_speed:
            steer.scale_to(self.max_speed)
        return steer

    def get_separation_vect(self):
        sum_vect = Vect2D(0, 0)
        for neighbor in self.neighbors:
            # add separation proportional to the inverse-square of the distance
            separation = self.pos - neighbor.pos
            distance = separation.get_magnitude()
            if distance < COLLISION_THRESHOLD:
                self.handle_collision(neighbor)
            separation.set_to_unit_vect()
            separation /= (distance ** 2) if distance != 0 else 1
            sum_vect += separation
        sum_vect.scale_to(self.max_speed)
        return self.get_steering_force(sum_vect)

    def get_alignment_vect(self):
        sum_vect = Vect2D(0, 0)
        for neighbor in self.neighbors:
            if neighbor.target == self.target:
                sum_vect += neighbor.vel
        sum_vect.scale_to(self.max_speed)
        return self.get_steering_force(sum_vect)

    def get_cohesion_vect(self):
        sum_vect = Vect2D(0, 0)
        for neighbor in self.neighbors:
            if neighbor.target == self.target:
                sum_vect += neighbor.pos
        sum_vect = (sum_vect / len(self.neighbors)) - self.pos
        sum_vect.scale_to(self.max_speed)
        return self.get_steering_force(sum_vect)
    
    def get_probe_end_pos(self):
        probe = self.vel
        if self.vel == Vect2D(0,0):
            probe = Vect2D(0, 0)
            probe.set_from_angle(self.heading, 10)
        probe.scale_to(self.prop_view_distance * AVOID_PREDICTIVENESS)
        end_pos = self.pos + probe
        return end_pos

    def get_avoid_bounds_vect(self):
        # Get the probe endpoint position
        end_pos = self.get_probe_end_pos()
        # Check for intersections with boundaries
        l_intersect = get_segment_intersection(end_pos, self.pos, Vect2D(0, 0), Vect2D(0, HEIGHT))
        r_intersect = get_segment_intersection(end_pos, self.pos, Vect2D(WIDTH, 0), Vect2D(WIDTH, HEIGHT))
        u_intersect = get_segment_intersection(end_pos, self.pos, Vect2D(0, 0), Vect2D(WIDTH, 0))
        d_intersect = get_segment_intersection(end_pos, self.pos, Vect2D(0, HEIGHT), Vect2D(WIDTH, HEIGHT))

        # Check for collisions and adjust desired velocity accordingly
        desired_vel = self.vel
        intersect = False
        if l_intersect:
            intersect = True
            desired_vel.x = self.max_speed
        if r_intersect:
            intersect = True
            desired_vel.x = -self.max_speed
        if u_intersect:
            intersect = True
            desired_vel.y = self.max_speed
        if d_intersect:
            intersect = True
            desired_vel.y = -self.max_speed

        # If there was an intersection, return the steering force
        if intersect:
            desired_vel.scale_to(self.max_speed)
            return self.get_steering_force(desired_vel)

        # If no intersection, return a zero vector
        return Vect2D(0, 0)
    
    def get_avoid_circular_obstacle_vect(self):
        # Get the end position of the probe
        end_pos = self.get_probe_end_pos()

        # Loop through the obstacles to find the closest one
        closest_obstacle = None
        min_dist = 0
        for obstacle in obstacles:
            if self.pos.get_distance_to(obstacle.pos) - obstacle.radius < COLLISION_THRESHOLD:
                self.handle_collision()
            # Check for intersection with the obstacle's circle (detects if the line intersects the obstacle's radius)
            if line_circle_intersection(self.pos, end_pos, obstacle.pos, obstacle.radius + self.radius):
                dist = self.pos.get_distance_to(obstacle.pos)
                if min_dist == 0 or dist < min_dist:
                    min_dist = dist
                    closest_obstacle = obstacle

        # If a closest obstacle is found
        if closest_obstacle is not None:
            # Get the closest point on the line between `self.pos` and `end_pos` relative to the obstacle
            length = self.pos.get_distance_to(end_pos)
            dot = ((closest_obstacle.pos.x - self.pos.x) * (end_pos.x - self.pos.x) + 
                   (closest_obstacle.pos.y - self.pos.y) * (end_pos.y - self.pos.y)) / (length ** 2)
            closest_point = Vect2D(self.pos.x + dot * (end_pos.x - self.pos.x), 
                                   self.pos.y + dot * (end_pos.y - self.pos.y))

            # Get the desired vector by subtracting the obstacle's position from the closest point
            desired_vec = closest_point - closest_obstacle.pos
            desired_vec.scale_to(self.max_speed)  # Scale to maximum velocity

            # Return the steering force to avoid the obstacle
            return self.get_steering_force(desired_vec)

        # If no obstacles, return zero vector
        return Vect2D(0, 0)
    
    def get_avoid_polygon_obstacle_vect(self):
        # Get the end position of the probe
        end_pos = self.get_probe_end_pos()

        # Loop through the all obstacles edges to find the closest one
        closest_obstacle = None
        intersection_point = None
        intersection_edge = None
        min_dist = 0
        for obstacle in obstacles:
            # Check for intersection with the obstacle's polygon
            for edge in obstacle.edges():
                p0, p1 = edge
                intersection = get_segment_intersection(self.pos, end_pos, p0, p1)
                if intersection:
                    dist = self.pos.get_distance_to(intersection)
                    if min_dist == 0 or dist < min_dist:
                        min_dist = dist
                        closest_obstacle = obstacle
                        intersection_point = intersection
                        intersection_edge = edge

        # If a closest obstacle is found
        if closest_obstacle is not None:
            # Use 90deg away from intersection as vector
            p0, p1 = intersection_edge
            edge_vector = p1 - p0
            perpendicular_vector = Vect2D(-edge_vector.y, edge_vector.x)  # 90 degrees CC
            # fix depending on what side Boid is on
            if (intersection_point - self.pos).cross(edge_vector) < 0:
                perpendicular_vector = perpendicular_vector * -1 

            perpendicular_vector.scale_to(self.max_speed)  # Scale to maximum velocity

            # Return the steering force to avoid the obstacle
            return self.get_steering_force(perpendicular_vector)

        # If no obstacles, return zero vector
        return Vect2D(0, 0)
    
    def get_avoid_obstacle_vect(self):
        # Get the end position of the probe
        end_pos = self.get_probe_end_pos()

        # Initialize variables for the closest obstacle and minimum distance
        closest_obstacle = None
        closest_point = None
        intersection_edge = None
        min_dist = float('inf')  # Start with a very large distance value

        # Loop through all obstacles (both circular and polygonal)
        for obstacle in obstacles:
            # --- Check Circular Obstacles ---
            if isinstance(obstacle, Circular_Obstacle):
                # Check for intersection with the obstacle's circle (detects if the line intersects the obstacle's radius)
                if self.pos.get_distance_to(obstacle.pos) - obstacle.radius < COLLISION_THRESHOLD:
                    self.collision_callback(self)
                if line_circle_intersection(self.pos, end_pos, obstacle.pos, obstacle.radius + self.radius):
                    dist = self.pos.get_distance_to(obstacle.pos)
                    if dist < min_dist:
                        min_dist = dist
                        closest_obstacle = obstacle

            # --- Check for Polygonal Obstacles ---
            elif isinstance(obstacle, Obstacle):
                for edge in obstacle.edges():
                    p0, p1 = edge
                    intersection = get_segment_intersection(self.pos, end_pos, p0, p1)
                    if intersection:
                        dist = self.pos.get_distance_to(intersection)
                        if dist < min_dist:
                            min_dist = dist
                            closest_obstacle = obstacle
                            closest_point = intersection
                            intersection_edge = edge

        # If a polygonal obstacle was found and has an intersection
        if closest_obstacle is not None:
            if isinstance(closest_obstacle, Circular_Obstacle):
                length = self.pos.get_distance_to(end_pos)
                dot = ((closest_obstacle.pos.x - self.pos.x) * (end_pos.x - self.pos.x) + 
                    (closest_obstacle.pos.y - self.pos.y) * (end_pos.y - self.pos.y)) / (length ** 2)
                closest_point = Vect2D(self.pos.x + dot * (end_pos.x - self.pos.x), 
                    self.pos.y + dot * (end_pos.y - self.pos.y))

                # Get the desired vector by subtracting the obstacle's position from the closest point
                desired_vec = closest_point - closest_obstacle.pos
                desired_vec.scale_to(self.max_speed)  # Scale to maximum velocity

                # Return the steering force to avoid the obstacle
                return self.get_steering_force(desired_vec)
            
            # Use 90 degrees away from the intersection as the avoidance vector for polygonal obstacles
            p0, p1 = intersection_edge
            edge_vector = p1 - p0
            perpendicular_vector = Vect2D(-edge_vector.y, edge_vector.x)  # Perpendicular vector (90 degrees CC)
            if (closest_point - self.pos).cross(edge_vector) < 0:
                perpendicular_vector = perpendicular_vector * -1

            perpendicular_vector.scale_to(self.max_speed)
            return self.get_steering_force(perpendicular_vector)

        # If no obstacles were encountered, return a zero vector (no avoidance needed)
        return Vect2D(0, 0)

    def get_target_vect(self):
        target = Vect2D.from_vector2(self.target.coord) - self.pos
        dist = target.get_magnitude()
        # Slow down if the boid is within the view distance (arrival behavior)
        if dist < self.view_distance:
            mag = (dist * self.max_speed) / self.view_distance
            target.scale_to(mag)
        else:
            target.scale_to(self.max_speed)
        return self.get_steering_force(target)

    def handle_collision(self, other=None):
        if other is not None:
            self.collision_callback(self, other)
        else:
            self.collision_callback(self)

    def update(self, dt):
        self.acc.set(0, 0)

        # Avoidance force (walls, obstacles, etc.)
        if avoiding:
            avoid_bounds_force = self.get_avoid_bounds_vect() * AVOID_WEIGHT
            avoid_obstacle_force = self.get_avoid_obstacle_vect() * AVOID_WEIGHT
            self.acc += avoid_bounds_force + avoid_obstacle_force

        # Steering forces (separation, alignment, cohesion)
        if len(self.neighbors) > 0 and flocking:
            separation_force = self.get_separation_vect() * SEPARATION_WEIGHT
            alignment_force = self.get_alignment_vect() * ALIGNMENT_WEIGHT
            cohesion_force = self.get_cohesion_vect() * COHESION_WEIGHT

            self.acc += separation_force + alignment_force + cohesion_force
            
        # Targeting
        if targeting:
# GAUTAM AND CHENGPEND VERSION ******************************************          
#  JOSH VERSION *********************************************************
            # threshold = DESTINATION_THRESHOLD if self.target.coord == self.destination.coord else DESTINATION_THRESHOLD * PASSTHROUGH_MULITPLIER
            # if self.target.coord.get_distance_to(self.pos) < threshold:
            #     if self.target.id == self.destination.id:
            #         self.destination_callback(self)
            #         return
            #     self.target = self.target.get_next_node(self.destination.id)
# END JOSH VERSION ******************************************************      

            target_force = self.get_target_vect() * TARGET_WEIGHT
            self.acc += target_force

        # Update velocity and position
        self.vel += self.acc * dt
        self.speed = self.vel.get_magnitude()
        if self.speed != 0:
            self.heading = math.atan2(self.vel.y, self.vel.x)
        if self.speed < self.min_speed:
            self.vel.scale_to(self.min_speed)
            self.speed = self.min_speed
        self.prop_view_distance = max(MIN_VIEW_DIST, self.view_distance * (self.speed / self.max_speed))

        self.pos += self.vel * dt
        # Wrap around screen edges or clamp....
        self.pos.set(self.pos.x % WIDTH, self.pos.y % HEIGHT)
        # self.pos.set(max(0, min(WIDTH, self.pos.x)), max(0, min(HEIGHT, self.pos.y)))
        self.statistics.update(dt)

    def draw_vector(self, v, color):
        pygame.draw.line(screen, color, self.pos, v)

    def draw(self):
        offset_pos = self.pos + camera_offset

        if self.selected and debug:
            # neighborhood
            pygame.draw.circle(screen, pygame.Color("#bbbbbb"), offset_pos.get_draw_format(), self.prop_view_distance*SCALE)

            if len(self.neighbors) > 0:
                if showNeighborVects:
                    for neighbor in self.neighbors:
                        pygame.draw.line(screen, self.color, offset_pos.get_draw_format(), neighbor.pos.get_draw_format())
                
                # acc
                v = self.acc + offset_pos
                pygame.draw.line(screen, pygame.Color("#f2a93a"), offset_pos.get_draw_format(), v.get_draw_format())

                # steering vectors
                if showSeparationVect:
                    v = self.get_separation_vect() + offset_pos
                    pygame.draw.line(screen, pygame.Color("#0077b6"), offset_pos.get_draw_format(), v.get_draw_format())
                if showAlignmentVect:
                    v = self.get_alignment_vect() + offset_pos
                    pygame.draw.line(screen, pygame.Color("#00b4d8"), offset_pos.get_draw_format(), v.get_draw_format())
                if showCohesionVect:
                    v = self.get_cohesion_vect() + offset_pos
                    pygame.draw.line(screen, pygame.Color("#90e0ef"), offset_pos.get_draw_format(), v.get_draw_format())

        # headings
        if showHeadings:
            v = self.vel/2 + offset_pos
            pygame.draw.line(screen, self.color, offset_pos.get_draw_format(), v.get_draw_format())
        # boid
        pygame.draw.circle(screen, self.color, offset_pos.get_draw_format(), int(self.radius*SCALE))

# FLOCK CLASS
class Flock:
    def __init__(self, num_boids: int, graph: Graph):
        print("A New FLOCK Has Been Created.")
        self.selected = [0]
        self.boids = []
        self.collided = []
        self.arrived = []
        self.num_boids = num_boids
        self.boid_cntr_pos = Vect2D(0, 0)
        self.graph = graph
        self.collisions = 0
        self.completed_routes = 0

        self.populate()

    def destination_callback(self, boid):
        if boid not in self.arrived:
            self.arrived.append(boid)
        if boid in self.boids:
            self.boids.remove(boid)
        self.completed_routes += 1

    def collision_callback(self, boid1, boid2=None):
        self.collisions += 1
        if boid1 not in self.collided:
            self.collided.append(boid1)
        if boid1 in self.boids:
            self.boids.remove(boid1)
        if boid2 is not None and boid2 in self.boids:
            self.boids.remove(boid2)

    def populate(self):
        for i in range(self.num_boids):

# GAUTAM AND CHENGPENG VERSION ****************************************
            selected = i in self.selected
            destination = random.choice(self.graph.nodes)
            boid = Boid(0.25, int(i), destination, self.graph, self.destination_callback, self.collision_callback, selected)
# JOSH VERSION *********************************************************
            # start = random.choice(self.graph.nodes)
            # possible_dests = self.graph.nodes.copy()
            # possible_dests.remove(start)
            # destination = random.choice(possible_dests)
            
            # pos = sample_point_in_circle(start.coord, SPAWN_RADIUS)

            # boid = Boid(boid_id = int(i), 
            #             destination = destination, 
            #             destination_callback = self.destination_callback, 
            #             collision_callback = self.collision_callback,
            #             pos = pos)
# END *******************************************************************
            nearest_node = self.graph.get_nearest_node(boid.pos)

            boid.set_target(nearest_node)
            self.boids.append(boid)

    def update(self, dt):
        self.boid_cntr_pos.set(0, 0)
        # for each boid, find neighborhood and update
        for i, boid in enumerate(self.boids):
            neighbors = []
            for j, other_boid in enumerate(self.boids):
                if j != i:
                    # test if flockmate is within neighborhood
                    dist = boid.pos.get_distance_to(other_boid.pos)
                    if dist < boid.prop_view_distance:
                        unit_vect = boid.pos.get_angle_unit_vect_to(other_boid.pos)
                        dot_product = max(-1, min(1, (unit_vect * boid.vel.get_unit_vect())))
                        angle = math.acos(dot_product)
                        if angle < boid.view_angle:
                            neighbors.append(other_boid)
            boid.neighbors = neighbors
            boid.update(dt)
            self.boid_cntr_pos += boid.pos

        # caclualte flock center and camera offset
        self.boid_cntr_pos /= self.num_boids
        camera_offset = Vect2D(screen.get_width() / (2*SCALE), screen.get_height() / (2*SCALE))
        if debug and follow:
            camera_offset -= self.boids[self.selected[0].pos]
        elif follow:
            camera_offset -= self.boid_cntr_pos
        else:
            camera_offset.set(0, 0)

        if len(self.boids) < 1:
            self.finalize()
            return

    def finalize(self):
        print("Simulation complete!")
        if not os.path.exists(STATISTICS_DIR):
            os.makedirs(STATISTICS_DIR)
        df = BoidStatistics.aggregate_statistics(self.collided + self.arrived)
        df.to_csv(os.path.join(STATISTICS_DIR, "boid_statistics.csv"))
        print(f"Statistics saved to '{STATISTICS_DIR}/boid_statistics.csv'")
        self.graph.save_graph(os.path.join(STATISTICS_DIR, "graph.txt"))
        print(f"Graph saved to '{STATISTICS_DIR}/graph.txt'")
        pygame.quit()
        sys.exit()


    def draw(self):
        for boid in self.boids:
            boid.draw()

# ==============================================================================
# BOID SIMULATION     by David Gabriel
# ==============================================================================
# SETUP GLOBAL VARIABLES
pygame.init()
screen = pygame.display.set_mode((WIDTH*SCALE, HEIGHT*SCALE))
pygame.display.set_caption("BOIDs")
clock = pygame.time.Clock()
camera_offset = Vect2D(0, 0)

graph = generate_graph()

flock = Flock(num_boids=100, graph=graph)
obstacles = []
obstacles.append(Circular_Obstacle(1, Vect2D(WIDTH / 2, HEIGHT / 2)))
obstacles.append(Obstacle([Vect2D(32,39),  Vect2D(30,42), Vect2D(30,42), Vect2D(35,40),]))

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
font = pygame.font.SysFont('Arial', 15)

# MAIN SIMULATION LOOP
running = True
while running:
    # EVENT HANDLING
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # UPDATE
    dt = clock.tick(60) / 1000 # limit to 60fps
    flock.update(dt)

    # RENDER
    screen.fill(WHITE)
    flock.draw()
    if showObstacles:
        for obstacle in obstacles:
            obstacle.draw()

    if showGraph:
        for node in graph.nodes:
            pygame.draw.circle(screen, RED, Vect2D.from_vector2(node.coord).get_draw_format(), 3)
            for neighbor in node.neighbors:
                pygame.draw.line(screen, RED, Vect2D.from_vector2(node.coord).get_draw_format(), Vect2D.from_vector2(neighbor.coord).get_draw_format())
    # FPS text
    if debug:
        fps = clock.get_fps()
        fps_text = font.render(f"FPS: {fps:.2f}", True, RED)
        completed_text = font.render(f"Completed Routes: {flock.completed_routes}", True, RED)
        collisions_text = font.render(f"Collisions: {flock.collisions}", True, RED)
        screen.blit(fps_text, (10, 10))
        screen.blit(completed_text, (10, 30))
        screen.blit(collisions_text, (10, 50))
    pygame.display.flip()

# Quit pygame
pygame.quit()
sys.exit()
