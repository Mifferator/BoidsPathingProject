# ==============================================================================
# VECT2D CLASS     by David Gabriel
# ==============================================================================
import math, random

class Vect2D:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

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
    
    def get_draw_format(self, scale):
        return (int(self.x*scale), int(self.y*scale))
    
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
    angle = get_random_float(0, 2 * math.pi)
    rad = get_random_float(0, radius)
    x = center.x + math.sin(angle) * rad
    y = center.y + math.cos(angle) * rad
    return Vect2D(x, y)