# ==============================================================================
# VECT2D CLASS     by David Gabriel
# ==============================================================================
import math
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

