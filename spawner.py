from abc import ABC, abstractmethod
from vect2d import Vect2D
import random, math
from typing import List
from enum import Enum

class SpawningStyle(Enum):
    RANDOM = 0
    NODE = 1

class Spawner(ABC):

    MAX_ATTEMPTS = 1000 # Maximum number of attempts to spawn a point

    @property
    @abstractmethod
    def obstacles(self) -> List[Vect2D]:
        pass

    @property
    @abstractmethod
    def spawned(self) -> List[Vect2D]:
        pass

    @property
    @abstractmethod
    def min_distance(self) -> int:
        pass

    @abstractmethod
    def generate_point(self) -> Vect2D:
        pass

    def _is_valid(self, point: Vect2D) -> bool:
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return False
        for spawned in self.spawned:
            if point.get_distance_to(spawned) < self.min_distance:
                return False

    def spawn(self) -> Vect2D:
        for _ in range(self.MAX_ATTEMPTS):
            point = self.generate_point()
            if self._is_valid(point):
                self.spawned.append(point)
                return point
        return self.generate_point() # give up and return a bad point


class RandomSpawner(Spawner):
    def __init__(self, x_range: tuple, y_range: tuple, min_distance: int, obstacles: list):
        self.x_range = x_range
        self.y_range = y_range
        self._min_distance = min_distance
        self._spawned = []
        self._obstacles = obstacles

    @property
    def spawned(self) -> List[Vect2D]:
        return self._spawned
    
    @property
    def min_distance(self) -> int:
        return self._min_distance
    
    @property
    def obstacles(self) -> list:
        return self._obstacles

    def generate_point(self) -> Vect2D:
        return Vect2D(random.randint(self.x_range[0], self.x_range[1]), random.randint(self.y_range[0], self.y_range[1]))
    
class NodeSpawner(Spawner):
    def __init__(self, node_coords: List[Vect2D], radius: int, min_distance: int, obstacles: list):
        self.node_coords = node_coords
        self.radius = radius
        self._min_distance = min_distance
        self._spawned = []
        self._obstacles = obstacles

    @property
    def spawned(self) -> List[Vect2D]:
        return self._spawned
    
    @property
    def min_distance(self) -> int:
        return self._min_distance
    
    @property
    def obstacles(self) -> list:
        return self._obstacles

    def generate_point(self) -> Vect2D:
        center = random.choice(self.node_coords)
        angle = random.uniform(0, 2 * math.pi)
        rad = random.randint(0, self.radius)
        x = center.x + math.sin(angle) * rad
        y = center.y + math.cos(angle) * rad
        return Vect2D(x, y)
