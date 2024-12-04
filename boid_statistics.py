import pandas as pd

class BoidStatistics:
    def __init__(self, boid):
        self.boid = boid
        self.data = {}
        self.data["time"] = []
        self.data['position_x'] = []
        self.data['position_y'] = []
        self.data['velocity_x'] = []
        self.data['velocity_y'] = []
        self.data['acceleration_x'] = []
        self.data['acceleration_y'] = []


    def update(self, dt):
        self.data["time"].append(dt)
        self.data['position_x'].append(self.boid.pos.x)
        self.data['position_y'].append(self.boid.pos.y)
        self.data['velocity_x'].append(self.boid.vel.x)
        self.data['velocity_y'].append(self.boid.vel.y)
        self.data['acceleration_x'].append(self.boid.acc.x)
        self.data['acceleration_y'].append(self.boid.acc.y)

    def to_dataframe(self):
        return pd.DataFrame(self.data)
    
    @staticmethod
    def aggregate_statistics(boids):

        combined_df = pd.concat(
            [boid.statistics.to_dataframe().assign(boid_id=boid.id) for boid in boids],
            ignore_index=True
        )

        return combined_df
