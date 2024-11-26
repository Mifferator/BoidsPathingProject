import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import os
from tqdm import tqdm

DEST_DIR = "boid_plots"

def load_data(filename):
    return pd.read_csv(filename)

def plot_boid_path(df, boid_id):
    boid_df = df[df['boid_id'] == boid_id].copy()

    boid_df.loc[:, 'speed'] = np.sqrt(boid_df['velocity_x']**2 + boid_df['velocity_y']**2)

    norm = plt.Normalize(boid_df['speed'].min(), boid_df['speed'].max())
    cmap = cm.magma

    plt.clf()
    fig, ax = plt.subplots()

    for i in range(len(boid_df) - 1):
        x = boid_df['position_x'].iloc[i:i+2]
        y = boid_df['position_y'].iloc[i:i+2]
        speed = boid_df['speed'].iloc[i]
        ax.plot(
            x, y, color=cmap(norm(speed))
        )

    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    plt.colorbar(sm, ax=ax, label="Speed")

    plt.title(f"Boid Path (ID: {boid_id})")
    plt.xlabel("Position X")
    plt.ylabel("Position Y")
    plt.savefig(os.path.join(DEST_DIR, f"boid_{boid_id}.png"), dpi=300)
    plt.close(fig)

if __name__ == "__main__":
    df = load_data("boid_statistics.csv")
    boids = df['boid_id'].unique().tolist()
    if not os.path.exists(DEST_DIR):
        os.makedirs(DEST_DIR)
    for boid in tqdm(boids):
        plot_boid_path(df, boid)
        pass