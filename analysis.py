import pandas as pd
import seaborn as sb
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import os
from tqdm import tqdm

DEST_DIR = "boid_plots"
SOURCE_DIR = "boid_statistics"

def load_data(filename):
    return pd.read_csv(filename)

def plot_graph(plt, graph_file):
    nodes = {}
    edges = []
    
    with open(graph_file, 'r') as f:
        lines = f.readlines()

        i = 0
        while lines[i].strip() != '#':
            node_id, x, y = lines[i].strip().split()
            nodes[node_id] = (float(x), float(y))
            i += 1
        
        i += 1
        
        while i < len(lines):
            x1, y1, x2, y2 = map(float, lines[i].strip().split())
            edges.append(((x1, y1), (x2, y2)))
            i += 1

    for node_id, (x, y) in nodes.items():
        plt.scatter(x, y, color='black', alpha=0.2)
        plt.text(x + 1, y, node_id, color='black', alpha=0.2)

    for (x1, y1), (x2, y2) in edges:
        plt.plot([x1, x2], [y1, y2], color='black', alpha=0.2)

def plot_boid_path(df, boid_id):
    boid_df = df[df['boid_id'] == boid_id].copy()

    boid_df.loc[:, 'speed'] = np.sqrt(boid_df['velocity_x']**2 + boid_df['velocity_y']**2)

    norm = plt.Normalize(boid_df['speed'].min(), boid_df['speed'].max())
    cmap = cm.magma

    plt.clf()
    fig, ax = plt.subplots()

    plot_graph(plt, os.path.join(SOURCE_DIR, "graph.txt"))

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

def plot_all_boid_paths(df):
    boids = df['boid_id'].unique().tolist()
    for boid in tqdm(boids):
        plot_boid_path(df, boid)

def plot_heat_map(df):
    plt.clf()
    fig, ax = plt.subplots()

    plot_graph(plt, os.path.join(SOURCE_DIR, "graph.txt"))

    x, y = zip(*df[['position_x', 'position_y']].values)
    ax.hist2d(x, y, bins=450, cmap='inferno')
    plt.colorbar(label="Boid Density")

    plt.title("Boid Density Heatmap")
    plt.xlabel("Position X")
    plt.ylabel("Position Y")
    plt.savefig(os.path.join(DEST_DIR, "boid_density.png"), dpi=300)
    plt.close(fig)

def plot_speed_hist(df):
    plt.clf()
    df.loc[:, 'speed'] = np.sqrt(df['velocity_x']**2 + df['velocity_y']**2)
    sb.histplot(df['speed'], color='blue', kde=True, line_kws={'linewidth': 1}, ec=None)
    plt.title("Boid Speed Histogram")
    plt.xlabel("Speed (m/s)")
    plt.ylabel("Frequency")
    plt.savefig(os.path.join(DEST_DIR, "boid_speed_hist.png"), dpi=300)

if __name__ == "__main__":
    df = load_data(os.path.join(SOURCE_DIR, "boid_statistics.csv"))
    boids = df['boid_id'].unique().tolist()
    if not os.path.exists(DEST_DIR):
        os.makedirs(DEST_DIR)
    
    plot_all_boid_paths(df)
    plot_heat_map(df)
    plot_speed_hist(df)