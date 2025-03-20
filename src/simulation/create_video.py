import matplotlib
matplotlib.use('Agg')  # Use the faster backend
import matplotlib.pyplot as plt
import numpy as np
import argparse
import json
import os
import multiprocessing
from tqdm import tqdm
from functools import partial
from numba import jit, prange

# Numba-optimized function to update trajectory lines
@jit(nopython=True, parallel=True)
def update_trajectories(trajectories, frame_idx, x_data, y_data):
    """
    Update trajectory lines for all agents up to the current frame.
    """
    num_agents = trajectories.shape[0]
    for i in prange(num_agents):  # Parallelize across agents
        x_data[i, :frame_idx + 1] = trajectories[i, :frame_idx + 1, 0]
        y_data[i, :frame_idx + 1] = trajectories[i, :frame_idx + 1, 1]

# Frame generation function
def generate_frame(frame_idx, trajectories, output_dir, x_data, y_data, world_size_x, world_size_y):
    """
    Generate and save a single frame.
    """
    # Create a new figure and axes for each process
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, world_size_x)
    ax.set_ylim(0, world_size_y)
    scatter = ax.scatter([], [], s=50)
    lines = [ax.plot([], [], '-', alpha=0.5)[0] for _ in range(trajectories.shape[0])]

    # Update scatter plot data for the current positions of agents
    current_positions = trajectories[:, frame_idx, :]
    scatter.set_offsets(current_positions)

    # Update trajectory lines using Numba-optimized function
    update_trajectories(trajectories, frame_idx, x_data, y_data)

    # Update lines in the Matplotlib plot
    for i, line in enumerate(lines):
        line.set_data(x_data[i, :frame_idx + 1], y_data[i, :frame_idx + 1])

    # Save the updated plot as a frame
    fig.savefig(f"{output_dir}/frame_{frame_idx:04d}.png")

    # Close the figure to release memory
    plt.close(fig)

def main():
    parser = argparse.ArgumentParser(description='Create video of agent trajectories')
    parser.add_argument('--data_file', type=str, required=True, help='JSON file containing trajectory data')
    parser.add_argument('--output', type=str, required=True, help='Output video filename')
    parser.add_argument('--world_size_x', type=float, required=True, help='World size in X dimension')
    parser.add_argument('--world_size_y', type=float, required=True, help='World size in Y dimension')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second for video')
    args = parser.parse_args()

    # Prepare output directory
    frames_dir = "simulation_frames"
    os.makedirs(frames_dir, exist_ok=True)

    # Load trajectory data
    with open(args.data_file, 'r') as f:
        trajectories = np.array(json.load(f))
    num_frames = trajectories.shape[1]
    num_agents = trajectories.shape[0]

    # Preallocate arrays for trajectory data (to avoid reallocation in every frame)
    x_data = np.zeros((num_agents, num_frames))
    y_data = np.zeros((num_agents, num_frames))

    # Multiprocessing pool
    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        generate_partial = partial(
            generate_frame,
            trajectories=trajectories,
            output_dir=frames_dir,
            x_data=x_data,
            y_data=y_data,
            world_size_x=args.world_size_x,
            world_size_y=args.world_size_y
        )
        list(tqdm(pool.imap_unordered(generate_partial, range(num_frames)), total=num_frames, desc="Generating frames"))

    # Create video
    os.system(f"ffmpeg -framerate {args.fps} -i {frames_dir}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p {args.output}")

if __name__ == "__main__":
    main()
