#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import json
import os
import sys

def main():
    parser = argparse.ArgumentParser(description='Create video of agent trajectories')
    parser.add_argument('--data_file', type=str, required=True, help='JSON file containing trajectory data')
    parser.add_argument('--output', type=str, required=True, help='Output video filename')
    parser.add_argument('--world_size_x', type=float, required=True, help='World size in X dimension')
    parser.add_argument('--world_size_y', type=float, required=True, help='World size in Y dimension')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second for video')
    args = parser.parse_args()

    # Check for ffmpeg
    try:
        import subprocess
        subprocess.check_output(['ffmpeg', '-version'])
        ffmpeg_available = True
    except (subprocess.SubprocessError, FileNotFoundError):
        ffmpeg_available = False
        print("Warning: ffmpeg not found. Will create a GIF instead of MP4.")
        # Change output extension to gif if it was mp4
        if args.output.lower().endswith('.mp4'):
            args.output = args.output[:-4] + '.gif'

    # Ensure output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.', exist_ok=True)

    # Load trajectory data
    with open(args.data_file, 'r') as f:
        trajectories = json.load(f)

    # Convert to numpy array for easier slicing
    trajectories = np.array(trajectories)
    num_agents = trajectories.shape[0]
    num_frames = trajectories.shape[1]

    # Create animation
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, args.world_size_x)
    ax.set_ylim(0, args.world_size_y)
    ax.set_title('Agent Simulation')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.grid(True)

    # Initialize scatter plot for agent positions
    scatter = ax.scatter([], [], s=50)

    # Initialize trajectory lines - one per agent
    lines = [ax.plot([], [], '-', alpha=0.5)[0] for _ in range(num_agents)]

    def init():
        scatter.set_offsets(np.empty((0, 2)))
        for line in lines:
            line.set_data([], [])
        return [scatter] + lines

    def animate(frame):
        # Update agent positions
        current_positions = trajectories[:, frame, :]
        scatter.set_offsets(current_positions)

        # Update trajectory lines
        for i, line in enumerate(lines):
            line.set_data(trajectories[i, :frame+1, 0], trajectories[i, :frame+1, 1])

        return [scatter] + lines

    ani = animation.FuncAnimation(fig, animate, frames=num_frames,
                                  init_func=init, blit=True, interval=1000/args.fps)

    try:
        if ffmpeg_available and args.output.lower().endswith('.mp4'):
            writer = animation.FFMpegWriter(fps=args.fps)
            ani.save(args.output, writer=writer)
        else:
            # Fallback to GIF
            if args.output.lower().endswith('.mp4'):
                output_file = args.output[:-4] + '.gif'
            else:
                output_file = args.output

            # Save as gif with Pillow writer
            ani.save(output_file, writer='pillow', fps=args.fps)

        print(f'Video saved to {args.output}')
    except Exception as e:
        print(f"Error saving animation: {str(e)}")
        print("Saving individual frames instead...")

        # Create frames directory
        frames_dir = "simulation_frames"
        os.makedirs(frames_dir, exist_ok=True)

        # Save individual frames
        for i in range(num_frames):
            animate(i)
            plt.savefig(f"{frames_dir}/frame_{i:04d}.png")
            if i % 10 == 0:
                print(f"Saved frame {i}/{num_frames}")

        print(f"All frames saved to {frames_dir}/ directory")
        print("To create a video manually, you can use:")
        print(f"ffmpeg -framerate {args.fps} -i {frames_dir}/frame_%04d.png -c:v libx264 -pix_fmt yuv420p {args.output}")

if __name__ == "__main__":
    main()
