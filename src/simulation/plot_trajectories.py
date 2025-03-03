#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import argparse
import json
import os

def main():
    parser = argparse.ArgumentParser(description='Plot agent trajectories')
    parser.add_argument('--data_file', type=str, required=True, help='JSON file containing trajectory data')
    parser.add_argument('--output', type=str, required=True, help='Output image filename')
    parser.add_argument('--world_size_x', type=float, required=True, help='World size in X dimension')
    parser.add_argument('--world_size_y', type=float, required=True, help='World size in Y dimension')
    args = parser.parse_args()

    # Ensure output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.', exist_ok=True)

    # Load trajectory data
    with open(args.data_file, 'r') as f:
        trajectories = json.load(f)

    plt.figure(figsize=(10, 8))
    for i, traj in enumerate(trajectories):
        traj = np.array(traj)
        plt.plot(traj[:, 0], traj[:, 1], '-', label=f'Agent {i}' if i < 10 else '')
        plt.plot(traj[0, 0], traj[0, 1], 'go')  # Start position
        plt.plot(traj[-1, 0], traj[-1, 1], 'ro')  # End position

    plt.xlim(0, args.world_size_x)
    plt.ylim(0, args.world_size_y)
    plt.title('Agent Trajectories')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)

    if len(trajectories) <= 10:
        plt.legend()

    plt.savefig(args.output, dpi=300)
    print(f"Plot saved to {args.output}")

if __name__ == "__main__":
    main()
