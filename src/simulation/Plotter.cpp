//
// Copyright 2025 Hugo Birch
// Created by hugo on 2/26/25.
//

#include "Plotter.h"

#include <sys/stat.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

Plotter::Plotter(const Simulation& sim, const SimulationParams& params)
    : simulation(sim), params(params) {}

void Plotter::plotTrajectories(const std::string& filename) {
  // Create a Python script to plot the trajectories
  std::ofstream pyScript("plot_trajectories.py");

  pyScript << "import matplotlib.pyplot as plt\n";
  pyScript << "import numpy as np\n\n";

  // Write trajectory data
  const auto& trajectories = simulation.getTrajectories();
  pyScript << "trajectories = [\n";

  for (const auto& agentTraj : trajectories) {
    pyScript << "    [\n";
    for (const auto& pos : agentTraj) {
      pyScript << "        [" << pos.first << ", " << pos.second << "],\n";
    }
    pyScript << "    ],\n";
  }
  pyScript << "]\n\n";

  // Plot code
  pyScript << "plt.figure(figsize=(10, 8))\n";
  pyScript << "for i, traj in enumerate(trajectories):\n";
  pyScript << "    traj = np.array(traj)\n";
  pyScript << "    plt.plot(traj[:, 0], traj[:, 1], '-', label=f'Agent {i}' if "
              "i < 10 else '')\n";
  pyScript << "    plt.plot(traj[0, 0], traj[0, 1], 'go')  # Start position\n";
  pyScript << "    plt.plot(traj[-1, 0], traj[-1, 1], 'ro')  # End position\n";

  pyScript << "plt.xlim(0, " << params.worldSizeX << ")\n";
  pyScript << "plt.ylim(0, " << params.worldSizeY << ")\n";
  pyScript << "plt.title('Agent Trajectories')\n";
  pyScript << "plt.xlabel('X Position')\n";
  pyScript << "plt.ylabel('Y Position')\n";
  pyScript << "plt.grid(True)\n";
  pyScript << "if len(trajectories) <= 10:\n";
  pyScript << "    plt.legend()\n";
  pyScript << "plt.savefig('" << filename << "', dpi=300)\n";

  pyScript.close();

  // Execute the Python script
  std::cout << "Generating trajectory plot..." << std::endl;
  std::system("python plot_trajectories.py");
  std::cout << "Plot saved to " << filename << std::endl;
}

void Plotter::createVideo(const std::string& filename, int fps) {
  // Create a Python script to generate the video
  std::ofstream pyScript("create_video.py");

  pyScript << "import matplotlib.pyplot as plt\n";
  pyScript << "import matplotlib.animation as animation\n";
  pyScript << "import numpy as np\n\n";

  // Write trajectory data
  const auto& trajectories = simulation.getTrajectories();
  pyScript << "trajectories = [\n";

  for (const auto& agentTraj : trajectories) {
    pyScript << "    [\n";
    for (const auto& pos : agentTraj) {
      pyScript << "        [" << pos.first << ", " << pos.second << "],\n";
    }
    pyScript << "    ],\n";
  }
  pyScript << "]\n\n";

  // Convert to numpy arrays for easier slicing
  pyScript << "trajectories = np.array(trajectories)\n";
  pyScript << "num_agents = trajectories.shape[0]\n";
  pyScript << "num_frames = trajectories.shape[1]\n\n";

  // Animation code
  pyScript << "fig, ax = plt.subplots(figsize=(10, 8))\n";
  pyScript << "ax.set_xlim(0, " << params.worldSizeX << ")\n";
  pyScript << "ax.set_ylim(0, " << params.worldSizeY << ")\n";
  pyScript << "ax.set_title('Agent Simulation')\n";
  pyScript << "ax.set_xlabel('X Position')\n";
  pyScript << "ax.set_ylabel('Y Position')\n";
  pyScript << "ax.grid(True)\n\n";

  pyScript << "# Initialize scatter plot for agent positions\n";
  pyScript << "scatter = ax.scatter([], [], s=50)\n\n";

  pyScript << "# Initialize trajectory lines - one per agent\n";
  pyScript << "lines = [ax.plot([], [], '-', alpha=0.5)[0] for _ in "
              "range(num_agents)]\n\n";

  pyScript << "def init():\n";
  pyScript << "    scatter.set_offsets(np.empty((0, 2)))\n";
  pyScript << "    for line in lines:\n";
  pyScript << "        line.set_data([], [])\n";
  pyScript << "    return [scatter] + lines\n\n";

  pyScript << "def animate(frame):\n";
  pyScript << "    # Update agent positions\n";
  pyScript << "    current_positions = trajectories[:, frame, :]\n";
  pyScript << "    scatter.set_offsets(current_positions)\n\n";

  pyScript << "    # Update trajectory lines\n";
  pyScript << "    for i, line in enumerate(lines):\n";
  pyScript << "        line.set_data(trajectories[i, :frame+1, 0], "
              "trajectories[i, :frame+1, 1])\n\n";

  pyScript << "    return [scatter] + lines\n\n";

  pyScript
      << "ani = animation.FuncAnimation(fig, animate, frames=num_frames,\n";
  pyScript << "                              init_func=init, blit=True, "
              "interval=1000/"
           << fps << ")\n\n";

  pyScript << "# Save animation\n";
  pyScript << "ani.save('" << filename << "', writer='ffmpeg', fps=" << fps
           << ")\n";
  pyScript << "print('Video saved to " << filename << "')\n";

  pyScript.close();

  // Execute the Python script
  std::cout << "Generating simulation video..." << std::endl;
  std::system("python create_video.py");
  std::cout << "Video saved to " << filename << std::endl;
}
