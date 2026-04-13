# Autonomous Spacecraft Docking Simulator

This project implements a hybrid control system for spacecraft proximity operations. It uses **RRT*** for global path planning and a **PID Controller** for high-precision docking.

## Features
* **Path Planning**: Adaptive RRT* algorithm with obstacle avoidance.
* **Precision Control**: PID-based translation for docking port alignment.
* **Safety**: Integrated "Keep-out Zone" (KOZ) checking using elliptical bounds.

## How it Works
1. **Initialization**: The Chaser spacecraft identifies the Target's docking cone.
2. **RRT* Phase**: Generates a collision-free path to a "Rendezvous Point."
3. **PID Phase**: Once within range, the PID controller takes over to zero out the position error between docking ports.

## Requirements
* `numpy`
* `matplotlib`
* `pandas`

## Simulation Results

### Planned Path vs Actual Trajectory
This plot shows the RRT* planned path (purple) and the actual PID-controlled trajectory (green).

![Planned Path](./assets/image_bd9fab.png)

### Docking Alignment
The chaser spacecraft successfully aligns its docking cone with the target.

![Docking Detail](./assets/image_bd9fcd.png)

