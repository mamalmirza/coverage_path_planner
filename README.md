# RoboCoverageSim

**RoboCoverageSim** is a Python-based visual simulation for Coverage Path Planning (CPP) algorithms. It uses `pygame` to visualize how a robot navigates and covers a grid environment filled with obstacles.

## Features

- **Visual Simulation**: Real-time rendering of the robot, grid, obstacles, and covered path using Pygame.
- **Random Environment**: Generates random obstacles for every simulation run.
- **Coverage Metrics**: tracks and displays the percentage of the area covered and the time elapsed.
- **Algorithm Support**: Designed to support multiple coverage algorithms.

## Supported Algorithms

The simulation includes implementations for several coverage strategies:

1.  **Lawn Mower (Boustrophedon)**: A standard pattern moving back and forth across the grid.
2.  **Spiral Coverage**: Spirals inward from the starting position.
3.  **Greedy / Nearest-Unvisited-Cell**: Moves to the closest unvisited cell using BFS when stuck.
4.  **Spanning Tree Coverage (STC)**: Uses a Minimum Spanning Tree (MST) to create a circumnavigating path that guarantees complete coverage.

## Installation

1.  **Clone the repository** (if you haven't already):
    ```bash
    git clone <repository_url>
    cd coverage_path_planner
    ```

2.  **Install Dependencies**:
    The project relies on `pygame` for visualization and `numpy` for grid management.
    ```bash
    pip install -r requirements.txt
    ```

## Usage

To start the simulation, run the `simulation.py` file:

```bash
python simulation.py
```

### Controls
- **Reset**: Click the "Reset" button in the UI to generate a new grid and restart the robot.
- **Configuration**: You can adjust parameters like `GRID_SIZE`, `OBSTACLE_PERCENT`, and `FPS` directly at the top of `simulation.py`.

### Changing Algorithms
To switch between algorithms, open `simulation.py` and modify the `moving_algorithm` variable:

```python
# Select your desired algorithm
# Options: lawn_mower, spiral_coverage, greedy_coverage, stc_coverage
moving_algorithm = stc_coverage
```
