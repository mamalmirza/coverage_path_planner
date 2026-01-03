import numpy as np
from collections import deque

def bfs_path(grid, start, goal):
    rows, cols = grid.shape
    queue = deque([start])
    came_from = {tuple(start): None}

    while queue:
        current = queue.popleft()
        if current == goal:
            break

        x, y = current
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < rows and 0 <= ny < cols and
                grid[nx, ny] != 1 and
                (nx, ny) not in came_from):
                came_from[(nx, ny)] = current
                queue.append((nx, ny))

    if goal not in came_from:
        return []

    # Reconstruct path
    path = []
    curr = goal
    while curr:
        path.append(curr)
        curr = came_from[curr]
    return path[::-1]



# ----------------------------
# LAWN-MOWER / BOUSTROPHEDON
# ----------------------------
def lawn_mower(grid, robot_pos, state=None):
    """Return next position for lawn-mower coverage."""
    x, y = robot_pos
    rows, cols = grid.shape

    # Determine direction (even rows -> right, odd rows -> left)
    direction = 1 if x % 2 == 0 else -1

    # Try moving in the current row
    next_y = y + direction
    if 0 <= next_y < cols and grid[x, next_y] != 1:
        return (x, next_y)

    # Move down to next row if possible
    next_x = x + 1
    if next_x < rows and grid[next_x, y] != 1:
        return (next_x, y)

    # Otherwise, stay in place (done)
    return (x, y)

# ----------------------------
# SPIRAL COVERAGE
# ----------------------------
def spiral_coverage(grid, robot_pos, state=None):
    """Return next position for spiral coverage."""
    x, y = robot_pos
    rows, cols = grid.shape

    # Directions: right, down, left, up
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx, ny] == 0:
            return (nx, ny)

    # Otherwise, stay
    return (x, y)

# ----------------------------
# GREEDY / NEAREST-UNVISITED-CELL
# ----------------------------
def greedy_coverage(grid, robot_pos, state):
    """
    Greedy coverage using BFS path planning.
    `state` holds the current path.
    """
    x, y = robot_pos

    # Follow existing path if any
    if state["path"]:
        return state["path"].pop(0)

    # Find nearest unvisited cell
    unvisited = np.argwhere(grid == 0)
    if len(unvisited) == 0:
        return (x, y)

    distances = np.abs(unvisited - np.array([x, y])).sum(axis=1)
    target = tuple(unvisited[np.argmin(distances)])

    path = bfs_path(grid, (x, y), target)
    if len(path) > 1:
        state["path"] = path[1:]
        return state["path"].pop(0)

    return (x, y)
