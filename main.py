import pygame
import numpy as np
import time
import sys

from algorithms import local_cluster_smoothing

# =========================
# CONFIGURATION
# =========================
GRID_ROWS = 20
GRID_COLS = 20
CELL_SIZE = 30
FPS = 5
OBSTACLE_RATIO = 0.2

UI_HEIGHT = 50
WINDOW_WIDTH = GRID_COLS * CELL_SIZE
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE + UI_HEIGHT

# =========================
# COLORS (Shadcn-inspired)
# =========================
COLORS = {
    "background": (248, 250, 252),
    "grid": (226, 232, 240),
    "free": (241, 245, 249),
    "obstacle": (15, 23, 42),
    "visited": (165, 243, 252),
    "robot": (14, 165, 233),
    "button": (30, 41, 59),
    "button_text": (241, 245, 249),
    "text": (15, 23, 42),
}

# =========================
# INITIALIZATION FUNCTIONS
# =========================
def generate_environment():
    grid = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)

    num_obstacles = int(GRID_ROWS * GRID_COLS * OBSTACLE_RATIO)
    obstacle_indices = np.random.choice(GRID_ROWS * GRID_COLS, num_obstacles, replace=False)

    for idx in obstacle_indices:
        r = idx // GRID_COLS
        c = idx % GRID_COLS
        grid[r, c] = -1  # obstacle

    return grid

def generate_subareas(grid):
    subareas = np.zeros_like(grid)

    label = 1
    for i in range(GRID_ROWS):
        for j in range(GRID_COLS):
            if grid[i, j] == 0:
                subareas[i, j] = label
                label += 1

    # Smooth regions
    for _ in range(3):
        subareas = local_cluster_smoothing(subareas)

    return subareas

# =========================
# ROBOT MOVEMENT (BASELINE)
# =========================
def lawn_mower_step(pos, grid, direction):
    r, c = pos

    if direction == 1:  # moving right
        if c + 1 < GRID_COLS and grid[r, c + 1] != -1:
            return (r, c + 1), direction
        elif r + 1 < GRID_ROWS:
            return (r + 1, c), -1
    else:  # moving left
        if c - 1 >= 0 and grid[r, c - 1] != -1:
            return (r, c - 1), direction
        elif r + 1 < GRID_ROWS:
            return (r + 1, c), 1

    return pos, direction

# =========================
# PYGAME SETUP
# =========================
pygame.init()
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("RoboCoverageSim")

clock = pygame.time.Clock()
font = pygame.font.SysFont("Segoe UI", 16)

# =========================
# RESET STATE
# =========================
def reset_simulation():
    grid = generate_environment()
    subareas = generate_subareas(grid)

    robot_pos = (0, 0)
    direction = 1
    start_time = time.time()

    return grid, subareas, robot_pos, direction, start_time

grid, subareas, robot_pos, direction, start_time = reset_simulation()

# =========================
# MAIN LOOP
# =========================
running = True
while running:
    clock.tick(FPS)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = event.pos
            if y > GRID_ROWS * CELL_SIZE:
                if 10 <= x <= 100:
                    grid, subareas, robot_pos, direction, start_time = reset_simulation()

    # Robot movement
    if grid[robot_pos] != -1:
        grid[robot_pos] = 1

    robot_pos, direction = lawn_mower_step(robot_pos, grid, direction)

    # =========================
    # DRAW GRID
    # =========================
    screen.fill(COLORS["background"])

    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            cell = grid[r, c]
            color = COLORS["free"]

            if cell == -1:
                color = COLORS["obstacle"]
            elif cell == 1:
                color = COLORS["visited"]

            pygame.draw.rect(
                screen,
                color,
                (c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE),
            )
            pygame.draw.rect(
                screen,
                COLORS["grid"],
                (c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE),
                1,
            )

    # Robot
    pygame.draw.circle(
        screen,
        COLORS["robot"],
        (
            robot_pos[1] * CELL_SIZE + CELL_SIZE // 2,
            robot_pos[0] * CELL_SIZE + CELL_SIZE // 2,
        ),
        CELL_SIZE // 3,
    )

    # =========================
    # UI BAR
    # =========================
    pygame.draw.rect(
        screen,
        COLORS["background"],
        (0, GRID_ROWS * CELL_SIZE, WINDOW_WIDTH, UI_HEIGHT),
    )

    # Coverage
    free_cells = np.sum(grid != -1)
    visited_cells = np.sum(grid == 1)
    coverage = (visited_cells / free_cells) * 100
    elapsed = time.time() - start_time

    coverage_text = font.render(f"Coverage: {coverage:.1f}%", True, COLORS["text"])
    time_text = font.render(f"Time: {elapsed:.1f}s", True, COLORS["text"])

    screen.blit(coverage_text, (120, GRID_ROWS * CELL_SIZE + 15))
    screen.blit(time_text, (250, GRID_ROWS * CELL_SIZE + 15))

    # Reset Button
    pygame.draw.rect(
        screen,
        COLORS["button"],
        (10, GRID_ROWS * CELL_SIZE + 10, 90, 30),
        border_radius=8,
    )
    reset_text = font.render("Reset", True, COLORS["button_text"])
    screen.blit(reset_text, (35, GRID_ROWS * CELL_SIZE + 16))

    pygame.display.flip()

pygame.quit()
sys.exit()
