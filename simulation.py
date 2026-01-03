import pygame
import numpy as np
import sys
import time
from algorithms import lawn_mower, spiral_coverage, greedy_coverage

# ----------------------------
# PARAMETERS
# ----------------------------
GRID_SIZE = (20, 20)
CELL_SIZE = 30
OBSTACLE_PERCENT = 0.05
FPS = 5
STATUS_BAR_HEIGHT = 50

# ----------------------------
# GRID CREATION
# ----------------------------
def create_grid():
    grid = np.zeros(GRID_SIZE, dtype=int)
    num_obstacles = int(OBSTACLE_PERCENT * GRID_SIZE[0] * GRID_SIZE[1])
    obstacle_indices = np.random.choice(GRID_SIZE[0]*GRID_SIZE[1], num_obstacles, replace=False)
    for idx in obstacle_indices:
        row = idx // GRID_SIZE[1]
        col = idx % GRID_SIZE[1]
        grid[row, col] = 1
    return grid

grid = create_grid()
robot_pos = [0, 0]
grid[robot_pos[0], robot_pos[1]] = 2
start_time = time.time()
algo_state = {"path": []}

# ----------------------------
# PYGAME INITIALIZATION
# ----------------------------
pygame.init()
screen = pygame.display.set_mode((GRID_SIZE[1]*CELL_SIZE, GRID_SIZE[0]*CELL_SIZE + STATUS_BAR_HEIGHT))
pygame.display.set_caption("RoboCoverageSim")

colors = {0: (245, 245, 245), 1: (40, 40, 40), 2: (120, 220, 120)}
robot_color = (255, 80, 80)
button_color = (240, 240, 245)
button_hover_color = (220, 220, 230)
button_shadow = (180, 180, 200)
font = pygame.font.SysFont("Segoe UI", 22)
clock = pygame.time.Clock()

button_width = 100
button_height = 36
reset_button_rect = pygame.Rect(400, GRID_SIZE[0]*CELL_SIZE + 7, button_width, button_height)

# ----------------------------
# DRAW BUTTON FUNCTION
# ----------------------------
def draw_button(rect, text, hover=False):
    color = button_hover_color if hover else button_color
    shadow_rect = rect.move(3, 3)
    pygame.draw.rect(screen, button_shadow, shadow_rect, border_radius=10)
    pygame.draw.rect(screen, color, rect, border_radius=10)
    text_surf = font.render(text, True, (50, 50, 50))
    text_rect = text_surf.get_rect(center=rect.center)
    screen.blit(text_surf, text_rect)

# ----------------------------
# SELECT ALGORITHM
# ----------------------------
# Options: lawn_mower, spiral_coverage, greedy_coverage
current_algorithm = greedy_coverage

# ----------------------------
# GAME LOOP
# ----------------------------
running = True
while running:
    mouse_pos = pygame.mouse.get_pos()
    mouse_click = pygame.mouse.get_pressed()[0]

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # RESET BUTTON CLICK
    if reset_button_rect.collidepoint(mouse_pos) and mouse_click:
        grid = create_grid()
        robot_pos = [0, 0]
        grid[robot_pos[0], robot_pos[1]] = 2
        start_time = time.time()
        algo_state = {"path": []}

    # ----------------------------
    # MOVE ROBOT USING ALGORITHM
    # ----------------------------
    # MOVE ROBOT USING ALGORITHM
    # ----------------------------
    next_pos = current_algorithm(grid, robot_pos, algo_state)
    robot_pos = list(next_pos)
    grid[robot_pos[0], robot_pos[1]] = 2

    # ----------------------------
    # DRAWING
    # ----------------------------
    screen.fill((245, 245, 250))  # background

    # Draw grid
    for row in range(GRID_SIZE[0]):
        for col in range(GRID_SIZE[1]):
            pygame.draw.rect(screen, colors[grid[row, col]],
                             (col*CELL_SIZE, row*CELL_SIZE, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(screen, (200, 200, 210),
                             (col*CELL_SIZE, row*CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

    # Draw robot
    pygame.draw.circle(screen, robot_color,
                       (robot_pos[1]*CELL_SIZE + CELL_SIZE//2, robot_pos[0]*CELL_SIZE + CELL_SIZE//2),
                       CELL_SIZE//3)

    # Status bar
    pygame.draw.rect(screen, (230, 230, 240),
                     (0, GRID_SIZE[0]*CELL_SIZE, GRID_SIZE[1]*CELL_SIZE, STATUS_BAR_HEIGHT))

    # Coverage & time
    total_free = np.sum(grid != 1)
    visited = np.sum(grid == 2)
    coverage_percent = (visited / total_free) * 100
    elapsed_time = time.time() - start_time
    coverage_text = font.render(f"Coverage: {coverage_percent:.1f}%", True, (50, 50, 50))
    time_text = font.render(f"Time: {elapsed_time:.1f}s", True, (50, 50, 50))
    screen.blit(coverage_text, (10, GRID_SIZE[0]*CELL_SIZE + 10))
    screen.blit(time_text, (200, GRID_SIZE[0]*CELL_SIZE + 10))

    # Draw Reset button
    draw_button(reset_button_rect, "Reset", hover=reset_button_rect.collidepoint(mouse_pos))

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()
