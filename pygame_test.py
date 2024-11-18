import pygame
import math
from heapq import heappush, heappop

# Initialize pygame
pygame.init()

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Set up display
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 60, 60  # Grid size
TILE_SIZE = WIDTH // ROWS

# Create screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("A* Pathfinding Visualization")

# Node class
class Node:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.color = WHITE
        self.neighbors = []

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, (self.col * TILE_SIZE, self.row * TILE_SIZE, TILE_SIZE, TILE_SIZE))

    def set_neighbors(self, grid):
        # Four possible directions (up, down, left, right)
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        self.neighbors = []
        for dr, dc in directions:
            new_row, new_col = self.row + dr, self.col + dc
            if 0 <= new_row < ROWS and 0 <= new_col < COLS and grid[new_row][new_col].color != BLACK:
                self.neighbors.append(grid[new_row][new_col])

# Heuristic function
def heuristic(node1, node2):
    return abs(node1.row - node2.row) + abs(node1.col - node2.col)

# Updated A* Algorithm to store coordinates instead of the Node instance
def a_star(start, end, grid):
    open_set = []
    heappush(open_set, (0, (start.row, start.col)))

    g_score = {node: float('inf') for row in grid for node in row}
    g_score[start] = 0
    parent_map = {}

    while open_set:
        current_coords = heappop(open_set)[1]
        current = grid[current_coords[0]][current_coords[1]]

        if current == end:
            path = []
            while current in parent_map:
                current.color = RED
                path.append(current)
                current = parent_map[current]
            end.color = BLUE
            return path[::-1]

        for neighbor in current.neighbors:
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                priority = tentative_g_score + heuristic(neighbor, end)
                heappush(open_set, (priority, (neighbor.row, neighbor.col)))
                parent_map[neighbor] = current
                neighbor.color = GREEN

        draw(grid)

    return None

# Drawing the grid
def draw_grid():
    for row in range(ROWS):
        pygame.draw.line(screen, BLACK, (0, row * TILE_SIZE), (WIDTH, row * TILE_SIZE))
        for col in range(COLS):
            pygame.draw.line(screen, BLACK, (col * TILE_SIZE, 0), (col * TILE_SIZE, HEIGHT))

# Drawing everything
def draw(grid):
    screen.fill(WHITE)
    for row in grid:
        for node in row:
            node.draw(screen)
    draw_grid()
    pygame.display.update()

# Main function
def main():
    grid = [[Node(row, col) for col in range(COLS)] for row in range(ROWS)]

    # Start and end nodes
    start = None
    end = None

    running = True
    while running:
        draw(grid)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Mouse actions for setting start, end, and obstacles
            if pygame.mouse.get_pressed()[0]:  # Left mouse button
                pos = pygame.mouse.get_pos()
                row, col = pos[1] // TILE_SIZE, pos[0] // TILE_SIZE
                node = grid[row][col]
                if not start:
                    start = node
                    start.color = BLUE
                elif not end and node != start:
                    end = node
                    end.color = RED
                elif node != start and node != end:
                    node.color = BLACK

            elif pygame.mouse.get_pressed()[2]:  # Right mouse button to reset
                pos = pygame.mouse.get_pos()
                row, col = pos[1] // TILE_SIZE, pos[0] // TILE_SIZE
                node = grid[row][col]
                node.color = WHITE
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    # Set neighbors for all nodes
                    for row in grid:
                        for node in row:
                            node.set_neighbors(grid)
                    a_star(start, end, grid)

                if event.key == pygame.K_r:
                    # Reset the grid
                    start = None
                    end = None
                    grid = [[Node(row, col) for col in range(COLS)] for row in range(ROWS)]

    pygame.quit()

if __name__ == "__main__":
    main()