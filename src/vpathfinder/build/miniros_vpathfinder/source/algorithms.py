import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial.distance import cdist
from heapq import heappush, heappop

def astar_heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def astar(grid, start, goal):
    moves = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),           (0, 1),
        (1, -1),  (1, 0),  (1, 1)
    ]
    
    open_set = [(0, start)]
    came_from = {}

    g_score = {start: 0}
    f_score = {start: astar_heuristic(start, goal)}

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for dx, dy in moves:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if not (0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]):
                continue
                
            if grid[neighbor] == 0:
                continue

            move_cost = 1.0 if dx == 0 or dy == 0 else np.sqrt(2)
            t_g = g_score[current] + move_cost

            if neighbor not in g_score or t_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = t_g
                f_score[neighbor] = t_g + astar_heuristic(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))
        
    return None

def local_update_path(global_path, current_pos, local_map, update_radius=10):
    current_idx = np.argmin([astar_heuristic(current_pos, p) for p in global_path])

    start_idx = max(0, current_idx - 2)
    end_idx = min(len(global_path) - 1, current_idx + update_radius)

    local_start = global_path[start_idx]
    local_goal = global_path[end_idx]

    has_collision = False
    for point in global_path:
        y, x = int(point[0]), int(point[1])
        if not (0 <= y < local_map.shape[0] and 0 <= x < local_map.shape[1]):
            continue

        if local_map[y, x] == 0:
            has_collision = True
            break

    if not has_collision:
        return global_path

    local_path = astar(local_map, local_start, local_goal)
    if not local_path:
        return global_path

    return global_path[:start_idx] + local_path + global_path[end_idx+1:]        

def smooth_path(path, grid, alpha=0.5, beta=0.1, iterations=100):
    """Сглаживание пути градиентным спуском"""
    path = np.array(path)
    for _ in range(iterations):
        for i in range(1, len(path)-1):
            original = path[i] - path[i]
            smoothness = (path[i-1] + path[i+1] - 2*path[i])
            obs_force = obstacle_force(path[i], grid)
            path[i] += (alpha * original + \
                    beta * smoothness + \
                    0.2 * obs_force).astype(np.uint32)
    return path

def obstacle_force(point, grid, radius=4):
    force = np.zeros(2)
    for dx in range(-radius, radius+1):
        for dy in range(-radius, radius+1):
            nx, ny = point[0]+dx, point[1]+dy
            if (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]) and grid[nx, ny]:
                dist = np.sqrt(dx**2 + dy**2)
                if dist < 1e-5: 
                    continue

                strength = 1.0 / max(dist, 0.1)
                force[0] -= strength * dx / dist
                force[1] -= strength * dy / dist
    
    return force

def prepare_map(map, robot_radius):
    obstacle_mask = map < 0
    y, x = np.ogrid[-robot_radius:robot_radius+1,-robot_radius:robot_radius+1]
    kernel = (x*x + y*y <= robot_radius ** 2).astype(np.uint8)
    return binary_dilation(obstacle_mask, structure=kernel)

def simplify_path(path, grid, max_lookahead=50, min_segment_length=10):
    if len(path) < 10:
        return [path[0], path[-1]]
    
    simplified = [path[0]]
    current_idx = 0
    
    while current_idx < len(path) - 1:
        next_idx = current_idx + 1
        best_idx = next_idx
        
        end_idx = min(current_idx + max_lookahead, len(path)-1)
        
        for test_idx in range(current_idx + 2, end_idx + 1):
            if is_line_safe(path[current_idx], path[test_idx], grid):
                # Проверяем, что сегмент достаточно длинный
                if np.linalg.norm(path[test_idx] - path[current_idx]) > min_segment_length:
                    best_idx = test_idx
        
        simplified.append(path[best_idx])
        current_idx = best_idx
    
    return np.array(simplified)

def is_line_safe(p1, p2, grid):
    points = bresenham_line(p1, p2)
    
    for point in points[1:-1]:
        y, x = point
        if (0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]):
            if grid[int(y), int(x)] == 1:
                return False
    return True

def bresenham_line(start, end):
    y1, x1 = start.astype(int)
    y2, x2 = end.astype(int)
    
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    
    while True:
        points.append([y1, x1])
        if x1 == x2 and y1 == y2:
            break
        
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    
    return np.array(points)

def adaptive_lookahead(speed, rotation_speed):
    base_distance = 50  # пикселей
    speed_factor = np.clip(speed / 0.5, 0.5, 2.0)  # Нормировка скорости
    rotation_factor = np.clip(1.0 / (abs(rotation_speed) + 0.1, 0.5, 2.0))
    return int(base_distance * speed_factor * rotation_factor)

from matplotlib import pyplot as plt
def visualize_comparison(original, simplified):
    plt.figure(figsize=(12, 8))
    plt.plot(original[:,1], original[:,0], 'b-', label='Original Path', alpha=0.5)
    plt.plot(simplified[:,1], simplified[:,0], 'ro-', label='Simplified Path')
    plt.legend()
    plt.title(f"Simplification: {len(original)} -> {len(simplified)} points")
    plt.show()


grid = np.zeros((100, 100), dtype=np.int16)

path = []
for x in range(100):
    path.append((x, 0))

for y in range(100):
    path.append((100, y))

path = np.asarray(path, dtype=np.int16)

simplified = simplify_path(path, grid)
simplified = smooth_path(simplified, grid, iterations=40)

visualize_comparison(path, simplified)