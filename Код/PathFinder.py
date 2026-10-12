#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Модуль поиска пути (Path Finder) с использованием алгоритма A*.
Работает с Occupancy Grid, созданной в OSMHandler.
Сохраняет найденный путь в текстовый файл (lat, lon, alt).
"""

import numpy as np
import heapq
import sys
import os
from datetime import datetime

# Попытка импорта matplotlib (необязателен, только для визуализации)
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# Параметры полёта
FLIGHT_ALTITUDE = 3.0   # высота полёта над точкой старта, метры


class Node:
    """Вспомогательный класс узла сетки для алгоритма A*."""
    __slots__ = ('row', 'col')

    def __init__(self, row, col):
        self.row = row
        self.col = col

    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    def __hash__(self):
        return hash((self.row, self.col))

    def __repr__(self):
        return f"Node({self.row}, {self.col})"


class AStarPlanner:
    """
    Планировщик A* на регулярной сетке.
    """
    def __init__(self, grid, resolution=3.0):
        """
        :param grid: 2D numpy array (0 – свободно, 1 – занято)
        :param resolution: размер ячейки в метрах
        """
        self.grid = grid
        self.rows, self.cols = grid.shape
        self.resolution = resolution
        self.straight_cost = resolution
        self.diag_cost = resolution * np.sqrt(2)

    def heuristic(self, a, b):
        """Евклидова эвристика (в метрах)."""
        dx = abs(a.row - b.row)
        dy = abs(a.col - b.col)
        return self.resolution * np.sqrt(dx * dx + dy * dy)

    def get_neighbors(self, node):
        """Возвращает список соседей (Node, cost) для 8-связной области."""
        neighbors = []
        row, col = node.row, node.col

        directions = [
            (-1,  0, self.straight_cost),
            ( 1,  0, self.straight_cost),
            ( 0, -1, self.straight_cost),
            ( 0,  1, self.straight_cost),
            (-1, -1, self.diag_cost),
            (-1,  1, self.diag_cost),
            ( 1, -1, self.diag_cost),
            ( 1,  1, self.diag_cost)
        ]

        for dr, dc, cost in directions:
            nr, nc = row + dr, col + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.grid[nr, nc] == 0:
                    neighbors.append((Node(nr, nc), cost))
        return neighbors

    def find_path(self, start_node, goal_node):
        """
        Запуск алгоритма A*.
        :param start_node: Node
        :param goal_node: Node
        :return: список Node от старта до цели или None, если путь не найден
        """
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start_node, goal_node), id(start_node), start_node))

        came_from = {}
        g_score = {start_node: 0.0}
        closed_set = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_node:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                path.reverse()
                return path

            closed_set.add(current)

            for neighbor, cost in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_node)
                    heapq.heappush(open_set, (f_score, id(neighbor), neighbor))

        return None


def save_path_to_txt(path, handler, altitude, output_dir="../Plans"):
    """Сохраняет путь в текстовый файл с координатами (lat, lon, alt)."""
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"path_{timestamp}.txt"
    filepath = os.path.join(output_dir, filename)

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("# lat,lon,alt\n")
        for node in path:
            lat, lon = handler.grid_to_geo(node.row, node.col)
            f.write(f"{lat:.8f},{lon:.8f},{altitude:.1f}\n")

    print(f"Путь сохранён в {filepath}")
    return filepath


# ---- Тестовый запуск ----
if __name__ == "__main__":
    from OSMHandler import OSMHandler

    print("=" * 50)
    print("ТЕСТИРОВАНИЕ PathFinder НА ОСНОВЕ OSM-КАРТЫ")
    print("=" * 50)

    osm_file = "/home/andrey/DroneNavigation/Maps/MapForPython.osm"
    handler = OSMHandler(osm_file, resolution=3.0, safety_margin=5.0, force_rebuild=False)
    grid = handler.grid
    print(f"Загружена сетка {handler.rows}x{handler.cols}, препятствий: {np.count_nonzero(grid)}")

    # Координаты старта и цели в формате Google Maps: (широта, долгота)
    start_lat, start_lon = 55.15987794961696, 61.36836654183546
    goal_lat, goal_lon = 55.16057934367334, 61.37538933239947

    start_row, start_col = handler.geo_to_grid(start_lon, start_lat)
    goal_row, goal_col = handler.geo_to_grid(goal_lon, goal_lat)

    print(f"Старт (Lat, Lon): ({start_lat:.6f}, {start_lon:.6f}) -> ячейка ({start_row}, {start_col})")
    print(f"Цель  (Lat, Lon): ({goal_lat:.6f}, {goal_lon:.6f}) -> ячейка ({goal_row}, {goal_col})")

    if grid[start_row, start_col] == 1:
        print("Стартовая точка находится внутри препятствия! Выберите другую.")
        sys.exit(1)
    if grid[goal_row, goal_col] == 1:
        print("Целевая точка находится внутри препятствия! Выберите другую.")
        sys.exit(1)

    planner = AStarPlanner(grid, resolution=2.0)
    start_node = Node(start_row, start_col)
    goal_node = Node(goal_row, goal_col)

    print("\nЗапуск A*...")
    path = planner.find_path(start_node, goal_node)

    if path is None:
        print("Путь не найден.")
    else:
        print(f"Найден путь из {len(path)} точек.")
        # Сохранение в txt
        saved_path = save_path_to_txt(path, handler, FLIGHT_ALTITUDE)

        # Визуализация (если доступен matplotlib)
        if MATPLOTLIB_AVAILABLE:
            plt.figure(figsize=(10, 8))
            plt.imshow(grid, cmap='gray', origin='lower')
            path_rows = [n.row for n in path]
            path_cols = [n.col for n in path]
            plt.plot(path_cols, path_rows, color='red', linewidth=2, marker='.', markersize=1)
            plt.scatter(start_col, start_row, color='green', s=50, label='Старт')
            plt.scatter(goal_col, goal_row, color='blue', s=50, label='Цель')
            plt.legend()
            plt.title("Путь A* на карте препятствий")
            plt.xlabel("Столбец")
            plt.ylabel("Строка")
            plt.tight_layout()
            plt.show()
        else:
            print("Matplotlib не установлен, визуализация пропущена.")
