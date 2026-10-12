# AStar.py
import heapq
import math

class AStar:
    """
    Реализация алгоритма A* для поиска пути в графе.
    """
    def __init__(self, nodes, edges):
        """
        :param nodes: словарь {node_id: (x, y)} координат узлов.
        :param edges: словарь {node_id: [(neighbor_id, cost), ...]} смежности.
        """
        self.nodes = nodes
        self.edges = edges

    def heuristic(self, a, b):
        """Евклидово расстояние между узлами a и b."""
        x1, y1 = self.nodes[a]
        x2, y2 = self.nodes[b]
        return math.hypot(x2 - x1, y2 - y1)

    def find_path(self, start_id, goal_id):
        """
        Поиск кратчайшего пути от start_id до goal_id.
        Возвращает список node_id, составляющих путь, или None, если путь не найден.
        """
        # Очередь с приоритетом: (f_score, counter, node_id)
        open_set = []
        counter = 0
        heapq.heappush(open_set, (0, counter, start_id))
        counter += 1

        # Словари для восстановления пути и хранения стоимости
        came_from = {}
        g_score = {start_id: 0}
        f_score = {start_id: self.heuristic(start_id, goal_id)}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_id:
                return self._reconstruct_path(came_from, current)

            for neighbor, cost in self.edges.get(current, []):
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_id)
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
                    counter += 1

        return None  # Путь не найден

    def _reconstruct_path(self, came_from, current):
        """Восстанавливает путь от старта до цели."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
