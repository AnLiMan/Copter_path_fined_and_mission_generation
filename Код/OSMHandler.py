#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Модуль обработки OSM-данных и построения Occupancy Grid.
Выполняет:
1. Импорт списка зданий из ReadOSM.
2. Определение границ области по всем узлам OSM-файла.
3. Проецирование координат в UTM для метрических расчётов.
4. Построение регулярной сетки (Occupancy Grid) с учётом буфера безопасности.
5. Кэширование сетки в .npz для быстрой загрузки.
6. Функции преобразования координат (широта/долгота <-> индексы сетки).
"""

import osmium
import numpy as np
import time
import os
from pyproj import Transformer
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union, transform
import colorama as cl

# Импортируем обработчик зданий из ReadOSM
from ReadOSM import BuildingHandler


processing_osm_file = "/home/andrey/DroneNavigation/Maps/MapForPython.osm"  #Путь до исследуемого файла

"""Обработчик для сбора всех узлов OSM-файла и вычисления bounding box."""
class BoundaryHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.min_lon = self.min_lat = float('inf')
        self.max_lon = self.max_lat = float('-inf')

    def node(self, n):
        lon, lat = n.location.lon, n.location.lat
        self.min_lon = min(self.min_lon, lon)
        self.max_lon = max(self.max_lon, lon)
        self.min_lat = min(self.min_lat, lat)
        self.max_lat = max(self.max_lat, lat)

""" Основной класс для подготовки карты проходимости.  """
class OSMHandler:
    def __init__(self, osm_file: str, resolution: float = 3.0, safety_margin: float = 5.0,
                 force_rebuild: bool = False):
        self.osm_file = osm_file
        self.resolution = resolution
        self.safety_margin = safety_margin
        self.force_rebuild = force_rebuild

        # Путь к кэш-файлу в той же папке, что и скрипт
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_name = os.path.splitext(os.path.basename(osm_file))[0]
        self.cache_path = os.path.join(script_dir, f"{base_name}_cache.npz")

        if not force_rebuild and os.path.exists(self.cache_path):
            self._load_cache()
        else:
            self._full_build()
    
    #Полный цикл: парсинг OSM, построение сетки, сохранение в кэш.
    def _full_build(self):
        
        # 1. Извлечение зданий
        building_handler = BuildingHandler()
        building_handler.apply_file(self.osm_file)
        self.buildings = building_handler.buildings
        print(cl.Fore.WHITE + f"Загружено зданий: {len(self.buildings)}")

        # 2. Определение границ области
        boundary_handler = BoundaryHandler()
        boundary_handler.apply_file(self.osm_file)
        self.bounds_wgs84 = (boundary_handler.min_lon, boundary_handler.min_lat,
                             boundary_handler.max_lon, boundary_handler.max_lat)
        print(cl.Fore.WHITE + f"Границы (WGS84): {self.bounds_wgs84}")

        # 3. Настройка проекции UTM
        self._init_projection()

        # 4. Построение Occupancy Grid
        self._build_grid()

        # 5. Сохранение в кэш
        self._save_cache()
        print(cl.Fore.GREEN + f"Данные сохранены в кэш: {self.cache_path}")
    
    #Автоматический выбор зоны UTM и создание трансформеров.
    def _init_projection(self):
        center_lon = (self.bounds_wgs84[0] + self.bounds_wgs84[2]) / 2
        center_lat = (self.bounds_wgs84[1] + self.bounds_wgs84[3]) / 2
        zone_number = int((center_lon + 180) // 6) + 1
        self.utm_crs = f"EPSG:326{zone_number:02d}" if center_lat >= 0 else f"EPSG:327{zone_number:02d}"
        self._setup_transformers()

        # Метрические границы
        self.min_x, self.min_y = self.transformer_to_utm.transform(
            self.bounds_wgs84[0], self.bounds_wgs84[1])
        self.max_x, self.max_y = self.transformer_to_utm.transform(
            self.bounds_wgs84[2], self.bounds_wgs84[3])
        print(cl.Fore.WHITE + f"Метрические границы (UTM): X[{self.min_x:.1f}, {self.max_x:.1f}] Y[{self.min_y:.1f}, {self.max_y:.1f}]")

    #Создание трансформеров на основе self.utm_crs (без расчёта границ).
    def _setup_transformers(self):
        self.transformer_to_utm = Transformer.from_crs("EPSG:4326", self.utm_crs, always_xy=True)
        self.transformer_to_wgs = Transformer.from_crs(self.utm_crs, "EPSG:4326", always_xy=True)

    #Создание Occupancy Grid: 0 – свободно, 1 – занято.
    def _build_grid(self):
        self.width_m = self.max_x - self.min_x
        self.height_m = self.max_y - self.min_y
        self.cols = int(np.ceil(self.width_m / self.resolution))
        self.rows = int(np.ceil(self.height_m / self.resolution))
        print(cl.Fore.WHITE + f"Размер сетки: {self.rows} x {self.cols} ячеек")

        self.grid = np.zeros((self.rows, self.cols), dtype=np.uint8)

        if self.buildings:
            buffered = []
            for poly, height in self.buildings:
                poly_utm = transform(self.transformer_to_utm.transform, poly)
                buffered.append(poly_utm.buffer(self.safety_margin))
            obstacles = unary_union(buffered)
        else:
            obstacles = None

        if obstacles is not None:
            print(cl.Fore.BLUE + "Начинаем растеризацию...")
            total_rows = self.rows
            report_interval = max(1, total_rows // 20)
            start_time = time.time()
            for row in range(self.rows):
                if row % report_interval == 0 or row == total_rows - 1:
                    elapsed = time.time() - start_time
                    pct = (row + 1) / total_rows * 100
                    print(cl.Fore.MAGENTA + f"Обработано {row+1}/{total_rows} рядов ({pct:.0f}%), прошло {elapsed:.1f} сек")
                for col in range(self.cols):
                    x = self.min_x + col * self.resolution + self.resolution / 2
                    y = self.min_y + row * self.resolution + self.resolution / 2
                    point = Point(x, y)
                    if obstacles.contains(point) or obstacles.intersects(point):
                        self.grid[row, col] = 1
            elapsed_total = time.time() - start_time
            print(cl.Fore.GREEN + f"Растеризация завершена за {elapsed_total:.1f} сек")
        else:
            print(cl.Fore.CYAN + "Препятствия отсутствуют, сетка полностью свободна.")
        print(f"Препятствий в сетке: {np.count_nonzero(self.grid)} ячеек")

    #Сохранение сетки и метаданных в .npz файл.
    def _save_cache(self):
        np.savez(self.cache_path,
                 grid=self.grid,
                 bounds_wgs84=np.array(self.bounds_wgs84),
                 resolution=self.resolution,
                 safety_margin=self.safety_margin,
                 utm_crs=self.utm_crs,
                 min_x=self.min_x,
                 min_y=self.min_y,
                 max_x=self.max_x,
                 max_y=self.max_y,
                 cols=self.cols,
                 rows=self.rows)

    #Загрузка сетки и метаданных из .npz файла
    def _load_cache(self):
        print(cl.Fore.WHITE + f"Загрузка кэшированной сетки из {self.cache_path}")
        data = np.load(self.cache_path)
        self.grid = data['grid']
        self.bounds_wgs84 = tuple(data['bounds_wgs84'])
        self.resolution = float(data['resolution'])
        self.safety_margin = float(data['safety_margin'])
        self.utm_crs = str(data['utm_crs'])
        self.min_x = float(data['min_x'])
        self.min_y = float(data['min_y'])
        self.max_x = float(data['max_x'])
        self.max_y = float(data['max_y'])
        self.cols = int(data['cols'])
        self.rows = int(data['rows'])
        self._setup_transformers()
        print(cl.Fore.WHITE + f"Загружена сетка {self.rows}x{self.cols}, препятствий: {np.count_nonzero(self.grid)}")

    # Преобразование географических координат в индексы (row, col) сетки.
    def geo_to_grid(self, lon, lat):
        x, y = self.transformer_to_utm.transform(lon, lat)
        col = int((x - self.min_x) / self.resolution)
        row = int((y - self.min_y) / self.resolution)
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return row, col

    #Преобразование индексов ячейки в географические координаты её центра.
    def grid_to_geo(self, row, col):
        x = self.min_x + col * self.resolution + self.resolution / 2
        y = self.min_y + row * self.resolution + self.resolution / 2
        lon, lat = self.transformer_to_wgs.transform(x, y)
        return lat, lon


# ---- Тестовый запуск ----
if __name__ == "__main__":
    print(cl.Fore.WHITE + "=" * 50)
    print(cl.Fore.YELLOW + "ТЕСТИРОВАНИЕ OSMHandler")
    print(cl.Fore.WHITE + "=" * 50)
    osm_file = processing_osm_file
    
    # При force_rebuild=True будет выполнен полный пересчёт и обновлён кэш
    handler = OSMHandler(osm_file, resolution=3.0, safety_margin=5.0, force_rebuild=True)

    # Проверка преобразования координат
    test_lon = (handler.bounds_wgs84[0] + handler.bounds_wgs84[2]) / 2
    test_lat = (handler.bounds_wgs84[1] + handler.bounds_wgs84[3]) / 2
    row, col = handler.geo_to_grid(test_lon, test_lat)
    lat, lon = handler.grid_to_geo(row, col)
    print(cl.Fore.BLUE + f"\nТест преобразования:")
    print(cl.Fore.WHITE + f"Исходные (Lon, Lat): ({test_lon:.6f}, {test_lat:.6f})")
    print(cl.Fore.WHITE + f"Индексы ячейки: ({row}, {col})")
    print(cl.Fore.WHITE + f"Обратно (Lat, Lon): ({lat:.6f}, {lon:.6f})")
    print(cl.Fore.WHITE + f"Статус ячейки: {'Занято' if handler.grid[row, col] == 1 else 'Свободно'}")
