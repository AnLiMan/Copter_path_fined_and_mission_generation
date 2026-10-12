#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#--Библиотеки--
import osmium
import shapely.wkb as wkblib
import colorama as cl

# Фабрика для создания Shapely-объектов из WKB (Well-Known Binary)
wkbfab = osmium.geom.WKBFactory()

"""Обработчик для извлечения полигонов зданий из OSM-файла."""
class BuildingHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.buildings = []  # Список кортежей (полигон Shapely, высота в метрах)

    def area(self, a):
        if 'building' in a.tags:
            try:
                wkb = wkbfab.create_multipolygon(a)
                poly = wkblib.loads(wkb, hex=True)

                # Определение высоты здания
                height_str = a.tags.get('height')
                if height_str:
                    height = float(height_str)
                else:
                    levels_str = a.tags.get('building:levels')
                    if levels_str:
                        height = float(levels_str) * 3.0
                    else:
                        height = 6.0   # высота по умолчанию (два этажа)

                self.buildings.append((poly, height))
            except Exception as e:
                print(cl.Fore.RED + f"Ошибка обработки объекта: {e}")


def main():
    # Путь к OSM-файлу относительно директории Scripts
    osm_file = "/home/andrey/DroneNavigation/Maps/MapForPython.osm"

    print(cl.Fore.WHITE + "=" * 50)
    print(cl.Fore.BLUE + "ТЕСТОВЫЙ ЗАПУСК МОДУЛЯ ЧТЕНИЯ OSM-КАРТЫ")
    print(cl.Fore.WHITE + "=" * 50)
    print(cl.Fore.YELLOW + f"Исходный файл: {osm_file}")

    # Создаем и применяем обработчик
    handler = BuildingHandler()
    try:
        handler.apply_file(osm_file)
    except FileNotFoundError:
        print(cl.Fore.RED + f"Ошибка: файл '{osm_file}' не найден.")
        print(cl.Fore.MAGENTA + "Убедитесь, что файл map.osm находится в папке ../Maps/")
        return
    except Exception as e:
        print(cl.Fore.RED + f"Непредвиденная ошибка при чтении файла: {e}")
        return

    buildings = handler.buildings
    print(cl.Fore.WHITE + f"\nЧтение завершено успешно.")
    print(cl.Fore.WHITE + f"Обнаружено зданий: {len(buildings)}")

    # Вывод информации о первых 5 зданиях для визуального контроля
    print(cl.Fore.GREEN + "\nПримеры извлечённых объектов (первые 5):")
    for i, (poly, height) in enumerate(buildings[:5]):
        centroid = poly.centroid
        print(cl.Fore.WHITE + f"  [{i+1}] Центр: ({centroid.x:.6f}, {centroid.y:.6f}) | "
              f"Площадь: {poly.area:.2f} м² | Высота: {height:.1f} м")

    if len(buildings) == 0:
        print(cl.Fore.RED + "\nВнимание: в файле не найдено объектов с тегом 'building'.")
        print(cl.Fore.MAGENTA + "Проверьте содержимое OSM-файла.")

    print(cl.Fore.WHITE + "\n" + "=" * 50)
    print(cl.Fore.BLUE + "ТЕСТОВЫЙ ЗАПУСК ЗАВЕРШЁН")
    print(cl.Fore.WHITE + "=" * 50)


if __name__ == "__main__":
    main()
