#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Генератор полётного задания (.plan) для QGroundControl.
Может работать в двух режимах:
  1) python3 mission_generator.py
     – создаёт план по встроенным демо-точкам.
  2) python3 mission_generator.py --input-file <путь_к_txt>
     – загружает координаты из текстового файла (lat,lon,alt)
       и генерирует соответствующий .plan.
"""

import json
import os
import argparse
from typing import List, Tuple, Optional

# Путь к папке для сохранения планов
PLANS_DIR = os.path.expanduser("~/DroneNavigation/Plans")


def read_path_file(filepath: str) -> List[Tuple[float, float, float]]:
    """
    Читает файл с координатами (lat, lon, alt).
    Пропускает пустые строки и строки, начинающиеся с '#'.
    Возвращает список кортежей (lat, lon, alt).
    """
    waypoints = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 3:
                lat = float(parts[0])
                lon = float(parts[1])
                alt = float(parts[2])
                waypoints.append((lat, lon, alt))
    return waypoints


def create_qgc_plan(
    waypoints: List[Tuple[float, float, Optional[float]]],
    altitude: float = 50.0,
    takeoff_alt: float = 50.0,
    planned_home: Optional[Tuple[float, float, float]] = None,
    cruise_speed: float = 15.0,
    hover_speed: float = 5.0,
    vehicle_type: int = 2,
    firmware_type: int = 12
) -> dict:
    """
    Генерирует словарь с миссией QGroundControl в формате .plan.

    :param waypoints: список точек. Каждая точка может быть:
                      - (lat, lon) – высота берётся из параметра altitude
                      - (lat, lon, alt) – используется своя высота
    :param altitude: высота полёта для точек без индивидуальной высоты.
    :param takeoff_alt: высота взлёта (обычно совпадает с altitude).
    :param planned_home: координаты домашней точки. Если None, берётся
                         первая точка списка (с нулевой высотой).
    :param cruise_speed: скорость полёта м/с.
    :param hover_speed: скорость висения м/с.
    :param vehicle_type: тип аппарата (2 - квадрокоптер).
    :param firmware_type: тип прошивки (12 - PX4).
    :return: словарь, готовый для сериализации в JSON.
    """
    if not waypoints:
        raise ValueError("Необходимо указать хотя бы одну путевую точку")

    if planned_home is None:
        home_lat = waypoints[0][0]
        home_lon = waypoints[0][1]
        planned_home = (home_lat, home_lon, 0.0)

    items = []

    # 1. Переключение в режим миссии
    items.append({
        "autoContinue": True,
        "command": 530,
        "doJumpId": 1,
        "frame": 2,
        "params": [0, 2, None, None, None, None, None],
        "type": "SimpleItem"
    })

    # 2. Взлёт
    items.append({
        "AMSLAltAboveTerrain": None,
        "Altitude": takeoff_alt,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 22,
        "doJumpId": 2,
        "frame": 3,
        "params": [0, 0, 0, None, planned_home[0], planned_home[1], takeoff_alt],
        "type": "SimpleItem"
    })

    # 3. Путевые точки
    for i, wp in enumerate(waypoints, start=3):
        lat = wp[0]
        lon = wp[1]
        if len(wp) >= 3 and wp[2] is not None:
            alt = float(wp[2])
        else:
            alt = altitude

        items.append({
            "AMSLAltAboveTerrain": None,
            "Altitude": alt,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": i,
            "frame": 3,
            "params": [0, 0, 0, None, lat, lon, alt],
            "type": "SimpleItem"
        })

    # 4. Возврат на старт
    items.append({
        "autoContinue": True,
        "command": 20,
        "doJumpId": len(items) + 1,
        "frame": 2,
        "params": [0, 0, 0, 0, 0, 0, 0],
        "type": "SimpleItem"
    })

    plan = {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": cruise_speed,
            "firmwareType": firmware_type,
            "globalPlanAltitudeMode": 0,
            "hoverSpeed": hover_speed,
            "items": items,
            "plannedHomePosition": [planned_home[0], planned_home[1], planned_home[2]],
            "vehicleType": vehicle_type,
            "version": 2
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1
    }
    return plan


def save_plan_to_file(plan: dict, filename: str = "mission.plan") -> str:
    """Сохраняет .plan файл в папку Plans и возвращает путь."""
    os.makedirs(PLANS_DIR, exist_ok=True)
    filepath = os.path.join(PLANS_DIR, filename)
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(plan, f, indent=4, ensure_ascii=False)
    return filepath


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Генератор .plan миссии для QGroundControl")
    parser.add_argument("--input-file", "-i", type=str, default=None,
                        help="Путь к текстовому файлу с координатами (lat,lon,alt). Если не указан, используются демо-точки.")
    parser.add_argument("--output", "-o", type=str, default="mission.plan",
                        help="Имя выходного .plan файла (по умолчанию mission.plan)")
    parser.add_argument("--altitude", type=float, default=50.0,
                        help="Высота полёта для демо-точек или для точек без индивидуальной высоты (по умолчанию 50 м)")
    parser.add_argument("--takeoff-alt", type=float, default=None,
                        help="Высота взлёта. Если не задана, равна --altitude")
    args = parser.parse_args()

    takeoff_alt = args.takeoff_alt if args.takeoff_alt is not None else args.altitude

    if args.input_file:
        waypoints = read_path_file(args.input_file)
        if not waypoints:
            print("Файл не содержит корректных точек.")
            exit(1)
        print(f"Загружено {len(waypoints)} точек из {args.input_file}")
        mission = create_qgc_plan(
            waypoints=waypoints,
            altitude=args.altitude,
            takeoff_alt=takeoff_alt,
            planned_home=None
        )
    else:
        points = [
            (55.16113621, 61.36945969),
            (55.16110283, 61.36876326),
            (55.16118427, 61.36864991),
            (55.16130576, 61.36863706),
            (55.16139188, 61.36868146),
            (55.16139788, 61.36895606),
            (55.16139588, 61.36931246)
        ]
        mission = create_qgc_plan(
            waypoints=points,
            altitude=args.altitude,
            takeoff_alt=takeoff_alt,
            planned_home=(55.161347, 61.3694382, 242)
        )

    saved_path = save_plan_to_file(mission, filename=args.output)
    print(f"План миссии сохранён: {saved_path}")
