# Copter_path_fined_and_mission_generation
Обработка osm-карт для поиска пути при помощи алгоритма A star и последующей генерации полетного задания в qground control

## Запуск скриптов рекомендуется проводить из стандартной папки
cd /home/%user_name%/DroneNavigation/Scripts

1. Обработка OSM-файла и построение карты проходимости
python3 ReadOSM.py

2. одуль обработки OSM-данных и построения Occupancy Grid.
python3 OSMHandler.py

3. Поиск пути
python3 PathFinder.py

4. Генерация файла миссии для QGroundControl. Имя файла сгенерируется и покажется автоматически скриптом, здесь в примере "path_20260815_180807.txt"
python3 mission_generator.py --input-file /home/andrey/DroneNavigation/Plans/path_20260815_180807.txt
