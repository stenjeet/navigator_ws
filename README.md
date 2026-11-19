# navigator_ws — симуляция робота «Навигатор-М»

Рабочее пространство ROS 2 проекта **navigator**: модель робота, карта полигона,
истинная квадратная траектория, модели СШП-дальностей и лидарных сканов,
EKF-локализация по СШП, сетка занятости, выделение ориентиров и EKF-SLAM.

Целевая среда — **Windows + WSL 2, Ubuntu 24.04, ROS 2 Jazzy**.
Пошаговая установка с нуля: [`WSL_SETUP.md`](WSL_SETUP.md). Код использует только `rclpy`,
стандартные сообщения и `launch`, поэтому собирается и на ROS 2 Humble (Ubuntu 22.04).

## Быстрый старт

```bash
cd ~/navigator_ws                   # проект должен лежать в файловой системе Linux, не в /mnt/c
./run.sh use_rviz:=false            # сборка + headless-симуляция; Ctrl+C для остановки
./run.sh                            # то же, но с окном RViz (Windows 11 — через WSLg)
```

Дальше — проверка во втором терминале (раздел [Проверка](#проверка)) или снятие картинок
для отчёта: `./run_visualize.sh`.

## Структура

```
navigator_ws/
├── README.md
├── WSL_SETUP.md                  # установка Windows + WSL 2 + Ubuntu 24.04
├── .gitignore                    # build/, install/, log/, results/plot_data.npz
├── run.sh                        # сборка + запуск симуляции
├── run_visualize.sh              # headless-прогон + рендер PNG в results/
├── run_localization_metrics.sh   # headless-прогон EKF + графики ошибок
├── run_map_compare.sh            # headless-прогон двух карт + сравнение
├── run_analysis.sh               # сценарный анализ точности (этап 9)
├── docs/stages7-9-contract.md    # контракт топиков/сообщений этапов 7-9
├── docs/assignment-implementation.md  # соответствие девяти этапам задания
├── results/                      # готовые рисунки для отчёта
├── tools/visualize.py            # сбор данных из топиков и отрисовка
├── tools/localization_metrics.py # RMSE и графики ошибок локализации
├── tools/map_compare.py          # сравнение карт (истина vs оценка)
├── tools/analysis.py             # сценарии: RMSE, карты, таблицы, графики
└── src/
    ├── navigator_description/    # только описание робота
    │   ├── CMakeLists.txt, package.xml
    │   ├── urdf/navigator_m.urdf
    │   └── config/navigator.rviz
    ├── navigator_sim/            # модели движения и датчиков
    │   ├── package.xml, setup.py, setup.cfg, resource/
    │   ├── config/{motion.yaml, sensors.yaml}
    │   └── navigator_sim/
    │       ├── world.py                  # единый источник геометрии полигона
    │       ├── polygon_markers_node.py   # карта (MarkerArray)
    │       ├── motion_node.py            # истинная траектория
    │       ├── uwb_model_node.py         # СШП-дальности
    │       └── lidar_model_node.py       # лидарные сканы
    ├── navigator_bringup/        # launch-файлы
    │   ├── CMakeLists.txt, package.xml
    │   └── launch/{simulation.launch.py, display.launch.py, localization.launch.py,
    │                mapping.launch.py, landmarks.launch.py, slam.launch.py}
    ├── navigator_interfaces/     # сообщения ориентиров (этапы 7-8)
    │   ├── CMakeLists.txt, package.xml
    │   └── msg/{LandmarkObservation, LandmarkObservationArray,
    │            Landmark, LandmarkMap}.msg
    ├── navigator_localization/   # этап 5: EKF-локализация по СШП
    │   ├── package.xml, setup.py, setup.cfg, resource/
    │   ├── config/ekf.yaml
    │   ├── test/{test_ekf.py, test_config.py}
    │   └── navigator_localization/
    │       ├── ekf.py            # чистый EKF (predict/update)
    │       └── ekf_node.py       # ROS-обёртка
    └── navigator_slam/           # этапы 6-8: карта, ориентиры, EKF-SLAM
        ├── package.xml, setup.py, setup.cfg, resource/
        ├── config/{occupancy.yaml, landmarks.yaml, ekf_slam.yaml}
        ├── test/{test_occupancy_grid.py, test_config.py,
        │         test_landmark_detection.py, test_ekf_slam.py, test_analysis.py}
        └── navigator_slam/
            ├── occupancy_grid.py          # сетка лог-оддсов (этап 6)
            ├── occupancy_grid_node.py     # ROS-обёртка
            ├── landmark_detection.py      # ориентиры и ассоциация (этап 7)
            ├── landmark_node.py           # ROS-обёртка
            ├── ekf_slam.py                # совместный EKF поза+карта (этап 8)
            └── ekf_slam_node.py           # ROS-обёртка
```

Разделение по ролям: `_description` можно переиспользовать с реальным роботом,
`_sim` заменяется на драйверы датчиков, `_bringup` — единственное место, где пакеты
собираются вместе. Этап 5 реализован в `navigator_localization`, этапы 6–8 — в
`navigator_slam`, сообщения ориентиров — в `navigator_interfaces`.

## Требования

- Ubuntu 24.04 + ROS 2 Jazzy: `ros-jazzy-desktop` (включает RViz2), `ros-dev-tools` (colcon).
- Только для рисунков: `python3-numpy`, `python3-matplotlib`.

## Сборка (один раз)

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Ожидаемо: `Summary: 6 packages finished` (`navigator_description`, `navigator_sim`,
`navigator_bringup`, `navigator_localization`, `navigator_slam`,
`navigator_interfaces`).

## Скрипты

| Команда | Что делает |
|---|---|
| `./run.sh [аргументы launch]` | `colcon build --symlink-install`, `source install/setup.bash`, `ros2 launch navigator_bringup simulation.launch.py`. Аргументы пробрасываются в launch: `./run.sh use_rviz:=false`. Собирает при каждом запуске (~пара секунд), чтобы не стартовать устаревший код. |
| `./run_visualize.sh [секунды]` | Запускает `tools/visualize.py` в фоне, поднимает headless-симуляцию, пишет данные (62 с по умолчанию, можно `./run_visualize.sh 90`), рендерит PNG в `results/`, затем гасит симуляцию. **Требует уже собранный `install/`** — после правок кода сначала `./run.sh` или `colcon build`. |
| `python3 tools/visualize.py --replot` | Перерисовывает графики из сохранённого `results/plot_data.npz` без запуска симуляции. Дополнительно: `--duration N`, `--out DIR`. |
| `./run_localization_metrics.sh [секунды]` | headless-прогон с EKF, запись `/ground_truth/odom` и `/estimate/odom` (62 с по умолчанию), PNG и RMSE в `results/`. |
| `python3 tools/localization_metrics.py --replot` | Перерисовывает метрики из `results/localization_data.npz`. |
| `./run_map_compare.sh [секунды]` | headless-прогон с двумя картографами (истинная поза и оценка EKF), сравнение карт, PNG, расстояние и IoU в `results/`. |
| `python3 tools/map_compare.py --replot` | Перерисовывает сравнение карт из `results/map_compare_data.npz`. |
| `./run_analysis.sh [секунды] [--quick]` | Этап 9: последовательно прогоняет сценарии (σ СШП, пропуски, окно без СШП, раскладки якорей), пишет таблицу и графики в `results/`. |
| `python3 tools/analysis.py --replot` | Перерисовывает таблицу и графики этапа 9 из `results/analysis/analysis_data.npz`. |

Все shell-скрипты берут дистрибутив ROS из `/opt/ros/jazzy/setup.bash`; другой путь можно
задать переменной, например `ROS_SETUP=/opt/ros/humble/setup.bash ./run.sh`.

## Запуск вручную

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# движение + СШП + лидар + карта + RViz
ros2 launch navigator_bringup simulation.launch.py

# без графики
ros2 launch navigator_bringup simulation.launch.py use_rviz:=false

# статичный показ модели и карты (без движения, этап 2 отчёта);
# use_rviz:=false — только headless smoke-test без окна
ros2 launch navigator_bringup display.launch.py

# этап 5: симуляция + EKF-локализация
ros2 launch navigator_bringup localization.launch.py
# эквивалент:
ros2 launch navigator_bringup simulation.launch.py use_localization:=true

# этап 6: сетка занятости по оценке EKF (EKF стартует автоматически)
ros2 launch navigator_bringup mapping.launch.py
# карта по истинной позе / обе карты сразу (для сравнения)
ros2 launch navigator_bringup mapping.launch.py pose_source:=ground_truth
ros2 launch navigator_bringup mapping.launch.py pose_source:=both

# этап 7: выделение ориентиров и ассоциация (поза — EKF)
ros2 launch navigator_bringup landmarks.launch.py
# этап 8: EKF-SLAM (совместная оценка позы и карты ориентиров)
ros2 launch navigator_bringup slam.launch.py

# этап 7 с истинной позой — чистая проверка детектора и ассоциации
ros2 launch navigator_bringup landmarks.launch.py pose_topic:=/ground_truth/odom
```

У всех launch-файлов есть аргумент `use_rviz` (по умолчанию `true`);
`simulation.launch.py` принимает `use_localization`, `sensors_params` и
`ekf_params` (сценарные YAML этапа 9), `mapping.launch.py` — `pose_source`
(`estimate` по умолчанию, `ground_truth`, `both`) и пробрасывает
`sensors_params`/`ekf_params`.

## Узлы и топики

| Узел (пакет) | Публикует |
|---|---|
| `robot_state_publisher` | `/robot_description`, `/tf` (звенья робота) |
| `static_transform_publisher` | `/tf_static` (`map → odom`, identity) |
| `motion_node` (`navigator_sim`) | `/ground_truth/odom`, `/ground_truth/path`, `/cmd_vel`, `/joint_states`, `/tf` (`odom → base_footprint`) |
| `polygon_markers` (`navigator_sim`) | `/polygon_markers` |
| `uwb_model_node` (`navigator_sim`) | `/uwb/ideal_ranges`, `/uwb/noisy_ranges`, `/uwb/anchors` |
| `lidar_model_node` (`navigator_sim`) | `/scan_ideal`, `/scan_noisy` |
| `ekf_node` (`navigator_localization`) | `/estimate/odom`, `/estimate/path`, `/estimate/error` |
| `occupancy_grid_ekf` (`navigator_slam`) | `/map/estimate` (сетка по оценённой позе) |
| `occupancy_grid_gt` (`navigator_slam`) | `/map/ground_truth` (сетка по истинной позе) |
| `landmark_node` (`navigator_slam`) | `/landmarks/observations`, `/landmarks/map`, `/landmarks/markers` |
| `ekf_slam_node` (`navigator_slam`) | `/slam/odom`, `/slam/path`, `/slam/landmarks`, `/slam/markers`, `/slam/error` |

| Топик | Тип | Назначение |
|-------|-----|-----------|
| `/ground_truth/odom` | nav_msgs/Odometry | истинная поза, курс, скорости |
| `/ground_truth/path` | nav_msgs/Path | накопленная истинная траектория |
| `/cmd_vel` | geometry_msgs/Twist | заданные v, ω |
| `/joint_states` | sensor_msgs/JointState | углы колёс |
| `/scan_ideal` | sensor_msgs/LaserScan | идеальный скан (frame `lidar_link`) |
| `/scan_noisy` | sensor_msgs/LaserScan | скан с шумом/смещением/пропусками |
| `/uwb/ideal_ranges` | std_msgs/Float32MultiArray | истинные дальности A1..A4 |
| `/uwb/noisy_ranges` | std_msgs/Float32MultiArray | зашумлённые (NaN = пропуск) |
| `/uwb/anchors` | visualization_msgs/MarkerArray | визуализация якорей |
| `/polygon_markers` | visualization_msgs/MarkerArray | стены, препятствия, ориентиры |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | `map → odom → base_footprint` |
| `/estimate/odom` | nav_msgs/Odometry | оценка позы EKF (frame `map`) |
| `/estimate/path` | nav_msgs/Path | накопленная оценка траектории |
| `/estimate/error` | geometry_msgs/Vector3Stamped | ошибка оценки vs истина (`x`, `y`, `yaw`) |
| `/map/estimate` | nav_msgs/OccupancyGrid | сетка занятости по оценке EKF (этап 6) |
| `/map/ground_truth` | nav_msgs/OccupancyGrid | сетка занятости по истинной позе (этап 6) |
| `/landmarks/observations` | navigator_interfaces/LandmarkObservationArray | ориентиры в системе робота (этап 7) |
| `/landmarks/map` | navigator_interfaces/LandmarkMap | карта ориентиров трекера (этап 7) |
| `/landmarks/markers` | visualization_msgs/MarkerArray | визуализация ориентиров трекера |
| `/slam/odom` | nav_msgs/Odometry | поза из EKF-SLAM (frame `map`) |
| `/slam/path` | nav_msgs/Path | траектория EKF-SLAM |
| `/slam/landmarks` | navigator_interfaces/LandmarkMap | ориентиры из совместного фильтра (этап 8) |
| `/slam/markers` | visualization_msgs/MarkerArray | визуализация ориентиров SLAM |
| `/slam/error` | geometry_msgs/Vector3Stamped | ошибка SLAM vs истина (`x`, `y`, `yaw`) |

## Проверка

Второй терминал, при запущенной симуляции:

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 pkg list | grep navigator             # 6 пакетов
ros2 node list                             # 6 без EKF; 7 с EKF; 8 с landmark_node; 8 со SLAM; 9 с двумя картографами (+1 с RViz)
ros2 topic list                            # все топики из таблицы выше
ros2 topic hz /scan_ideal                  # ~10 Гц
ros2 topic hz /uwb/noisy_ranges            # ~10 Гц
ros2 topic echo --once /uwb/noisy_ranges   # 4 дальности (NaN = пропуск)
ros2 topic echo --once /ground_truth/odom  # истинная поза
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /estimate/odom               # ~20 Гц (этап 5, если включён EKF)
ros2 topic echo --once /estimate/error     # ошибка оценки (этап 5)
ros2 topic hz /map/estimate                # ~1 Гц (этап 6, если запущен маппер)
ros2 topic echo --once /map/estimate       # OccupancyGrid: -1/0/100
ros2 topic hz /landmarks/observations      # ~10 Гц (этап 7, если запущен landmark_node)
ros2 topic echo --once /landmarks/map      # ориентиры с id и ковариацией
ros2 topic hz /slam/odom                   # ~20 Гц (этап 8)
ros2 topic echo --once /slam/landmarks     # карта ориентиров из EKF-SLAM
ros2 topic echo --once /slam/error         # ошибка SLAM vs истина
```

Ожидаемо: TF `map → base_footprint` движется по квадрату 4 × 4 м со скоростью 0.35 м/с,
лидар даёт 360 лучей; ближайшие стены ≈ 1.4 м от старта, дальние лучи
ограничены `range_max = 8 м` (`inf` = пропуск/нет отражения).

## Этап 5 — EKF-локализация

`navigator_localization` оценивает позу `[x, y, θ]` в `map`:

- **прогноз** — по дифференциальной модели из `/cmd_vel` (`v`, `ω`);
- **коррекция** — по дальностям `/uwb/noisy_ranges` к якорям A1..A4
  (`NaN` = пропуск, такое измерение пропускается).

Выходы: `/estimate/odom`, `/estimate/path`, `/estimate/error`. Узел **не**
публикует TF `odom → base_footprint` (это делает `motion_node`), чтобы не было
конфликта; дисплеи `/estimate/odom` и `/estimate/path` уже добавлены в
`navigator.rviz` (синий цвет).

Важное ограничение: одна СШП-метка в статике задаёт только свою позицию,
поэтому **курс по дальностям не наблюдается** (якобиан ранга 2). Курс
оценивается через модель движения и уточняется на поворотах; начальный курс
в `ekf.yaml` задавайте разумно.

Запуск и проверка:

```bash
ros2 launch navigator_bringup localization.launch.py use_rviz:=false
ros2 topic hz /estimate/odom
ros2 topic echo --once /estimate/error
```

Unit-тесты (без ROS):

```bash
PYTHONPATH=src/navigator_localization:src/navigator_sim \
  python3 -m unittest discover -s src/navigator_localization/test -v
```

Метрики и графики ошибок одной командой:

```bash
./run_localization_metrics.sh          # 62 с
./run_localization_metrics.sh 90       # 90 с
```

Результаты: `results/result_localization_trajectory.png` (истина vs оценка),
`results/result_localization_error.png` (ошибки `x`, `y`, `yaw` во времени),
RMSE печатается в консоль.

## Этап 6 — Построение сетки занятости

`navigator_slam` строит `nav_msgs/OccupancyGrid` по сканам лидара и позе
(`occupancy_grid_node`):

- **свободные ячейки** — вдоль каждого валидного луча (шаг `resolution / 2`);
- **занятые ячейки** — в точке отражения луча;
- вклад копится в лог-оддсах: `+log(p_occ/(1-p_occ))` за попадание,
  `−log(p_free/(1-p_free))` за проход (порог в `occupancy.yaml`), значения
  ограничены `[log_odds_min, log_odds_max]`, поэтому карта обратима;
- `inf`/`NaN` и дальности вне `[range_min, range_max]` (пропуски лидара)
  игнорируются; поза берётся ближайшая по штампу из буфера (допуск
  `pose_tolerance`).

Ячейка публикуется как `100` (занята), `0` (свободна) или `-1` (неизвестна)
по порогам `occupied_threshold` / `free_threshold`. Сетка 180 × 180 ячеек
(0.05 м, −0.5…8.5 м) покрывает полигон вместе со стенами; публикация 1 Гц.

Сравнение карт — тот же узел, запущенный дважды на одних и тех же сканах, но с
разной позой (`mapping.launch.py pose_source:=both`):

| Топик | Поза |
|---|---|
| `/map/estimate` | `/estimate/odom` — оценка EKF (этап 5) |
| `/map/ground_truth` | `/ground_truth/odom` — истинная поза (эталон) |

Запуск, проверка и тесты:

```bash
ros2 launch navigator_bringup mapping.launch.py use_rviz:=false pose_source:=both
ros2 topic hz /map/estimate                 # ~1 Гц
ros2 topic echo --once /map/estimate        # width/height 180, data из -1/0/100

PYTHONPATH=src/navigator_slam:src/navigator_sim \
  python3 -m unittest discover -s src/navigator_slam/test -v
```

Сравнение одной командой:

```bash
./run_map_compare.sh          # 62 с
./run_map_compare.sh 90       # 90 с
```

Метрики сравнения (`tools/map_compare.py`):

- **расстояние** от занятых ячеек одной карты до ближайшей занятой ячейки другой
  (среднее и максимум, метры) — основная метрика точности карты;
- IoU занятых ячеек и доля совпавших ячеек среди известных в обеих картах;
- число ячеек, занятых только в одной из карт.

IoU по занятым ячейкам чувствителен к сдвигу: стены и ориентиры занимают 1–2
ячейки, поэтому сдвиг даже на одну ячейку (0.05 м) заметно его снижает — карты
считаются совпадающими прежде всего по расстоянию. Результат —
`results/result_map_compare.png` (истина, оценка, различия).

Важное ограничение: обе карты строятся по одним и тем же сканам, поэтому
различие отражает только ошибку позы. Карта по истинной позе — эталон
симуляции; на реальном роботе такой эталон появляется лишь после EKF-SLAM
(этап 8).

## Этап 7 — Выделение и сопоставление ориентиров

`navigator_slam/landmark_detection.py` извлекает ориентиры из скана и
сопоставляет их с ранее наблюдавшимися:

- **сегментация** — разрывы по скачку дальности с адаптивным порогом
  (`max(jump_threshold, 3σ) + |Δθ|·r`), короткие разрывы `inf`/`NaN`
  сшиваются, полный круг скана замыкается циклически;
- **геометрия** — кластер принимается, если в нём `min_points..max_points`
  лучей, ширина ≤ `max_cluster_width` (0.20 м), а круг радиуса 0.04–0.25 м
  описывает дугу с residual ≤ `max_residual`; для коротких дуг используется
  `fit_arc` (хорда/стрелка), и кластер всё равно отбрасывается, если радиус
  или residual не проходят пороги;
- **наблюдение** (`navigator_interfaces/LandmarkObservationArray`) — в системе
  `base_footprint`: `range` и `bearing` от центра робота (смещение лидара
  скомпенсировано), `radius` и собственные дисперсии детектора;
- **ассоциация** — махаланобисово расстояние с гейтом `gate = 9.21`
  (χ², 2 степени свободы, 99 %), каждый id используется не более одного раза
  за скан, несопоставленные наблюдения получают новый id.

Узел `landmark_node` публикует `/landmarks/observations` на каждый скан, а при
наличии позы (`pose_topic`) — карту `/landmarks/map` (экспоненциальное
сглаживание `map_alpha`) и маркеры `/landmarks/markers`. Пустой `pose_topic`
отключает карту и маркеры — так `landmark_node` работает внутри EKF-SLAM.

Ориентир попадает в карту только после `min_hits` согласованных наблюдений в
пределах `tentative_gate` (0.35 м): короткая дуга на угле стены описывается
окружностью так же хорошо, как настоящая метка, и одиночные ложные срабатывания
(≈1.7 % сканов) иначе закреплялись бы в карте навсегда.

Проверка детектора (офлайн, 720 синтетических сканов по геометрии полигона):
средняя ошибка положения относительно `world.LANDMARKS` 3 мм без шума,
1.7 % ложных срабатываний с шумом лидара (σ = 0.02 м, bias = 0.01 м,
1 % пропусков), 0.4 мс на скан.

```bash
ros2 launch navigator_bringup landmarks.launch.py use_rviz:=false
ros2 topic hz /landmarks/observations
ros2 topic echo --once /landmarks/map
```

## Этап 8 — EKF-SLAM

`navigator_slam/ekf_slam.py` оценивает **совместно** позу и координаты
ориентиров:

```
mu    = [x, y, θ, m1x, m1y, ..., mNx, mNy]
Sigma = cov(mu)                      # (2N+3) x (2N+3)
```

- **прогноз** — дифференциальная модель по `/cmd_vel`, полный якобиан процесса
  (кросс-корреляции с ориентирами сохраняются);
- **коррекция по СШП** — та же модель дальности, что на этапе 5 (смещение
  метки, 3D-дистанция до якорей, `NaN` = пропуск);
- **коррекция по ориентирам** — модель «дальность/азимут от центра робота»,
  аналитические якобианы по блоку робота и по связанному ориентиру,
  форма Джозефа;
- **инициализация ориентира** — обратная модель наблюдения, ковариация через
  `G·Σ·Gᵀ + M·R·Mᵀ`, диагональ не ниже `initial_landmark_std²`;
- **ассоциация** — по **совместной** ковариации (блоки робота и ориентира
  вместе с кросс-корреляциями), гейт χ² 99 %;
- **подтверждение** — ориентир добавляется в состояние только после
  `min_landmark_hits` (8) согласованных наблюдений: настоящая метка видна
  сотни раз, ложное срабатывание на угле стены — единицы.

Выходы: `/slam/odom`, `/slam/path`, `/slam/landmarks`, `/slam/markers`,
`/slam/error`; TF не публикуется (истинная поза по-прежнему владеет
`odom → base_footprint`).

Проверка замыкания (unit-тест, два круга по полигону, шум 2 см / 0.02 рад):
ошибка позы в конце **3.4 см**, курса 0.004 рад, средняя ошибка ориентиров
**3.4 мм**, число ориентиров равно числу настоящих (4) — повторное наблюдение
после замкнутой траектории не создаёт дубликатов.

Офлайн-прогон полной связки «реальный детектор → EKF-SLAM» (1136 синтетических
сканов, два круга): ошибка позы 2.7 см в среднем, **4 ориентира** с ошибкой
1.6–4.5 см, без фантомов на стенах.

Для runtime-проверки замыкания поставьте `loops: 2` в
`navigator_sim/config/motion.yaml` (штатно `loops: 1`); после второго круга
`/slam/landmarks` должен сохранить те же 4 id без дубликатов.

```bash
ros2 launch navigator_bringup slam.launch.py use_rviz:=false
ros2 topic hz /slam/odom
ros2 topic echo --once /slam/landmarks
```

## Этап 9 — Анализ эффективности

`tools/analysis.py` последовательно прогоняет сценарии, варьируя параметры через
сгенерированные YAML (`sensors_params`/`ekf_params`), и агрегирует метрики:

| Сценарий | Что меняется |
|---|---|
| `baseline` | номинальные `sensors.yaml` и `ekf.yaml` |
| `uwb_sigma_0.02/0.10/0.20` | σ СШП в модели **и** в фильтре |
| `uwb_mismatch_r_low` | модель 0.10 м, фильтр 0.05 м (оптимистичный R) |
| `dropout_0.10/0.25` | вероятность пропуска СШП |
| `outage_20_40` | все дальности `NaN` с 20-й по 40-ю секунду |
| `anchors_wide` / `anchors_compact` | углы полигона / якоря у старта |

Метрики: RMSE и максимум ошибки позы (`/estimate/error`), IoU занятых ячеек и
среднее расстояние между картами `/map/estimate` и `/map/ground_truth`.
Точность EKF-SLAM проверяется отдельным запуском `slam.launch.py`
(`/slam/error` и RMSE в логе `ekf_slam_node`), в сценарную таблицу она не входит. Результаты:
`results/analysis/summary.md` (таблица + автоматические выводы),
`results/analysis/summary.csv`, `results/analysis_rmse.png`,
`results/analysis_maps.png`.

```bash
./run_analysis.sh --quick      # baseline + σ 0.10 + dropout 0.25, по 30 с (~3 мин)
./run_analysis.sh              # полная таблица, по 62 с (~13 мин)
python3 tools/analysis.py --replot   # таблица и графики без запуска симуляции
```

Первые 3 с после первого `/estimate/error` отбрасываются как прогрев
(`--warmup`, по умолчанию 3 с): EKF стартует с фиксированной начальной позой,
пока робот уже движется, и этот переходный процесс иначе даёт большой
`max |e|`, не относящийся к установившейся точности.

## Остановка

`Ctrl+C` в терминале запуска. Если после аварийного прерывания остались процессы:

```bash
pkill -9 -f lib/navigator_sim
pkill -9 -f lib/navigator_slam
pkill -9 -f /opt/ros/jazzy/lib/robot_state_publisher
pkill -9 -f /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher
```

## Параметры

- `navigator_sim/config/motion.yaml` — старт (1.5; 1.5; 0), сторона 4.0 м, v = 0.35 м/с,
  ω = 0.60 рад/с, Δt = 0.05 с, колея 0.42 м, колесо r = 0.09 м, `loops`.
- `navigator_sim/config/sensors.yaml` — seed 12025, σ/смещения/пропуски СШП и лидара,
  дальности лидара 0.12–8.0 м, 360 лучей, 10 Гц.
- `navigator_description/urdf/navigator_m.urdf` — геометрия «Навигатор-М»
  (корпус r = 0.24 м, h = 0.22 м; лидар `0.18 0 0.34`, СШП-метка
  `-0.15 0.12 0.46`, колея 0.42 м).
- `navigator_localization/config/ekf.yaml` — начальная поза и её σ, `Q`, `σ` СШП,
  смещение метки, якоря A1..A4, частота прогноза 20 Гц (этап 5).
- `navigator_slam/config/occupancy.yaml` — разрешение 0.05 м, границы сетки,
  `p_occupied`/`p_free`, клип лог-оддсов, пороги публикации, смещение лидара,
  `max_range`, допуск по позе, частота публикации (этап 6).
- `navigator_slam/config/landmarks.yaml` — пороги кластеризации (`jump_threshold`,
  `min/max_points`, `max_cluster_width`), допустимый радиус и residual,
  `sigma_range`/`sigma_bearing`, гейт ассоциации, `map_alpha`, `max_landmarks`,
  подтверждение кандидатов (`min_hits`, `tentative_gate`) (этап 7).
- `navigator_slam/config/ekf_slam.yaml` — начальная поза и σ, `Q`, σ СШП и якоря,
  `use_uwb`, `gate`, `sigma_range`/`sigma_bearing`, `initial_landmark_std`,
  `max_landmarks`, подтверждение ориентиров (`min_landmark_hits`,
  `tentative_gate`), частоты публикации (этап 8).
- `navigator_sim/config/sensors.yaml` — одна секция на ноду (`uwb_model_node`,
  `lidar_model_node`) и параметры сценариев этапа 9: `anchors` (плоские тройки
  `x, y, z`; если ключ не задан — якоря полигона `world.UWB_ANCHORS`) и окно без
  СШП `outage_start_s`/`outage_end_s`. Пустой список `anchors: []` в YAML писать
  нельзя: ROS не выводит тип пустого массива и объявляет параметр
  неинициализированным.

## Рисунки для отчёта

```bash
./run_visualize.sh        # headless-прогон ~62 с, PNG в results/
./run_visualize.sh 90     # более длинная запись
```

| Файл | Что показывает |
|------|----------------|
| `results/result_map.png` | вид сверху: полигон, истинная траектория, накопленная карта лидара |
| `results/result_scan.png` | один скан: идеальные и зашумлённые точки в системе лидара |
| `results/result_uwb.png` | дальности A1–A4: идеал vs шум + смещение + пропуски |
| `results/result_localization_*.png` | локализация EKF: траектория и ошибки (этап 5) |
| `results/result_map_compare.png` | сетка занятости по истинной и оценённой позе + различия (этап 6) |
| `results/analysis_rmse.png` | RMSE локализации по сценариям этапа 9 |
| `results/analysis_maps.png` | IoU и расстояние между картами по сценариям этапа 9 |
| `results/analysis/summary.md` | таблица метрик и автоматические выводы этапа 9 |

Сырые данные прогона сохраняются в `results/plot_data.npz`. Повторная отрисовка без
запуска симуляции:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 tools/visualize.py --replot
```

## RViz

`navigator_description/config/navigator.rviz` задаёт вид сверху: модель робота, TF,
истинную траекторию и оценку EKF (`/estimate/odom`, `/estimate/path`), маркеры
полигона, два лидарных скана (`/scan_ideal`, `/scan_noisy`), карты сетки
занятости (`/map/estimate` включён, `/map/ground_truth` выключен по умолчанию),
а также выключенные по умолчанию `/slam/path`, `/slam/odom`, `/slam/markers` и
`/landmarks/markers` — их достаточно включить галочкой при запуске
`landmarks.launch.py` или `slam.launch.py`. Fixed Frame — `map`. В Windows 11 окно RViz открывается через WSLg; на headless-машине
можно снимать скриншоты через виртуальный дисплей:

```bash
sudo apt install -y xvfb imagemagick
Xvfb :99 -screen 0 1600x1000x24 &
export DISPLAY=:99
ros2 launch navigator_bringup simulation.launch.py
import -window root rviz.png
```

## Проблемы и решения

| Симптом | Решение |
|---|---|
| `ros2: command not found` | не подключены `/opt/ros/jazzy/setup.bash` и `install/setup.bash` |
| `run_visualize.sh`: «Workspace is not built yet» | выполнить `colcon build --symlink-install` или `./run.sh` |
| `run_visualize.sh` падает на импорте | установить `python3-numpy python3-matplotlib` |
| `Package 'navigator_bringup' not found` | workspace не собран или не подключён `install/setup.bash` |
| `/map/estimate` отсутствует | не запущен `mapping.launch.py` (`pose_source:=estimate`/`both`) или ещё нет `/estimate/odom` |
| `map_compare.py`: «no map on …» | маппер запущен без `pose_source:=both` |
| `ros2 topic echo` пишет «message type … is invalid» | в этом терминале не подключён workspace: `source install/setup.bash` (типы `navigator_interfaces` видны только оттуда) |
| `/landmarks/observations` молчит | `landmark_node` не запущен (`landmarks.launch.py`/`slam.launch.py`) или в скане нет ориентиров |
| `/slam/odom` отсутствует | не запущен `slam.launch.py`; SLAM не использует `/estimate/odom` |
| `analysis.py`: «no /estimate/error» | EKF не стартовал: сценарий не дождался записи, смотрите `--startup-timeout` |
| `run_analysis.sh` долго молчит | это нормально: сценарии идут последовательно, статус печатается по мере завершения |
| RViz не открывается | Windows 10 без X-сервера — используйте `use_rviz:=false` или настройте `DISPLAY` |
| Сборка идёт очень долго | проект лежит в `/mnt/c` — скопируйте в `~` (см. `WSL_SETUP.md`, раздел 3) |

Подробности по установке WSL и ROS 2 Jazzy — в [`WSL_SETUP.md`](WSL_SETUP.md).
