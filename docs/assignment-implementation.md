# Соответствие заданию курсового проекта

Документ отвечает на три вопроса:

1. **что требуется** в каждом из девяти этапов задания;
2. **где это реализовано** (пакет, файл, функция/класс);
3. **какой командой показать**, что пункт работает.

Дополнительно собраны легенда цветов, карта топиков, принятые допущения и
чек-лист приёмки в WSL. Все команды выполняются из корня проекта
`~/navigator_ws` после `source /opt/ros/jazzy/setup.bash` и
`source install/setup.bash`, если не указано иное. `ros2 launch` занимает
терминал: команды `ros2 topic …` и `tf2_echo` выполняются во **втором**
терминале с теми же `source`.

> **Статус проверки на момент оформления.** В среде без ROS (macOS) выполнены:
> `py_compile` всех Python-файлов, разбор URDF/launch, статическая проверка
> геометрии сцены и **197 чистых unit-тестов** (177 в `navigator_slam`,
> 12 в `navigator_localization`, 8 в `navigator_sim`) — все проходят.
> Полный runtime-прогон ROS 2
> (RViz, топики, сценарии этапа 9) выполняется в WSL по командам из разделов
> ниже: это финальный шаг приёмки, а не отсутствующая функциональность.

---

## 0. Короткий вывод по этапам

| Этап | Статус | Основной модуль |
|---|---|---|
| 1. Окружение, WSL 2, ROS 2, RViz | реализовано | `WSL_SETUP.md`, `run.sh` |
| 2. URDF, полигон, якоря, ориентиры | реализовано | `navigator_description`, `navigator_sim/world.py` |
| 3. Истинная траектория | реализовано | `navigator_sim/motion_node.py` |
| 4. Модели СШП и лидара | реализовано | `uwb_model_node.py`, `lidar_model_node.py` |
| 5. EKF-локализация по СШП | реализовано | `navigator_localization` |
| 6. Сетка занятости и сравнение карт | реализовано | `navigator_slam/occupancy_grid*.py` |
| 7. Ориентиры и ассоциация | реализовано | `navigator_slam/landmark_detection.py` |
| 8. EKF-SLAM и замыкание петли | реализовано | `navigator_slam/ekf_slam.py` |
| 9. Анализ эффективности | реализовано; таблицы/графики создаются прогоном | `tools/analysis.py` |

Артефакты `results/*.png` и `results/analysis/*` **генерируются командами**
(разделы 2.4, 5–9). В репозитории лежат три ранних рисунка
`results/result_map.png`, `result_scan.png`, `result_uwb.png`; после
перестановки препятствий их нужно пересоздать через `./run_visualize.sh`.
Таблица и графики этапа 9 появляются после `./run_analysis.sh`.

---

## 1. Карта репозитория

| Пакет | Роль | Этапы |
|---|---|---|
| `navigator_description` | URDF робота и конфигурация RViz | 2 |
| `navigator_sim` | геометрия полигона, истинное движение, СШП, лидар, маркеры | 2–4 |
| `navigator_bringup` | launch-файлы, сборка всех узлов | 1–9 |
| `navigator_localization` | EKF-локализация по СШП | 5 |
| `navigator_slam` | сетка занятости, ориентиры, EKF-SLAM | 6–8 |
| `navigator_interfaces` | сообщения ориентиров и карты | 7–8 |
| `tools/` | сбор данных и отрисовка рисунков | 3–9 |

Полезные документы:

- `README.md` — запуск, топики, математика этапов 5–9;
- `WSL_SETUP.md` — установка Windows 11 + WSL 2 + Ubuntu 24.04 + ROS 2 Jazzy;
- `docs/stages7-9-contract.md` — контракт топиков и сообщений этапов 7–9;
- `PLAN.md` — исходный план и ADR (локальный файл, в git не входит).

---

## 2. Этапы задания

### Этап 1 — Настройка окружения

**Требование:** WSL 2, ROS 2, RViz, рабочее пространство; проверка запуска
узлов и обмена сообщениями.

**Где реализовано**

- `WSL_SETUP.md:13–36` — установка WSL 2 и Ubuntu 24.04;
- `WSL_SETUP.md:39–76` — ROS 2 Jazzy Desktop (включает RViz 2), `ros-dev-tools`,
  `python3-numpy`, `python3-yaml`, опционально `matplotlib`/`pytest`;
- `WSL_SETUP.md:80–112` — перенос проекта в файловую систему Linux и
  `colcon build --symlink-install` (6 пакетов);
- `WSL_SETUP.md:173–199`, `README.md:206–231` — `ros2 node list`,
  `ros2 topic list`, `ros2 topic hz`, `ros2 topic echo`, `tf2_echo`;
- `run.sh` — сборка + запуск одной командой; `ROS_SETUP` позволяет указать
  другой дистрибутив.

**Команды проверки**

```powershell
# Windows PowerShell
wsl --install -d Ubuntu-24.04
wsl -l -v                  # ожидается VERSION 2
```

```bash
# Ubuntu (WSL)
source /opt/ros/jazzy/setup.bash
cd ~/navigator_ws
colcon build --symlink-install
source install/setup.bash

ros2 pkg list | grep navigator      # 6 пакетов
ros2 launch navigator_bringup simulation.launch.py use_rviz:=false
```

Во втором терминале:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 node list
ros2 topic list
ros2 topic hz /scan_ideal            # ~10 Гц
ros2 topic hz /uwb/noisy_ranges      # ~10 Гц
ros2 run tf2_ros tf2_echo map base_footprint
```

Ожидаемо: 6 узлов без EKF; `map → odom → base_footprint` публикуется, TF
движется по квадрату.

---

### Этап 2 — Модель робота и карта полигона

**Требование:** упрощённая модель в URDF; стены, препятствия, СШП-якоря и
ориентиры SLAM; индивидуальная конфигурация полигона с объяснением выбора.

**Где реализовано**

- `navigator_description/urdf/navigator_m.urdf` — дифференциальный привод:
  `base_link` (корпус r = 0.24 м, h = 0.22 м), `left_wheel_joint` /
  `right_wheel_joint` (continuous, колея 0.42 м, r = 0.09 м), кастеры,
  `lidar_joint` в `(0.18, 0, 0.34)`, `uwb_tag_joint` в `(-0.15, 0.12, 0.46)`,
  `computer_link`. Материалы задают цвета звеньев (см. раздел 3).
- `navigator_sim/world.py` — единый источник геометрии:
  - `COLORS` — палитра сцены;
  - `WALLS` — внешние стены 8 × 8 м, толщина 0.12 м;
  - `BOX_OBSTACLES` — синяя Z-стена из трёх боксов;
  - `CYLINDER_OBSTACLES` — два цилиндра r = 0.30 и 0.42 м;
  - `TRIANGLE_OBSTACLE` — бирюзовая треугольная призма;
  - `UWB_ANCHORS` — A1–A4;
  - `LANDMARKS` — L1–L4 (r = 0.09 м) с индивидуальными цветами;
  - `LIDAR_OFFSET`, `UWB_TAG_OFFSET` — монтажные смещения.
- `navigator_sim/polygon_markers_node.py` — публикует всё это как
  `MarkerArray` в `/polygon_markers`, читая цвета из `world.COLORS`.
- `navigator_sim/lidar_model_node.py` — импортирует те же таблицы для
  ray-casting, поэтому картинка и измерения не расходятся.
- `navigator_bringup/launch/display.launch.py` — статичный показ модели и
  полигона без движения.

**Почему полигон такой**

- Арена 8 × 8 м: квадратная траектория 4 × 4 м (`1.5…5.5` м) оставляет по
  1.5 м внешнего коридора для якорей и объезда.
- Все препятствия отстоят от линии движения не менее чем на 0.45 м
  (робот r = 0.24 м + запас), поэтому визуально и по лидару робот ни во что
  не «въезжает»; модель столкновений в проекте не нужна.
- Z-стена, треугольник и цилиндры дают разные типы отражений: плоские грани,
  наклонные кромки и круглые дуги. Это проверяет ray-casting и детектор
  ориентиров на разных геометриях.
- Ориентиры L1–L4 стоят у четырёх стен и видны с соответствующих участков
  траектории; каждый цвет однозначно соответствует стороне света.
- СШП-якоря A1–A4 разнесены по углам, чтобы геометрия дальностей давала
  наблюдаемую позицию (курс по дальностям в статике не наблюдается —
  якобиан ранга 2, см. этап 5).
- Большой цилиндр отодвинут от якоря A4: их маркеры больше не пересекаются
  (зазор > 5 м).

**Команды проверки**

```bash
ros2 launch navigator_bringup display.launch.py
ros2 topic echo --once --qos-durability transient_local /robot_description | head
ros2 topic echo --once /polygon_markers | head
ros2 run tf2_ros tf2_echo map base_footprint
```

В RViz включены `RobotModel`, `TF`, `MarkerArray` (`/polygon_markers`) и
`Grid`. Для headless-проверки: `display.launch.py use_rviz:=false`.

---

### Этап 3 — Истинная траектория движения

**Требование:** кинематическая модель, траектория, публикация истинных
координат, курса и скоростей; воспроизводимые запуски.

**Где реализовано**

- `navigator_sim/motion_node.py:97–145` — дифференциальная модель:
  интегрирование `x, y, yaw`, клип последнего шага сегмента, обратная
  кинематика колёс `v_left = v − ωL/2`, `v_right = v + ωL/2`;
- `motion_node.py:68–76` — траектория: 4 прямых по 4 м + 4 поворота на π/2,
  повторяется `loops` раз; в конце — точное возвращение в старт;
- `motion_node.py:147–196` — публикация `/ground_truth/odom`,
  `/ground_truth/path`, `/cmd_vel`, `/joint_states` и TF
  `odom → base_footprint`;
- `navigator_sim/config/motion.yaml` — все параметры запуска (старт, сторона,
  скорости, шаг, колея, колесо, число кругов);
- `run.sh` / `simulation.launch.py` — одинаковый запуск на любой машине.

**Команды проверки**

```bash
ros2 launch navigator_bringup simulation.launch.py use_rviz:=false
```

Второй терминал:

```bash
ros2 topic hz /ground_truth/odom          # ~20 Гц (шаг 0.05 с)
ros2 topic echo --once /ground_truth/odom
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /joint_states
ros2 run tf2_ros tf2_echo map base_footprint
```

Ожидаемо: поза проходит квадрат `(1.5,1.5) → (5.5,1.5) → (5.5,5.5) →
(1.5,5.5) → (1.5,1.5)`; `v = 0.35 м/с`, `ω = 0.60 рад/с`.

---

### Этап 4 — Моделирование измерений

**Требование:** математические модели СШП-дальностей и лидарных сканов;
идеальные измерения, затем шум, смещения и пропуски; допущения.

**Где реализовано**

- СШП: `navigator_sim/uwb_model_node.py`
  - `tag_position()` — положение метки с учётом смещения и курса;
  - `timer_callback()` — 3D-дальность до каждого якоря, затем
    `bias + N(0, σ²)` и Bernoulli-пропуск (`NaN`), окно полного отказа
    `outage_start_s..outage_end_s`;
  - публикует `/uwb/ideal_ranges`, `/uwb/noisy_ranges`, `/uwb/anchors`;
- лидар: `navigator_sim/lidar_model_node.py`
  - `WorldGeometry` / `raycast` — пересечение луча с AABB, окружностями и
    отрезками треугольника;
  - `range_min = 0.12`, `range_max = 8.0`, 360 лучей, 10 Гц;
  - `z = clip(r + bias + N(0, σ²), r_min, r_max)`, пропуск → `+inf`;
  - публикует `/scan_ideal`, `/scan_noisy` (frame `lidar_link`);
- параметры и seed: `navigator_sim/config/sensors.yaml`
  (`random_seed: 12025`, σ/bias/dropout для обоих датчиков).

`/uwb/anchors` — разовый снимок при старте узла; в RViz якоря рисуются из
`/polygon_markers`, поэтому отдельный подписчик на `/uwb/anchors` не нужен.

**Принятые допущения**

- СШП-якоря — радиоинфраструктура, в лидарную геометрию не входят.
- Лидар — идеальный 2D-сканер в плоскости `z = 0.34`; шум аддитивный
  гауссов, смещение постоянное, пропуски независимые.
- Все препятствия выше плоскости лидара (0.45–0.85 м), поэтому 2D-модель
  корректно описывает их как вертикальные препятствия; сам корпус робота луч
  не загораживает (самоэкранирование не моделируется).
- Дальность СШП — евклидово 3D-расстояние между меткой и якорем; стены и
  препятствия радиосигнал не загораживают.
- Генераторы случайных чисел инициализированы фиксированным seed, поэтому
  прогоны воспроизводимы; порядок вызовов RNG не зависит от сценария.
- Модель движения идеальна: проскальзывания и шум управления в штатном
  режиме не моделируются (шум добавляют только датчики; в тестах SLAM
  управляющий шум задаётся отдельно).

**Команды проверки**

```bash
ros2 launch navigator_bringup simulation.launch.py use_rviz:=false
ros2 topic hz /scan_ideal                 # ~10 Гц
ros2 topic hz /scan_noisy                 # ~10 Гц
ros2 topic echo --once /scan_noisy | head
ros2 topic hz /uwb/noisy_ranges           # ~10 Гц
ros2 topic echo --once /uwb/noisy_ranges  # 4 числа, NaN = пропуск

./run_visualize.sh                        # ~62 с
```

Результаты: `results/result_map.png` (полигон + траектория + карта),
`results/result_scan.png` (идеальный и шумный сканы),
`results/result_uwb.png` (дальности A1–A4).

---

### Этап 5 — Локализация на основе EKF

**Требование:** прогноз по модели движения, коррекция по СШП; начальные
условия, матрицы шума, оценка ошибок координат и курса.

**Где реализовано**

- `navigator_localization/navigator_localization/ekf.py`
  - `EKFLocalization.predict()` — дифференциальная модель, якобиан процесса;
  - `EKFLocalization.update()` — батч-коррекция по всем валидным дальностям,
    `NaN` пропускается, форма Джозефа для ковариации;
  - `measurement_and_jacobian()` — модель дальности с учётом смещения метки;
- `navigator_localization/navigator_localization/ekf_node.py`
  - подписки `/cmd_vel`, `/uwb/noisy_ranges`, `/ground_truth/odom`;
  - публикации `/estimate/odom`, `/estimate/path`, `/estimate/error`;
  - RMSE в лог раз в секунду;
- `navigator_localization/config/ekf.yaml` — начальная поза `(1.5, 1.5, 0)`,
  σ = 0.10/0.10/0.0873, `Q = diag(1e-4, 1e-4, 1e-4)`, `R = uwb_sigma²`,
  смещение метки и якоря.

**Команды проверки**

```bash
ros2 launch navigator_bringup localization.launch.py use_rviz:=false
ros2 topic hz /estimate/odom          # ~20 Гц
ros2 topic echo --once /estimate/odom
ros2 topic echo --once /estimate/error   # x, y, yaw
./run_localization_metrics.sh            # 62 с
```

Тесты:

```bash
PYTHONPATH=src/navigator_localization:src/navigator_sim \
  python3 -m unittest discover -s src/navigator_localization/test -v   # 12 тестов
```

Результаты: `results/result_localization_trajectory.png`,
`results/result_localization_error.png`, RMSE в консоли.

Ограничение: одна СШП-метка в статике не наблюдает курс (якобиан ранга 2);
курс уточняется на поворотах. Это отражено в `README.md` и тестах.

---

### Этап 6 — Построение сетки занятости

**Требование:** обновлять свободные и занятые ячейки по лидарным измерениям и
оценённой позе; сравнить карты по истинной и оценённой позе.

**Где реализовано**

- `navigator_slam/occupancy_grid.py`
  - `LogOddsGrid.update_scan()` — свободные ячейки вдоль луча (шаг
    `resolution/2`), занятая ячейка в конце луча, лог-оддсы с клипом;
  - `occupancy_data()` — публикация `-1 / 0 / 100`;
  - `compare_occupancy_data()` / `compare_grids()` — IoU, согласие, расстояние
    до ближайшей занятой ячейки;
- `navigator_slam/occupancy_grid_node.py` — ROS-обёртка, буфер поз, выбор
  ближайшей по времени, публикация 1 Гц;
- `navigator_slam/config/occupancy.yaml` — 180 × 180, 0.05 м, origin
  `(-0.5, -0.5)`, `p_occupied = 0.70`, `p_free = 0.30`;
- `navigator_bringup/launch/mapping.launch.py` — `pose_source`:
  `estimate` (по умолчанию), `ground_truth`, `both` (два маппера на одних
  сканах);
- `tools/map_compare.py` + `run_map_compare.sh` — запись двух карт, PNG и
  метрики.

**Команды проверки**

```bash
ros2 launch navigator_bringup mapping.launch.py use_rviz:=false pose_source:=both
ros2 topic hz /map/estimate                 # ~1 Гц
ros2 topic echo --once /map/estimate        # width/height 180, data -1/0/100
ros2 topic echo --once /map/ground_truth
./run_map_compare.sh                        # 62 с
```

Тесты: `PYTHONPATH=src/navigator_slam:src/navigator_sim python3 -m unittest
discover -s src/navigator_slam/test -v` — входит в 177 тестов SLAM.

Результат: `results/result_map_compare.png`, IoU и среднее расстояние в
консоли. Ограничение: обе карты строятся по одним сканам, поэтому различие
отражает только ошибку позы.

---

### Этап 7 — Выделение и сопоставление ориентиров

**Требование:** получать из сканов наблюдения ориентиров (дальность и
направление); определять, наблюдался ли ориентир ранее.

**Где реализовано**

- `navigator_slam/landmark_detection.py`
  - `segment_clusters()` — кластеризация по адаптивному скачку дальности;
  - `fit_circle()` / `fit_arc()` — круг или дуга по хорде/стрелке;
  - `extract_landmarks()` — фильтры `min/max_points`, ширины, радиуса,
    residual; наблюдение в `base_footprint` (`range`, `bearing`, `radius`,
    дисперсии), смещение лидара скомпенсировано;
  - `associate_observations()` — махаланобисов гейт χ² 9.21, жадное
    сопоставление, новый id для несопоставленных;
- `navigator_slam/landmark_node.py` — `/landmarks/observations` каждый скан,
  `/landmarks/map` и `/landmarks/markers` при наличии позы; подтверждение
  кандидатов (`min_hits`, `tentative_gate`) через `tentative.py`;
- `navigator_interfaces/msg/*.msg` — `LandmarkObservation`,
  `LandmarkObservationArray`, `Landmark`, `LandmarkMap`;
- `navigator_slam/config/landmarks.yaml` — пороги кластеризации, радиус
  0.04–0.25 м (настоящие ориентиры r = 0.09 м, цилиндры 0.30/0.42 м
  отсекаются), гейт, `map_alpha`;
- `navigator_bringup/launch/landmarks.launch.py`.

**Команды проверки**

```bash
ros2 launch navigator_bringup landmarks.launch.py use_rviz:=false
ros2 topic hz /landmarks/observations       # ~10 Гц
ros2 topic echo --once /landmarks/observations
ros2 topic echo --once /landmarks/map       # id + ковариация
ros2 topic echo --once /landmarks/markers
```

Чистая проверка детектора (без ошибки EKF):

```bash
ros2 launch navigator_bringup landmarks.launch.py \
  pose_topic:=/ground_truth/odom use_rviz:=false
```

Тесты: 28 тестов в `test_landmark_detection.py`, включая сканы по реальной
геометрии `world.py`, шум, пропуски и устойчивость id.

---

### Этап 8 — Реализация EKF-SLAM

**Требование:** совместно оценивать позу и координаты неизвестных
ориентиров; проверить повторное наблюдение после замкнутой траектории.

**Где реализовано**

- `navigator_slam/ekf_slam.py`
  - состояние `mu = [x, y, θ, m1x, m1y, …]`, полная ковариация;
  - `predict()` — дифференциальная модель, кросс-корреляции с ориентирами
    сохраняются;
  - `update_uwb()` — та же модель, что на этапе 5;
  - `associate()` — по совместной ковариации (робот + ориентир), гейт χ²;
  - `add_landmark()` — обратная модель наблюдения и ковариация нового
    ориентира;
  - `update_landmarks()` — модель «дальность/азимут», аналитические якобианы,
    форма Джозефа;
  - `confirm_observation()` / `process_observations()` — подтверждение после
    `min_landmark_hits`, защита от фантомов на углах стен;
- `navigator_slam/ekf_slam_node.py` — подписки `/cmd_vel`,
  `/landmarks/observations`, `/uwb/noisy_ranges`, `/ground_truth/odom`;
  публикации `/slam/odom`, `/slam/path`, `/slam/landmarks`, `/slam/markers`,
  `/slam/error`; TF не публикует (им владеет `motion_node`);
- `navigator_slam/config/ekf_slam.yaml` — начальная поза, `Q`, СШП, гейт,
  `sigma_range/bearing`, `initial_landmark_std`, подтверждение;
- `navigator_bringup/launch/slam.launch.py` — симуляция без EKF этапа 5,
  `landmark_node` с пустым `pose_topic` и `ekf_slam_node`.

**Команды проверки**

```bash
ros2 launch navigator_bringup slam.launch.py use_rviz:=false
ros2 topic hz /slam/odom                 # ~20 Гц (прогноз)
ros2 topic echo --once /slam/landmarks   # 4 ориентира с id
ros2 topic echo --once /slam/error       # ошибка vs истина
```

Проверка замыкания петли (два круга, без ROS):

```bash
PYTHONPATH=src/navigator_slam:src/navigator_sim python3 -m unittest \
  src.navigator_slam.test.test_ekf_slam.TestLoopClosure.test_double_loop_closes_and_keeps_the_ids -v
```

Тест проверяет: после двух кругов число ориентиров равно 4, дубликатов нет,
ошибка позы и ориентиров остаётся в допуске. Дополнительно 49 тестов
`test_ekf_slam.py` покрывают ковариацию, ассоциацию, пропуски, отказ СШП и
подтверждение ориентиров.

Для **runtime-демонстрации** замкнутой траектории штатный
`motion.yaml` содержит `loops: 1`: поставьте `loops: 2` (в своей копии
конфигурации или локально) и запустите `slam.launch.py`; после второго круга
`/slam/landmarks` должен сохранить те же 4 id без дубликатов.

---

### Этап 9 — Анализ эффективности

**Требование:** сравнить точность локализации и карт при разных шумах,
размещении якорей и временном отсутствии СШП; представить графики, таблицы и
выводы.

**Где реализовано**

- `tools/analysis.py`
  - таблица сценариев `DEFAULT_PROFILE`: `baseline`,
    `uwb_sigma_0.02/0.10/0.20`, `uwb_mismatch_r_low`, `dropout_0.10/0.25`,
    `outage_20_40`, `anchors_wide`, `anchors_compact`;
  - для каждого сценария генерируются YAML `sensors_params` и `ekf_params`,
    запускается `mapping.launch.py pose_source:=both`, пишутся RMSE
    (`/estimate/error`) и метрики карт;
  - `--quick` — 3 сценария по 30 с; полный профиль — 10 сценариев по 62 с;
  - `--warmup` — сколько секунд после первого `/estimate/error` отбросить как
    переходный процесс (по умолчанию 3 с), чтобы стартовая ошибка EKF не
    определяла `max |e|`;
  - `--replot` — отрисовка из `analysis_data.npz` без ROS;
  - `summary.md` содержит таблицу и автоматические выводы;
- `tools/visualize.py`, `tools/localization_metrics.py`,
  `tools/map_compare.py` — рисунки этапов 3–6;
- `run_analysis.sh`, `run_visualize.sh`, `run_localization_metrics.sh`,
  `run_map_compare.sh` — обёртки запуска;
- `src/navigator_slam/test/test_analysis.py` — 49 тестов: слияние
  override-ов, генерация YAML, агрегация RMSE, метрики карт, таблица,
  выводы, `--replot`, команда запуска.

**Команды проверки**

```bash
./run_analysis.sh --quick          # ~3 мин, 3 сценария
./run_analysis.sh                  # ~13 мин, полная таблица
python3 tools/analysis.py --replot # без ROS, из сохранённого npz
```

Результаты:

| Файл | Содержимое |
|---|---|
| `results/analysis/summary.md` | таблица RMSE/IoU/расстояний + выводы |
| `results/analysis/summary.csv` | те же метрики машинно-читаемо |
| `results/analysis_rmse.png` | RMSE позы и курса по сценариям |
| `results/analysis_maps.png` | IoU и расстояние между картами |
| `results/analysis/analysis_data.npz` | данные для `--replot` |

Тесты этапа 9 входят в 177 тестов `navigator_slam`; отдельно:

```bash
PYTHONPATH=src/navigator_slam:src/navigator_sim python3 -m unittest \
  src.navigator_slam.test.test_analysis -v
```

---

## 3. Цвета и обозначения

### 3.1 Сцена (`world.COLORS`, `world.LANDMARKS`)

| Объект | RGB (0–255) | HEX | Где задано |
|---|---|---|---|
| Пол | 230, 232, 237 | `#E6E8ED` | `world.COLORS['floor']` |
| Внешние стены | 61, 66, 77 | `#3D424D` | `world.COLORS['walls']` |
| Z-стена (синяя) | 38, 97, 199 | `#2661C7` | `world.COLORS['box_obstacle']` |
| Цилиндры (оранжевые) | 242, 115, 20 | `#F27314` | `world.COLORS['cylinder_obstacle']` |
| Треугольник (бирюзовый) | 20, 166, 153 | `#14A699` | `world.COLORS['triangle_obstacle']` |
| СШП-якоря A1–A4 (янтарные) | 255, 140, 0 | `#FF8C00` | `world.COLORS['uwb_anchor']`, `uwb_model_node.py:223–225` |
| Подписи якорей | 26, 26, 26 | `#1A1A1A` | `world.COLORS['label']` |
| Ориентир L1 (юг) | 217, 20, 20 | `#D91414` | `world.LANDMARKS` |
| Ориентир L2 (восток) | 20, 191, 38 | `#14BF26` | `world.LANDMARKS` |
| Ориентир L3 (север) | 20, 64, 217 | `#1440D9` | `world.LANDMARKS` |
| Ориентир L4 (запад) | 242, 178, 13 | `#F2B20D` | `world.LANDMARKS` |

Геометрия сцены: внешние стены 0.12 × 0.50 м; Z-стена состоит из трёх боксов
высотой 0.70 м; цилиндры h = 0.60/0.85 м; треугольник h = 0.55 м; ориентиры
r = 0.09 м, h = 0.45 м.

### 3.2 Робот (`navigator_m.urdf`)

| Звено | RGB | HEX | Примечание |
|---|---|---|---|
| `base_link` (корпус) | 209, 214, 222 | `#D1D6DE` | светло-серый, r = 0.24 м, h = 0.22 м |
| `front_panel_link` | 217, 20, 20 | `#D91414` | красная передняя панель |
| Колёса | 10, 10, 10 | `#0A0A0A` | чёрные |
| Кастеры | 166, 171, 179 | `#A6ABB3` | серебристые |
| `lidar_link` | 0, 179, 217 | `#00B3D9` | голубой лидар |
| `computer_link` | 242, 178, 13 | `#F2B30D` | жёлтый вычислитель |
| `uwb_tag_link` | 230, 230, 230 | `#E6E6E6` | белая СШП-метка |

### 3.3 RViz (`navigator_description/config/navigator.rviz`)

| Элемент | Цвет | Комментарий |
|---|---|---|
| Фон | белый | `Background Color: 255;255;255` |
| `/ground_truth/path` | зелёный `#19FF00` | истинная траектория |
| `/estimate/path`, `/estimate/odom` | синий `#0055FF` | оценка EKF (этап 5) |
| `/ground_truth/odom` | красно-оранжевый `#FF1900` | стрелка истинной позы |
| `/scan_ideal` | серый `#A0A0A0` | идеальный лидар |
| `/scan_noisy` | голубой `#00FFFF` | шумный лидар |
| `/slam/path`, `/slam/odom` | оранжевый `#FF8C00` | EKF-SLAM, по умолчанию выключено |
| `/landmarks/markers` | зелёный `#1ABF40` | трекер ориентиров, белые подписи id |
| `/slam/markers` | цикл цветов | ориентиры SLAM: красный, зелёный, синий, жёлтый, фиолетовый, голубой; тёмные подписи id |
| `/map/*` | схема `map` RViz | `100` — занято (чёрный), `0` — свободно (белый), `-1` — неизвестно (серый) |

Важно: зелёные цилиндры `/landmarks/markers` — это **оценки трекера**, а не
истинные ориентиры. Истинные L1–L4 всегда нарисованы цветами из таблицы 3.1.
Ориентиры SLAM (`/slam/markers`) окрашены по id, поэтому их цвет не совпадает
с цветом реального ориентира — это сделано, чтобы различать соседние id.

### 3.4 Дисплеи RViz

Файл `navigator_description/config/navigator.rviz` включает:

- `Grid`, `RobotModel`, `TF` — сетка, модель и системы координат;
- `Path` (`/ground_truth/path`) и `Odometry` (`/ground_truth/odom`) — истина;
- `EKF path` (`/estimate/path`) и `EKF odometry` (`/estimate/odom`) — этап 5;
- `MarkerArray` (`/polygon_markers`) — стены, препятствия, якоря, ориентиры;
- `LaserScan` (`/scan_ideal`, серый) и `LaserScan` (`/scan_noisy`, голубой);
- `Map (estimated pose)` (`/map/estimate`) — этап 6; `Map (true pose)`
  (`/map/ground_truth`) выключен по умолчанию;
- `SLAM path`, `SLAM pose`, `SLAM landmarks` (`/slam/*`) — этап 8, выключены
  по умолчанию, включаются галочкой при `slam.launch.py`;
- `Landmark observations` (`/landmarks/markers`) — этап 7, выключен по
  умолчанию, включается при `landmarks.launch.py`.

---

## 4. Топики: кто что публикует

| Топик | Тип | Узел | Этап |
|---|---|---|---|
| `/ground_truth/odom`, `/ground_truth/path` | `nav_msgs/Odometry`, `Path` | `motion_node` | 3 |
| `/cmd_vel`, `/joint_states` | `geometry_msgs/Twist`, `sensor_msgs/JointState` | `motion_node` | 3 |
| `/scan_ideal`, `/scan_noisy` | `sensor_msgs/LaserScan` | `lidar_model_node` | 4 |
| `/uwb/ideal_ranges`, `/uwb/noisy_ranges`, `/uwb/anchors` | `Float32MultiArray`, `MarkerArray` | `uwb_model_node` | 4 |
| `/polygon_markers` | `MarkerArray` | `polygon_markers` | 2 |
| `/estimate/odom`, `/estimate/path`, `/estimate/error` | `Odometry`, `Path`, `Vector3Stamped` | `ekf_node` | 5 |
| `/map/estimate`, `/map/ground_truth` | `nav_msgs/OccupancyGrid` | `occupancy_grid_ekf`, `occupancy_grid_gt` | 6 |
| `/landmarks/observations` | `navigator_interfaces/LandmarkObservationArray` | `landmark_node` | 7 |
| `/landmarks/map`, `/landmarks/markers` | `LandmarkMap`, `MarkerArray` | `landmark_node` | 7 |
| `/slam/odom`, `/slam/path`, `/slam/error` | `Odometry`, `Path`, `Vector3Stamped` | `ekf_slam_node` | 8 |
| `/slam/landmarks`, `/slam/markers` | `LandmarkMap`, `MarkerArray` | `ekf_slam_node` | 8 |

---

## 5. Тесты и статические проверки

| Команда | Что проверяет | Ожидаемо |
|---|---|---|
| `PYTHONPATH=src/navigator_localization:src/navigator_sim python3 -m unittest discover -s src/navigator_localization/test -v` | EKF, конфиг, смещение метки | 12 тестов, OK |
| `PYTHONPATH=src/navigator_slam:src/navigator_sim python3 -m unittest discover -s src/navigator_slam/test -v` | сетка, ориентиры, EKF-SLAM, анализ | 177 тестов, OK |
| `PYTHONPATH=src/navigator_sim python3 -m unittest discover -s src/navigator_sim/test -v` | геометрия полигона, цвета, URDF | 8 тестов, OK |
| `python3 -m py_compile $(find src tools -name '*.py')` | синтаксис всех модулей | без ошибок |
| `colcon test && colcon test-result --verbose` | тесты всех пакетов через colcon | без ошибок, `navigator_sim` больше не падает с «NO TESTS RAN» |
| `ros2 doctor` | окружение ROS | без критических ошибок |

Тесты не требуют ROS: чистые модули не импортируют `rclpy` и сообщения,
поэтому запускаются и на машине без ROS (что и было сделано при оформлении).

---

## 6. Полный чек-лист приёмки в WSL

1. **Сборка**

   ```bash
   cd ~/navigator_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Этапы 2–4** (модель, движение, датчики)

   ```bash
   ros2 launch navigator_bringup display.launch.py
   ros2 launch navigator_bringup simulation.launch.py
   # второй терминал: ros2 node list / topic list / topic hz
   ```

3. **Этап 5**

   ```bash
   ros2 launch navigator_bringup localization.launch.py
   ros2 topic echo --once /estimate/error
   ./run_localization_metrics.sh
   ```

4. **Этап 6**

   ```bash
   ros2 launch navigator_bringup mapping.launch.py pose_source:=both
   ros2 topic echo --once /map/estimate
   ./run_map_compare.sh
   ```

5. **Этап 7**

   ```bash
   ros2 launch navigator_bringup landmarks.launch.py
   ros2 topic echo --once /landmarks/observations
   ros2 topic echo --once /landmarks/map
   ```

6. **Этап 8**

   ```bash
   ros2 launch navigator_bringup slam.launch.py
   ros2 topic hz /slam/odom
   ros2 topic echo --once /slam/landmarks
   ros2 topic echo --once /slam/error
   ```

7. **Этап 9**

   ```bash
   ./run_analysis.sh --quick
   cat results/analysis/summary.md
   ```

8. **Тесты**

   ```bash
   colcon test
   colcon test-result --verbose
   ```

Если все пункты 1–8 проходят, проект готов к слиянию в `main`.

---

## 7. Известные ограничения и замечания

- **Runtime-артефакты этапа 9.** В репозитории нет `results/analysis/*`:
  таблица и графики создаются только после запуска `run_analysis.sh` в
  среде с ROS. В текущей среде (macOS без ROS) полный прогон не выполнялся;
  код и 49 тестов этапа 9 проверены статически.
- **Рисунки `results/*.png`.** Три файла в `results/` сделаны до последней
  перестановки препятствий. Пересоздать: `./run_visualize.sh`.
- **Курс по СШП.** Одна метка в статике не наблюдает курс; это физическое
  свойство модели, а не дефект. Курс уточняется на поворотах.
- **Числа в README/PLAN.** Часть числовых результатов этапов 6–8 получена до
  перестановки препятствий (коммиты `f3df9d3`, `bb5ea54`). Код и тесты
  соответствуют новой сцене, но точные метрики в тексте нужно подтвердить
  повторным прогоном `run_map_compare.sh`, `slam.launch.py` и
  `run_analysis.sh`.
- **ROS-обёртки.** Unit-тестами покрыта математика (чистые модули); сами ROS-узлы
  (`occupancy_grid_node`, `landmark_node`, `ekf_slam_node`) проверяются
  запуском launch-файлов и `ros2 topic …`, а не unit-тестами.
- **Шум этапа 9.** Сценарная таблица варьирует σ СШП, пропуски, отказ и
  раскладку якорей; шум лидара зафиксирован (`lidar_sigma = 0.02`). Это
  соответствует требованию «разные шумы» в части СШП; при необходимости
  добавьте сценарий с `lidar_sigma` в `tools/analysis.py`.
- **TF.** `odom → base_footprint` публикует только `motion_node`; EKF и
  EKF-SLAM публикуют позы в `map`, но не TF, чтобы не было конфликта.
  `map → odom` — фиксированная identity-трансформация симуляции.
- **Столкновения.** Физическая модель столкновений отсутствует; вместо неё
  геометрия полигона проверена на зазор ≥ 0.45 м до траектории
  (робот r = 0.24 м).
- **Анкоры и лидар.** СШП-якоря не входят в лидарную геометрию (радио не
  отражается лидаром); они видны только как маркеры.
- **Задержка наблюдений.** Наблюдения ориентиров применяются по текущей позе
  фильтра, без интерполяции на штамп скана; на номинальной скорости 0.35 м/с
  это даёт смещение порядка 3–4 см. Для курсового уровня это принято как
  допущение; при необходимости добавьте буфер поз, как в
  `occupancy_grid_node.py`.
- **Шум процесса.** `Q` в EKF задаётся на один шаг прогноза и не масштабируется
  на `dt`; при штатном таймере 20 Гц это эквивалентно постоянному шуму.
  Матрицы вынесены в YAML и легко настраиваются.
- **Смещения СШП.** В модели якорей есть постоянные смещения
  `uwb_biases = [0.02, −0.03, 0.04, 0.01]` м (`sensors.yaml`), а фильтр
  считает их нулевыми: часть систематической ошибки остаётся
  нескомпенсированной. Это осознанное упрощение этапа 5.
- **Дискретное время.** Симуляция использует фиксированный шаг таймера и не
  подключает `/clock`; геометрическая траектория детерминирована, но при
  задержках callback-ов привязка позы ко времени может немного отличаться
  между запусками. Для отчёта важна повторяемость формы траектории и
  фиксированный seed датчиков.
