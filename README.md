# navigator_ws — симуляция робота «Навигатор-М»

Рабочее пространство ROS 2 проекта **navigator**: модель робота, карта полигона,
истинная квадратная траектория, модели СШП-дальностей и лидарных сканов.

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
├── results/                      # готовые рисунки для отчёта
├── tools/visualize.py            # сбор данных из топиков и отрисовка
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
    └── navigator_bringup/        # launch-файлы
        ├── CMakeLists.txt, package.xml
        └── launch/{simulation.launch.py, display.launch.py}
```

Разделение по ролям: `_description` можно переиспользовать с реальным роботом,
`_sim` заменяется на драйверы датчиков, `_bringup` — единственное место, где пакеты
собираются вместе. Следующие этапы (5–9) добавляются рядом как
`navigator_localization`, `navigator_slam`, `navigator_interfaces`.

## Требования

- Ubuntu 24.04 + ROS 2 Jazzy: `ros-jazzy-desktop` (включает RViz2), `ros-dev-tools` (colcon).
- Только для рисунков: `python3-numpy`, `python3-matplotlib`.

## Сборка (один раз)

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Ожидаемо: `Summary: 3 packages finished` (`navigator_description`, `navigator_sim`,
`navigator_bringup`).

## Скрипты

| Команда | Что делает |
|---|---|
| `./run.sh [аргументы launch]` | `colcon build --symlink-install`, `source install/setup.bash`, `ros2 launch navigator_bringup simulation.launch.py`. Аргументы пробрасываются в launch: `./run.sh use_rviz:=false`. Собирает при каждом запуске (~пара секунд), чтобы не стартовать устаревший код. |
| `./run_visualize.sh [секунды]` | Запускает `tools/visualize.py` в фоне, поднимает headless-симуляцию, пишет данные (62 с по умолчанию, можно `./run_visualize.sh 90`), рендерит PNG в `results/`, затем гасит симуляцию. **Требует уже собранный `install/`** — после правок кода сначала `./run.sh` или `colcon build`. |
| `python3 tools/visualize.py --replot` | Перерисовывает графики из сохранённого `results/plot_data.npz` без запуска симуляции. Дополнительно: `--duration N`, `--out DIR`. |

Оба shell-скрипта берут дистрибутив ROS из `/opt/ros/jazzy/setup.bash`; другой путь можно
задать переменной, например `ROS_SETUP=/opt/ros/humble/setup.bash ./run.sh`.

## Запуск вручную

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# движение + СШП + лидар + карта + RViz
ros2 launch navigator_bringup simulation.launch.py

# без графики
ros2 launch navigator_bringup simulation.launch.py use_rviz:=false

# статичный показ модели и карты (без движения, этап 2 отчёта)
ros2 launch navigator_bringup display.launch.py use_rviz:=false
```

У обоих launch-файлов есть аргумент `use_rviz` (по умолчанию `true`).

## Узлы и топики

| Узел (пакет) | Публикует |
|---|---|
| `robot_state_publisher` | `/robot_description`, `/tf` (звенья робота) |
| `static_transform_publisher` | `/tf_static` (`map → odom`, identity) |
| `motion_node` (`navigator_sim`) | `/ground_truth/odom`, `/ground_truth/path`, `/cmd_vel`, `/joint_states`, `/tf` (`odom → base_footprint`) |
| `polygon_markers` (`navigator_sim`) | `/polygon_markers` |
| `uwb_model_node` (`navigator_sim`) | `/uwb/ideal_ranges`, `/uwb/noisy_ranges`, `/uwb/anchors` |
| `lidar_model_node` (`navigator_sim`) | `/scan_ideal`, `/scan_noisy` |

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

## Проверка

Второй терминал, при запущенной симуляции:

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 pkg list | grep navigator             # 3 пакета
ros2 node list                             # 6 узлов без RViz (7 с RViz), без дублей
ros2 topic list                            # все топики из таблицы выше
ros2 topic hz /scan_ideal                  # ~10 Гц
ros2 topic hz /uwb/noisy_ranges            # ~10 Гц
ros2 topic echo --once /uwb/noisy_ranges   # 4 дальности (NaN = пропуск)
ros2 topic echo --once /ground_truth/odom  # истинная поза
ros2 run tf2_ros tf2_echo map base_footprint
```

Ожидаемо: TF `map → base_footprint` движется по квадрату 4 × 4 м со скоростью 0.35 м/с,
лидар даёт 360 конечных дальностей в диапазоне ≈ 0.97–7.32 м.

## Остановка

`Ctrl+C` в терминале запуска. Если после аварийного прерывания остались процессы:

```bash
pkill -9 -f lib/navigator_sim
pkill -9 -f /opt/ros/jazzy/lib/robot_state_publisher
pkill -9 -f /opt/ros/jazzy/lib/tf2_ros/static_transform_publisher
```

## Параметры

- `navigator_sim/config/motion.yaml` — старт (1.5; 1.5; 0), сторона 4.0 м, v = 0.35 м/с,
  ω = 0.60 рад/с, Δt = 0.05 с, колея 0.42 м, колесо r = 0.09 м, `loops`.
- `navigator_sim/config/sensors.yaml` — seed 12025, σ/смещения/пропуски СШП и лидара,
  дальности лидара 0.12–8.0 м, 360 лучей, 10 Гц.
- `navigator_description/urdf/navigator_m.urdf` — геометрия «Навигатор-М»
  (лидар `0.18 0 0.34`, СШП-метка `-0.15 0.12 0.46`, колея 0.42 м).

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

Сырые данные прогона сохраняются в `results/plot_data.npz`. Повторная отрисовка без
запуска симуляции:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 tools/visualize.py --replot
```

## RViz

`navigator_description/config/navigator.rviz` задаёт вид сверху: модель робота, TF,
траекторию, маркеры полигона и два лидарных скана (`/scan_ideal`, `/scan_noisy`),
Fixed Frame — `map`. В Windows 11 окно RViz открывается через WSLg; на headless-машине
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
| RViz не открывается | Windows 10 без X-сервера — используйте `use_rviz:=false` или настройте `DISPLAY` |
| Сборка идёт очень долго | проект лежит в `/mnt/c` — скопируйте в `~` (см. `WSL_SETUP.md`, раздел 3) |

Подробности по установке WSL и ROS 2 Jazzy — в [`WSL_SETUP.md`](WSL_SETUP.md).
