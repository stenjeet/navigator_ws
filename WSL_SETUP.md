# Запуск проекта в Windows 11 + WSL 2 (Ubuntu 24.04, ROS 2 Jazzy)

Пошаговая инструкция: от чистой Windows до работающей симуляции.
Проект — папка `navigator_ws` (внутри `src/` шесть пакетов: `navigator_description`,
`navigator_sim`, `navigator_bringup`, `navigator_localization`, `navigator_slam`,
`navigator_interfaces`).

> Соответствие версий: **Ubuntu 24.04 → ROS 2 Jazzy**. ROS 2 Humble ставится только
> на Ubuntu 22.04, поэтому здесь он не используется.

---

## 1. Установить WSL 2 и Ubuntu 24.04

В **PowerShell от имени администратора**:

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
```

После перезагрузки при первом запуске Ubuntu задать имя пользователя и пароль.
Проверка версии WSL:

```powershell
wsl -l -v
```

Должно быть `VERSION 2`. Если `1`:

```powershell
wsl --set-version Ubuntu-24.04 2
```

Все дальнейшие команды выполняются **внутри Ubuntu** (терминал «Ubuntu-24.04»).

---

## 2. Установить ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo apt update

# официальный ROS-репозиторий
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y

# ROS 2 Jazzy Desktop (включает RViz2) + инструменты сборки
sudo apt install -y ros-jazzy-desktop ros-dev-tools

echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Проверка: `printenv ROS_DISTRO` → `jazzy`.

Обязательно для EKF-локализации (этап 5):

```bash
sudo apt install -y python3-numpy python3-yaml
```

Опционально (рисунки `run_visualize.sh` и запуск тестов):

```bash
sudo apt install -y python3-matplotlib python3-pytest
```

---

## 3. Скопировать проект в файловую систему Linux

Собирать в `/mnt/c/...` нельзя: медленно и ломаются права. Проект должен лежать
в домашней папке WSL, например `~/navigator_ws`.

Если папка лежит в Windows, например `C:\Users\<Имя>\Downloads\navigator_ws`:

```bash
cp -r "/mnt/c/Users/<Имя>/Downloads/navigator_ws" ~/navigator_ws
cd ~/navigator_ws
ls        # README.md  WSL_SETUP.md  run.sh  src/
```

Если скачан zip-архив:

```bash
cd ~
unzip "/mnt/c/Users/<Имя>/Downloads/navigator_ws.zip"
```

---

## 4. Собрать

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Ожидаемо: `Summary: 6 packages finished` (`navigator_description`, `navigator_sim`,
`navigator_bringup`, `navigator_localization`, `navigator_slam`,
`navigator_interfaces`).

---

## 5. Запустить

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# с RViz (Windows 11 — WSLg, окно откроется само):
ros2 launch navigator_bringup simulation.launch.py

# без графики (гарантированно работает на любой конфигурации):
ros2 launch navigator_bringup simulation.launch.py use_rviz:=false
```

Короткий вариант — скрипт (сборка + запуск):

```bash
./run.sh                     # RViz
./run.sh use_rviz:=false     # headless
```

Статичный показ модели и карты (этап 2 отчёта):

```bash
ros2 launch navigator_bringup display.launch.py
```

Этап 5 — симуляция + EKF-локализация:

```bash
ros2 launch navigator_bringup localization.launch.py
# эквивалент:
ros2 launch navigator_bringup simulation.launch.py use_localization:=true
```

Этап 6 — сетка занятости по оценённой позе (EKF запускается автоматически):

```bash
ros2 launch navigator_bringup mapping.launch.py
# карта по истинной позе (без EKF):
ros2 launch navigator_bringup mapping.launch.py pose_source:=ground_truth
# обе карты одновременно — для сравнения:
ros2 launch navigator_bringup mapping.launch.py pose_source:=both
```

Этап 7 — выделение ориентиров (поза из EKF) и этап 8 — EKF-SLAM:

```bash
ros2 launch navigator_bringup landmarks.launch.py     # ориентиры + карта трекера
ros2 launch navigator_bringup slam.launch.py          # совместный EKF-SLAM
```

Остановить — `Ctrl+C`.

---

## 6. Проверка (второй терминал Ubuntu)

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 pkg list | grep navigator             # 6 пакетов
ros2 node list                              # 6 без EKF, 7 с EKF, 9 с картами, 10 со SLAM
ros2 topic list                             # список топиков
ros2 topic hz /scan_ideal                   # ~10 Гц
ros2 topic hz /uwb/noisy_ranges             # ~10 Гц
ros2 topic echo --once /uwb/noisy_ranges    # 4 дальности (NaN = пропуск)
ros2 topic echo --once /ground_truth/odom   # истинная поза
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /estimate/odom                # ~20 Гц (если EKF включён)
ros2 topic echo --once /estimate/error      # ошибка оценки (этап 5)
ros2 topic hz /map/estimate                 # ~1 Гц (этап 6, если запущен маппер)
ros2 topic echo --once /map/estimate         # OccupancyGrid: -1/0/100
ros2 topic hz /landmarks/observations        # ~10 Гц (этап 7)
ros2 topic hz /slam/odom                     # ~20 Гц (этап 8)
ros2 topic echo --once /slam/landmarks       # карта ориентиров EKF-SLAM
ros2 topic echo --once /slam/error           # ошибка SLAM vs истина
```

Ожидаемо: 6 узлов без EKF (7 с `use_localization:=true`), все топики из
`README.md`, частоты ~10 Гц, TF движется по квадрату 4 × 4 м со скоростью
0.35 м/с; `/estimate/odom` — ~20 Гц.

### Проверка EKF (этап 5)

Unit-тесты (без запуска симуляции):

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
PYTHONPATH=src/navigator_localization:src/navigator_sim \
  python3 -m unittest discover -s src/navigator_localization/test -v
# или через colcon:
colcon test --packages-select navigator_localization
colcon test-result --verbose
```

Запуск с EKF и проверка (второй терминал):

```bash
ros2 launch navigator_bringup localization.launch.py use_rviz:=false
ros2 topic hz /estimate/odom
ros2 topic echo --once /estimate/odom
ros2 topic echo --once /estimate/error
```

В логе `ekf_node` раз в секунду печатается `RMSE x=... y=... yaw=...`.
На зашумлённых СШП ошибка положения — единицы сантиметров. Курс по одной
метке в статике не наблюдается, поэтому он уточняется на поворотах.

Графики ошибок и итоговый RMSE одной командой:

```bash
cd ~/navigator_ws
./run_localization_metrics.sh          # 62 с
./run_localization_metrics.sh 90       # 90 с
```

Результаты — `results/result_localization_trajectory.png` (истина vs оценка)
и `results/result_localization_error.png` (ошибки `x`, `y`, `yaw`); RMSE
печатается в консоль.

### Проверка сетки занятости (этап 6)

Unit-тесты (без ROS):

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
PYTHONPATH=src/navigator_slam:src/navigator_sim \
  python3 -m unittest discover -s src/navigator_slam/test -v
colcon test --packages-select navigator_slam
```

Запуск и проверка (второй терминал):

```bash
ros2 launch navigator_bringup mapping.launch.py use_rviz:=false pose_source:=both
ros2 topic hz /map/estimate            # ~1 Гц
ros2 topic echo --once /map/estimate   # width/height 180, resolution 0.05, data из -1/0/100
```

В логе `occupancy_grid_ekf` раз в секунду печатаются число сканов и количество
занятых/свободных/неизвестных ячеек. Сравнение карт одной командой:

```bash
cd ~/navigator_ws
./run_map_compare.sh          # 62 с
./run_map_compare.sh 90       # 90 с
```

Результат — `results/result_map_compare.png` (карта по истинной позе, карта по
оценке и карта различий) плюс IoU занятых ячеек и доля совпавших ячеек в консоли.
Повторная отрисовка: `python3 tools/map_compare.py --replot`.

### Проверка ориентиров и EKF-SLAM (этапы 7-8)

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select navigator_slam && colcon test-result --verbose   # 154 теста
```

Запуск: `landmarks.launch.py` поднимает симуляцию + EKF + `landmark_node`
(`/landmarks/observations`, `/landmarks/map`, `/landmarks/markers`);
`slam.launch.py` — симуляцию без EKF, `landmark_node` с пустым `pose_topic`
(только наблюдения) и `ekf_slam_node` (`/slam/odom`, `/slam/path`,
`/slam/landmarks`, `/slam/markers`, `/slam/error`). В логе `ekf_slam_node`
раз в секунду печатается RMSE позы относительно истины.

Проверки:

```bash
ros2 topic echo --once /landmarks/observations   # 1-4 ориентира, base_footprint
ros2 topic echo --once /slam/landmarks           # 4 ориентира с id и ковариацией
ros2 topic echo --once /slam/error               # x, y, yaw vs /ground_truth/odom
```

### Проверка анализа эффективности (этап 9)

```bash
cd ~/navigator_ws
./run_analysis.sh --quick      # 3 сценария по 30 с (~3 мин)
./run_analysis.sh              # 10 сценариев по 62 с (~13 мин)
```

Результаты: `results/analysis/summary.md` (таблица + выводы),
`results/analysis/summary.csv`, `results/analysis_rmse.png`,
`results/analysis_maps.png`. Повторная отрисовка:
`python3 tools/analysis.py --replot`.

---

## 7. Картинки для отчёта (опционально)

Скрипт запускает headless-симуляцию, пишет ~62 с данных и рендерит PNG в `results/`:

```bash
cd ~/navigator_ws
./run_visualize.sh          # 62 с
./run_visualize.sh 90       # 90 с
```

Повторная отрисовка из сохранённых данных (без симуляции):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 tools/visualize.py --replot
```

---

## 8. Частые проблемы

| Симптом | Причина / решение |
|---|---|
| `ros2: command not found` | не подключены `source /opt/ros/jazzy/setup.bash` и `source install/setup.bash` |
| `E: Unable to locate package ros-jazzy-desktop` | не выполнен `apt update` после репозитория или Ubuntu не 24.04 |
| Сборка долгая / ошибки прав | проект в `/mnt/c` — скопировать в `~` (раздел 3) |
| `colcon build` ведёт себя странно | запускать из корня `~/navigator_ws`, не из `src` |
| RViz не открывается (Windows 10) | WSLg нет: установить VcXsrv/Xming и задать `export DISPLAY=$(ip route show default \| awk '{print $3}'):0.0` |
| RViz падает с ошибкой OpenGL | `sudo apt install -y mesa-utils libgl1-mesa-dri` и перезапустить WSL (`wsl --shutdown`) |
| После `Ctrl+C` остались процессы | `pkill -9 -f lib/navigator_sim; pkill -9 -f lib/navigator_slam` |
| `plot_data.npz`/PNG не создались | не установлены `python3-numpy` и `python3-matplotlib` — см. раздел 2 |
| `ModuleNotFoundError: No module named 'numpy'` | не установлен `python3-numpy` — `sudo apt install -y python3-numpy` |
| `Package 'navigator_localization' not found` | workspace не пересобран — `colcon build --symlink-install`, затем `source install/setup.bash` |
| EKF не стартует | по умолчанию `use_localization:=false`; используйте `localization.launch.py` или `use_localization:=true` |
| `/map/estimate` пустой или отсутствует | маппер не запущен или поза не найдена: `mapping.launch.py` (`pose_source:=estimate`/`both`) и непустой `/estimate/odom` |
| В `map_compare.py` «no map on ...» | запущен `mapping.launch.py` без `pose_source:=both` |
| `/landmarks/observations` молчит | не запущен `landmark_node` (`landmarks.launch.py` / `slam.launch.py`) или в поле зрения нет ориентиров |
| `/slam/odom` отсутствует | не запущен `slam.launch.py`; EKF-локализация этапа 5 для SLAM не нужна |
| `analysis.py` падает на старте сценария | EKF не успел опубликовать `/estimate/error` — увеличьте `--startup-timeout` |
| «sensor параметры не применились» | проверьте секции `sensors.yaml`: имя секции должно совпадать с именем ноды (`uwb_model_node`, `lidar_model_node`) |
