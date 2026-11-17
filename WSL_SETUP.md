# Запуск проекта в Windows 11 + WSL 2 (Ubuntu 24.04, ROS 2 Jazzy)

Пошаговая инструкция: от чистой Windows до работающей симуляции.
Проект — папка `navigator_ws` (внутри `src/` три пакета: `navigator_description`,
`navigator_sim`, `navigator_bringup`).

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

Опционально (нужно только для `run_visualize.sh`):

```bash
sudo apt install -y python3-numpy python3-matplotlib
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

Ожидаемо: `Summary: 3 packages finished` (`navigator_description`, `navigator_sim`,
`navigator_bringup`).

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

Остановить — `Ctrl+C`.

---

## 6. Проверка (второй терминал Ubuntu)

```bash
cd ~/navigator_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 pkg list | grep navigator             # 3 пакета
ros2 node list                              # 6 узлов, без дублей
ros2 topic list                             # список топиков
ros2 topic hz /scan_ideal                   # ~10 Гц
ros2 topic hz /uwb/noisy_ranges             # ~10 Гц
ros2 topic echo --once /uwb/noisy_ranges    # 4 дальности (NaN = пропуск)
ros2 topic echo --once /ground_truth/odom   # истинная поза
ros2 run tf2_ros tf2_echo map base_footprint
```

Ожидаемо: 6 узлов, все топики из `README.md`, частоты ~10 Гц, TF движется
по квадрату 4 × 4 м со скоростью 0.35 м/с.

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
| После `Ctrl+C` остались процессы | `pkill -9 -f lib/navigator_sim` |
| `plot_data.npz`/PNG не создались | не установлены `python3-numpy` и `python3-matplotlib` — см. раздел 2 |
