# ROS 2 Jazzy Setup Reference

## Architecture Overview

ROS 2 Jazzy Jalisco is installed inside a containerized Ubuntu 24.04 Noble environment via **Distrobox** backed by **Podman**, isolating ROS 2 dependencies from Arch/Omarchy rolling release updates while providing native desktop integration (shared `$HOME`, X11/Wayland display, and networking).

```
Host (Omarchy / Arch Linux)
 ├── Podman (rootless container runtime)
 ├── Distrobox container: `ros2-jazzy` (Ubuntu 24.04 LTS)
 │    ├── ROS 2 Jazzy Jalisco Desktop (`/opt/ros/jazzy`)
 │    ├── Build tools: `colcon`, `rosdep`, `build-essential`
 │    └── Shared `$HOME` filesystem
 ├── Host Wrappers: `ros2`, `colcon`, `ros2-shell` in `~/.local/bin/`
 └── Desktop Launcher: `~/.local/share/applications/rviz2.desktop`
```

---

## What Got Installed & Where

| Component | Location | How It Was Installed |
| :--- | :--- | :--- |
| **Podman & Distrobox** | `/usr/bin/podman`<br>`/usr/bin/distrobox` | Host pacman: `sudo pacman -S podman distrobox` |
| **Container Image & Files** | `~/.local/share/containers/storage/` | `distrobox create --name ros2-jazzy --image ubuntu:24.04` |
| **ROS 2 Jazzy Desktop** | Container `/opt/ros/jazzy/` | Container apt: `ros-jazzy-desktop` from Open Robotics apt repo |
| **ROS Build Tools** | Container `/usr/bin/` | Container apt: `python3-colcon-common-extensions`, `python3-rosdep`, `build-essential` |
| **Workspace** | `/home/vj/ros2_ws/` | Standard directory created in shared home |
| **`ros2` CLI Wrapper** | `/home/vj/.local/bin/ros2` | Host bash wrapper invoking container `ros2` with env sourced |
| **`colcon` CLI Wrapper** | `/home/vj/.local/bin/colcon` | Host bash wrapper invoking container `colcon` with env sourced |
| **`ros2-shell` Shortcut** | `/home/vj/.local/bin/ros2-shell` | Wrapper script executing `distrobox enter ros2-jazzy -- bash` |
| **RViz2 Desktop Entry** | `/home/vj/.local/share/applications/rviz2.desktop` | Standard `.desktop` entry for host application menus |

---

## Daily Usage

### 1. Direct Host Commands
Because wrappers are in `~/.local/bin` (on your `$PATH`), run ROS 2 commands directly from any host terminal:
```bash
# Check ROS 2 version / environment
ros2 --help

# Run demo publisher
ros2 run demo_nodes_cpp talker

# Build packages in workspace
cd ~/ros2_ws
colcon build --symlink-install
```

### 2. Interactive Container Shell
For development sessions requiring frequent internal commands, compilers, or interactive bash scripts:
```bash
ros2-shell
```
Inside the container, `/opt/ros/jazzy/setup.bash` is automatically sourced via `~/.bashrc`.

### 3. Launching RViz2 (GUI)
- **App Menu**: Launch **RViz2 (ROS 2 Jazzy)** from your desktop application launcher.
- **Terminal**: Run `rviz2` inside `ros2-shell`, or run on host:
  ```bash
  distrobox enter ros2-jazzy -- rviz2
  ```

---

## Container Maintenance

```bash
# Check container status
distrobox list

# Enter container as root (to install extra apt packages)
distrobox enter --root ros2-jazzy

# Stop container
distrobox stop ros2-jazzy
```
