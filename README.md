# UAV ROS 2 Workspace – Quick Setup

Follow these steps to create a clean workspace called **`uav`**, clone this package, and build everything with **colcon**.

---

## 1 · Create the workspace skeleton
```bash
# make new workspace in your home folder
mkdir -p ~/uav/src
cd ~/uav

# bootstrap build + install directories (empty first build)
colcon build
```
## 2 · Clone this package
```bash
cd ~/uav/src
git clone https://github.com/<your‑org>/<your‑package>.git   # ← replace with real URL
```
## 3 · Build the workspace
```bash
cd ~/uav
colcon build --symlink-install
source ~/uav/install/setup.bash
```
