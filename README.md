# UAV ROS 2 Workspace 

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
git clone https://github.com/konthamaashini/uav.git
```
## 3 · Build the workspace
```bash
cd ~/uav
colcon build --symlink-install
source ~/uav/install/setup.bash
```
The fish geometry was modeled in Onshape (https://cad.onshape.com/documents/aaca298587ed8e68033344ff/w/b1920aaef25add5d3ec70db8/e/f61702870397c57547016114?renderMode=0&uiState=682af8d07aab7f5080a9108d).
![image](https://github.com/user-attachments/assets/363d55a5-c043-4d59-a68d-9513f89ab172)


Using the Onshape‑to‑URDF exporter with our config.json, we generated the URDF.
All exported STL meshes are stored in meshes/.
The resulting URDF is located at urdf/fish.urdf.
Both the meshes and urdf/fish.urdf are contained within the fish_hpurv package.
