# UAV ROS 2 Workspace 

Follow these steps to create a clean workspace called **`uav`**, clone this package, and build everything with **colcon**.

---

## 1 · Create the workspace skeleton
```bash
# make new workspace in your home folder
mkdir -p ~/uav
cd ~/uav

# bootstrap build + install directories (empty first build)
colcon build
```
## 2 · Clone this package
```bash
git clone https://github.com/konthamaashini/src.git
```
## 3 · Build the workspace
```bash
cd ~/uav
colcon build --symlink-install
source ~/uav/install/setup.bash
```
## 4 · Run the display launch file
```bash
ros2 launch fish_hpurv display.launch.py
```


## Gazebo Environment Setup
The following code lines are included in the launch file (e.g., `display.launch.py`) to configure Gazebo's plugin and model paths and define file paths for the simulation:

```python
# Set GAZEBO_PLUGIN_PATH to include uuv_gazebo_plugins
set_gazebo_plugin_path = SetEnvironmentVariable(
    name='GAZEBO_PLUGIN_PATH',
    value=os.path.join(get_package_share_directory('uuv_gazebo_plugins'), 'lib')
)

# Set GAZEBO_MODEL_PATH to include necessary model directories
set_gazebo_model_path = SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=os.path.join(get_package_share_directory('uuv_gazebo_worlds'), 'models')
)

# Define package and file paths
package_name = 'fish_hpurv'
xacro_file = 'urdf/fish_hpurv_macro.urdf.xacro'
world_file = 'worlds/ocean_waves.world'

# Build full paths
model_file_path = os.path.join(get_package_share_directory(package_name), xacro_file)
world_file_path = os.path.join(get_package_share_directory('uuv_gazebo_worlds'), world_file)
```

The fish geometry was modeled in Onshape (https://cad.onshape.com/documents/aaca298587ed8e68033344ff/w/b1920aaef25add5d3ec70db8/e/f61702870397c57547016114?renderMode=0&uiState=682af8d07aab7f5080a9108d).
![image](https://github.com/user-attachments/assets/363d55a5-c043-4d59-a68d-9513f89ab172)


Using the Onshape‑to‑URDF exporter with our config.json, we generated the URDF.
All exported STL meshes are stored in meshes/.
The resulting URDF is located at urdf/fish.urdf.
Both the meshes and urdf/fish.urdf are contained within the fish_hpurv package.

Taking the fish.urdf model now we  include three extra sensors and save it as fish_hpurv_macro.urdf under the ~/uav/src/fish_hpurv/urdf:


- **Laser range‑finder**  
  - Link : `link_base`  
  - Plugin : `gazebo_ros_laser`  
  - Topic : `/scan`

- **IMU**  
  - Link : `link_base`  
  - Plugin : `gazebo_ros_imu`  
  - Topic : `/imu/data` (50 Hz)

- **DVL – Doppler Velocity Log**  
  - Link : `link_base`  
  - Plugin : `libfish_dvl_plugin.so` (custom)  
  - Output : bottom‑tracking velocity

_All sensors are declared inside their own `<gazebo>` blocks, so they are spawned automatically when the model loads in Gazebo._
