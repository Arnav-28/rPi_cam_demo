# Setup Guide: RPi 4 V4L2 Camera & ROS 2 Humble (Source Build)

This guide covers enabling the legacy V4L2 camera stack for the Raspberry Pi CSI camera on Ubuntu 22.04 and building ROS 2 Humble from source.



---

## Part 1: Enable V4L2 (Legacy Camera Stack)

Ubuntu 22.04 uses libcamera by default. To use standard ROS drivers (like `v4l2_camera` or `usb_cam`), we must enable the legacy Broadcom drivers.

### 1. Install Kernel Extras

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install linux-modules-extra-raspi -y
```

### 2. Configure Firmware

Open the boot configuration file:

```bash
sudo nano /boot/firmware/config.txt
```

Make the following changes:

- Look for `camera_auto_detect=1` and comment it out (add `#`)
- Add `start_x=1` and `gpu_mem=128` to the bottom of the file

The section should look like this:

```
# camera_auto_detect=1
start_x=1
gpu_mem=128
```

Save (`Ctrl+O`) and Exit (`Ctrl+X`).

### 3. Reboot and Load Module

Reboot the Pi to apply changes:

```bash
sudo reboot
```

Once back online, load the driver and verify:

```bash
sudo modprobe bcm2835-v4l2
ls -l /dev/video0
```

You should see `/dev/video0` listed.

### 4. Auto-load Module on Boot (Recommended)

To automatically load the module on every boot:

```bash
echo "bcm2835-v4l2" | sudo tee -a /etc/modules
```

### 5. Verify Camera Capabilities

Check if the camera is detected:

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext
```

**Install v4l-utils if not available:**

```bash
sudo apt install v4l-utils -y
```

---

## Part 2: Build Dependencies


### 1. Add ROS 2 APT Repository

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2. Install Build Tools and Dependencies

```bash
sudo apt-get update && sudo apt-get full-upgrade -y

sudo apt-get install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  libpython3-dev \
  python3-numpy \
  python3-yaml \
  wget
```
---

## Part 3: Install and Run v4l2_camera Package

### 1. Install v4l2_camera Package

Navigate to your ROS 2 workspace:

```bash
cd ~/ros2_humble/src

# Clone the v4l2_camera package
git clone -b humble https://github.com/ros-drivers/v4l2_camera.git
```

### 2. Install Additional Dependencies

```bash
cd ~/ros2_humble

# Install dependencies for v4l2_camera
rosdep install --from-paths src --ignore-src -y

# Install image transport plugins (for viewing images)
sudo apt install ros-humble-image-transport-plugins -y 2>/dev/null || \
rosdep install --from-paths src --ignore-src -y --skip-keys "image_transport_plugins"
```

### 3. Build the Package

```bash
cd ~/ros2_humble
colcon build --symlink-install --packages-select v4l2_camera
source install/setup.bash
```

### 4. Run the v4l2_camera Node

**Launch command:**
(change resolution to [320,240] if stream lags)


```bash
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480] \ 
```

### 5. View Camera Stream

In a **new terminal**, verify topics are publishing:

```bash
source ~/ros2_humble/install/setup.bash
ros2 topic list
ros2 topic echo /image_raw --once
```

**To view the image (requires GUI or remote display):**

Install image view tools:

```bash
sudo apt update
sudo apt install ros-humble-rqt-image-view
```

View the stream:

```bash
ros2 run rqt_image_view rqt_image_view
```

### 7. Create Launch File (Optional)

For easier startup, create a launch file:

```bash
mkdir -p ~/ros2_humble/src/v4l2_camera/launch
nano ~/ros2_humble/src/v4l2_camera/launch/camera.launch.py
```

Add the following content:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'time_per_frame': [1, 30],
            }]
        )
    ])
```

Save and rebuild:

```bash
cd ~/ros2_humble
colcon build --symlink-install --packages-select v4l2_camera
source install/setup.bash
```

Launch with:

```bash
ros2 launch v4l2_camera camera.launch.py
```

---

## Troubleshooting

### Camera Not Detected

```bash
# Check if camera is physically connected
vcgencmd get_camera

# Should show: supported=1 detected=1

# If not detected, check cable connection and reboot
```

### Permission Denied on /dev/video0

```bash
sudo chmod 666 /dev/video0
# Or add permanent udev rule:
echo 'KERNEL=="video[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-camera.rules
sudo udevadm control --reload-rules
```

### Module bcm2835-v4l2 Not Found

```bash
# Ensure kernel extras are installed
sudo apt install linux-modules-extra-raspi -y
sudo reboot
```

