# Setup Guide: RPi 4 V4L2 Camera & ROS 2 Humble (Source Build)

This guide covers enabling the legacy V4L2 camera stack for the Raspberry Pi CSI camera on Ubuntu 22.04 and building ROS 2 Humble from source.

## Prerequisites

- **Hardware:** Raspberry Pi 4 with CSI Camera Module
- **OS:** Ubuntu 22.04 Server
- **Storage:** Ensure at least **15 GB** of free space for compilation

---

## Part 1: Enable V4L2 (Legacy Camera Stack)

Ubuntu 22.04 uses `libcamera` by default. To use standard ROS drivers (like `v4l2_camera` or `usb_cam`), we must enable the legacy Broadcom drivers.

### 1. Install Kernel Extras

```bash
sudo apt update
sudo apt install linux-modules-extra-raspi
```

### 2. Configure Firmware

Open the boot configuration file:

```bash
sudo nano /boot/firmware/config.txt
```

Make the following changes:

1. Look for `camera_auto_detect=1` and comment it out (add `#`)
2. Add `start_x=1` and `gpu_mem=128` to the bottom of the file

The section should look like this:

```ini
# camera_auto_detect=1
start_x=1
gpu_mem=128
```

Save (**Ctrl+O**) and Exit (**Ctrl+X**).

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

> **Optional:** Add `bcm2835-v4l2` to `/etc/modules` to load it automatically on boot.

---

## Part 2: Build ROS 2 Humble from Source

### 1. Install Build Tools

```bash
sudo apt-get update && sudo apt-get full-upgrade

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
    python3-rosdep2 \
    python3-colcon-core \
    vcstool \
    build-essential
```

### 2. Create Workspace & Fetch Source

```bash
# Create directory
mkdir -p ros2_humble/src
cd ros2_humble

# Download repos file
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos

# Import source code
vcs import src < ros2.repos
```

### 3. Install Dependencies

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
```

### 4. Compile

> **Note:** This process may take a long time.

```bash
colcon build --symlink-install
```

---

## Part 3: Setup Environment & Verify

### 1. Source the Workspace

Add the install setup to your bash configuration so ROS 2 is available in every terminal:

```bash
echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Verification (Talker/Listener)

**Terminal 1 (Talker):**

```bash
ros2 run demo_nodes_cpp talker
```

Expected Output:

```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

**Terminal 2 (Listener):**

```bash
ros2 run demo_nodes_py listener
```

Expected Output:

```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

---

## Troubleshooting

- **Camera not detected:** Verify cable connection and ensure camera is enabled in `config.txt`
- **Build failures:** Check available disk space and RAM. Consider using swap space for compilation
- **Module not loading:** Ensure kernel extras are installed and firmware config is correct

---

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Raspberry Pi Camera Documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html)
- [V4L2 Camera Node](https://github.com/ros-drivers/v4l2_camera)

---

**Guide Version:** 1.0  
**Last Updated:** January 2026
3. Install Dependencies
Bash

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"
4. Compile
Note: This process may take a long time.

Bash

colcon build --symlink-install
Part 3: Setup Environment & Verify
1. Source the Workspace
Add the install setup to your bash configuration so ROS 2 is available in every terminal.

Bash

echo "source ~/ros2_humble/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
2. Verification (Talker/Listener)
Terminal 1 (Talker):

Bash

ros2 run demo_nodes_cpp talker
Expected Output:

Plaintext

[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
Terminal 2 (Listener):

Bash

ros2 run demo_nodes_py listener
Expected Output:

Plaintext

[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...