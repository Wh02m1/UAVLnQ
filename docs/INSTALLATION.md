# UAVLnQ Installation Guide
Simulation Environment:

**ArduPilot**: 4.6.0<br/>
**Gazebo**: Harmonic<br/>
**QGroundControl**: 5.0.6 (64-bit)<br/>
**NS-3**: 3.45<br/>
**OS**: Ubuntu 24.04.2 LTS<br/>

This guide provides a step-by-step walkthrough for setting up the UAV simulation environment used in this repository.

Tested on:  
- **Ubuntu 24.04.2 LTS**  
- Works on both **VMware VM** and **native laptop installation**  
- **VMware** → Enable 3D acceleration, allocate ≥4 CPU cores & 8 GB RAM
---

## Table of Contents
1. [Prerequisites](#prerequisites)  
2. [Install ArduPilot (SITL)](#1-install-ardupilot-sitl)  
3. [Install Gazebo (Harmonic)](#2-install-gazebo)  
4. [Install ArduPilot Gazebo Plugin](#3-install-ardupilot-gazebo-plugin)  
5. [Install QGroundControl](#4-install-qgroundcontrol)  
6. [Install NS-3](#5-install-ns-3-network-simulator-3)  
7. [Use Case Setup](#6-Use-Case-Setup)  
8. [Troubleshooting](#troubleshooting)  
9. [References](#references)  

---

## Prerequisites  

Update system and install base tools:  

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git wget curl build-essential cmake g++ gcc     python3 python3-pip python3-dev     libxml2-dev libxslt1-dev     libprotobuf-dev protobuf-compiler     libssl-dev libffi-dev     pkg-config lsb-release
```

---

## 1. Install ArduPilot (SITL)  

### 1. Prerequisites
```bash
sudo apt install git
```

### 2. Clone ArduPilot
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

### 3. Install Dependencies
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```
### 4. Build SITL (Quadcopter Example)
```bash
./waf configure --board sitl
./waf copter
```
### 5. Adding `sim_vehicle.py` to PATH

To run `sim_vehicle.py` from anywhere, add it to your shell `PATH`.

#### 1. Open your shell config
For **bash**:
```bash
nano ~/.bashrc
```
#### 2. Add the script path
Append this line:
```bash
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"
```
#### 3. Reload your shell
```bash
source ~/.bashrc  
```

### 5. Run SITL (Quick Test)
```bash
sim_vehicle.py -v ArduCopter --console --map
```

> When running SITL, you should see something like:

<pre>
'build' finished successfully (…)
Starting ArduCopter ...
Connect tcp:127.0.0.1:5760 source_system=255
Detected vehicle 1:1 on link 0
Received 1359 parameters (ftp)
Saved 1359 parameters to mav.parm
</pre>

> MAVProxy Console

The MAVProxy console will open with the prompt:

<pre>
STABILIZE>
</pre>

> Map Window
A map window will open. It may briefly say "map not ready", but it loads once GPS data is available.

---

## 2. Install Gazebo  

The Gazebo version called **Harmonic** is the only one currently compatible with the latest Ubuntu 24.04.

Gazebo simulates the physical environment and drone dynamics, allowing testing of UAVs in realistic 3D worlds without real hardware.

### 1. Update System Packages
```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

### 2. Install Gazebo Harmonic
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg]  https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main"  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic
```
### 3. Verify Installation
```bash
gz sim --version
```

## 3. Install ArduPilot Gazebo Plugin

Integrates ArduPilot with Gazebo to simulate drones and evaluate their behavior in 3D environments.

### 1. Update package lists
```bash
sudo apt update
```

### 2. Install Gazebo simulation library and JSON parser
```bash
sudo apt install libgz-sim8-dev rapidjson-dev -y
```
### 3. Install OpenCV and GStreamer multimedia libraries for visualization and video streaming
```bash
sudo apt install libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-libav gstreamer1.0-gl -y
```
### 4. Clone the plugin repository
```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
```
### 5. Create build directory and enter it
```bash
mkdir build && cd build
```
### 6. Configure the build with CMake
```bash
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
### 7. Compile the plugin using 4 cores
```bash
make -j4
make build
```
### 8. Install the plugin system-wide
```bash
sudo make install
```
### 9. Set Gazebo Environment Variables

These variables tell Gazebo where to find your plugin and model/world files.

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/enter/path/to/your/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/enter/path/to/your/ardupilot_gazebo/models:/enter/path/to/your/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

> **Important:** Replace `/enter/path/to/your/...` with the **actual path** to your `ardupilot_gazebo` directory.

### 10. Gazebo Simulation Setup and Use Cases
A step-by-step guide for running single-UAV and multi-UAV simulations in Gazebo with ArduPilot, including setup, configuration, and testing.

[Gazebo Setup Guide](INSTALLATION_GAZEBO.md)

---

## 4. Install QGroundControl  

Provides the user interface and control for drones, supporting mission planning, monitoring, and real-time operation.


```bash
# Download QGroundControl AppImage (64-bit x86)
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage

# Make the AppImage executable
chmod +x QGroundControl-x86_64.AppImage

# Run QGroundControl
./QGroundControl-x86_64.AppImage
```

---

## 5. Install NS-3 (Network Simulator 3)

We use **ns-3 3.45** for network simulation.  

NS-3 simulates the network for the UAV project, enabling testing of communication between drones and ground stations, and analysis of speed, delays, and data loss under different network conditions without real hardware.

### 1. Install Build Tools
```bash
sudo apt update
sudo apt install -y cmake ninja-build build-essential
sudo apt-get install libzmq3-dev libczmq-dev
sudo apt install libzmq3-dev cppzmq-dev
```

### 2. Clone NS3 repository and MAVLink Library  
```bash
git clone https://github.com/nsnam/ns-3-dev-git
cd ns-3-dev-git
git clone https://github.com/mavlink/c_library_v2
```

### 3. Configure and build NS3
```bash
./ns3 configure
./ns3 build
```
>If the use case setup has not been done yet, skip to that section. Once ready, the two use cases can be run with the following commands:

1. To run the main ns3 topology without the attacker 
```bash
./ns3 run scratch/drone-mesh/drone_mesh
```
2. To run the main ns3 topology with the attacker 
```bash
./ns3 run scratch/drone-mesh-with-attacker/drone_mesh
```

---
## 6. Use Case Setup

### 1. Clone UAVLnQ repository
```bash
cd ../
git clone https://github.com/Wh02m1/UAVLnQ.git
cd UAVLnQ
```
### 2. Update Output Paths in NS-3 Scripts

Before running the simulations, update the paths in the following files to specify where the PCAP files should be saved:

<pre>scratch/drone-mesh/main.cc 
scratch/drone-mesh-with-attack/main.cc</pre>

Look for the paths:

<pre>/path/to/change/drone-mesh-
/path/to/change/drone-mesh-anim_</pre>

and change them to the desired location for saving PCAP and anim files.

### 3. Copy NS3 Network Simulation Scripts
```bash
cp -r scratch/*  ../ns-3-dev-git/scratch
```

### 4. Build NS3 Network Simulation Scripts
```bash
cd ../ns-3-dev-git
./ns3 configure
./ns3 build
```

### 5. Create a Virtual Environment

Create a virtual environment with **Python 3.9** to ensure compatibility with the `dronekit` library:

#### Prerequisite: Python 3.9 Installation (Ubuntu)

Before proceeding, ensure that **Python 3.9** is installed on your system.

##### 1. Check if Python 3.9 is installed
Open a terminal and run:

```bash
python3.9 --version
```
You should see output similar to:
```bash
Python 3.9.x
```
If Python 3.9 is not installed, follow these steps:
```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.9 python3.9-venv python3.9-dev -y
```
Verify the installation:
```bash
python3.9 --version
```
##### 2. Create the Virtual Environment

```bash
cd ../UAVLnQ
python3.9 -m venv drone_env
source drone_env/bin/activate
```
### 6. Install Requirements
```bash
pip install --upgrade pip
pip install dronekit pymavlink pyzmq
```
---
## Troubleshooting  

- **QGC not connecting** → run:  
  ```bash
  ./build/sitl/bin/arducopter --model quad --serial0=udp:127.0.0.1:14550
  ```
- **NS-3 build errors** → clean & rebuild:  
  ```bash
  ./ns3 clean && ./ns3 build
  ```

---


## Optional: Install NetAnim (NS-3 Visualizer)

NetAnim is a visualization tool for NS-3 that allows you to view drone mobility, connectivity, and data exchange over time using the generated `.anim` files.
These `.anim` files are automatically created after the simulation finishes and can be opened in NetAnim to visualize the results.


1. Install Qt5 (required for NetAnim)

```bash
sudo apt update
sudo apt install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools -y
```

2. Build and Install NetAnim

```bash
cd ns-3-dev-git
git clone https://gitlab.com/nsnam/netanim.git
cd netanim
qmake NetAnim.pro
make
```

4. Run NetAnim

After simulation, open the `.anim` file generated by NS-3:

```bash
./NetAnim <path-to-file>.anim
```

You should see nodes (representing drones) moving and exchanging data links over time.

https://github.com/user-attachments/assets/26c4dbeb-9f81-4eb1-9476-23962083f38d


## References  

- [ArduPilot Docs](https://ardupilot.org/dev/docs/building-setup-linux.html)  
- [Gazebo Docs](http://gazebosim.org/)
- [ArduPilot Gazebo GitHub](https://github.com/ArduPilot/ardupilot_gazebo)  
- [Gazebo Installation Guide](http://gazebosim.org/tutorials?tut=install_ubuntu)  
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/using-sitl-with-gazebo.html)
- [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)  
- [NS-3 Manual](https://www.nsnam.org/docs/release/3.45/installation/singlehtml/index.html#download)  
