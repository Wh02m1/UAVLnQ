# UAVLnQ - Setup Guide

An open-source co-simulation framework integrating ArduPilot SITL with ns-3 for UAV swarm security research.

---
## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [1. Python 3.9](#1-python-39)
  - [2. ArduPilot SITL](#2-ardupilot-sitl)
  - [3. ns-3 Network Simulator](#3-ns-3-network-simulator)
  - [4. QGroundControl](#4-qgroundcontrol)
  - [5. UAVLnQ Framework](#5-uavlnq-framework)
- [Configuration](#configuration)
- [Quick Start](#quick-start)
- [Output Files](#output-files)
- [Optional: Gazebo (3D Drone Simulator)](#optional-gazebo-3d-drone-simulator)
- [Optional: NetAnim (NS-3 Visualizer)](#optional-netanim-ns-3-visualizer)
- [References](#references)
---

## Prerequisites

- Ubuntu 20.04/22.04 LTS
- Python 3.9
- Git

---

## Installation

### 1. Python 3.9

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install required dependencies
sudo apt install -y software-properties-common

# Add deadsnakes PPA
sudo add-apt-repository ppa:deadsnakes/ppa -y

# Update package list
sudo apt update

# Install Python 3.9 and venv
sudo apt install -y python3.9 python3.9-venv python3.9-dev

# Verify installation
python3.9 --version
```

---
### 2. ArduPilot SITL
```bash
# Install git
sudo apt-get install -y git gitk 

# Clone ArduPilot
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git

cd ardupilot

# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload path (required once after installation)
. ~/.profile

# Add ArduPilot to PATH permanently so you don't need to run . ~/.profile every time
echo 'export PATH="$HOME/ardupilot/Tools/autotest:$PATH"' >> ~/.bashrc
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

# Apply changes to current terminal
source ~/.bashrc

# Build for SITL
./waf configure --board sitl
./waf copter

# Verify installation
which sim_vehicle.py
```

#### Test Single Drone 
```bash
sim_vehicle.py -v copter -I0 --sysid=1 --console \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14552 \
    --out=udp:127.0.0.1:14553
```
---

### 3. ns-3 Network Simulator

#### 3.1 Install Prerequisites

```bash
# Install G++ compiler
sudo apt install g++ -y

# Install CMake
sudo apt install cmake -y

# Install Clang
sudo apt install clang -y

# Install ZeroMQ libraries
sudo apt install libzmq3-dev -y
sudo apt install libzmq5 -y
sudo apt install cppzmq-dev -y
```

#### 3.2 Clone and Build ns-3

```bash
# Clone ns-3
git clone https://gitlab.com/nsnam/ns-3-dev.git
cd ns-3-dev

# Clone MAVLink C library (required for MAVLink packet handling)
git clone https://github.com/mavlink/c_library_v2

# Configure and build
./ns3 configure
./ns3 build
```

---

### 4. QGroundControl 

QGroundControl provides visualization for UAV positions and status.

#### 4.1 Enable Serial Port Access

```bash
sudo usermod -aG dialout "$(id -un)"
```

> **Note:** Log out and back in for group changes to take effect.

#### 4.2 Install Dependencies

```bash
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

#### 4.3 Download and Install QGroundControl

```bash
mkdir -p ~/QGroundControl
cd ~/QGroundControl

wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage

# Make executable
chmod +x QGroundControl-x86_64.AppImage

# Run QGroundControl
./QGroundControl-x86_64.AppImage
```

#### 4.4 Configure Multiple UAVs in QGC

To view multiple drones, add UDP ports in QGroundControl:

1. Open **Application Settings** → **Comm Links**
2. Add new UDP connections for each drone:
   - Drone 1: Port `14550`
   - Drone 2: Port `14560`
   - Drone 3: Port `14570`
and so on depending on your number of drones simulated

---

### 5. UAVLnQ Framework

#### 5.1 Clone Repository

```bash
git clone https://github.com/Wh02m1/UAVLnQ.git
cd UAVLnQ
```

#### 5.2 Setup ns-3 Scripts

```bash
# Copy UAVLnQ ns-3 scripts to ns-3 scratch directory
cp -r ns3-scripts/* ~/ns-3-dev/scratch/

# Rebuild ns-3
cd ~/ns-3-dev
./ns3 build
```

#### 5.3 Create Python Virtual Environment

```bash
cd ~/UAVLnQ

# Create virtual environment with Python 3.9
python3.9 -m venv .venv

# Activate virtual environment
source .venv/bin/activate

# Install requirements
pip install -r requirements.txt
```

---

## Configuration

Edit `drones_config.json` to configure your drone setup.

### Configuration File Structure
```json
{
  "Drones_config": [
    {
      "id": 1,
      "dronekit_connection": "udp:127.0.0.1:14551",
      "mavlink_connection": "udp:127.0.0.1:14552",
      "mavlink_parser_connection": "udp:127.0.0.1:14553",
      "qgroundcontrol_port": 14550
    },
    {
      "id": 2,
      "dronekit_connection": "udp:127.0.0.1:14561",
      "mavlink_connection": "udp:127.0.0.1:14562",
      "mavlink_parser_connection": "udp:127.0.0.1:14563",
      "qgroundcontrol_port": 14560
    },
    {
      "id": 3,
      "dronekit_connection": "udp:127.0.0.1:14571",
      "mavlink_connection": "udp:127.0.0.1:14572",
      "mavlink_parser_connection": "udp:127.0.0.1:14573",
      "qgroundcontrol_port": 14570
    }
  ],
  "NS3_config": {
    "ns3_bin": "/Path/To/ns-3-dev/build/scratch/NS3-Multi-Drone/ns3.44-NS3-Multi-Drone-default",
    "parameters": "--n=3"
  }
}
```

### Drone Configuration Fields

| Field | Description |
|-------|-------------|
| `id` | Unique drone identifier (matches ArduPilot `--sysid` parameter) |
| `dronekit_connection` | UDP port for DroneKit API connection (mission control) |
| `mavlink_connection` | UDP port for MAVLink telemetry capture (sent to ns-3) |
| `mavlink_parser_connection` | UDP port for receiving commands from ns-3 parser |
| `qgroundcontrol_port` | UDP port for QGroundControl visualization |

### NS3 Configuration Fields

| Field | Description |
|-------|-------------|
| `ns3_bin` | Full path to the compiled ns-3 executable |
| `parameters` | Command line arguments for ns-3 (e.g., `--n=3` for 3 drones) |

### Adding More Drones

To add a 4th drone, append a new entry following the port pattern (+10 for each drone):
```json
{
  "id": 4,
  "dronekit_connection": "udp:127.0.0.1:14581",
  "mavlink_connection": "udp:127.0.0.1:14582",
  "mavlink_parser_connection": "udp:127.0.0.1:14583",
  "qgroundcontrol_port": 14580
}
```

Then update `NS3_config.parameters` to `"--n=4"`.

> **Important:** The ports in `drones_config.json` must match the `--out` parameters when launching ArduPilot SITL.

---

## Quick Start

### Terminal 1: Launch Drone 1

```bash
cd ~/ardupilot
. ~/.profile
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --sysid=1 --model JSON --console \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14552 \
    --out=udp:127.0.0.1:14553
```

### Terminal 2: Launch Drone 2

```bash
cd ~/ardupilot
. ~/.profile
sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --sysid=2 --model JSON --console \
    --out=udp:127.0.0.1:14561 \
    --out=udp:127.0.0.1:14562 \
    --out=udp:127.0.0.1:14563
```

### Terminal 3: Launch Drone 3

```bash
cd ~/ardupilot
. ~/.profile
sim_vehicle.py -v ArduCopter -f gazebo-iris -I2 --sysid=3 --model JSON --console \
    --out=udp:127.0.0.1:14571 \
    --out=udp:127.0.0.1:14572 \
    --out=udp:127.0.0.1:14573
```

### Terminal 4: Run UAVLnQ Connector

```bash
cd ~/UAVLnQ
source .venv/bin/activate
python connect.py
```

### Terminal 5: Run MAVLink Parser

```bash
cd ~/UAVLnQ
source .venv/bin/activate
python Mavlink-NS3-Parser.py
```

---

## Output Files

| File Type | Location | Description |
|-----------|----------|-------------|
| PCAP | `ns-3-dev/` | Network traffic capture |
| Telemetry CSV | `UAVLnQ/logs/` | Per-drone flight data |
| Animation XML | `ns-3-dev/` | NetAnim visualization |

---
## Optional: Gazebo (3D Drone Simulator)

You can additionally simulate the drones motion in 3D using Gazebo.
Follow the installation guide here: [Gazebo Setup Guide](INSTALLATION_GAZEBO.md)

---
## Optional: NetAnim (NS-3 Visualizer)

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


## References  

- [ArduPilot Docs](https://ardupilot.org/dev/docs/building-setup-linux.html)  
- [Gazebo Docs](http://gazebosim.org/)
- [ArduPilot Gazebo GitHub](https://github.com/ArduPilot/ardupilot_gazebo)  
- [Gazebo Installation Guide](http://gazebosim.org/tutorials?tut=install_ubuntu)  
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/using-sitl-with-gazebo.html)
- [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)  
- [NS-3 Manual](https://www.nsnam.org/docs/release/3.45/installation/singlehtml/index.html#download)  
