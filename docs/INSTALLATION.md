In order to run the code in this repository, follow the setup steps in order.

# NS-3 Setup

## 1. Install Build Tools

```bash
sudo apt update
sudo apt install -y cmake ninja-build build-essential
sudo apt-get install libzmq3-dev libczmq-dev
sudo apt install libzmq3-dev cppzmq-dev
```

## 2. Clone NS3 repository and MAVLink Library  

```bash
git clone https://github.com/nsnam/ns-3-dev-git
cd ns-3-dev-git
git clone https://github.com/mavlink/c_library_v2
```

## 3. Configure and build NS3
```bash

./ns3 configure
./ns3 build

```
---
# UAVSwarmAttackSim Setup

## 1. Clone UAVSwarmAttackSim repository
```bash
git clone https://github.com/Wh02m1/UAVSwarmAttackSim.git
```

## 2. Moving NS3 Script

```bash
mv  UAVSwarmAttackSim/scratch/*  ns-3-dev-git/scratch
```
## 3. Build NS3 Drone Network Script 

```bash
cd ../ns-3-dev-git
./ns3 build
```

## 4. Create a Virtual Environment

Create a virtual environment with **Python 3.9** to ensure compatibility with the `dronekit` library:

```bash
cd ../UAVSwarmAttackSim
python3.9 -m venv drone_env
source drone_env/bin/activate
```
## 5. install requiremts
```
pip install --upgrade pip
pip install dronekit pymavlink pyzmq
```
---
# QGroundControl Setup

To visualize and control your UAVs, you can install **QGroundControl**.

## 1. Download QGroundControl

Download the AppImage for Linux:  

- [QGroundControl x86_64 AppImage](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)

## 2. Make the AppImage Executable

```bash
chmod +x QGroundControl-<arch>.AppImage
```
## 3. Run QGroundControl
```bash
./QGroundControl-<arch>.AppImage
```



