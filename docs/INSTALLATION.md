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

# 2. Moving NS3 Script

```bash
mv  UAVSwarmAttackSim/scratch ns-3-dev-git/scratch
```
# 3. Create a virtula environemt 

```bash
cd UAVSwarmAttackSim
python3 -m venv venv
source venv/bin/activate
```
