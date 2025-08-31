# UAVLnQ (UAV Network Simulator)


# System Architecture

Overall system architecture Without Attacker:

![Architecture](docs/Architecture.png)

Overall system architecture with Attacker Sending malformed mavlink packets:

![Architecture with Attacker](docs/Architecture_with_attacker.png)

# Simulated Drone Attacks (NS-3 + MAVLink)

This repository contains implementations of various **MAVLink attack scenarios** targeting UAV swarms.  
Each attack creates and injects malicious MAVLink packets into the network to manipulate drone behavior or mislead operators.

---

## Attack Scenarios 

### Drone SITL Attacks
| # | Attack Name                            | Packet Builder Function(s)            | Description |
|---|----------------------------------------|----------------------------------------|-------------|
| 1 | Malicious Waypoint Injection (Hijack Routes) | `CreateMavlinkMissionPacket`         | Injects fake `MISSION_ITEM` messages so followers fly to unintended waypoints or become isolated. |
| 2 | Speed Manipulation                     | `CreateChangeSpeedPacket`              | Injects `COMMAND_LONG` with `MAV_CMD_DO_CHANGE_SPEED` to force slower/faster flight, disrupting mission timing. |
| 3 | Forced Return-to-Launch (RTL)          | `CreateForcedReturnHomePacket`         | Sends a `SET_MODE` to force drones into RTL mode, causing premature mission abortion. |
| 4 | Forced Disarm                          | `CreateForcedDisarmPacket`             | Sends `MAV_CMD_COMPONENT_ARM_DISARM` with the force-disarm flag; immediately disarms motors mid-flight. |
| 5 | Flight Termination                     | `CreateFlightTerminationPacket`        | Sends `MAV_CMD_DO_FLIGHTTERMINATION` causing immediate motor cutoff. |
| 6 | Home Position Hijack                   | `CreateSetHomePositionPacket`          | Maliciously sets home location (near attacker). On RTL, vehicles return to the attacker’s position. |

### Network-Level Attacks
| # | Attack Name                            | Packet Builder Function(s)            | Description |
|---|----------------------------------------|----------------------------------------|-------------|
| 7 | GPS Spoofing (Network Injection)       | `CreateFakeGpsPacket`                  | Sends fake `GPS_RAW_INT` for a legitimate  Real drones (SYSIDs 1-3); spoofs position, altitude, and velocity. |
| 8 | Heartbeat Flood DoS                    | `CreateHeartbeatPacket`                | Floods the network with many spoofed `HEARTBEAT` frames (e.g., SYSIDs 4–20) to overload links and processing. |

### QGroundControl (GCS)–Focused Attacks
| # | Attack Name                            | Packet Builder Function(s)            | Description |
|---|----------------------------------------|----------------------------------------|-------------|
| 9  | Drone Location Spoofing (UI Deception) | `CreateQgcLocationSpoofPacket`        | Spoofs Real drones (SYSIDs 1-3) positions in QGC by sending fake `GLOBAL_POSITION_INT`, making them appear somewhere else. |
| 10 | Battery Status Spoofing                | `CreateSpoofedBatteryStatusPacket`     | Spoofs `BATTERY_STATUS` (percentage/voltage). |
| 11 | Ghost Drones (Fake Fleet Flood)        | `CreateSpoofedDroneGpsPacket`          | Generates non-existent drones (SYSIDs 4–10) so QGC shows “phantom” UAVs. |



# Installation
- [Installation Guide](docs/INSTALLATION.md)

# Multi-Drone SITL + Gazebo + NS3 Setup Guide

This guide explains how to set up and run **multiple ArduCopter SITL drones** with Gazebo, NS3, and QGroundControl.

---
## 1. Prerequisites
> Follow the ArduPilot and Gazebo installation guides to fulfill the prerequisites.

- ArduPilot installed and `sim_vehicle.py` added to your `PATH`.
- Gazebo installed with the custom world file (`worlds/Custom_3_uav.sdf`).
- QGroundControl AppImage downloaded (`QGroundControl-x86_64.AppImage`).
- Python scripts:  
  - `connect.py`  
  - `Mavlink-NS3-Parser.py`  

---

## 1. Modify Source Code 

> ⚠️ Before running, make sure to update the NS3 line in `connect.py` depending on your use case — either the normal case or the attacker case.
> You need to modify both the NS-3 path and, within NS-3, update the path where you want to save the traffic `.pcap` and `.anim` files.

The line to be modified:
```bash
'/home/boda/Desktop/ns-3-dev/ns3','run','scratch/drone-mesh/drone_mesh'
```

Normal Senario:
```bash
'/your/path/to/ns-3-dev/ns3','run','scratch/drone-mesh/drone_mesh'
```

With Attacker Senario:
```bash
'/your/path/to/ns-3-dev/ns3','run','scratch/drone-mesh-with-attacker/drone_mesh'
```

---

## 2. Launch Gazebo

Run Gazebo with the custom multi-drone world:
```bash
gz sim worlds/Custom_3_uav.sdf
```
And when the 3D simulator opens, press the ▶️ Start button

---

## 3. Launch QGroundControl

Run QGroundControl in a separate terminal:
```bash
./QGroundControl-x86_64.AppImage
```

---

## 4. Launch Multiple Drones

Run each of the following commands inside the ArduPilot directory in separate terminals.
⚠️ Make sure to execute them only after launching Gazebo and starting the 3D simulator (▶️ Start button).

### Drone 1

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --sysid=1 --model JSON --console \
--out=udp:127.0.0.1:14551 \
--out=udp:127.0.0.1:14552 \
--out=udp:127.0.0.1:14553
```
### Drone 2
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --sysid=2 --model JSON --console \
--out=udp:127.0.0.1:14561 \
--out=udp:127.0.0.1:14562 \
--out=udp:127.0.0.1:14563
```

### Drone 3
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -I2 --sysid=3 --model JSON --console \
--out=udp:127.0.0.1:14571 \
--out=udp:127.0.0.1:14572 \
--out=udp:127.0.0.1:14573
```

---

Finally, open two new terminals inside this project directory. In each terminal, activate your Python 3.9 virtual environment (recommended):

```bash
source drone_env/bin/activate
```

In first terminal, run the connector:
```bash
python connect.py
```

In secound terminal, run the NS3 parser:
```bash
python Mavlink-NS3-Parser.py
```
<img width="1668" height="943" alt="image" src="https://github.com/user-attachments/assets/793066ca-c297-45a3-a243-5f63f132888f" />






