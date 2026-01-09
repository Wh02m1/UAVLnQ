# UAVLnQ (UAV-Network Simulator)

An Architecture for Security Analysis of Cyber-Physical Network Behavior in UAV Swarms integrates ArduPilot SITL with the ns-3 network simulator using ZeroMQ middleware, providing a testbed for studying UAV behavior and for developing and testing intrusion detection systems (IDS) in UAV swarms.

The details about the design of the simulator and some preliminary results for some use case scenarios are presented in our paper(link will be here). If you find this code useful in your research, please consider citing the paper:

```bibtex
@software{uavlnq2026,
  author    = {Yousef, Abdelrahman and Tsai, Chinya and Mistry, Nikita Nilesh and Real, Maria Mendez and Gogniat, Guy},
  title     = {UAVLnQ: An Architecture for Security Analysis of Cyber-Physical Network Behavior in UAV Swarms},
  year      = {2026},
  booktitle = {DASIP 2026: Workshop on Design and Architectures for Signal and Image Processing},
  url       = {Will be updated upon publication},
}

```

---

## Overview

UAVLnQ bridges the gap between UAV simulation and network security research by enabling realistic UAV swarm communication simulation over MAVLink protocol with attack injection capabilities. The framework generates PCAP files containing legitimate MAVLink communication patterns and CSV files containing physical telemetry data, making it ideal for IDS training, security analysis, and cybersecurity research.

---
## System Architecture

Overall system architecture Without Attacker:

![Architecture](docs/Architecture.png)

Overall system architecture with Attacker Sending malformed mavlink packets:

![Architecture with Attacker](docs/Architecture_with_attacker.png)


## NS-3 Simulation Scripts

The framework includes three ns-3 scratch configurations:

| Script | Parameters | Description |
|--------|------------|-------------|
| `3-Leader-Follower-Drone-Mesh` | `--simTime`, `--o` | Fixed 3 drone swarm in WiFi ad-hoc mesh topology |
| `3-Leader-Follower-Drone-Mesh-with-attacker` | `--simTime`, `--attack`, `--attackTime`, `--o` | Same as above but with an attacker node and attack scenarios |
| `Multi-Leader-Follower-Drone-Mesh` | `--n`, `--simTime`, `--o` | Scalable multi-UAV swarm with configurable drone count via `--n` parameter |

### Leader-Follower Topology

The leader-follower topology and MAVLink communication architecture is based on the approach described in:

> Adoni, W.Y.H.; Fareedh, J.S.; Lorenz, S.; Gloaguen, R.; Madriz, Y.; Singh, A.; Kühne, T.D. **Intelligent Swarm: Concept, Design and Validation of Self-Organized UAVs Based on Leader–Followers Paradigm for Autonomous Mission Planning.** *Drones* 2024, 8, 575. [https://doi.org/10.3390/drones8100575](https://doi.org/10.3390/drones8100575)

In this topology, the leader drone acts as a gateway broker, relaying MAVLink messages (HEARTBEAT, GPS_RAW_INT, SYS_STATUS, MISSION_ITEM) between the GCS and follower UAVs over a WiFi ad-hoc network using UDP.
### Parameters

| Parameter | Description |
|-----------|-------------|
| `--n` | Number of drones in the swarm |
| `--simTime` | Simulation duration in seconds |
| `--attack` | Attack type to execute during simulation |
| `--attackTime` | Time in seconds when attack starts |
| `--o` | Output directory for PCAP and animation files |


## Supported Attacks

UAVLnQ implements attack scenarios for the 3 UAV mesh topology that allow an attacker in ns-3 to craft and send malicious MAVLink packets to ns-3 nodes to record them in PCAP files, and to ArduPilot SITL to affect the physical telemetry data as well. Attacks are organized into three categories:

### Drone SITL Attacks
| Attack | Command | Description |
|--------|---------|-------------|
| ***ForceDisarm*** | `--attack=ForceDisarm` | Sends **MAV_CMD_COMPONENT_ARM_DISARM** with force-disarm magic number to shut down motors mid-flight |
| ***FlightTermination*** | `--attack=FlightTermination` | Sends **MAV_CMD_DO_FLIGHTTERMINATION** to abort flight and terminate all motors |
| ***ForceRTL*** | `--attack=ForceRTL` | Forces drone into **Return-to-Launch** mode, abandoning current mission |
| ***SpeedManipulation*** | `--attack=SpeedManipulation` | Alters drone speed via **MAV_CMD_DO_CHANGE_SPEED**, disrupting mission timing |
| ***HomePositionHijack*** | `--attack=HomePositionHijack` | Changes home position to attacker location via **MAV_CMD_DO_SET_HOME** |
| ***MissionInjection*** | `--attack=MissionInjection` | Injects malicious **MISSION_ITEM** waypoints appearing from leader drone |

### Network Level Attacks
| Attack | Command | Description |
|--------|---------|-------------|
| ***GPSSpoofing*** | `--attack=GPSSpoofing` | Injects fake **GPS_RAW_INT** packets to corrupt position awareness |
| ***HeartbeatFlood*** | `--attack=HeartbeatFlood` | DoS attack flooding network with **HEARTBEAT** packets from spoofed IDs |

### QGroundControl Attacks
| Attack | Command | Description |
|--------|---------|-------------|
| ***BatterySpoofing*** | `--attack=BatterySpoofing` | Sends fake **BATTERY_STATUS** showing critical levels (5%) |
| ***QGCLocationSpoofing*** | `--attack=QGCLocationSpoofing` | Spoofs **GLOBAL_POSITION_INT** to display incorrect drone positions |
| ***GhostDroneFlood*** | `--attack=GhostDroneFlood` | Creates phantom drones (sysid 4-10) flooding QGC with fake UAVs |


---

## Installation

Detailed installation instructions are available in the following guides:

- **[Main Installation Guide](https://github.com/Wh02m1/UAVLnQ/blob/main/docs/INSTALLATION.md)** - Complete setup for ArduPilot, ns-3, QGroundControl, and UAVLnQ
- **[Gazebo Installation Guide](https://github.com/Wh02m1/UAVLnQ/blob/main/docs/INSTALLATION_GAZEBO.md)** - Optional 3D visualization setup

---

## Quick Start

### 1. Launch ArduPilot SITL Instances

```bash
# Terminal 1 - Drone 1 (Leader)
sim_vehicle.py -v ArduCopter -I0 --sysid=1 --console \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14552 \
    --out=udp:127.0.0.1:14553

# Terminal 2 - Drone 2 (Follower)
sim_vehicle.py -v ArduCopter -I1 --sysid=2 --console \
    --out=udp:127.0.0.1:14561 \
    --out=udp:127.0.0.1:14562 \
    --out=udp:127.0.0.1:14563

# Terminal 3 - Drone 3 (Follower)
sim_vehicle.py -v ArduCopter -I2 --sysid=3 --console \
    --out=udp:127.0.0.1:14571 \
    --out=udp:127.0.0.1:14572 \
    --out=udp:127.0.0.1:14573
```
### 2. Configure drones_config.json

Edit `drones_config.json` to match your SITL setup:
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
        "ns3_bin": "/Path/To/ns-3-dev/build/scratch/3-Leader-Follower-Drone-Mesh-with-attacker/ns3.44-drone_mesh-default",
        "parameters": "--o=/Path/To/UAVLnQ/ns3-output --simTime=200 --attack=FlightTermination --attackTime=50"
    }
}
```

> **Note:** Update `ns3_bin` to match the absolute path to your compiled ns-3 simulation binary. The `parameters` field should be adjusted depending on which ns-3 script you are using.


### 3. Start UAVLnQ Components

```bash
# Terminal 4 - Mission Controller
cd UAVLnQ
source .venv/bin/activate
python connect.py

# Terminal 5 - MAVLink Parser
cd UAVLnQ
source .venv/bin/activate
python Mavlink-NS3-Parser.py
```

### 4. Launch QGroundControl (Optional)

```bash
./QGroundControl-x86_64.AppImage
```


---

## Extending UAVLnQ

will add this later 

---

## Contributing

Contributions are welcome—fork the repo, create a branch, and submit a pull request.


### Areas for Contribution

- New attack implementations
- Additional MAVLink message support
- Performance optimizations
- Documentation improvements
- IDS integration examples
---


## Contact

For questions, issues, or collaboration inquiries, please open an issue on GitHub.
