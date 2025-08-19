# System Architecture

Overall system architecture Without Attacker:

![Architecture](images/Architecture.png)

Overall system architecture with Attacker Sending malformed mavlink packets:

![Architecture with Attacker](images/Architecture_with_attacker.jpg)

# Setup
### Drone 1
```bash
./sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --sysid=1 --model JSON --console \
--out=udp:127.0.0.1:14551 \
--out=udp:127.0.0.1:14552 \
--out=udp:127.0.0.1:14553
```
### Drone 2
```bash
./sim_vehicle.py -v ArduCopter -f gazebo-iris -I1 --sysid=2 --model JSON --console \
--out=udp:127.0.0.1:14561 \
--out=udp:127.0.0.1:14562 \
--out=udp:127.0.0.1:14563
```

### Drone 3
```bash
./sim_vehicle.py -v ArduCopter -f gazebo-iris -I2 --sysid=3 --model JSON --console \
--out=udp:127.0.0.1:14571 \
--out=udp:127.0.0.1:14572 \
--out=udp:127.0.0.1:14573
```
