# Gazebo Installation & Setup Guide

---

## Table of Contents

- [1. Install ArduPilot Gazebo Plugin](#1-install-ardupilot-gazebo-plugin)
- [2. Running a Single UAV](#2-running-a-single-uav)
- [3. Running Multiple UAVs](#3-running-multiple-uavs)
  - [Step 1: Duplicate and Modify UAV Models](#step-1-duplicate-and-modify-uav-models)
  - [Step 2: Create a Custom World](#step-2-create-a-custom-world)
  - [Step 3: Run the Custom World](#step-3-run-the-custom-world)
  - [Step 4: Connect Multiple Drones to ArduPilot](#step-4-connect-multiple-drones-to-ardupilot)

---

## 1. Install ArduPilot Gazebo Plugin

Integrates ArduPilot with Gazebo to simulate drones and evaluate their behavior in 3D environments.

### 1.1 Update package lists
```bash
sudo apt update
```

### 1.2 Install Gazebo simulation library and JSON parser
```bash
sudo apt install libgz-sim8-dev rapidjson-dev -y
```

### 1.3 Install OpenCV and GStreamer multimedia libraries for visualization and video streaming
```bash
sudo apt install libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-libav gstreamer1.0-gl -y
```

### 1.4 Clone the plugin repository
```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
```

### 1.5 Create build directory and enter it
```bash
mkdir build && cd build
```

### 1.6 Configure the build with CMake
```bash
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 1.7 Compile the plugin using 4 cores
```bash
make -j4
make build
```

### 1.8 Install the plugin system-wide
```bash
sudo make install
```

### 1.9 Set Gazebo Environment Variables

These variables tell Gazebo where to find your plugin and model/world files.
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/enter/path/to/your/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/enter/path/to/your/ardupilot_gazebo/models:/enter/path/to/your/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

> **Important:** Replace `/enter/path/to/your/...` with the **actual path** to your `ardupilot_gazebo` directory.

---

## 2. Running a Single UAV

Gazebo can be used to simulate a single drone with ArduPilot. Follow these steps to run the simulation.

Run the following command to start Gazebo with the default `iris_runway.sdf` world:
```bash
gz sim worlds/iris_runway.sdf
```

This will launch the simulator.  
Click the **Run** button to start the simulation:

![Gazebo Runway](images/gazebo_runway.png)

![Gazebo Run Button](images/gazebo_run.png)

Now, connect ArduPilot to the simulator with:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

If everything is set correctly, the `MAVProxy` console will display green status messages:

![MAVProxy Connected](images/mavproxy_connected.png)

You can then issue the following commands in `MAVProxy` to take off:
```bash
mode guided
arm throttle
takeoff 5
```

![Drone Takeoff](images/drone_takeoff.png)

---

## 3. Running Multiple UAVs

By default, the `iris_runway.sdf` world supports a single UAV. To simulate **multiple UAVs**, you will need to create a custom world. Below are the steps.

### Step 1: Duplicate and Modify UAV Models

Navigate to the models folder and create copies of the `iris_with_gimbal` model:
```bash
cd /ardupilot_gazebo/models

cp iris_with_gimbal -r custom_iris_9002
cp iris_with_gimbal -r custom_iris_9012
cp iris_with_gimbal -r custom_iris_9022
```

Edit each model's `model.sdf` file and update the ports:
```xml
<!-- custom_iris_9002 -->
<fdm_port_in>9002</fdm_port_in>
<fdm_port_out>9003</fdm_port_out>

<!-- custom_iris_9012 -->
<fdm_port_in>9012</fdm_port_in>
<fdm_port_out>9013</fdm_port_out>

<!-- custom_iris_9022 -->
<fdm_port_in>9022</fdm_port_in>
<fdm_port_out>9023</fdm_port_out>
```

![Custom Model 1](images/model1.png)  
![Custom Model 2](images/model2.png)  
![Custom Model 3](images/model3.png)

Each new SITL instance uses ports spaced by 10 units.  
For example: `9002/9003`, `9012/9013`, `9022/9023`, etc.

---

### Step 2: Create a Custom World

Navigate to the worlds folder and copy the default world:
```bash
cd /ardupilot_gazebo/worlds
cp iris_runway.sdf Custom_3_uav.sdf
```

Edit `Custom_3_uav.sdf`:

- Remove the default `iris_with_gimbal` model:
```xml
<include>
  <uri>model://iris_with_gimbal</uri>
  <pose degrees="true">0 0 0.195 0 0 90</pose>
</include>
```

- Add your custom UAV models:
```xml
<include>
  <uri>model://custom_iris_9002</uri>
  <name>iris_with_gimbal_9002</name>
  <pose degrees="true">0 0 0.195 0 0 90</pose>
</include>

<include>
  <uri>model://custom_iris_9012</uri>
  <name>iris_with_gimbal_9012</name>
  <pose degrees="true">2 0 0.195 0 0 90</pose>
</include>

<include>
  <uri>model://custom_iris_9022</uri>
  <name>iris_with_gimbal_9022</name>
  <pose degrees="true">4 0 0.195 0 0 90</pose>
</include>
```

This places drones in a straight line, spaced 2 meters apart.

---

### Step 3: Run the Custom World
```bash
gz sim worlds/Custom_3_uav.sdf
```

![Custom 3 UAV World](images/uav_world.png)

---

### Step 4: Connect Multiple Drones to ArduPilot

Use separate terminals for each drone instance:
```bash
# Terminal 1
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# Terminal 2
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I1

# Terminal 3
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I2
```

You can now control three UAVs simultaneously within Gazebo.
