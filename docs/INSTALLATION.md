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

## 3. Configure and build ns-3
```bash

./ns3 configure
./ns3 build

```
