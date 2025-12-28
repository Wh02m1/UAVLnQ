/*
================================================================================
Drone Mesh Network Simulation with MAVLink/ZMQ Integration - ns-3
================================================================================

ARCHITECTURE:
------------
Network Topology:
   - 3 Drones in WiFi ad-hoc mesh (802.11a)
   - IP Range: 10.1.1.0/24
     * Drone0: 10.1.1.1
     * Drone1: 10.1.1.2
     * Drone2: 10.1.1.3

Communication Protocols:
   - MAVLink over UDP for drone commands and status
   - ZMQ for external communication
   - WiFi ad-hoc (802.11a) for physical layer

Port Mapping:
   ┌─────────────┬─────────────┬──────────────────────────────┐
   │   Node      │   Port      │          Purpose             │
   ├─────────────┼─────────────┼──────────────────────────────┤
   │ All Drones  │ 20000/UDP   │ MAVLink command reception,   │
   │             │             │ GPS position sharing,        │
   │             │             │ system status, heartbeat     │
   ├─────────────┼─────────────┼──────────────────────────────┤
   │ External    │ 5555/TCP    │ ZMQ command publishing       │
   │ External    │ 5556/TCP    │ ZMQ position updates         │
   └─────────────┴─────────────┴──────────────────────────────┘

Node Relationships:
        ---------------------------------------
        |                                      |
   +----⬇-----+       +----------+       +-----⬇----+
   |  Drone0  |◀─────▶|  Drone1  |◀─────▶|  Drone2  |
   | (10.1.1.1|       | (10.1.1.2|       | (10.1.1.3|
   +----------+       +----------+       +----------+
                                   


- Mission Control: Drone0 sends MAVLink commands to other drones via port 20000
- Position Sharing: Drones broadcast mavlink GPS_INT, SYS_STATUS and HEARTBEAT messages via UDP port 20000
- All MAVLink communication consolidated to single port (20000/UDP)
================================================================================
*/
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/realtime-simulator-impl.h"
#include "ns3/udp-socket-factory.h"
#include <common/mavlink.h>

#include <zmq.hpp>
#include <thread>
#include <atomic>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneZmqMeshNetwork");

static std::atomic<bool> keepRunning(true);
std::mutex positionMutex;
std::queue<std::vector<uint8_t>> positionQueue;  // Changed to binary queue

NodeContainer drones;
std::vector<Ptr<MobilityModel>> droneMobilityModels;

// Global GPS reference point
static double s_refLat = -35.3633;
static double s_refLon = 149.165;
static double s_refAlt = 0.0;
static double s_metersPerDegreeLat = 111320.0;
static double s_metersPerDegreeLon = s_metersPerDegreeLat * std::cos(s_refLat * M_PI / 180.0);

// Global variables for drone IPs
std::vector<Ipv4Address> droneIpAddresses;

// ZMQ context and publisher for commands
zmq::context_t g_zmqContext(1);
zmq::socket_t* g_commandPublisher = nullptr;

// Global vector for drone sockets
std::vector<Ptr<Socket>> g_droneSockets;

// MAVLink parser state
mavlink_status_t mavlink_status;

// Create MAVLink packet using official library
std::vector<uint8_t> CreateMavlinkPacket(uint8_t target_system, uint8_t target_component,
                                         float lat, float lon, float alt) {
    mavlink_message_t msg;
    uint8_t system_id = 1;   // Drone 0 (sender)
    uint8_t component_id = 1; // Component ID

    // Initialize mission item structure
    mavlink_mission_item_t mission_item = {0};
    mission_item.target_system = target_system;
    mission_item.target_component = target_component;
    mission_item.seq = 0;
    mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
    mission_item.current = 0;
    mission_item.autocontinue = 1;
    mission_item.param1 = 0; // Hold time (seconds)
    mission_item.param2 = 0; // Acceptance radius (meters)
    mission_item.param3 = 0; // Pass through waypoint
    mission_item.param4 = 0; // Yaw angle (NaN for unchanged)
    mission_item.x = lat;
    mission_item.y = lon;
    mission_item.z = alt;

    // Pack the message
    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &mission_item);

    // Serialize to byte vector
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

// Send waypoint commands directly from Drone0 to other drones
void SendWaypointPairFromDrone0(int pairIndex) {
    Ptr<Node> drone0 = NodeList::GetNode(0);
    Ptr<Socket> socket = Socket::CreateSocket(drone0, UdpSocketFactory::GetTypeId());
    socket->Bind();
    
    // Define waypoints
    static const std::vector<std::tuple<float, float, float>> drone1_waypoints = {
        {50, 60, 30},
        {10, 30, 30},
        {60, 10, 30}
    };
    
    static const std::vector<std::tuple<float, float, float>> drone2_waypoints = {
        {50, 60, 30},
        {20, 60, 30},
        {20, 30, 30}
    };

    if (pairIndex < 0 || pairIndex >= drone1_waypoints.size()) return;

    auto [lat1, lon1, alt1] = drone1_waypoints[pairIndex];
    auto [lat2, lon2, alt2] = drone2_waypoints[pairIndex];
    
    std::vector<uint8_t> pkt1 = CreateMavlinkPacket(1, 0, lat1, lon1, alt1);
    std::vector<uint8_t> pkt2 = CreateMavlinkPacket(2, 0, lat2, lon2, alt2);
    
    Ptr<Packet> packet1 = Create<Packet>(pkt1.data(), pkt1.size());
    Ptr<Packet> packet2 = Create<Packet>(pkt2.data(), pkt2.size());
    
    // Send directly to drone IPs
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 20000));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 20000));

    
    NS_LOG_INFO("Drone0 sent waypoint pair " << pairIndex + 1 << " at " 
                << Simulator::Now().GetSeconds() << "s");
    
    // Publish commands to ZMQ
    if (g_commandPublisher) {
        zmq::message_t zmqMsg1(pkt1.data(), pkt1.size());
        zmq::message_t zmqMsg2(pkt2.data(), pkt2.size());
        
        g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
        g_commandPublisher->send(zmqMsg2, zmq::send_flags::none);
        NS_LOG_INFO("Published commands to ZMQ 5555");
    }
}

// Process binary MAVLink GPS_RAW_INT, SYS_STATUS, and HEARTBEAT messages that will received from ZMQ 
// ZMQ message format: [message_type (1 byte), drone_id (1 byte), MAVLink message bytes]
// Message types: 0 = GPS_RAW_INT, 1 = SYS_STATUS, 2 = HEARTBEAT
void ProcessMavlinkMessage(const std::vector<uint8_t>& data) {
    if (data.size() < 2) return; // Need at least message type and drone ID
    
    // First byte is message type (0 for GPS, 1 for system status, 2 for heartbeat)
    uint8_t msg_type = data[0];
    // Second byte is drone ID (0 or 1 or 2)
    uint8_t droneId = data[1];
    
    // Parse MAVLink message from the remaining bytes (starting from index 2)
    mavlink_message_t msg;
    for (size_t i = 2; i < data.size(); i++) {
        // Parse each byte as part of a MAVLink message
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &mavlink_status)) {
            // If the message is a GPS_RAW_INT (contains GPS data)
            if (msg_type == 0 && msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                mavlink_gps_raw_int_t gps;
                // Decode the GPS_RAW_INT message to extract GPS fields
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                
                // Convert MAVLink GPS fields to degrees/meters
                double lat = gps.lat / 1e7;      // Latitude in degrees
                double lon = gps.lon / 1e7;      // Longitude in degrees
                double alt = gps.alt / 1000.0;   // Altitude in meters
                
                // If the droneId is valid, update its position in the simulation
                if (droneId < droneMobilityModels.size()) {
                    // Convert GPS coordinates to local simulation XYZ coordinates
                    double x = (lon - s_refLon) * s_metersPerDegreeLon;
                    double y = (lat - s_refLat) * s_metersPerDegreeLat;
                    double z = alt - s_refAlt;
                    
                    // Set the drone's position in the simulation
                    droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
                    
                    // Log the updated position 
                    NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                                << " RAW GPS: lat=" << lat << " lon=" << lon << " alt=" << alt
                                << " → x=" << x << " y=" << y << " z=" << z);
                }
            }
            // If the message is a SYS_STATUS (contains system status data)
            else if (msg_type == 1 && msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
                mavlink_sys_status_t sys_status;
                // Decode the SYS_STATUS message to extract system status fields
                mavlink_msg_sys_status_decode(&msg, &sys_status);
                
                // Log the system status 
                NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                            << " SYS_STATUS: battery_voltage=" << sys_status.voltage_battery
                            << " battery_remaining=" << sys_status.battery_remaining
                            << " comms_drop_rate=" << sys_status.drop_rate_comm);
            }
            // NEW: If the message is a HEARTBEAT (contains system health data)
            else if (msg_type == 2 && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat;
                // Decode the HEARTBEAT message to extract system health fields
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                
                // Log the heartbeat information
                NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                            << " HEARTBEAT: type=" << static_cast<int>(heartbeat.type)
                            << " autopilot=" << static_cast<int>(heartbeat.autopilot)
                            << " base_mode=" << static_cast<int>(heartbeat.base_mode)
                            << " custom_mode=" << heartbeat.custom_mode
                            << " system_status=" << static_cast<int>(heartbeat.system_status));
            }
            
            // Forward the MAVLink packet to all other drones (except itself) on port 20000
            // We forward only the raw MAVLink message (without message type and drone ID)
            for (uint32_t j = 0; j < drones.GetN(); j++) {
                if (j != droneId) {
                    // Create a packet containing only the MAVLink message (excluding message type and drone ID)
                    Ptr<Packet> packet = Create<Packet>(data.data() + 2, data.size() - 2);
                    // Send the packet to the other drone's port 20000
                    g_droneSockets[droneId]->SendTo(packet, 0, 
                        InetSocketAddress(droneIpAddresses[j], 20000));
                    
                    // Log the forwarding event
                    NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                                << " forwarded MAVLink packet (type=" << static_cast<int>(msg_type)
                                << ") to Drone " << j);
                }
               
                /**
                For example, if Drone 0 receives a MAVLink GPS_RAW_INT, SYS_STATUS, or HEARTBEAT message:
                - Drone extracts the message type (0 for GPS, 1 for system status, 2 for heartbeat) and its own ID from the message.
                - It decodes the message data (GPS coordinates, system status information, or system health).
                    - For GPS messages, it updates its own position in the simulation using the coordinates.
                    - For system status messages, it logs battery voltage, remaining percentage, and communication drop rate.
                    - For heartbeat messages, it logs system type, autopilot type, and system status.
                - It sends the MAVLink packet (without the message type and drone ID prefix) to the other drone's IP address on port 20000.
                This ensures that every drone knows the position, system status, and health of every other drone
                */
            }
        }
    }
}

// Modified ZMQ receiver to handle binary data
void ZmqPositionReceiverThread() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");
    subscriber.set(zmq::sockopt::subscribe, "");
    NS_LOG_INFO("ZMQ Position subscriber connected to port 5556");

    while (keepRunning.load()) {
        zmq::message_t message;
        if (subscriber.recv(message, zmq::recv_flags::dontwait)) {
            uint8_t* data = static_cast<uint8_t*>(message.data());
            size_t length = message.size();
            std::vector<uint8_t> msg_vec(data, data + length);
            {
                std::lock_guard<std::mutex> lock(positionMutex);
                positionQueue.push(msg_vec);
            }
            NS_LOG_INFO("ZMQ Position update received (" << length << " bytes)");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void UpdatePositionsFromQueue() {
    std::queue<std::vector<uint8_t>> localQueue;
    {
        std::lock_guard<std::mutex> lock(positionMutex);
        std::swap(localQueue, positionQueue);
    }

    while (!localQueue.empty()) {
        ProcessMavlinkMessage(localQueue.front());
        localQueue.pop();
    }

    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);
}

void PrintDronePositions(NodeContainer nodes, double interval, double simTime) {
    Time now = Simulator::Now();
    if (now.GetSeconds() > simTime)
        return;

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<MobilityModel> mobility = nodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        std::cout << "Time " << now.GetSeconds() << "s, Drone " << i
                  << " Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    }

    Simulator::Schedule(Seconds(interval), &PrintDronePositions, nodes, interval, simTime);
}

void PacketReceived(Ptr<const Packet> p, const Address& addr, uint32_t droneId) {
    InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(addr);
    NS_LOG_INFO("Drone " << droneId << " received " << p->GetSize() 
                << "B packet from " << inetAddr.GetIpv4() << ":" << inetAddr.GetPort());
    
    // Attempt to parse as MAVLink message
    uint8_t buffer[256];
    uint32_t bytesToCopy = std::min(p->GetSize(), static_cast<uint32_t>(256));
    p->CopyData(buffer, bytesToCopy);
    
    mavlink_message_t msg;
    mavlink_status_t status;
    for (uint32_t i = 0; i < bytesToCopy; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                NS_LOG_INFO("MAVLink GPS_RAW_INT: lat=" << gps.lat/1e7 
                            << " lon=" << gps.lon/1e7 << " alt=" << gps.alt/1000.0);
            }
        }
    }
}

void InstallPacketSinks(NodeContainer nodes, std::vector<uint16_t> ports, double simTime) {
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        for (uint16_t port : ports) {
            PacketSinkHelper sink("ns3::UdpSocketFactory",
                                  InetSocketAddress(Ipv4Address::GetAny(), port));
            ApplicationContainer sinkApp = sink.Install(nodes.Get(i));
            sinkApp.Start(Seconds(0.0));
            sinkApp.Stop(Seconds(simTime));

            Ptr<PacketSink> ps = sinkApp.Get(0)->GetObject<PacketSink>();
            Callback<void, std::string, Ptr<const Packet>, const Address&> cb =
                [i](std::string /*context*/, Ptr<const Packet> p, const Address& addr) {
                    PacketReceived(p, addr, i);
                };
            ps->TraceConnect("Rx", "Drone" + std::to_string(i), cb);
        }
    }
}

int main(int argc, char *argv[]) {
    uint32_t numDrones = 3;
    double simTime = 200.0;
    double refLat = -35.3633;
    double refLon = 149.165;
    double refAlt = 0.0;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("refLat", "Reference latitude", refLat);
    cmd.AddValue("refLon", "Reference longitude", refLon);
    cmd.AddValue("refAlt", "Reference altitude", refAlt);
    cmd.Parse(argc, argv);

    s_refLat = refLat;
    s_refLon = refLon;
    s_refAlt = refAlt;
    s_metersPerDegreeLon = s_metersPerDegreeLat * std::cos(s_refLat * M_PI / 180.0);

    // Setup ZMQ command publisher
    g_commandPublisher = new zmq::socket_t(g_zmqContext, ZMQ_PUB);
    g_commandPublisher->bind("tcp://*:5555");
    NS_LOG_INFO("ZMQ command publisher bound to port 5555");

    Simulator::SetImplementation(CreateObject<RealtimeSimulatorImpl>());

    // Create only drone nodes
    drones.Create(numDrones);

    // Drones start at (0,0,0)
    MobilityHelper mobilityDrones;
    Ptr<ListPositionAllocator> dronePositionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numDrones; ++i) {
        dronePositionAlloc->Add(Vector(0, 0, 0));
    }
    mobilityDrones.SetPositionAllocator(dronePositionAlloc);
    mobilityDrones.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityDrones.Install(drones);

    // Cache MobilityModel pointers
    droneMobilityModels.clear();
    for (uint32_t i = 0; i < drones.GetN(); ++i) {
        droneMobilityModels.push_back(drones.Get(i)->GetObject<MobilityModel>());
    }

    // Setup WiFi ad-hoc network
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");
    Ssid ssid = Ssid("drone-mesh");
    wifiMac.SetType("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));

    NetDeviceContainer droneDevices = wifi.Install(wifiPhy, wifiMac, drones);

    // Install internet stack
    InternetStackHelper internet;
    internet.Install(drones);

    // Assign IP addresses
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer droneInterfaces = ipv4.Assign(droneDevices);

    // Store drone IP addresses
    for (uint32_t i = 0; i < droneInterfaces.GetN(); ++i) {
        droneIpAddresses.push_back(droneInterfaces.GetAddress(i));
        NS_LOG_INFO("Drone " << i << " IP: " << droneInterfaces.GetAddress(i));
    }

    // Create UDP sockets for drone-to-drone communication
    for (uint32_t i = 0; i < drones.GetN(); i++) {
        Ptr<Socket> socket = Socket::CreateSocket(drones.Get(i), UdpSocketFactory::GetTypeId());
        socket->Bind();
        g_droneSockets.push_back(socket);
        NS_LOG_INFO("Created UDP socket for Drone " << i);
    }

    // Install packet sinks for MAVLink ports and GPS forwarding port
    std::vector<uint16_t> ports = {20000};
    InstallPacketSinks(drones, ports, simTime);

    // Schedule waypoint commands
    Simulator::Schedule(Seconds(20.0), &SendWaypointPairFromDrone0, 0);
    Simulator::Schedule(Seconds(30.0), &SendWaypointPairFromDrone0, 1);
    Simulator::Schedule(Seconds(40.0), &SendWaypointPairFromDrone0, 2);

    // Setup output files
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "ns3-output/multi-drone-mesh-" // Change this to where you want to save the PCAP files
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string outputPrefix = oss.str();

    std::ostringstream animOss;
    animOss << "ns3-output/multi-drone-mesh-anim_"  // Change this to where you want to save the anim output 
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".xml";
    std::string animFile = animOss.str();

    wifiPhy.EnablePcapAll(outputPrefix);

    // Setup animation
    AnimationInterface anim(animFile);
    anim.SetMobilityPollInterval(Seconds(0.1));
    anim.EnablePacketMetadata(true);
    
    for (uint32_t i = 0; i < drones.GetN(); ++i) {
        anim.UpdateNodeSize(drones.Get(i)->GetId(), 5, 5);
        anim.UpdateNodeDescription(drones.Get(i), "Drone " + std::to_string(i));
        anim.UpdateNodeColor(drones.Get(i), 100, 0, 0);  // Red for all drones
    }

    // Initialize MAVLink parser
    memset(&mavlink_status, 0, sizeof(mavlink_status));

    // Start ZMQ position receiver thread
    std::thread zmqPositionThread(ZmqPositionReceiverThread);

    // Schedule position updates from queue
    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);
    
    // Schedule position logging
    Simulator::Schedule(Seconds(1.0), &PrintDronePositions, drones, 1.0, simTime);

    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Cleanup
    keepRunning.store(false);
    zmqPositionThread.join();
    delete g_commandPublisher;
    Simulator::Destroy();

    return 0;
}
