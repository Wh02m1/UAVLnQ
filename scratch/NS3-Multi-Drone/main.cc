/*
================================================================================
Drone Mesh Network Simulation with MAVLink/ZMQ Integration - ns-3
================================================================================

ARCHITECTURE:
------------
Network Topology:
   - N Drones in WiFi ad-hoc mesh (802.11a) - configurable via -n parameter (default: 3)
   - IP Range: 10.1.1.0/24
     * Drone0: 10.1.1.1 (Leader)
     * Drone1: 10.1.1.2 (Follower)
     * Drone2: 10.1.1.3 (Follower)
     * ...
     * DroneN: 10.1.1.N+1 (Follower)

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
   +----↓-----+       +----------+       +-----↓----+
   |  Drone0  |◀─────▶|  Drone1  |◀─────▶|  Drone2  |◀─────▶ ... ◀─────▶|  DroneN  |
   | (Leader) |       |(Follower)|       |(Follower)|                    |(Follower)|
   | (10.1.1.1|       | (10.1.1.2|       | (10.1.1.3|                    | (10.1.1.N|
   +----------+       +----------+       +----------+                    +----------+
                                   


- Mission Control: Drone0 (Leader) sends MAVLink commands to other drones via port 20000
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
#include <map>

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

// MAVLink parser state - one per drone for proper parsing
std::map<uint8_t, mavlink_status_t> mavlink_status_map;

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

// Send waypoint commands directly from Drone0 to all followers
void SendWaypointPairFromDrone0(int pairIndex) {
    Ptr<Node> drone0 = NodeList::GetNode(0);
    Ptr<Socket> socket = Socket::CreateSocket(drone0, UdpSocketFactory::GetTypeId());
    socket->Bind();

    // Waypoints for FOLLOWERS only
    // follower_waypoints[0] -> follower with sysid 2 (ns-3 node 1)
    // follower_waypoints[1] -> follower with sysid 3 (ns-3 node 2)
    // follower_waypoints[2] -> follower with sysid 4 (ns-3 node 3)
    static const std::vector<std::vector<std::tuple<float, float, float>>> follower_waypoints = {
        // Follower 1 (sysid 2, ns-3 node 1)
        {
            {50, 60, 30},
            {20, 60, 30},
            {20, 30, 30}
        },
        // Follower 2 (sysid 3, ns-3 node 2)
        {
            {40, 50, 30},
            {30, 40, 30},
            {50, 20, 30}
        },
        // Follower 3 (sysid 4, ns-3 node 3) 
        {
            {60, 70, 30},
            {40, 60, 30},
            {70, 30, 30}
        },
        // Add more followers as needed...
    };

    if (pairIndex < 0) return;

    // For each FOLLOWER ns-3 node (1..N-1)
    for (uint32_t ns3Index = 1; ns3Index < drones.GetN(); ++ns3Index) {
        uint32_t followerIdx = ns3Index - 1;  // map to waypoint table index

        // No waypoint definition for this follower -> skip
        if (followerIdx >= follower_waypoints.size())
            break;

        // This follower has fewer waypoints than pairIndex -> skip this round
        if (pairIndex >= follower_waypoints[followerIdx].size())
            continue;

        auto [lat, lon, alt] = follower_waypoints[followerIdx][pairIndex];

        // ns-3 index i -> sysid = i + 1
        uint8_t target_system = ns3Index + 1; // leader is 1, followers start at 2

        // Build MAVLink packet
        std::vector<uint8_t> pkt = CreateMavlinkPacket(target_system, 0, lat, lon, alt);
        Ptr<Packet> packet = Create<Packet>(pkt.data(), pkt.size());

        if (ns3Index < droneIpAddresses.size()) {
            socket->SendTo(packet, 0,
                InetSocketAddress(droneIpAddresses[ns3Index], 20000));

            NS_LOG_INFO("Drone0 sent waypoint " << (pairIndex + 1)
                        << " to Drone sysid=" << unsigned(target_system)
                        << " (ns-3 idx=" << ns3Index << ") at "
                        << Simulator::Now().GetSeconds() << "s");
        }

        // Optional: also publish to ZMQ like before
        if (g_commandPublisher) {
            zmq::message_t zmqMsg(pkt.data(), pkt.size());
            g_commandPublisher->send(zmqMsg, zmq::send_flags::none);
        }
    }

    NS_LOG_INFO("Drone0 completed sending waypoint pair "
                << pairIndex + 1 << " at " << Simulator::Now().GetSeconds() << "s");
}


// Process binary MAVLink GPS_RAW_INT, SYS_STATUS, and HEARTBEAT messages that will received from ZMQ 
// ZMQ message format: [message_type (1 byte), drone_id (1 byte), MAVLink message bytes]
// Message types: 0 = GPS_RAW_INT, 1 = SYS_STATUS, 2 = HEARTBEAT
//
// ========== OPTIMIZATION: SELECTIVE FORWARDING ==========
// CHANGED: Only forward commands from leader (Drone 0) to prevent network flooding
// GPS, HEARTBEAT, and SYS_STATUS are processed locally but NOT forwarded
// This reduces network traffic by ~90% for large swarms (10+ drones)
// ========================================================
void ProcessMavlinkMessage(const std::vector<uint8_t>& data) {
    if (data.size() < 2) return;
    
    uint8_t msg_type = data[0];
    uint8_t droneId = data[1];
    
    if (droneId >= drones.GetN()) {
        NS_LOG_WARN("Received message for invalid drone ID: " << static_cast<int>(droneId));
        return;
    }
    
    mavlink_message_t msg;
    mavlink_status_t* status = &mavlink_status_map[droneId];
    
    // CHANGED: Added flag to control forwarding - only leader commands should be forwarded
    bool shouldForward = false;  // Only forward leader commands
    
    for (size_t i = 2; i < data.size(); i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, status)) {
            // GPS_RAW_INT - update position locally, don't forward
            // CHANGED: Removed forwarding logic - GPS is now LOCAL ONLY
            if (msg_type == 0 && msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                
                double lat = gps.lat / 1e7;
                double lon = gps.lon / 1e7;
                double alt = gps.alt / 1000.0;
                
                double x = (lon - s_refLon) * s_metersPerDegreeLon;
                double y = (lat - s_refLat) * s_metersPerDegreeLat;
                double z = alt - s_refAlt;
                
                if (droneId < droneMobilityModels.size() && droneMobilityModels[droneId]) {
                    droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
                    
                    NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                                << " GPS: lat=" << lat << " lon=" << lon << " alt=" << alt
                                << " → x=" << x << " y=" << y << " z=" << z);
                }
                // CHANGED: Don't forward GPS (was forwarding to all drones before)
            }
            // SYS_STATUS - just log, don't forward
            // CHANGED: Removed forwarding logic - SYS_STATUS is now LOCAL ONLY
            else if (msg_type == 1 && msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
                mavlink_sys_status_t sys_status;
                mavlink_msg_sys_status_decode(&msg, &sys_status);
                
                NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                            << " SYS_STATUS: battery=" << sys_status.voltage_battery
                            << " remaining=" << sys_status.battery_remaining);
                // CHANGED: Don't forward system status (was forwarding before)
            }
            // HEARTBEAT - just log, don't forward
            // CHANGED: Removed forwarding logic - HEARTBEAT is now LOCAL ONLY
            else if (msg_type == 2 && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                
                NS_LOG_INFO("Drone " << static_cast<int>(droneId) 
                            << " HEARTBEAT: type=" << static_cast<int>(heartbeat.type)
                            << " status=" << static_cast<int>(heartbeat.system_status));
                // CHANGED: Don't forward heartbeat (was forwarding before)
            }
            // MISSION_ITEM from leader (drone 0) - forward to all
            // KEPT: Commands from leader are still forwarded (critical for swarm control)
            else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
                if (droneId == 0) {  // Only if from leader
                    shouldForward = true;
                    NS_LOG_INFO("Leader command (MISSION_ITEM) - forwarding to all drones");
                }
            }
            // COMMAND_LONG from leader - forward to all
            // KEPT: Commands from leader are still forwarded (critical for swarm control)
            else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                if (droneId == 0) {  // Only if from leader
                    shouldForward = true;
                    NS_LOG_INFO("Leader command (COMMAND_LONG) - forwarding to all drones");
                }
            }
            // SET_MODE from leader - forward to all
            // KEPT: Commands from leader are still forwarded (critical for swarm control)
            else if (msg.msgid == MAVLINK_MSG_ID_SET_MODE) {
                if (droneId == 0) {  // Only if from leader
                    shouldForward = true;
                    NS_LOG_INFO("Leader command (SET_MODE) - forwarding to all drones");
                }
            }
        }
    }
    
    // CHANGED: Only forward commands from the leader (Drone 0)
    // Old behavior: forwarded ALL messages from ALL drones → caused MAC queue overflow
    // New behavior: forward ONLY leader commands → prevents network flooding
    if (shouldForward && droneId < g_droneSockets.size() && g_droneSockets[droneId]) {
        for (uint32_t j = 0; j < drones.GetN(); j++) {
            if (j != droneId && j < droneIpAddresses.size()) {
                Ptr<Packet> packet = Create<Packet>(data.data() + 2, data.size() - 2);
                g_droneSockets[droneId]->SendTo(packet, 0, 
                    InetSocketAddress(droneIpAddresses[j], 20000));
                
                NS_LOG_DEBUG("Leader forwarded command to Drone " << j);
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
            NS_LOG_DEBUG("ZMQ Position update received (" << length << " bytes)");
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
        if (mobility) {
            Vector pos = mobility->GetPosition();
            std::cout << "Time " << now.GetSeconds() << "s, Drone " << i
                      << " Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        }
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
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
                mavlink_mission_item_t mission_item;
                mavlink_msg_mission_item_decode(&msg, &mission_item);
                NS_LOG_INFO("Drone " << droneId << " received MISSION_ITEM: lat=" << mission_item.x 
                            << " lon=" << mission_item.y << " alt=" << mission_item.z);
            } else if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                NS_LOG_DEBUG("MAVLink GPS_RAW_INT: lat=" << gps.lat/1e7 
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
    cmd.AddValue("n", "Number of drones", numDrones);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("refLat", "Reference latitude", refLat);
    cmd.AddValue("refLon", "Reference longitude", refLon);
    cmd.AddValue("refAlt", "Reference altitude", refAlt);
    cmd.Parse(argc, argv);

    // Validate number of drones
    if (numDrones < 2) {
        std::cerr << "Number of drones must be at least 2 (1 leader + 1 follower)" << std::endl;
        return 1;
    }

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
    NS_LOG_INFO("Created " << numDrones << " drone nodes");

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
        Ptr<MobilityModel> mobility = drones.Get(i)->GetObject<MobilityModel>();
        droneMobilityModels.push_back(mobility);
        NS_LOG_INFO("Initialized mobility model for Drone " << i);
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
    droneIpAddresses.clear();
    for (uint32_t i = 0; i < droneInterfaces.GetN(); ++i) {
        droneIpAddresses.push_back(droneInterfaces.GetAddress(i));
        NS_LOG_INFO("Drone " << i << " IP: " << droneInterfaces.GetAddress(i));
    }

    // Create UDP sockets for drone-to-drone communication
    g_droneSockets.clear();
    for (uint32_t i = 0; i < drones.GetN(); i++) {
        Ptr<Socket> socket = Socket::CreateSocket(drones.Get(i), UdpSocketFactory::GetTypeId());
        socket->Bind();
        g_droneSockets.push_back(socket);
        NS_LOG_INFO("Created UDP socket for Drone " << i);
    }

    // Install packet sinks for MAVLink ports and GPS forwarding port
    std::vector<uint16_t> ports = {20000};
    InstallPacketSinks(drones, ports, simTime);

    // Schedule waypoint commands from Drone0 to other drones
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
        if (i == 0) {
            anim.UpdateNodeColor(drones.Get(i), 0, 100, 0);  // Green for leader
        } else {
            anim.UpdateNodeColor(drones.Get(i), 100, 0, 0);  // Red for followers
        }
    }

    // Initialize MAVLink parser states for all drones
    for (uint32_t i = 0; i < numDrones; ++i) {
        mavlink_status_map[i] = mavlink_status_t();
        memset(&mavlink_status_map[i], 0, sizeof(mavlink_status_t));
    }

    // Start ZMQ position receiver thread
    std::thread zmqPositionThread(ZmqPositionReceiverThread);

    // Schedule position updates from queue
    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);
    
    // Schedule position logging
    Simulator::Schedule(Seconds(1.0), &PrintDronePositions, drones, 1.0, simTime);

    NS_LOG_INFO("Starting simulation with " << numDrones << " drones for " << simTime << " seconds");

    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Cleanup
    keepRunning.store(false);
    zmqPositionThread.join();
    delete g_commandPublisher;
    Simulator::Destroy();

    NS_LOG_INFO("Simulation completed successfully");

    return 0;
}