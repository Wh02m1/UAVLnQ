/*
================================================================================
DRONE MESH NETWORK SIMULATION - TOPOLOGY OVERVIEW
================================================================================

Internal Network Topology (ns-3 simulation):
------------------------------------------
1. Drones (Nodes 0-2):
   - Mobility: Initially at (0,0,0), updated via ZMQ position stream
   - IPs: 10.1.1.1 (Drone0), 10.1.1.2 (Drone1), 10.1.1.3 (Drone2)
   - Ports: 
        5550: Drone0 command channel
        5551: Drone1 command channel
        5552: Drone2 command channel
        10000-10002: Drone-to-drone communication ports

2. Communication:
   - 802.11a Ad-hoc WiFi network
   - All nodes in same subnet (10.1.1.0/24)
   - Direct UDP communication between drones
   - MAVLink commands for navigation
   - UDP echo for drone-to-drone communication

External Interfaces (ZMQ):
--------------------------
1. Position Update Channel:
   - tcp://localhost:5556 : Position updates (format: "droneId,lat,lon,alt")

2. Command Publisher:
   - tcp://*:5555 : Publishes NAV_WAYPOINT commands to external systems

Coordinate System:
------------------
- Global reference point: (refLat, refLon, refAlt)
- Local Cartesian conversion:
    x = (lon - refLon) * metersPerDegreeLon
    y = (lat - refLat) * metersPerDegreeLat
    z = alt - refAlt

Simulation Workflow:
-------------------
1. Setup mesh network topology
2. Start ZMQ thread for position updates
3. Schedule waypoint commands at 20s, 30s, 40s
4. Install UDP echo servers and clients for drone-to-drone communication
5. Process incoming ZMQ position messages
6. Update drone positions from ZMQ stream
7. Visualize with NetAnim and log positions
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
std::queue<std::string> positionQueue;

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
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 5551));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 5552));
    
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

void ProcessPositionMessage(const std::string &msg) {
    std::istringstream iss(msg);
    std::string token;
    int droneId = -1;
    double lat = 0, lon = 0, alt = 0;

    if (std::getline(iss, token, ',')) droneId = std::stoi(token);
    if (std::getline(iss, token, ',')) lat = std::stod(token);
    if (std::getline(iss, token, ',')) lon = std::stod(token);
    if (std::getline(iss, token, ',')) alt = std::stod(token);

    if (droneId >= 0 && static_cast<uint32_t>(droneId) < drones.GetN()) {
        double x = (lon - s_refLon) * s_metersPerDegreeLon;
        double y = (lat - s_refLat) * s_metersPerDegreeLat;
        double z = alt - s_refAlt;

        droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
        NS_LOG_INFO("Updated Drone " << droneId << " position to (" << x << "," << y << "," << z << ")");
    } else {
        NS_LOG_WARN("Invalid droneId or out of range: " << droneId);
    }
}

void ZmqPositionReceiverThread() {
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");
    subscriber.set(zmq::sockopt::subscribe, "");
    NS_LOG_INFO("ZMQ Position subscriber connected to port 5556");

    while (keepRunning.load()) {
        zmq::message_t message;
        if (subscriber.recv(message, zmq::recv_flags::dontwait)) {
            std::string msg_str(static_cast<char *>(message.data()), message.size());
            {
                std::lock_guard<std::mutex> lock(positionMutex);
                positionQueue.push(msg_str);
            }
            NS_LOG_INFO("ZMQ Position update received: " << msg_str);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void UpdatePositionsFromQueue() {
    std::queue<std::string> localQueue;
    {
        std::lock_guard<std::mutex> lock(positionMutex);
        std::swap(localQueue, positionQueue);
    }

    while (!localQueue.empty()) {
        ProcessPositionMessage(localQueue.front());
        localQueue.pop();
    }

    Simulator::Schedule(Seconds(0.1), &UpdatePositionsFromQueue);
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
    
    uint8_t buffer[32];
    uint32_t bytesToCopy = std::min(p->GetSize(), static_cast<uint32_t>(32));
    p->CopyData(buffer, bytesToCopy);
    std::string packetData(reinterpret_cast<char*>(buffer), bytesToCopy);
    NS_LOG_INFO("Packet content: " << packetData);
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

    // Install packet sinks for MAVLink ports
    std::vector<uint16_t> mavlinkPorts = {5550, 5551, 5552};
    InstallPacketSinks(drones, mavlinkPorts, simTime);

    // ========================================================================
    // ADDED: Install UDP echo servers for drone-to-drone communication
    // ========================================================================
    uint16_t basePort = 10000;
    std::vector<uint16_t> echoPorts;
    
    // Install UDP echo servers on each drone
    for (uint32_t i = 0; i < drones.GetN(); ++i) {
        uint16_t dronePort = basePort + i;
        echoPorts.push_back(dronePort);
        
        UdpEchoServerHelper echoServer(dronePort);
        ApplicationContainer serverApp = echoServer.Install(drones.Get(i));
        serverApp.Start(Seconds(0.0));
        serverApp.Stop(Seconds(simTime));
        NS_LOG_INFO("Installed UDP echo server on Drone " << i << " at port " << dronePort);
    }

    // Install UDP echo clients for drone-to-drone communication
    for (uint32_t i = 0; i < drones.GetN(); ++i) {
        for (uint32_t j = 0; j < drones.GetN(); ++j) {
            if (i != j) { // Avoid self-communication
                UdpEchoClientHelper echoClient(droneIpAddresses[j], basePort + j);
                echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
                echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                echoClient.SetAttribute("PacketSize", UintegerValue(512)); // Smaller packets for drone-to-drone
                
                ApplicationContainer clientApp = echoClient.Install(drones.Get(i));
                clientApp.Start(Seconds(2.0)); // Start after servers
                clientApp.Stop(Seconds(simTime));
                
                NS_LOG_INFO("Installed UDP echo client on Drone " << i 
                            << " to communicate with Drone " << j << " on port " << (basePort + j));
            }
        }
    }
    // ========================================================================
    // END OF ADDED SECTION
    // ========================================================================

    // Schedule waypoint commands
    Simulator::Schedule(Seconds(20.0), &SendWaypointPairFromDrone0, 0);
    Simulator::Schedule(Seconds(30.0), &SendWaypointPairFromDrone0, 1);
    Simulator::Schedule(Seconds(40.0), &SendWaypointPairFromDrone0, 2);

    // Setup output files
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << "/home/boda/Desktop/Final-DroneNS3/ns3-output/drone-mesh-"
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string outputPrefix = oss.str();

    std::ostringstream animOss;
    animOss << "/home/boda/Desktop/Final-DroneNS3/ns3-output/drone-mesh-anim_"
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

    // Start ZMQ position receiver thread
    std::thread zmqPositionThread(ZmqPositionReceiverThread);

    // Schedule position updates from queue
    Simulator::Schedule(Seconds(0.1), &UpdatePositionsFromQueue);
    
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