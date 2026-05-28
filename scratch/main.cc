#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/realtime-simulator-impl.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/point-to-point-module.h"
#include "ns3/aodv-module.h"
#include "ns3/flow-monitor-module.h"
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

NetDeviceContainer droneDevices;
Ipv4InterfaceContainer droneInterfaces;
std::vector<Ptr<MobilityModel>> droneMobilityModels;

// Global GPS reference point
static double s_refLat = -35.3633;
static double s_refLon = 149.165;
static double s_refAlt = 0.0;
static double s_metersPerDegreeLat = 111320.0;
static double s_metersPerDegreeLon = s_metersPerDegreeLat * std::cos(s_refLat * M_PI / 180.0);
std::vector<double> starting_x_position = {0.0, 80.0, 160.0, 240.0};

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
// This was altered to fit a multihop topology
std::vector<uint8_t> CreateMavlinkPacket(
    uint8_t system_id,
    uint8_t target_system,
    uint8_t target_component,
    float lat, float lon, float alt)
{
    mavlink_message_t msg;

    uint8_t component_id = 1;

    mavlink_mission_item_t mission_item{};
    mission_item.target_system = target_system;
    mission_item.target_component = target_component;
    mission_item.seq = 0;
    mission_item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mission_item.command = MAV_CMD_NAV_WAYPOINT;
    mission_item.current = 0;
    mission_item.autocontinue = 1;

    mission_item.param1 = 0;
    mission_item.param2 = 0;
    mission_item.param3 = 0;
    mission_item.param4 = 0;

    mission_item.x = lat;
    mission_item.y = lon;
    mission_item.z = alt;

    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &mission_item);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    return std::vector<uint8_t>(buffer, buffer + len);
}

    // Send waypoint commands directly from Drone0 to all followers
    // Waypoints format: {x, y, altitude} in meters (local coordinates)
    // Each follower has an array of waypoints sent at different times:
    //   - waypoints[0] sent at t=20s (pairIndex 0)
    //   - waypoints[1] sent at t=30s (pairIndex 1)
    //   - waypoints[2] sent at t=40s (pairIndex 2)
    // So each follower has a 3 waypoint mission send to him at different times (Can be extended and changed)
void SendWaypointPairFromDrone0(int pairIndex)
{
    if (pairIndex < 0) return;

    static const std::vector<std::vector<std::tuple<float,float,float>>> follower_waypoints = {

        // Drone 1
        {{50,60,30},{20,60,30},{20,30,30}},

        // Drone 2
        {{40,50,30},{30,40,30},{50,20,30}},

        // Drone 3
        {{60,70,30},{40,60,30},{70,30,30}},

        // Drone 4
        {{80,80,30},{60,80,30},{60,40,30}}
};

    for (uint32_t follower = 1; follower < drones.GetN(); ++follower)
    {
        uint32_t idx = follower - 1;

        if (idx >= follower_waypoints.size()) break;
        if (pairIndex >= follower_waypoints[idx].size()) continue;

        auto [x, y, z] = follower_waypoints[idx][pairIndex];

        uint8_t target_system = follower + 1;

        std::vector<uint8_t> pkt =
            CreateMavlinkPacket(1, target_system, 0, x, y, z);

        Ptr<Packet> packet = Create<Packet>(pkt.data(), pkt.size());

        InetSocketAddress dest(droneIpAddresses[follower], 20000);

        g_droneSockets[0]->SendTo(packet, 0, dest);

        NS_LOG_INFO("Leader sent waypoint to drone "
                    << follower << " via MANET routing");
    }
}


// Process binary MAVLink GPS_RAW_INT, SYS_STATUS, and HEARTBEAT messages that will received from ZMQ 
// ZMQ message format: [message_type (1 byte), drone_id (1 byte), MAVLink message bytes]
// Message types: 0 = GPS_RAW_INT, 1 = SYS_STATUS, 2 = HEARTBEAT
//
// ========== OPTIMIZATION: SELECTIVE FORWARDING==========
// Only forward commands from leader (Drone 0) to prevent network flooding
// GPS, HEARTBEAT, and SYS_STATUS are processed locally but NOT forwarded
// This reduces network traffic by ~90% for large swarms (10+ drones)
// This was altered to fit a multihop topology
// ========================================================
void ProcessMavlinkMessage(const std::vector<uint8_t>& data)
{
    if (data.size() < 2) return;

    //uint8_t msg_type = data[0];
    uint8_t droneId = data[1];

    if (droneId >= drones.GetN()) {
        NS_LOG_WARN("Invalid drone ID: " << (int)droneId);
        return;
    }

    mavlink_message_t msg;
    mavlink_status_t* status = &mavlink_status_map[droneId];

    for (size_t i = 2; i < data.size(); i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, status))
        {
            // ---------------- GPS (local update only)
            if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
            {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);

                double lat = gps.lat / 1e7;
                double lon = gps.lon / 1e7;
                double alt = gps.alt / 1000.0;

                double x = (lon - s_refLon) * s_metersPerDegreeLon;
                double y = (lat - s_refLat) * s_metersPerDegreeLat;
                double z = alt - s_refAlt;

                if (droneId < droneMobilityModels.size() &&
                    droneMobilityModels[droneId])
                {
                    droneMobilityModels[droneId]->SetPosition(Vector(starting_x_position[droneId], 0.0, alt - s_refAlt));
                }
            }

            // ---------------- SYS_STATUS (log only)
            else if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS)
            {
                mavlink_sys_status_t sys;
                mavlink_msg_sys_status_decode(&msg, &sys);
            }

            // ---------------- HEARTBEAT (log only)
            else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
            {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
            }

            // ---------------- CONTROL MESSAGES (DO NOT FORWARD HERE)
            // IMPORTANT: no routing logic here anymore
            else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
            {
                NS_LOG_INFO("Mission item received by drone " << (int)droneId);
            }
            else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
            {
                NS_LOG_INFO("Command received by drone " << (int)droneId);
            }
            else if (msg.msgid == MAVLINK_MSG_ID_SET_MODE)
            {
                NS_LOG_INFO("Mode change received by drone " << (int)droneId);
            }
        }
    }
}

// Modified ZMQ receiver to handle binary data
// This was altered to fit a multihop topology
void ZmqPositionReceiverThread()
{
    zmq::socket_t subscriber(g_zmqContext, ZMQ_SUB);

    subscriber.connect("tcp://localhost:5556");
    subscriber.set(zmq::sockopt::subscribe, "");

    NS_LOG_INFO("ZMQ subscriber connected to port 5556");

    const size_t MAX_QUEUE_SIZE = 1000;

    while (keepRunning.load())
    {
        zmq::message_t message;

        if (subscriber.recv(message, zmq::recv_flags::dontwait))
        {
            uint8_t* data =
                static_cast<uint8_t*>(message.data());

            size_t length = message.size();

            if (length < 3)
                continue;

            std::vector<uint8_t> msg_vec(data, data + length);

            {
                std::lock_guard<std::mutex> lock(positionMutex);

                if (positionQueue.size() < MAX_QUEUE_SIZE)
                {
                    positionQueue.push(std::move(msg_vec));
                }
            }

            NS_LOG_DEBUG("ZMQ MAVLink message received ("
                         << length << " bytes)");
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(2));
    }
}
// This was altered to fit a multihop topology
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
// This was altered to fit a multihop topology
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
// This was altered to fit a multihop topology
void PacketReceived(Ptr<const Packet> p,
                    const Address& addr,
                    uint32_t droneId)
{
    InetSocketAddress inetAddr =
        InetSocketAddress::ConvertFrom(addr);

    NS_LOG_INFO("Drone "
        << droneId
        << " received MANET packet from "
        << inetAddr.GetIpv4()
        << " size=" << p->GetSize());

    std::vector<uint8_t> buffer(p->GetSize());

    p->CopyData(buffer.data(), buffer.size());

    mavlink_message_t msg;
    mavlink_status_t status;

    for (size_t i = 0; i < buffer.size(); i++)
    {
        if (mavlink_parse_char(
                MAVLINK_COMM_0,
                buffer[i],
                &msg,
                &status))
        {
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
            {
                mavlink_mission_item_t mission;

                mavlink_msg_mission_item_decode(
                    &msg,
                    &mission);

                NS_LOG_INFO("Drone "
                    << droneId
                    << " received waypoint: "
                    << mission.x << ", "
                    << mission.y << ", "
                    << mission.z);
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


int main(int argc, char *argv[])
{
    uint32_t nNodes = 4;
    double simTime = 50.0;
    double txRate = 1.0; // packets/sec
    std::string outputDir = "/home/ubuntu/UAVLnQ/ns3-output"; 


    CommandLine cmd;
    cmd.AddValue("nNodes", "Number of nodes", nNodes);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("o", "Output directory for PCAP and animation files", outputDir);
    cmd.Parse(argc, argv);

    // -----------------------
    // 1. Create nodes
    // -----------------------
    NodeContainer nodes;
    nodes.Create(nNodes);

    // -----------------------
    // 2. Mobility (linear chain)
    // -----------------------
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();

    for (uint32_t i = 0; i < nNodes; i++)
        pos->Add(Vector(i * 80, 0, 20));

    mobility.SetPositionAllocator(pos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // -----------------------
    // 3. WiFi Adhoc
    // -----------------------
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange", DoubleValue(120));

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // -----------------------
    // 4. AODV ONLY
    // -----------------------
    AodvHelper aodv;

    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // -----------------------
    // 5. UDP traffic (simple chain)
    // -----------------------
    uint16_t port = 9;

    // Sink at last node
    PacketSinkHelper sink("ns3::UdpSocketFactory",
                          InetSocketAddress(Ipv4Address::GetAny(), port));
    
    sink.Install(nodes.Get(1));
    ApplicationContainer sinkApp = sink.Install(nodes.Get(nNodes - 1)); 
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    // Source at node 0
    OnOffHelper app("ns3::UdpSocketFactory",
                    InetSocketAddress(interfaces.GetAddress(nNodes - 1), port)); 
    app.Install(nodes.Get(0));
    app.SetAttribute("DataRate", StringValue("1kbps"));
    app.SetAttribute("PacketSize", UintegerValue(64));
    app.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    app.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer srcApp = app.Install(nodes.Get(0));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simTime));
    
    // Setup output files
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << outputDir << "/multihop-aodv"
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string outputPrefix = oss.str();

    std::ostringstream animOss;
    animOss << outputDir << "/multihop-aodv-anim_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".xml";
    std::string animFile = animOss.str();

    AnimationInterface anim(animFile);
    anim.SetMobilityPollInterval(Seconds(0.1));
    anim.EnablePacketMetadata(true);

    anim.UpdateNodeDescription(nodes.Get(0), "Source");
    anim.UpdateNodeDescription(nodes.Get(nodes.GetN() - 1), "Destination");
    anim.EnableIpv4RouteTracking("routes.xml", Seconds(0), Seconds(10), Seconds(5));
    anim.UpdateNodeColor(nodes.Get(0), 0, 255, 0);
    anim.UpdateNodeColor(nodes.Get(1), 0, 0, 255);
    anim.UpdateNodeColor(nodes.Get(2), 255, 0, 0);
    anim.UpdateNodeColor(nodes.Get(3), 255, 0, 0);
    // -----------------------
    // 6. Flow Monitor
    // -----------------------
    phy.EnablePcapAll(outputPrefix);
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

    auto stats = monitor->GetFlowStats();

    std::cout << "\n===== AODV BASELINE RESULTS =====\n";

    for (auto &flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        double pdr = (flow.second.txPackets > 0)
                       ? (double)flow.second.rxPackets / flow.second.txPackets
                       : 0;

        std::cout << "Flow " << flow.first
                  << " " << t.sourceAddress << " -> " << t.destinationAddress << "\n"
                  << "TX=" << flow.second.txPackets
                  << " RX=" << flow.second.rxPackets
                  << " LOST=" << flow.second.lostPackets
                  << " PDR=" << pdr << "\n\n";
    }

    Simulator::Destroy();
    return 0;
}

