/* *** main.cc ***
 *
 * Wormhole Attack Simulation integrated into AODV Drone Mesh Network
 *
 * Original AODV drone mesh code extended with an out-of-band wormhole attack:
 *   - W1 is placed near Node 0 (leader)
 *   - W2 is placed near Node 3 (last follower)
 *   - Both wormhole nodes share the main 802.11a channel so AODV sees them
 *     as legitimate hops, while also being connected via a private P2P tunnel
 *     that makes the W1->W2 path appear artificially short.
 *
 * Network topology (linear chain + wormhole):
 *
 *   [Node0] -- [Node1] -- [Node2] -- [Node3]
 *      |                                |
 *     [W1] ======= P2P tunnel ======= [W2]
 *
 * IP addressing:
 *   Main Wi-Fi channel : 10.1.1.0/24  (all nodes including W1, W2)
 *   P2P wormhole tunnel: 10.1.2.0/24  (W1 and W2 only)
 *
 * RUN IT WITH:  NS_LOG="DroneWormholeMesh=info" ~/ns-3-dev/build/scratch/multihop-with-attacker/ns3-dev-multihop-with-attacker-default --nNodes=7 --simTime=50 --o=/home/ubuntu/UAVLnQ/ns3-output/ 2>&1 | tee /home/ubuntu/UAVLnQ/ns3-output/wh.log
 *
 * Reference Code: 
 * HLM2022, Wormhole-for-NS3, ver. 1.0, [Source code], 2022. [Online]. Available: https://github.com/HLM2022/Wormhole-for-NS3. Accessed: May 26, 2026.
 * 
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/aodv-module.h"
#include "ns3/flow-monitor-module.h"

// ZMQ / MAVLink headers (keep if building with full drone stack)
// Comment these out if building standalone without ZMQ/MAVLink
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

NS_LOG_COMPONENT_DEFINE("DroneWormholeMesh");

// ---------------------------------------------------------------------------
// Globals (drone / ZMQ state)
// ---------------------------------------------------------------------------
static std::atomic<bool> keepRunning(true);
std::mutex positionMutex;
std::queue<std::vector<uint8_t>> positionQueue;

NodeContainer drones;          // normal drone nodes (nNodes)
NodeContainer p2pNodes;        // wormhole nodes W1, W2
NodeContainer allNodes;        // drones + p2pNodes

NetDeviceContainer droneDevices;
Ipv4InterfaceContainer droneInterfaces;
std::vector<Ptr<MobilityModel>> droneMobilityModels;

// GPS reference (Canberra)
static double s_refLat            =  -35.3633;
static double s_refLon            =  149.165;
static double s_refAlt            =    0.0;
static double s_metersPerDegreeLat = 111320.0;
static double s_metersPerDegreeLon = s_metersPerDegreeLat *
                                     std::cos(s_refLat * M_PI / 180.0);

std::vector<Ipv4Address>   droneIpAddresses;
zmq::context_t             g_zmqContext(1);
zmq::socket_t*             g_commandPublisher = nullptr;
std::vector<Ptr<Socket>>   g_droneSockets;
std::map<uint8_t, mavlink_status_t> mavlink_status_map;

// ---------------------------------------------------------------------------
// MAVLink helpers (unchanged from original)
// ---------------------------------------------------------------------------
std::vector<uint8_t> CreateMavlinkPacket(
    uint8_t system_id,
    uint8_t target_system,
    uint8_t target_component,
    float lat, float lon, float alt)
{
    mavlink_message_t msg;
    uint8_t component_id = 1;

    mavlink_mission_item_t mi{};
    mi.target_system    = target_system;
    mi.target_component = target_component;
    mi.seq              = 0;
    mi.frame            = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    mi.command          = MAV_CMD_NAV_WAYPOINT;
    mi.current          = 0;
    mi.autocontinue     = 1;
    mi.x = lat;  mi.y = lon;  mi.z = alt;

    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &mi);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

// ---------------------------------------------------------------------------
// Waypoint dispatch (leader -> followers via AODV mesh)
// ---------------------------------------------------------------------------
void SendWaypointPairFromDrone0(int pairIndex)
{
    if (pairIndex < 0) return;

    static const std::vector<std::vector<std::tuple<float,float,float>>>
        follower_waypoints = {
            // Drone 1
            {{50, 60,30},{20, 60,30},{20, 30,30}},
            // Drone 2
            {{40, 50,30},{30, 40,30},{50, 20,30}},
            // Drone 3
            {{60, 70,30},{40, 60,30},{70, 30,30}},
            // Drone 4
            {{80, 80,30},{60, 80,30},{60, 40,30}},
            // Drone 5
            {{100, 60,30},{80, 50,30},{90, 30,30}},
            // Drone 6
            {{120, 70,30},{100, 80,30},{110, 40,30}}
        };

    for (uint32_t follower = 1; follower < drones.GetN(); ++follower)
    {
        uint32_t idx = follower - 1;
        if (idx >= follower_waypoints.size()) break;
        if (pairIndex >= (int)follower_waypoints[idx].size()) continue;

        auto [x, y, z] = follower_waypoints[idx][pairIndex];
        uint8_t target_system = follower + 1;

        std::vector<uint8_t> pkt =
            CreateMavlinkPacket(1, target_system, 0, x, y, z);

        Ptr<Packet> packet = Create<Packet>(pkt.data(), pkt.size());
        InetSocketAddress dest(droneIpAddresses[follower], 20000);
        g_droneSockets[0]->SendTo(packet, 0, dest);

        NS_LOG_INFO("Leader sent waypoint pair " << pairIndex
                    << " to drone " << follower << " via AODV mesh");
    }
}

// ---------------------------------------------------------------------------
// MAVLink message processor
// ---------------------------------------------------------------------------
void ProcessMavlinkMessage(const std::vector<uint8_t>& data)
{
    if (data.size() < 2) return;

    uint8_t droneId = data[1];
    if (droneId >= allNodes.GetN()) {
        NS_LOG_WARN("Invalid drone ID: " << (int)droneId);
        return;
    }

    mavlink_message_t msg;
    mavlink_status_t* status = &mavlink_status_map[droneId];

    for (size_t i = 2; i < data.size(); i++)
    {
        if (!mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, status))
            continue;

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
                droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
            }
        }
        else if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS)
        {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(&msg, &sys);
        }
        else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
        {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
        }
        else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
        {
            NS_LOG_INFO("Mission item received by node " << (int)droneId);
        }
        else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        {
            NS_LOG_INFO("Command received by node " << (int)droneId);
        }
        else if (msg.msgid == MAVLINK_MSG_ID_SET_MODE)
        {
            NS_LOG_INFO("Mode change received by node " << (int)droneId);
        }
    }
}

// ---------------------------------------------------------------------------
// ZMQ receiver thread
// ---------------------------------------------------------------------------
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
            uint8_t* data   = static_cast<uint8_t*>(message.data());
            size_t   length = message.size();
            if (length < 3) continue;

            std::vector<uint8_t> msg_vec(data, data + length);
            {
                std::lock_guard<std::mutex> lock(positionMutex);
                if (positionQueue.size() < MAX_QUEUE_SIZE)
                    positionQueue.push(std::move(msg_vec));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// ---------------------------------------------------------------------------
// Simulator-side helpers
// ---------------------------------------------------------------------------
void UpdatePositionsFromQueue()
{
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

void PrintDronePositions(NodeContainer nodes, double interval, double simTime)
{
    Time now = Simulator::Now();
    if (now.GetSeconds() > simTime) return;

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
        if (mob) {
            Vector pos = mob->GetPosition();
            std::cout << "Time " << now.GetSeconds()
                      << "s, Node " << i
                      << " Position: (" << pos.x << ", "
                      << pos.y << ", " << pos.z << ")\n";
        }
    }
    Simulator::Schedule(Seconds(interval),
                        &PrintDronePositions, nodes, interval, simTime);
}

void PacketReceived(Ptr<const Packet> p, const Address& addr, uint32_t nodeId)
{
    InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(addr);
    NS_LOG_INFO("Node " << nodeId
                << " received MANET packet from " << inetAddr.GetIpv4()
                << " size=" << p->GetSize());

    std::vector<uint8_t> buffer(p->GetSize());
    p->CopyData(buffer.data(), buffer.size());

    mavlink_message_t msg;
    mavlink_status_t  status;

    for (size_t i = 0; i < buffer.size(); i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
        {
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
            {
                mavlink_mission_item_t mission;
                mavlink_msg_mission_item_decode(&msg, &mission);
                NS_LOG_INFO("Node " << nodeId
                            << " received waypoint: "
                            << mission.x << ", "
                            << mission.y << ", "
                            << mission.z);
            }
        }
    }
}

void InstallPacketSinks(NodeContainer nodes,
                        std::vector<uint16_t> ports,
                        double simTime)
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        for (uint16_t port : ports)
        {
            PacketSinkHelper sink("ns3::UdpSocketFactory",
                InetSocketAddress(Ipv4Address::GetAny(), port));
            ApplicationContainer sinkApp = sink.Install(nodes.Get(i));
            sinkApp.Start(Seconds(0.0));
            sinkApp.Stop(Seconds(simTime));

            Ptr<PacketSink> ps = sinkApp.Get(0)->GetObject<PacketSink>();
            Callback<void, std::string, Ptr<const Packet>, const Address&> cb =
                [i](std::string, Ptr<const Packet> p, const Address& addr) {
                    PacketReceived(p, addr, i);
                };
            ps->TraceConnect("Rx", "Node" + std::to_string(i), cb);
        }
    }
}

// ===========================================================================
// main()
// ===========================================================================
int main(int argc, char *argv[])
{
    LogComponentEnable("DroneWormholeMesh", LOG_LEVEL_INFO);

    // -----------------------------------------------------------------------
    // 0. Simulation parameters
    // -----------------------------------------------------------------------
    uint32_t nNodes   = 7;     // normal drone nodes (N0 \u2026 N(nNodes-1))
    double   simTime  = 50.0;
    std::string outputDir = "/home/ubuntu/UAVLnQ/ns3-output";

    // Wormhole node positions (scenario 2: within range of N0 and N(nNodes-1))
    double W1X = -10.0,  W1Y = -30.0;   // near Node 0  (x=0)
    double W2X =  0.0,   W2Y = -30.0;   // updated in main after nNodes parsed

    CommandLine cmd;
    cmd.AddValue("nNodes",  "Number of normal drone nodes", nNodes);
    cmd.AddValue("simTime", "Simulation time (s)",          simTime);
    cmd.AddValue("o",       "Output directory",             outputDir);
    cmd.Parse(argc, argv);

    // Now that nNodes is known, place W2 next to the last normal node
    W2X = (nNodes - 1) * 80.0 + 10.0;

    // -----------------------------------------------------------------------
    // 1. Create nodes
    //    drones   : N0 \u2026 N(nNodes-1)  \u2014 normal mesh nodes
    //    p2pNodes : W1, W2            \u2014 wormhole nodes
    //    allNodes : drones + p2pNodes \u2014 everything (for stack install)
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Creating nodes: " << nNodes << " drones + 2 wormhole nodes");

    drones.Create(nNodes);

    // Wormhole nodes \u2014 created BEFORE stack install so AODV covers them
    p2pNodes.Create(2);

    allNodes.Add(drones);
    allNodes.Add(p2pNodes);

    // -----------------------------------------------------------------------
    // 2. Mobility
    //    Normal nodes: linear chain, 80 m spacing, z=20 m
    //    Wormhole nodes: near N0 and N(nNodes-1), below the chain
    // -----------------------------------------------------------------------
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();

    // Normal drone positions
    for (uint32_t i = 0; i < nNodes; i++)
        pos->Add(Vector(i * 80.0, 0.0, 20.0));

    // Wormhole node positions
    pos->Add(Vector(W1X, W1Y, 20.0));   // W1
    pos->Add(Vector(W2X, W2Y, 20.0));   // W2

    mobility.SetPositionAllocator(pos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(allNodes);

    // Cache mobility model pointers for ZMQ GPS updates
    for (uint32_t i = 0; i < allNodes.GetN(); ++i)
        droneMobilityModels.push_back(
            allNodes.Get(i)->GetObject<MobilityModel>());

    // -----------------------------------------------------------------------
    // 3a. Main 802.11a channel  (all nodes share this \u2014 wormhole is invisible)
    // -----------------------------------------------------------------------
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel",
                               "MaxRange", DoubleValue(80.0));

    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode",    StringValue("OfdmRate6Mbps"),
        "ControlMode", StringValue("OfdmRate6Mbps"));

    // Install main Wi-Fi on ALL nodes (drones + wormhole nodes)
    // This is what makes the wormhole nodes appear as normal AODV hops
    NetDeviceContainer devices = wifi.Install(phy, mac, allNodes);

    // -----------------------------------------------------------------------
    // 3b. Second 802.11a channel  (wormhole nodes only \u2014 private channel)
    //     This mimics the out-of-band link that makes W1 and W2 appear close
    // -----------------------------------------------------------------------
    YansWifiChannelHelper channel2;
    channel2.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel2.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
        "SystemLoss",    DoubleValue(1.0),
        "HeightAboveZ",  DoubleValue(1.5));

    YansWifiPhyHelper phy2;
    phy2.SetChannel(channel2.Create());

    WifiMacHelper mac2;
    mac2.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi2;
    wifi2.SetStandard(WIFI_STANDARD_80211a);
    wifi2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode",    StringValue("OfdmRate6Mbps"),
        "ControlMode", StringValue("OfdmRate6Mbps"));

    // Second Wi-Fi interface installed only on the two wormhole nodes
    NetDeviceContainer wormholeWifiDevices = wifi2.Install(phy2, mac2, p2pNodes);

    // -----------------------------------------------------------------------
    // 3c. Point-to-point tunnel between W1 and W2  (the actual wormhole)
    // -----------------------------------------------------------------------
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay",    StringValue("2ms"));

    NetDeviceContainer p2pDevices = p2p.Install(p2pNodes);

    // -----------------------------------------------------------------------
    // 4. Internet stack with AODV on every node
    // -----------------------------------------------------------------------
    AodvHelper aodv;

    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    // -----------------------------------------------------------------------
    // 5. IP addressing
    //    10.1.1.0/24 \u2014 main Wi-Fi (all allNodes devices)
    //    10.1.2.0/24 \u2014 P2P tunnel (W1 <-> W2 only)
    // -----------------------------------------------------------------------
    Ipv4AddressHelper ipv4;

    // Main subnet \u2014 assign to devices in the same order as allNodes
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer allInterfaces = ipv4.Assign(devices);

    // Slice out the normal drone interface addresses
    for (uint32_t i = 0; i < nNodes; i++)
        droneIpAddresses.push_back(allInterfaces.GetAddress(i));

    // P2P subnet
    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces = ipv4.Assign(p2pDevices);

    NS_LOG_INFO("W1 main IP : " << allInterfaces.GetAddress(nNodes));
    NS_LOG_INFO("W2 main IP : " << allInterfaces.GetAddress(nNodes + 1));
    NS_LOG_INFO("W1 P2P  IP : " << p2pInterfaces.GetAddress(0));
    NS_LOG_INFO("W2 P2P  IP : " << p2pInterfaces.GetAddress(1));

    // -----------------------------------------------------------------------
    // 6. UDP traffic: Node 0 (leader) -> Node (nNodes-1) via AODV
    //    AODV will route through the wormhole if it looks like a shorter path
    // -----------------------------------------------------------------------
    uint16_t port = 9;

    PacketSinkHelper sink("ns3::UdpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sink.Install(drones.Get(nNodes - 1));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(simTime));

    OnOffHelper app("ns3::UdpSocketFactory",
        InetSocketAddress(droneIpAddresses[nNodes - 1], port));
    app.SetAttribute("DataRate",   StringValue("1kbps"));
    app.SetAttribute("PacketSize", UintegerValue(64));
    app.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    app.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer srcApp = app.Install(drones.Get(0));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simTime));

    // -----------------------------------------------------------------------
    // 7. Scheduled waypoint dispatches (t=20, 30, 40 s)
    // -----------------------------------------------------------------------
    Simulator::Schedule(Seconds(20.0), &SendWaypointPairFromDrone0, 0);
    Simulator::Schedule(Seconds(30.0), &SendWaypointPairFromDrone0, 1);
    Simulator::Schedule(Seconds(40.0), &SendWaypointPairFromDrone0, 2);

    // -----------------------------------------------------------------------
    // 8. Position reporting and ZMQ position queue
    // -----------------------------------------------------------------------
    Simulator::Schedule(Seconds(0.0),  &PrintDronePositions, allNodes, 5.0, simTime);
    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);

    // -----------------------------------------------------------------------
    // 9. PCAP + routing table dumps
    // -----------------------------------------------------------------------
    auto t  = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << outputDir << "/wormhole-aodv-"
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string prefix = oss.str();

    phy.EnablePcapAll(prefix + "-main");
    p2p.EnablePcap(prefix + "-wormhole-W1", p2pDevices.Get(0));
    p2p.EnablePcap(prefix + "-wormhole-W2", p2pDevices.Get(1));

    Ptr<OutputStreamWrapper> routingStream =
        Create<OutputStreamWrapper>(prefix + "-routing.txt", std::ios::out);
    aodv.PrintRoutingTableAllAt(Seconds(0.5), routingStream);
    aodv.PrintRoutingTableAllAt(Seconds(2.5), routingStream);
    aodv.PrintRoutingTableAllAt(Seconds(25.0), routingStream);

    // -----------------------------------------------------------------------
    // 10. NetAnim
    // -----------------------------------------------------------------------
    AnimationInterface anim(prefix + "-anim.xml");

    for (uint32_t i = 0; i < nNodes; i++)
    {
        anim.UpdateNodeDescription(drones.Get(i), "N" + std::to_string(i));
        anim.UpdateNodeColor(drones.Get(i), 0, 200, 0);   // green
        anim.UpdateNodeSize(i, 5.0, 5.0);
    }
    anim.UpdateNodeDescription(p2pNodes.Get(0), "W1");
    anim.UpdateNodeDescription(p2pNodes.Get(1), "W2");
    anim.UpdateNodeColor(p2pNodes.Get(0), 0, 0, 255);     // blue
    anim.UpdateNodeColor(p2pNodes.Get(1), 0, 0, 255);
    anim.UpdateNodeSize(nNodes,     5.0, 5.0);
    anim.UpdateNodeSize(nNodes + 1, 5.0, 5.0);
    anim.EnablePacketMetadata(true);

    // -----------------------------------------------------------------------
    // 11. Flow monitor
    // -----------------------------------------------------------------------
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // -----------------------------------------------------------------------
    // 12. Start ZMQ thread and run simulation
    // -----------------------------------------------------------------------
    std::thread zmqThread(ZmqPositionReceiverThread);

    NS_LOG_INFO("Starting simulation (simTime=" << simTime << "s)");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -----------------------------------------------------------------------
    // 13. Post-simulation: flow statistics
    // -----------------------------------------------------------------------
    keepRunning.store(false);
    zmqThread.join();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    auto stats = monitor->GetFlowStats();

    std::cout << "\n===== WORMHOLE + AODV RESULTS =====\n";
    for (auto& flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(flow.first);
        double pdr = (flow.second.txPackets > 0)
                     ? (double)flow.second.rxPackets / flow.second.txPackets
                     : 0.0;

        std::cout << "Flow " << flow.first
                  << "  " << ft.sourceAddress
                  << " -> " << ft.destinationAddress << "\n"
                  << "  TX="   << flow.second.txPackets
                  << "  RX="   << flow.second.rxPackets
                  << "  LOST=" << flow.second.lostPackets
                  << "  PDR="  << pdr
                  << "\n\n";
    }

    Simulator::Destroy();
    return 0;
}
