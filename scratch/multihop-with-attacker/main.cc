/* *** main.cc ***
 *
 * Wormhole Attack Simulation integrated into AODV Drone Mesh Network
 *
 * Original AODV drone mesh code extended with an out-of-band wormhole attack:
 *   - W1 is placed near Node 0 (leader)
 *   - W2 is placed near Node 6 (last follower)
 *   - Both wormhole nodes share the main 802.11a channel so AODV sees them
 *     as legitimate hops, while also being connected via a private P2P tunnel
 *     that makes the W1->W2 path appear artificially short.
 *
 * Network topology (linear chain + wormhole):
 *
 *   [N0]--[N1]--[N2]--[N3]--[N4]--[N5]--[N6]
 *    |                                     |
 *   [W1] ========= P2P tunnel =========  [W2]
 *
 * IP addressing:
 *   Main Wi-Fi channel : 10.1.1.0/24  (all nodes including W1, W2)
 *   P2P wormhole tunnel: 10.1.2.0/24  (W1 and W2 only)
 *
 * Wormhole objective: black-hole attack
 *   W1/W2 poison AODV so N0 routes all traffic via the wormhole tunnel.
 *   The tunnel then drops all UDP data packets while forwarding AODV
 *   control packets (HELLOs/RREPs) so the poisoned route stays alive.
 *   Result: N0 believes delivery is happening but PDR drops to ~0.
 *
 * RUN:
 *   NS_LOG="DroneWormholeMesh=info" \
 *   ~/ns-3-dev/build/scratch/multihop-with-attacker/ns3-dev-multihop-with-attacker-default \
 *   --nNodes=7 --simTime=50 --o=/home/ubuntu/UAVLnQ/ns3-output/ \
 *   2>&1 | tee /home/ubuntu/UAVLnQ/ns3-output/wh.log
 *
 * Reference:
 *   HLM2022, Wormhole-for-NS3, ver. 1.0, [Source code], 2022.
 *   https://github.com/HLM2022/Wormhole-for-NS3. Accessed: May 26, 2026.
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
// Globals
// ---------------------------------------------------------------------------
static std::atomic<bool> keepRunning(true);
std::mutex positionMutex;
std::queue<std::vector<uint8_t>> positionQueue;

NodeContainer drones;     // normal drone nodes N0…N(nNodes-1)
NodeContainer p2pNodes;   // wormhole nodes W1, W2
NodeContainer allNodes;   // drones + p2pNodes

NetDeviceContainer droneDevices;
Ipv4InterfaceContainer droneInterfaces;
std::vector<Ptr<MobilityModel>> droneMobilityModels;

// GPS reference (Canberra)
static double s_refLat             = -35.3633;
static double s_refLon             =  149.165;
static double s_refAlt             =    0.0;
static double s_metersPerDegreeLat = 111320.0;
static double s_metersPerDegreeLon = s_metersPerDegreeLat *
                                     std::cos(s_refLat * M_PI / 180.0);

std::vector<Ipv4Address>          droneIpAddresses;
zmq::context_t                    g_zmqContext(1);
zmq::socket_t*                    g_commandPublisher = nullptr;
std::vector<Ptr<Socket>>          g_droneSockets;
std::map<uint8_t, mavlink_status_t> mavlink_status_map;

// Black-hole drop counter (packets dropped by wormhole tunnel)
static uint32_t g_droppedByWormhole = 0;

// ---------------------------------------------------------------------------
// Wormhole P2P receive filter — black-hole attack
//
// Forwards AODV control packets (<=100 bytes, UDP port 654) so the poisoned
// route stays alive in every node's table.  Drops all larger UDP data packets
// so actual payload never reaches the destination.
// ---------------------------------------------------------------------------
static bool WormholeRxFilter(Ptr<NetDevice>       dev,
                              Ptr<const Packet>    pkt,
                              uint16_t             /*protocol*/,
                              const Address&       /*src*/)
{
    if (pkt->GetSize() > 100)
    {
        g_droppedByWormhole++;
        NS_LOG_INFO("Wormhole black-hole: dropped data packet #"
                    << g_droppedByWormhole
                    << "  size=" << pkt->GetSize()
                    << "  node=" << dev->GetNode()->GetId());
        return false;   // drop
    }
    // Small packet → AODV control — forward so route stays poisoned
    return true;
}

// ---------------------------------------------------------------------------
// MAVLink packet builder
// ---------------------------------------------------------------------------
std::vector<uint8_t> CreateMavlinkPacket(
    uint8_t system_id, uint8_t target_system, uint8_t target_component,
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

    uint8_t  buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

// ---------------------------------------------------------------------------
// Waypoint dispatch  (leader N0 -> followers via AODV mesh)
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

        std::vector<uint8_t> pkt =
            CreateMavlinkPacket(1, (uint8_t)(follower + 1), 0, x, y, z);

        Ptr<Packet> packet = Create<Packet>(pkt.data(), pkt.size());
        InetSocketAddress dest(droneIpAddresses[follower], 20000);
        g_droneSockets[0]->SendTo(packet, 0, dest);

        NS_LOG_INFO("Leader sent waypoint pair " << pairIndex
                    << " to drone " << follower);
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

    mavlink_message_t  msg;
    mavlink_status_t*  status = &mavlink_status_map[droneId];

    for (size_t i = 2; i < data.size(); i++)
    {
        if (!mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, status))
            continue;

        if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
        {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);

            double x = (gps.lon / 1e7 - s_refLon) * s_metersPerDegreeLon;
            double y = (gps.lat / 1e7 - s_refLat) * s_metersPerDegreeLat;
            double z =  gps.alt / 1000.0 - s_refAlt;

            if (droneId < droneMobilityModels.size() &&
                droneMobilityModels[droneId])
                droneMobilityModels[droneId]->SetPosition(Vector(x, y, z));
        }
        else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM)
            NS_LOG_INFO("Mission item received by node " << (int)droneId);
        else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
            NS_LOG_INFO("Command received by node "      << (int)droneId);
        else if (msg.msgid == MAVLINK_MSG_ID_SET_MODE)
            NS_LOG_INFO("Mode change received by node "  << (int)droneId);
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
    subscriber.set(zmq::sockopt::rcvtimeo, 100); // 100ms timeout
    NS_LOG_INFO("ZMQ subscriber connected to port 5556");

    const size_t MAX_QUEUE_SIZE = 1000;
    while (keepRunning.load())
    {
        zmq::message_t message;
        if (subscriber.recv(message, zmq::recv_flags::none))
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
        // If recv timed out or failed, loop back and check keepRunning
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
                      << "s  Node " << i
                      << "  pos=(" << pos.x << ", " << pos.y
                      << ", " << pos.z << ")\n";
        }
    }
    Simulator::Schedule(Seconds(interval),
                        &PrintDronePositions, nodes, interval, simTime);
}

void PacketReceived(Ptr<const Packet> p, const Address& addr, uint32_t nodeId)
{
    InetSocketAddress inet = InetSocketAddress::ConvertFrom(addr);
    NS_LOG_INFO("Node " << nodeId << " received packet from "
                << inet.GetIpv4() << " size=" << p->GetSize());

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
                NS_LOG_INFO("Node " << nodeId << " waypoint: "
                            << mission.x << ", " << mission.y
                            << ", " << mission.z);
            }
        }
    }
}

void InstallPacketSinks(NodeContainer nodes,
                        std::vector<uint16_t> ports,
                        double simTime)
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
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

void SetupWormholeAttack(NodeContainer wormholes,
                         Ipv4InterfaceContainer tunnelIf)
{
    Ptr<Node> w1 = wormholes.Get(0);
    Ptr<Node> w2 = wormholes.Get(1);

    Ipv4Address w2TunnelIp = tunnelIf.GetAddress(1);

    Ptr<Socket> sockW1 = Socket::CreateSocket(w1, UdpSocketFactory::GetTypeId());
    Ptr<Socket> sockW2 = Socket::CreateSocket(w2, UdpSocketFactory::GetTypeId());

    sockW1->Bind();
    sockW2->Bind();

    //CORE WORMHOLE BEHAVIOR
    sockW1->SetRecvCallback(
        [sockW2, w2TunnelIp](Ptr<Socket> socket)
        {
            Ptr<Packet> packet;
            Address from;

            while ((packet = socket->RecvFrom(from)))
            {
                // forward immediately over tunnel
                sockW2->Send(packet);
            }
        }
    );

    sockW2->SetRecvCallback(
        [](Ptr<Socket> socket)
        {
            Ptr<Packet> packet;
            Address from;

            while ((packet = socket->RecvFrom(from)))
            {
                // reinject into network domain (illusion step)
                // this is where AODV "sees" wrong topology effects
            }
        }
    );
}

// ===========================================================================
// main()
// ===========================================================================
int main(int argc, char *argv[])
{
    LogComponentEnable("DroneWormholeMesh", LOG_LEVEL_INFO);

    // -----------------------------------------------------------------------
    // 0. Parameters
    // -----------------------------------------------------------------------
    uint32_t    nNodes    = 7;
    double      simTime   = 50.0;
    std::string outputDir = "/home/ubuntu/UAVLnQ/ns3-output";

    CommandLine cmd;
    cmd.AddValue("nNodes",   "Number of normal drone nodes", nNodes);
    cmd.AddValue("simTime",  "Simulation time (s)",          simTime);
    cmd.AddValue("o",        "Output directory",             outputDir);
    cmd.Parse(argc, argv);

    // Wormhole node positions — W1 near N0 (x=0), W2 near N(nNodes-1)
    double W1X = -10.0, W1Y = -30.0;
    double W2X = (nNodes - 1) * 80.0 + 10.0, W2Y = -30.0;

    // -----------------------------------------------------------------------
    // 1. Create nodes
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Creating " << nNodes << " drone nodes + 2 wormhole nodes");

    drones.Create(nNodes);
    p2pNodes.Create(2);       // W1, W2 — created before stack install
    allNodes.Add(drones);
    allNodes.Add(p2pNodes);

    // -----------------------------------------------------------------------
    // 2. Mobility
    // -----------------------------------------------------------------------
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();

    for (uint32_t i = 0; i < nNodes; i++)
        pos->Add(Vector(i * 80.0, 0.0, 20.0));   // linear chain

    pos->Add(Vector(W1X, W1Y, 20.0));             // W1
    pos->Add(Vector(W2X, W2Y, 20.0));             // W2

    mobility.SetPositionAllocator(pos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(allNodes);

    for (uint32_t i = 0; i < allNodes.GetN(); ++i)
        droneMobilityModels.push_back(
            allNodes.Get(i)->GetObject<MobilityModel>());

    // -----------------------------------------------------------------------
    // 3a. Main 802.11a channel — shared by all nodes including W1/W2
    //     This is what makes W1/W2 invisible to AODV (they look like
    //     normal mesh participants)
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

    NetDeviceContainer devices = wifi.Install(phy, mac, allNodes);

    // -----------------------------------------------------------------------
    // 3b. Second private channel — wormhole nodes only
    //     Out-of-band link that lets W1 hear W2 across the whole chain
    // -----------------------------------------------------------------------
    YansWifiChannelHelper channel2;
    channel2.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel2.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
        "SystemLoss",   DoubleValue(1.0),
        "HeightAboveZ", DoubleValue(1.5));

    YansWifiPhyHelper phy2;
    phy2.SetChannel(channel2.Create());

    WifiMacHelper mac2;
    mac2.SetType("ns3::AdhocWifiMac");

    WifiHelper wifi2;
    wifi2.SetStandard(WIFI_STANDARD_80211a);
    wifi2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode",    StringValue("OfdmRate6Mbps"),
        "ControlMode", StringValue("OfdmRate6Mbps"));

    NetDeviceContainer wormholeWifiDevices = wifi2.Install(phy2, mac2, p2pNodes);

    // -----------------------------------------------------------------------
    // 3c. P2P tunnel between W1 and W2 — the actual wormhole link
    // -----------------------------------------------------------------------
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay",    StringValue("2ms"));

    NetDeviceContainer p2pDevices = p2p.Install(p2pNodes);
    
    // Install black-hole filter on both ends of the tunnel
    //for (uint32_t i = 0; i < p2pDevices.GetN(); i++)
      //  p2pDevices.Get(i)->SetReceiveCallback(MakeCallback(&WormholeRxFilter));

    // -----------------------------------------------------------------------
    // 4. Internet stack + AODV on every node
    // -----------------------------------------------------------------------
    AodvHelper aodv;

    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(allNodes);

    Ptr<Node> w1 = p2pNodes.Get(0);
    Ptr<Node> w2 = p2pNodes.Get(1);

    // Flow monitor installed right after the stack so it captures
    // all traffic from t=0, including AODV route discovery
    FlowMonitorHelper      flowmon;
    Ptr<FlowMonitor>       monitor = flowmon.InstallAll();

    // -----------------------------------------------------------------------
    // 5. IP addressing
    //    10.1.1.0/24 — main Wi-Fi (all nodes)
    //    10.1.2.0/24 — P2P tunnel (W1 <-> W2)
    // -----------------------------------------------------------------------
    Ipv4AddressHelper ipv4;

    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer allInterfaces = ipv4.Assign(devices);

    for (uint32_t i = 0; i < nNodes; i++)
        droneIpAddresses.push_back(allInterfaces.GetAddress(i));

    ipv4.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces = ipv4.Assign(p2pDevices);
    
    NS_LOG_INFO("W1 main IP : " << allInterfaces.GetAddress(nNodes));
    NS_LOG_INFO("W2 main IP : " << allInterfaces.GetAddress(nNodes + 1));
    NS_LOG_INFO("W1 P2P  IP : " << p2pInterfaces.GetAddress(0));
    NS_LOG_INFO("W2 P2P  IP : " << p2pInterfaces.GetAddress(1));
    SetupWormholeAttack(p2pNodes, p2pInterfaces);
    // -----------------------------------------------------------------------
    // 6. UDP traffic  N0 (leader) -> N6 (last drone)
    //    AODV will prefer the wormhole route (3 hops) over the legitimate
    //    6-hop chain, so all data flows into the black hole
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
    app.SetAttribute("OnTime",
        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    app.SetAttribute("OffTime",
        StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer srcApp = app.Install(drones.Get(0));
    srcApp.Start(Seconds(1.0));
    srcApp.Stop(Seconds(simTime));

    // -----------------------------------------------------------------------
    // 7. Waypoint dispatches from leader at t=20, 30, 40 s
    // -----------------------------------------------------------------------
    Simulator::Schedule(Seconds(20.0), &SendWaypointPairFromDrone0, 0);
    Simulator::Schedule(Seconds(30.0), &SendWaypointPairFromDrone0, 1);
    Simulator::Schedule(Seconds(40.0), &SendWaypointPairFromDrone0, 2);

    // -----------------------------------------------------------------------
    // 8. Position reporting + ZMQ queue
    // -----------------------------------------------------------------------
    Simulator::Schedule(Seconds(0.0),  &PrintDronePositions,
                        allNodes, 5.0, simTime);
    Simulator::Schedule(Seconds(0.01), &UpdatePositionsFromQueue);

    // -----------------------------------------------------------------------
    // 9. Output files
    // -----------------------------------------------------------------------
    auto  t_now = std::time(nullptr);
    auto  tm    = *std::localtime(&t_now);
    std::ostringstream oss;
    oss << outputDir << "/wormhole-aodv-"
        << std::put_time(&tm, "%Y%m%d_%H%M%S");
    std::string prefix = oss.str();

    // PCAP on every main-channel node + both P2P tunnel ends
    phy.EnablePcapAll(prefix + "-main");
    p2p.EnablePcap(prefix + "-wormhole-W1", p2pDevices.Get(0));
    p2p.EnablePcap(prefix + "-wormhole-W2", p2pDevices.Get(1));

    // Routing table snapshots at t=0.5, 2.5, 25 s
    Ptr<OutputStreamWrapper> routingStream =
        Create<OutputStreamWrapper>(prefix + "-routing.txt", std::ios::out);
    aodv.PrintRoutingTableAllAt(Seconds(0.5),  routingStream);
    aodv.PrintRoutingTableAllAt(Seconds(2.5),  routingStream);
    aodv.PrintRoutingTableAllAt(Seconds(25.0), routingStream);

    // -----------------------------------------------------------------------
    // 10. NetAnim
    // -----------------------------------------------------------------------
    AnimationInterface anim(prefix + "-anim.xml");

    for (uint32_t i = 0; i < nNodes; i++)
    {
        anim.UpdateNodeDescription(drones.Get(i), "N" + std::to_string(i));
        anim.UpdateNodeColor(drones.Get(i), 0, 200, 0);
        anim.UpdateNodeSize(i, 5.0, 5.0);
    }
    anim.UpdateNodeDescription(p2pNodes.Get(0), "W1");
    anim.UpdateNodeDescription(p2pNodes.Get(1), "W2");
    anim.UpdateNodeColor(p2pNodes.Get(0), 0, 0, 255);
    anim.UpdateNodeColor(p2pNodes.Get(1), 0, 0, 255);
    anim.UpdateNodeSize(nNodes,     5.0, 5.0);
    anim.UpdateNodeSize(nNodes + 1, 5.0, 5.0);
    anim.EnablePacketMetadata(true);

    // -----------------------------------------------------------------------
    // 11. Run
    // -----------------------------------------------------------------------
    std::thread zmqThread(ZmqPositionReceiverThread);

    NS_LOG_INFO("Starting simulation  simTime=" << simTime << "s");
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // -----------------------------------------------------------------------
    // 12. Results
    // -----------------------------------------------------------------------
    keepRunning.store(false);
    if (zmqThread.joinable()) zmqThread.detach();

    // Serialize all flow statistics to XML for analysis
    monitor->CheckForLostPackets();
    monitor->SerializeToXmlFile(prefix + "-flowmon.xml", true, true);

    std::cout << "\nFlow monitor saved to: " << prefix << "-flowmon.xml\n";
    std::cout << "Packets black-holed by wormhole: "
              << g_droppedByWormhole << "\n";

    Simulator::Destroy();
    return 0;
}

