// attacks.h
#pragma once
#include <vector>
#include <cstdint>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"

#include <zmq.hpp>

extern std::vector<ns3::Ipv4Address> droneIpAddresses;
extern zmq::socket_t* g_commandPublisher;
extern ns3::NodeContainer Attacker;
extern double s_refLat;
extern double s_refLon;
extern double s_refAlt;
extern double s_metersPerDegreeLat;
extern double s_metersPerDegreeLon;

std::vector<uint8_t> CreateMavlinkMissionPacket(uint8_t target_system, uint8_t target_component,float lat, float lon, float alt);
std::vector<uint8_t> CreateFakeGpsPacket(uint8_t droneId, double lat, double lon, double alt);
std::vector<uint8_t> CreateChangeSpeedPacket(uint8_t target_system, float speedType, float speed);
std::vector<uint8_t> CreateForcedReturnHomePacket(uint8_t target_system);
std::vector<uint8_t> CreateForcedDisarmPacket(uint8_t target_system);
std::vector<uint8_t> CreateFlightTerminationPacket(uint8_t target_system);
std::vector<uint8_t> CreateSetHomePositionPacket(uint8_t target_system, double lat, double lon, double alt);
std::vector<uint8_t> CreateHeartbeatPacket(uint8_t system_id);


void ExecuteFlightTerminationAttack(ns3::Ptr<ns3::Socket> socket);
void ExecuteForceDisarmAttack(ns3::Ptr<ns3::Socket> socket);
void ExecuteForceRTLAttack(ns3::Ptr<ns3::Socket> socket);
void ExecuteGpsSpoofingAttack(ns3::Ptr<ns3::Socket> socket);
void ExecuteSpoofedDroneFloodAttack(ns3::Ptr<ns3::Socket> socket);
void ExecuteSetHomeAttack(ns3::Ptr<ns3::Socket> socket);
void ExecuteHeartbeatFloodAttack(ns3::Ptr<ns3::Socket> socket);
void SendWaypointPairFromAttacker(int pairIndex);
