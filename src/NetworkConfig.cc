#include "NetworkConfig.h"
#include <iostream>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <cstring>

namespace ORB_SLAM2 {

NetworkConfig::RobotConfig NetworkConfig::GetRobotConfig(int robot_id, 
                                                         const std::string& server_ip, 
                                                         int base_port) {
    RobotConfig config;
    
    // Auto-assign robot_id if not provided
    if (robot_id == -1) {
        const char* env_robot_id = std::getenv("EDGE_SLAM_ROBOT_ID");
        if (env_robot_id) {
            robot_id = std::stoi(env_robot_id);
        } else {
            robot_id = 1; // Default to robot 1
            std::cout << "NetworkConfig: Using default robot_id = 1. Set EDGE_SLAM_ROBOT_ID environment variable to override." << std::endl;
        }
    }
    
    // Use environment variables if available
    const char* env_server_ip = std::getenv("EDGE_SLAM_SERVER_IP");
    const char* env_base_port = std::getenv("EDGE_SLAM_BASE_PORT");
    
    std::string final_server_ip = server_ip;
    int final_base_port = base_port;
    
    if (env_server_ip && strlen(env_server_ip) > 0) {
        final_server_ip = std::string(env_server_ip);
    }
    
    if (env_base_port) {
        final_base_port = std::stoi(env_base_port);
    }
    
    config.robot_id = robot_id;
    config.client_ip = GetLocalIP();
    config.server_ip = final_server_ip;
    
    // Server ports: Fixed ports for all robots to connect to
    config.keyframe_server_port = final_base_port + PORT_OFFSET_KEYFRAME;  // 15000
    config.frame_server_port = final_base_port + PORT_OFFSET_FRAME;        // 15001  
    config.map_server_port = final_base_port + PORT_OFFSET_MAP;            // 15002
    
    // Client ports: Robot-specific ports to avoid conflicts
    int client_port_base = final_base_port + 10 + (robot_id * PORTS_PER_ROBOT);
    config.keyframe_client_port = client_port_base + PORT_OFFSET_KEYFRAME;  // 15013, 15016, 15019...
    config.frame_client_port = client_port_base + PORT_OFFSET_FRAME;        // 15014, 15017, 15020...
    config.map_client_port = client_port_base + PORT_OFFSET_MAP;            // 15015, 15018, 15021...
    
    std::cout << "NetworkConfig for Robot " << robot_id << ":" << std::endl;
    std::cout << "  Client IP: " << config.client_ip << std::endl;
    std::cout << "  Server IP: " << config.server_ip << std::endl;
    std::cout << "  Client Ports: " << config.keyframe_client_port << ", " << config.frame_client_port << ", " << config.map_client_port << std::endl;
    std::cout << "  Server Ports: " << config.keyframe_server_port << ", " << config.frame_server_port << ", " << config.map_server_port << std::endl;
    
    return config;
}

// Cache the config to avoid repeated initialization and logging
NetworkConfig::RobotConfig NetworkConfig::GetCachedRobotConfig() {
    static RobotConfig cached_config;
    static bool initialized = false;
    
    if (!initialized) {
        cached_config = GetRobotConfig();
        initialized = true;
    }
    
    return cached_config;
}

std::string NetworkConfig::GetLocalIP() {
    // For Isaac Sim testing, default to localhost
    const char* env_client_ip = std::getenv("EDGE_SLAM_CLIENT_IP");
    if (env_client_ip && strlen(env_client_ip) > 0) {
        return std::string(env_client_ip);
    }
    
    return "127.0.0.1";
}

}