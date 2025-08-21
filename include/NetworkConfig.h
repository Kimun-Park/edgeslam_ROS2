#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H

#include <string>

namespace ORB_SLAM2 {

class NetworkConfig {
public:
    struct RobotConfig {
        int robot_id;
        std::string client_ip;
        std::string server_ip;
        int keyframe_client_port;
        int keyframe_server_port;
        int frame_client_port;
        int frame_server_port;
        int map_client_port;
        int map_server_port;
    };

    static RobotConfig GetRobotConfig(int robot_id = -1, 
                                    const std::string& server_ip = "127.0.0.1", 
                                    int base_port = 15000);
    
    // Cache the config to avoid repeated initialization
    static RobotConfig GetCachedRobotConfig();
    
    static std::string GetLocalIP();
    
private:
    static constexpr int PORT_OFFSET_KEYFRAME = 0;
    static constexpr int PORT_OFFSET_FRAME = 1;
    static constexpr int PORT_OFFSET_MAP = 2;
    static constexpr int PORTS_PER_ROBOT = 3;
    static constexpr int DEFAULT_BASE_PORT = 15000;
};

}

#endif // NETWORK_CONFIG_H