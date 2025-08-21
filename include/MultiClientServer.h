#ifndef MULTI_CLIENT_SERVER_H
#define MULTI_CLIENT_SERVER_H

#include "TcpSocket.h"
#include "blockingconcurrentqueue.h"
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>

namespace ORB_SLAM2 {

struct ClientConnection {
    int client_id;
    int robot_id;
    TcpSocket* socket;
    std::thread* handler_thread;
    bool is_active;
    
    ClientConnection(int cid, TcpSocket* sock) 
        : client_id(cid), robot_id(-1), socket(sock), handler_thread(nullptr), is_active(true) {}
};

class MultiClientServer {
public:
    MultiClientServer(const std::string& server_ip, int port, const std::string& channel_name);
    ~MultiClientServer();
    
    // Start accepting new connections
    void StartServer();
    void StopServer();
    
    // Data queues for communication
    void SetReceiveQueue(moodycamel::BlockingConcurrentQueue<std::string>* queue);
    void SetSendQueue(moodycamel::BlockingConcurrentQueue<std::string>* queue);
    
    // Client management
    int GetConnectedClientCount() const;
    std::vector<int> GetConnectedRobotIds() const;
    
private:
    // Server socket for accepting connections
    TcpSocket* server_socket;
    std::string server_ip;
    int server_port;
    std::string channel_name;
    
    // Client connections
    std::vector<ClientConnection*> clients;
    mutable std::mutex clients_mutex;
    int next_client_id;
    
    // Server control
    std::atomic<bool> server_running;
    std::thread* accept_thread;
    
    // Data queues
    moodycamel::BlockingConcurrentQueue<std::string>* receive_queue;
    moodycamel::BlockingConcurrentQueue<std::string>* send_queue;
    std::thread* send_thread;
    
    // Internal methods
    void AcceptConnections();
    void HandleClientReceive(ClientConnection* client);
    void HandleSendToClients();
    void RemoveClient(int client_id);
    int ExtractRobotIdFromData(const std::string& data);
    std::string AddRobotIdToData(const std::string& data, int robot_id);
};

}

#endif // MULTI_CLIENT_SERVER_H