#include "MultiClientServer.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

namespace ORB_SLAM2 {

MultiClientServer::MultiClientServer(const std::string& server_ip, int port, const std::string& channel_name)
    : server_ip(server_ip), server_port(port), channel_name(channel_name),
      next_client_id(1), server_running(false), accept_thread(nullptr),
      receive_queue(nullptr), send_queue(nullptr), send_thread(nullptr) {
    
    server_socket = new TcpSocket(server_ip, port);
    std::cout << "MultiClientServer [" << channel_name << "] created on port " << port << std::endl;
}

MultiClientServer::~MultiClientServer() {
    StopServer();
    
    // Clean up client connections
    std::lock_guard<std::mutex> lock(clients_mutex);
    for (auto* client : clients) {
        if (client->handler_thread && client->handler_thread->joinable()) {
            client->handler_thread->join();
        }
        delete client->handler_thread;
        delete client->socket;
        delete client;
    }
    clients.clear();
    
    delete server_socket;
}

void MultiClientServer::StartServer() {
    server_running = true;
    
    // Start accepting connections
    accept_thread = new std::thread(&MultiClientServer::AcceptConnections, this);
    
    // Start send thread if send_queue is set
    if (send_queue) {
        send_thread = new std::thread(&MultiClientServer::HandleSendToClients, this);
    }
    
    std::cout << "MultiClientServer [" << channel_name << "] started on port " << server_port << std::endl;
}

void MultiClientServer::StopServer() {
    server_running = false;
    
    if (accept_thread && accept_thread->joinable()) {
        accept_thread->join();
        delete accept_thread;
        accept_thread = nullptr;
    }
    
    if (send_thread && send_thread->joinable()) {
        send_thread->join();
        delete send_thread;
        send_thread = nullptr;
    }
}

void MultiClientServer::SetReceiveQueue(moodycamel::BlockingConcurrentQueue<std::string>* queue) {
    receive_queue = queue;
}

void MultiClientServer::SetSendQueue(moodycamel::BlockingConcurrentQueue<std::string>* queue) {
    send_queue = queue;
}

void MultiClientServer::AcceptConnections() {
    std::cout << "MultiClientServer [" << channel_name << "] listening on port " << server_port << "..." << std::endl;
    
    // Set up the server socket for listening (only once)
    if (listen(server_socket->getMySocket(), 5) == -1) {
        std::cerr << "MultiClientServer [" << channel_name << "] listen failed" << std::endl;
        return;
    }
    
    while (server_running) {
        try {
            // Accept new connections directly using accept()
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            std::cout << "MultiClientServer [" << channel_name << "] waiting for connection..." << std::endl;
            
            int client_fd = accept(server_socket->getMySocket(), (sockaddr*)&client_addr, &client_len);
            
            if (client_fd >= 0) {
                std::lock_guard<std::mutex> lock(clients_mutex);
                
                // Create a new TcpSocket wrapper for the client connection
                TcpSocket* client_socket = new TcpSocket();
                client_socket->setMySocket(client_fd);
                client_socket->setSocketHandle(client_fd);
                client_socket->setOtherSocket(client_fd);
                
                ClientConnection* new_client = new ClientConnection(next_client_id++, client_socket);
                clients.push_back(new_client);
                
                // Start handler thread for this client
                new_client->handler_thread = new std::thread(&MultiClientServer::HandleClientReceive, this, new_client);
                
                std::cout << "✓ Robot " << new_client->client_id << " connected to [" << channel_name << "] channel (port " << server_port << ")" << std::endl;
                
            } else {
                if (!server_running) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
        } catch (const std::exception& e) {
            std::cerr << "MultiClientServer [" << channel_name << "] accept error: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
}

void MultiClientServer::HandleClientReceive(ClientConnection* client) {
    while (server_running && client->is_active) {
        try {
            std::string received_data = client->socket->recieveMessage();
            
            if (!received_data.empty() && receive_queue) {
                // Extract robot_id from data if available
                if (client->robot_id == -1) {
                    client->robot_id = ExtractRobotIdFromData(received_data);
                    if (client->robot_id != -1) {
                        std::cout << "Client " << client->client_id << " identified as Robot " << client->robot_id << std::endl;
                    }
                }
                
                // Add client info to data and forward to processing queue
                std::string tagged_data = AddRobotIdToData(received_data, client->robot_id);
                receive_queue->enqueue(tagged_data);
            }
            
        } catch (const std::exception& e) {
            std::cerr << "MultiClientServer [" << channel_name << "] receive error from client " << client->client_id << ": " << e.what() << std::endl;
            break;
        }
    }
    
    // Mark client as inactive
    client->is_active = false;
    std::cout << "Client " << client->client_id << " disconnected from [" << channel_name << "]" << std::endl;
}

void MultiClientServer::HandleSendToClients() {
    std::string data_to_send;
    
    while (server_running) {
        if (send_queue->wait_dequeue_timed(data_to_send, std::chrono::milliseconds(100))) {
            std::lock_guard<std::mutex> lock(clients_mutex);
            
            // Extract target robot_id from data
            int target_robot_id = ExtractRobotIdFromData(data_to_send);
            
            for (auto* client : clients) {
                if (client->is_active && (target_robot_id == -1 || client->robot_id == target_robot_id)) {
                    try {
                        client->socket->sendMessage(const_cast<std::string&>(data_to_send));
                    } catch (const std::exception& e) {
                        std::cerr << "Send error to client " << client->client_id << ": " << e.what() << std::endl;
                    }
                }
            }
        }
    }
}

void MultiClientServer::RemoveClient(int client_id) {
    std::lock_guard<std::mutex> lock(clients_mutex);
    
    clients.erase(std::remove_if(clients.begin(), clients.end(),
        [client_id](ClientConnection* client) {
            if (client->client_id == client_id) {
                client->is_active = false;
                if (client->handler_thread && client->handler_thread->joinable()) {
                    client->handler_thread->join();
                }
                delete client->handler_thread;
                delete client->socket;
                delete client;
                return true;
            }
            return false;
        }), clients.end());
}

int MultiClientServer::ExtractRobotIdFromData(const std::string& data) {
    // Simple format: "ROBOT_ID:actual_data"
    size_t colon_pos = data.find(":");
    if (colon_pos != std::string::npos && colon_pos > 0) {
        try {
            std::string robot_id_str = data.substr(0, colon_pos);
            if (robot_id_str.substr(0, 8) == "ROBOT_ID") {
                return std::stoi(robot_id_str.substr(8));
            }
        } catch (...) {
            // Parsing failed, return -1
        }
    }
    return -1;
}

std::string MultiClientServer::AddRobotIdToData(const std::string& data, int robot_id) {
    if (robot_id != -1) {
        return "ROBOT_ID" + std::to_string(robot_id) + ":" + data;
    }
    return data;
}

int MultiClientServer::GetConnectedClientCount() const {
    std::lock_guard<std::mutex> lock(clients_mutex);
    return std::count_if(clients.begin(), clients.end(),
        [](const ClientConnection* client) { return client->is_active; });
}

std::vector<int> MultiClientServer::GetConnectedRobotIds() const {
    std::lock_guard<std::mutex> lock(clients_mutex);
    std::vector<int> robot_ids;
    
    for (const auto* client : clients) {
        if (client->is_active && client->robot_id != -1) {
            robot_ids.push_back(client->robot_id);
        }
    }
    
    return robot_ids;
}

}