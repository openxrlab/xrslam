#ifndef __CLIENT_H__
#define __CLIENT_H__

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <tuple>  
#include <fstream>  
#include <thread>
#include <xrslam/xrslam.h>
#include <xrslam/utility/message.h>

namespace xrslam{

#define TEXT_COLOR_RESET   "\033[0"  
#define TEXT_COLOR_RED     "\033[0;31"  
#define TEXT_COLOR_GREEN   "\033[0;32"  
#define TEXT_COLOR_YELLOW  "\033[0;33"  
#define TEXT_COLOR_BLUE    "\033[0;34"  
#define TEXT_COLOR_MAGENTA "\033[0;35"  
#define TEXT_COLOR_CYAN    "\033[0;36"


inline void VIZ_LOGGER(const std::string& text, const std::string& colorCode) {  
    std::cout << "\033[" << colorCode << "m" << text << "\033[0m" << std::endl;  
} 


class SocketBase{

public:
    std::string name;
    int _sock;

    // client
    struct sockaddr_in _server_addr;

    // server
    int _server_fd;
    int _addrlen;
    struct sockaddr_in _address;  
    std::string _port;

    std::ofstream logfile;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    
    SocketBase(){}

    void writeToLog(const std::string& action, size_t dataSize) { 
        auto now = std::chrono::high_resolution_clock::now();  
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();  
        logfile << elapsed_time << " - " << action << " " << dataSize << " bytes\n";  
    }  

    std::tuple<DataType, std::string> readData(){
        
        char type;
        uint32_t messageLength = 0;  
        std::vector<char> buffer;
        try{
            
            if (!readNBytes(&type, 1)) {  
                perror("read type error");  
                exit(EXIT_FAILURE);
            }

            
            if (!readNBytes(&messageLength, sizeof(messageLength))) {  
                perror("read length error");  
                exit(EXIT_FAILURE);
            }
            messageLength = ntohl(messageLength); // 网络字节序转换为主机字节序  

            buffer.reserve(messageLength);    
            if (!readNBytes(buffer.data(), messageLength)) {  
                perror("read data error");  
                exit(EXIT_FAILURE);  
            } 
        }
        catch(const std::exception& e)
        {
            VIZ_LOGGER("[BACKEND ERROR]: package lost!", TEXT_COLOR_RED);
        }   

        if(messageLength == 0){
            return std::make_tuple(DT_UNKNOWN, std::string(""));
        }

        std::string message;
        message.assign(buffer.data(), messageLength);
        
        writeToLog("recv", messageLength);
        
        return std::make_tuple(static_cast<DataType>(type), message);  
    }

    bool readNBytes(void* buffer, size_t size) {  
        size_t bytesRead = 0;  
        while (bytesRead < size) {  
            int result = recv(_sock, static_cast<char*>(buffer) + bytesRead, size - bytesRead, 0);  
            if (result < 0) {  
                return false; // 读取失败  
            } else if (result == 0) {  
                return false; // 连接已关闭  
            }  
            bytesRead += result;  
        }  
        return true;  
    }

    void sendData(std::string buffer, DataType type= DataType::DT_FRAME){
        writeToLog("send", buffer.length());
        char header = static_cast<char>(type); 
        buffer = header + addLengthPrefix(buffer);
        send(_sock, buffer.c_str(), buffer.length(), 0);
    }
    
    std::string addLengthPrefix(std::string message) {
        size_t messageLength = message.length();
    
        std::string lengthPrefix;
        lengthPrefix.push_back((messageLength >> 24) & 0xFF);
        lengthPrefix.push_back((messageLength >> 16) & 0xFF);
        lengthPrefix.push_back((messageLength >> 8) & 0xFF);
        lengthPrefix.push_back(messageLength & 0xFF);
        
        return lengthPrefix + message;
    }

};

class VizServer: public SocketBase{

public:
    VizServer(std::string port="8080"){
        _port = port;
        _addrlen = sizeof(_address);
        int opt = 1; 
        if ((_server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {  
            perror("socket failed");  
            exit(EXIT_FAILURE);  
        }  

        if (setsockopt(_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {  
            perror("setsockopt");  
            exit(EXIT_FAILURE);  
        }  
    
        _address.sin_family = AF_INET;  
        _address.sin_addr.s_addr = INADDR_ANY;  
        _address.sin_port = htons(std::stoi(port));  
    
        if (bind(_server_fd, (struct sockaddr *)&_address, sizeof(_address))<0) {  
            perror("bind failed");  
            exit(EXIT_FAILURE);  
        }  

        if (listen(_server_fd, 3) < 0) {  
            perror("listen");  
            exit(EXIT_FAILURE);  
        }  

        if ((_sock = accept(_server_fd, (struct sockaddr *)&_address, (socklen_t*)&_addrlen))<0) {  
            perror("accept");  
            exit(EXIT_FAILURE);  
        }  
    }

    void getClientInfo(){
        std::string ip = inet_ntoa(_address.sin_addr);
        std::string port = std::to_string(ntohs(_address.sin_port));
        VIZ_LOGGER("[Server INFO]: server listening on port " +  _port, TEXT_COLOR_YELLOW);
        VIZ_LOGGER("[Server INFO]: connection accepted from " + ip + ":" + port, TEXT_COLOR_YELLOW);
    }
};


class VizClient: public SocketBase{

public:
    VizClient(std::string name, std::string ip, std::string port) {  

        this->name = name;
        // logfile.open("network_log_" + name + ".txt", std::ios_base::out); 
        startTime = std::chrono::high_resolution_clock::now();  

        bool connected = false;  

        while (!connected) {  
            if ((_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {    
                VIZ_LOGGER("[Client ERROR]: socket creation error", TEXT_COLOR_RED);
                exit(EXIT_FAILURE);    
            }    
    
            _server_addr.sin_family = AF_INET;    
            _server_addr.sin_port = htons(std::stoi(port));    
        
            if (inet_pton(AF_INET, ip.c_str(), &_server_addr.sin_addr) <= 0) {
                VIZ_LOGGER("[Client ERROR]: invalid address/ address not supported", TEXT_COLOR_RED);
                exit(EXIT_FAILURE);    
            }    
        
            if (connect(_sock, (struct sockaddr *)&_server_addr, sizeof(_server_addr)) < 0) {    
                VIZ_LOGGER("[Client ERROR]: connection failed. retrying in 1 second...", TEXT_COLOR_RED);
                std::this_thread::sleep_for(std::chrono::seconds(1));  
                close(_sock);
            } else {  
                connected = true; 
            }  
        }
        VIZ_LOGGER("[Client INFO]: connection success!", TEXT_COLOR_GREEN);
    }  

    std::shared_ptr<Notifier> notifier;
    
    std::mutex transform_mutex;
    bool registered = false;
    matrix<4> T_wmatch_wquery = matrix<4>::Identity();

    matrix<4> get_transform(){
        std::unique_lock<std::mutex> lock(transform_mutex);
        return T_wmatch_wquery;
    }
    void set_transform(const matrix<4> &T_wmatch_wquery){
        std::unique_lock<std::mutex> lock(transform_mutex);
        this->T_wmatch_wquery = T_wmatch_wquery;
    }

    void set_registered(bool registered){
        std::unique_lock<std::mutex> lock(transform_mutex);
        this->registered = registered;
    }
    bool check_registered(){
        std::unique_lock<std::mutex> lock(transform_mutex);
        return registered;
    }

    std::mutex local_map_mutex;
    LocalMapMsg local_map;
    bool local_map_available = false;
    
    void update_local_map(LocalMapMsg msg){
        std::unique_lock<std::mutex> lk(local_map_mutex);
        local_map_available = true;
        local_map = msg;
    }

    LocalMapMsg get_local_map(){
        std::unique_lock<std::mutex> lk(local_map_mutex);
        return local_map;
    }

    bool check_local_map_available(){
        std::unique_lock<std::mutex> lk(local_map_mutex);
        return local_map_available;
    }

    std::mutex object_mutex;
    std::queue<Pose> object_list;
    
    void add_object_pose(Pose pose){
        std::unique_lock<std::mutex> lk(object_mutex);
        object_list.push(pose);
    }
    
    void send_object_pose(Pose pose){
        ObjectMsg msg;
        msg.pose = pose;
        msg.name = name;
        this->sendData(msg.serialized(), DT_OBJ);
    }
    Pose get_latest_object(){
        std::unique_lock<std::mutex> lk(object_mutex);
        Pose pose = object_list.front();
        object_list.pop();

        matrix<4> transform = this->get_transform();

        std::cout << "transform: " << transform << std::endl;
        std::cout << "pose: " << pose.to_matrix() << std::endl;
        pose = Pose::from_matrix(transform.inverse() * pose.to_matrix());
        return pose;
    }

    bool is_object_exist(){
        std::unique_lock<std::mutex> lk(object_mutex);
        return !object_list.empty();
    }

    void listen(){
        while(true){
            usleep(1000000);
            // auto buffer = this->readData(); 
            // DataType type = std::get<0>(buffer);
            // std::string data = std::get<1>(buffer);
            // if(type == DT_NOTIFY){
            //     NotifyMsg msg = NotifyMsg::deserialized(data);
            //     if(notifier){
            //         notifier->update_status(msg);
            //         notifier->cv.notify_one();
            //     }
            // }else if(type == DT_MESSAGE){
            //     NotifyMsg msg = NotifyMsg::deserialized(data);
            //     std::cout << msg.deserialized(data).message << std::endl;
            // }else if(type == DT_ALIGN){
            //     AlignMsg msg = AlignMsg::deserialized(data);
            //     this->set_transform(msg.T_wmatch_wquery);
            //     this->set_registered(true);
            // }else if(type == DT_SWT_MAP){
            //     LocalMapMsg msg = LocalMapMsg::deserialized(data);
            //     this->update_local_map(msg);
            // }
            // else if(type == DT_OBJ){
            //     ObjectMsg msg = ObjectMsg::deserialized(data);
            //     VIZ_LOGGER("[CLIENT INFO]: get an object pose", TEXT_COLOR_BLUE);
            //     std::cout << msg.pose.p.transpose() << std::endl;
            //     this->add_object_pose(msg.pose);
            // }
            // else{
            //     std::cout << "unknow type" << std::endl;
            // }
        }
    }
};
}
#endif
