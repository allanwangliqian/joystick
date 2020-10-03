#include <ped_includes.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream> 

#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#define MAX_AGENT 100
#define MAX_OBSTACLE 1000
#define PORT 2111

struct Agent
{
    std::string name;
    float x;
    float y;
    float th;
    float vx;
    float vy;
    float vth;
    float goal_x;
    float goal_y;
    float goal_th;
    float radius;
};

struct Robot
{
    float x;
    float y;
    float th;
    float vx;
    float vy;
    float vth;
    float goal_x;
    float goal_y;
    float goal_th;
    float radius;
};

struct Obstacle
{
    float x1;
    float x2;
    float y1;
    float y2;
};

struct Info
{
    int num_agents;
    int num_obstacles;
    struct Robot robot;
    struct Agent agents[MAX_AGENT];
    struct Obstacle obstacles[MAX_OBSTACLE];
    float delta_t;
};

struct Info information;

int establishConnection() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        std::cout << "socket failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        std::cout << "setsockopt failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );

    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        std::cout << "bind_failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        std::cout << "listen failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        std::cout << "accept failed" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    std::cout << "Waiting for signal..." << std::endl;

    return new_socket;
}

void parseInformation(std::string info_string) {
    std::istringstream f(info_string);
    std::string line;
    std::string element;
    std::vector<std::string> elem_list;
    int mode = 0;
    bool skip = false;
    information.num_obstacles = 0;
    information.num_agents = 0;
    while (std::getline(f, line)) {
        std::stringstream line_tmp(line);
        while (std::getline(line_tmp, element, ',')) {
            elem_list.push_back(element);
        }
        if (elem_list.size() == 1) {
            element = elem_list[0];
            if (element.compare("delta_t") == 0) {
                mode = 1;
                skip = true;
            }
            if (element.compare("obstacles") == 0) {
                mode = 2;
                skip = true;
            }
            if (element.compare("robot") == 0) {
                mode = 3;
                skip = true;
            }
            if (element.compare("agents") == 0) {
                mode = 4;
                skip = true;
            }
            if (element.compare("End") == 0) {
                mode = 0;
                skip = true;
            }
        }

        if (skip == false) {
            if (mode == 1) {
                element = elem_list[0];
                information.delta_t = std::stof(element);
            }
            if (mode == 2) {
                information.obstacles[information.num_obstacles].x1 = std::stof(elem_list[0]);
                information.obstacles[information.num_obstacles].y1 = std::stof(elem_list[1]);
                information.obstacles[information.num_obstacles].x2 = std::stof(elem_list[2]);
                information.obstacles[information.num_obstacles].y2 = std::stof(elem_list[3]);
                information.num_obstacles++;
            }
            if (mode == 3) {
                information.robot.x = std::stof(elem_list[0]);
                information.robot.y = std::stof(elem_list[1]);
                information.robot.th = std::stof(elem_list[2]);
                information.robot.vx = std::stof(elem_list[3]);
                information.robot.vy = std::stof(elem_list[4]);
                information.robot.vth = std::stof(elem_list[5]);
                information.robot.goal_x = std::stof(elem_list[6]);
                information.robot.goal_y = std::stof(elem_list[7]);
                information.robot.goal_th = std::stof(elem_list[8]);
                information.robot.radius = std::stof(elem_list[9]);
            }
            if (mode == 4) {
                information.agents[information.num_agents].name = elem_list[0];
                information.agents[information.num_agents].x = std::stof(elem_list[1]);
                information.agents[information.num_agents].y = std::stof(elem_list[2]);
                information.agents[information.num_agents].th = std::stof(elem_list[3]);
                information.agents[information.num_agents].vx = std::stof(elem_list[4]);
                information.agents[information.num_agents].vy = std::stof(elem_list[5]);
                information.agents[information.num_agents].vth = std::stof(elem_list[6]);
                information.agents[information.num_agents].goal_x = std::stof(elem_list[7]);
                information.agents[information.num_agents].goal_y = std::stof(elem_list[8]);
                information.agents[information.num_agents].goal_th = std::stof(elem_list[9]);
                information.agents[information.num_agents].radius = std::stof(elem_list[10]);
                information.num_agents++;
            }
        } else {
            skip = false;
        }
        
        elem_list.clear();
    }
    return;
}

int receiveInformation(int socket) {
    std::string info_str;
    char buffer[1024];
    int byte_count;
    do {
        byte_count = read(socket, buffer, 1023);
        buffer[byte_count] = '\0';
        info_str += buffer;
    } while (byte_count == 1023);
    //std::cout << info_str << std::endl;
    if (info_str.compare("OFF") == 0) {
        return 1;
    } else {
        parseInformation(info_str);
    }
    return 0;
    
}

void sendCommands(int socket) {
    const char* command_c = "Hello";
    send(socket, command_c, strlen(command_c), 0);
    return;
}

int main()
{
    int socket;
    int end_flag;
    socket = establishConnection();

    while (true) {
        while (true) {
            end_flag = receiveInformation(socket);
            if (end_flag == 1) {
                break;
            }

            sendCommands(socket);
        }
    }
            
    return 0;
}
