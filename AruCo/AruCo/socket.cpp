// TestSocketClient.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")
using SOCKET_T = SOCKET;
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

using SOCKET_T = int;
const int INVALID_SOCKET = -1;
const int SOCKET_ERROR = -1;
#endif

bool sendMessage(const char* message) {
    SOCKET_T sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Socket creation failed." << std::endl;
        return false;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(1755); // Port number Unity listens on

#ifdef _WIN32
    InetPton(AF_INET, L"10.0.0.130", &serverAddr.sin_addr);
#else
    inet_pton(AF_INET, "10.0.0.130", &serverAddr.sin_addr);
#endif

    if (connect(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Connection failed." << std::endl;
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return false;
    }

    int sendResult = send(sock, message, (int)strlen(message), 0);
    if (sendResult == SOCKET_ERROR) {
        std::cerr << "Send failed." << std::endl;
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return false;
    }

    std::cout << "Message sent: " << message << std::endl;

#ifdef _WIN32
    closesocket(sock);
#else
    close(sock);
#endif

    return true;
}

int main()
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed." << std::endl;
        return 1;
    }
#endif

    const char* message = "Hello, Unity!";

    while (true) {
        if (!sendMessage(message)) {
            std::cerr << "Failed to send message." << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}