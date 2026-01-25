
#include "common/UdpClient.h"
#include <cstring> // for memset

UdpClient::UdpClient(const std::string& ip, int port)
    : mIP(ip), mPort(port) 
{
#ifdef _WIN32
    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock.");
    }
#endif

    // Create the socket
    mSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (mSocket < 0) {
        throw std::runtime_error("Failed to create socket.");
    }

    // Configure the server address
    memset(&mServerAddress, 0, sizeof(mServerAddress));
    mServerAddress.sin_family = AF_INET;
    mServerAddress.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &mServerAddress.sin_addr) <= 0) {
        throw std::invalid_argument("Invalid IP address.");
    }
}

UdpClient::~UdpClient() 
{
    // Close the socket
#ifdef _WIN32
    closesocket(mSocket);
    WSACleanup();
#else
    close(mSocket);
#endif
}

bool UdpClient::send(const std::string& message) 
{
    if (sendto(mSocket, message.c_str(), message.size(), 0,
               reinterpret_cast<struct sockaddr*>(&mServerAddress),
               sizeof(mServerAddress)) < 0) 
    {
        return false; 
    }

    return true; 
}
