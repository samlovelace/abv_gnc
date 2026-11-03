#ifndef UDPCLIENT_H
#define UDPCLIENT_H

#include <string>
#include <stdexcept>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h> // for inet_pton
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <arpa/inet.h>
    #include <sys/socket.h>
    #include <unistd.h>
#endif

class UdpClient 
{
public:
    UdpClient(const std::string& ip, int port);
    ~UdpClient();

    bool send(const std::string& message);

private:
    int mSocket;
    struct sockaddr_in mServerAddress;
    std::string mIP;
    int mPort;
};

#endif // UDPCLIENT_H
