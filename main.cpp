#include <iostream>

#include "kcpnet.h"

std::shared_ptr<KCPNetServer::KCPContext> gRetainThis;

std::shared_ptr<KCPNetServer::KCPContext> validateConnection(std::string lIP, uint16_t lPort, std::shared_ptr<KCPNetServer::KCPContext> &rpCtx) {
    std::cout << "Connecting IP:port > " << lIP << ":" << unsigned(lPort) << std::endl;
    rpCtx->mValue = 50;
    gRetainThis = rpCtx;
   return rpCtx;
}

void gotDataServer(const char* pData, size_t lSize, KCPNetServer::KCPContext* pCTX) {
    std::cout << "Data server mofo" << std::endl;
}

void gotDataClient() {
    std::cout << "Data client mofo" << std::endl;
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    uint8_t lData[1000];

    KCPNetServer lKcpServer ( "127.0.0.1", 8000, nullptr);
    lKcpServer.mClientConnected = std::bind(&validateConnection, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3);

    lKcpServer.mGotDataServer = std::bind(&gotDataServer, std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3);

    KCPNetClient lKcpClient ( "127.0.0.1", 8000, 10, nullptr);
   lKcpClient.mGotDataClient = std::bind(&gotDataClient);
    lKcpClient.sendData((const char*)&lData[0], 1000);

    std::this_thread::sleep_for(std::chrono::seconds (1));
    lKcpServer.sendData((const char*)&lData[0], 1000, gRetainThis.get());
    std::this_thread::sleep_for(std::chrono::seconds (1));
    return 0;
}
