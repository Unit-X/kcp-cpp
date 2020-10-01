#include <iostream>

#include "kcpnet.h"

std::shared_ptr<KCPContext> gRetainThis;

std::shared_ptr<KCPContext> validateConnection(std::string lIP, uint16_t lPort, std::shared_ptr<KCPContext> &rpCtx) {

    //If you want to retain this connection context this is the place to do that. All other calls use the raw pointer to the object.

    std::cout << "Connecting IP:port > " << lIP << ":" << unsigned(lPort) << std::endl;
    rpCtx->mValue = 50;
    gRetainThis = rpCtx;
    return rpCtx;
}

void gotDataServer(const char* pData, size_t lSize, KCPContext* pCTX) {
    std::cout << "The server got -> " << unsigned(lSize) << " bytes of data" << std::endl;
}

void gotDataClient(const char* pData, size_t lSize, KCPContext* pCTX) {
    std::cout << "The client got -> " << unsigned(lSize) << " bytes of data" << std::endl;
}

int main() {
    std::cout << "KCP-cpp test" << std::endl;

    uint8_t lData[1000];

    KCPNetServer lKcpServer ( "127.0.0.1", 8000, nullptr);
    lKcpServer.mClientConnected = std::bind(&validateConnection, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3);

    lKcpServer.mGotDataServer = std::bind(&gotDataServer, std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3);

    KCPNetClient lKcpClient ( "127.0.0.1", 8000, 10, nullptr);
    lKcpClient.mGotDataClient = std::bind(&gotDataClient, std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3);
    lKcpClient.sendData((const char*)&lData[0], 1000);

    std::this_thread::sleep_for(std::chrono::seconds (1));
    lKcpServer.sendData((const char*)&lData[0], 1000, gRetainThis.get());
    std::this_thread::sleep_for(std::chrono::seconds (1));
    return 0;
}
