#include <iostream>

#include "kcpnet.h"
#include <vector>

std::shared_ptr<KCPContext> gRetainThis;

//Validate this connection
//Return a nullptr if you want to reject the new connection
//If you want to retain this connection context this is the place to do that. All other calls use the raw pointer to the object.
//You can also skip retaining this and just pass a smart_pointer to std::any within the KCPContext
std::shared_ptr<KCPContext> validateConnection(std::string lIP, uint16_t lPort, std::shared_ptr<KCPContext> &rpCtx) {
    std::cout << "Connecting IP:port > " << lIP << ":" << unsigned(lPort) << std::endl;
    rpCtx->mValue = 50;
    gRetainThis = rpCtx;

    //You can optionally configure the KCP connection here also.
    //rpCtx->mSettings.mMtu = 1000;

    //And set the wanted ID.
    //rpCtx->mID = 10; (10 is default)

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

    std::vector<uint8_t> lData(4000);
    std::generate(lData.begin(), lData.end(), [n = 0]() mutable { return n++; });

    //Create the server and register the receive data callback and the validate connection callback
    KCPNetServer lKcpServer ( "127.0.0.1", 8000, nullptr);
    lKcpServer.mValidateConnectionCallback = std::bind(&validateConnection, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3);

    lKcpServer.mGotDataServer = std::bind(&gotDataServer, std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3);

    KCPNetClient lKcpClient ( "127.0.0.1", 8000, 10, nullptr);
    lKcpClient.mGotDataClient = std::bind(&gotDataClient, std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3);

    KCPSettings lSettingsClient;
    if(lKcpClient.configureKCP(lSettingsClient)) {
        std::cout << "Failed configuring KCP Client" << std::endl;
    }

    lKcpClient.sendData((const char*)lData.data(), 4000);

    std::this_thread::sleep_for(std::chrono::seconds (1));
    lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());
    std::this_thread::sleep_for(std::chrono::seconds (1));
    return 0;
}
