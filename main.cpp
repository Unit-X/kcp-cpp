#include <iostream>

#include "KCPNet.h"
#include <vector>
#include <algorithm>

std::shared_ptr<KCPContext> gRetainThis;

int gPacketNum = 0;

// Validate this connection
// Return a nullptr if you want to reject the new connection
// If you want to retain this connection context this is the place to do that. All other calls use the raw pointer to the object.
// You can also skip retaining this and just pass a smart_pointer to std::any within the KCPContext
std::shared_ptr<KCPContext> validateConnection(std::string lIP, uint16_t lPort, std::shared_ptr<KCPContext> &rpCtx) {
    std::cout << "Connecting IP:port > " << lIP << ":" << unsigned(lPort) << std::endl;
    rpCtx->mValue = 50;
    gRetainThis = rpCtx;

    // You can optionally configure the KCP connection here also.
    //rpCtx->mSettings.mMtu = 1000;

    // And set the wanted ID.
    //rpCtx->mID = 10; (10 is default)

    return rpCtx;
}

void gotDataServer(const char* pData, size_t lSize, KCPContext* pCTX) {
    std::cout << "The server got -> " << unsigned(lSize) << " bytes of data. pk num -> " << gPacketNum++ << std::endl;
}

void gotDataClient(const char* pData, size_t lSize, KCPContext* pCTX) {
    std::cout << "The client got -> " << unsigned(lSize) << " bytes of data" << std::endl;
}

void noConnectionServer(KCPContext* pCTX) {
    std::cout << "The server timed out a client." << std::endl;
}

void noConnectionClient(KCPContext* pCTX) {
    std::cout << "The server is not active." << std::endl;
}

class TestClass {
    int mTestValue = 100;
};

int main() {
    std::cout << "KCP-cpp test" << std::endl;

    std::vector<uint8_t> lData(4000);
    std::generate(lData.begin(), lData.end(), [n = 0]() mutable { return n++; });

    // Create the server and register the receive data callback and the validate connection callback
    std::shared_ptr<KCPContext> lx = std::make_shared<KCPContext>("");
    lx->mObject = std::make_shared<TestClass>();
    KCPNetServer lKcpServer;
    if (lKcpServer.configureKCP(gotDataServer,
                            noConnectionServer,
                            validateConnection,
                            "::1",
                            8000,
                            lx)) {
        std::cout << "Failed to configure the KCP Server" << std::endl;
    }

    KCPNetClient lKcpClient;

    KCPSettings lSettingsClient;
    if(lKcpClient.configureKCP(lSettingsClient,
                               gotDataClient,
                               noConnectionClient,
                               "::1",
                               8000,
                               10,
                               nullptr)) {
        std::cout << "Failed to configure the KCP Client" << std::endl;
    }

/*
    for ( uint64_t i=0 ; i<10000 ; i++ ) {
        *(uint64_t*)lData.data() = i;
        lKcpClient.sendData((const char*)lData.data(), 1000);
        std::cout << "Pushed -> " << unsigned(i) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds (1));
    }
    std::cout << "STOP SENDING" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds (2));
*/

    lKcpClient.sendData((const char*)lData.data(), 4000);

    std::this_thread::sleep_for(std::chrono::seconds (1));
    lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());
    std::this_thread::sleep_for(std::chrono::seconds (1));

    lKcpServer.mDropAll = true;

    std::this_thread::sleep_for(std::chrono::seconds (1));
    lKcpClient.sendData((const char*)lData.data(), 4000);
    lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());
    std::this_thread::sleep_for(std::chrono::seconds (1));
    lKcpClient.sendData((const char*)lData.data(), 4000);
    lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());
    std::this_thread::sleep_for(std::chrono::seconds (20));
    lKcpClient.sendData((const char*)lData.data(), 4000);
    lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());

    return 0;
}
