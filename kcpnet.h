//
//  _   _______ ______ _   _      _
// | | / /  __ \| ___ \ \ | |    | |
// | |/ /| /  \/| |_/ /  \| | ___| |_
// |    \| |    |  __/| . ` |/ _ \ __|
// | |\  \ \__/\| |   | |\  |  __/ |_
// \_| \_/\____/\_|   \_| \_/\___|\__|
//
//
// Created by Unit-X @ Edgeware AB on 2020-09-30.
//

// C++ wrapper around KCP

#ifndef KCP_CPP_KCPNET_H
#define KCP_CPP_KCPNET_H

#include "ikcp.h"
#include "kissnet/kissnet.hpp"
#include <any>
#include <thread>
#include <unordered_map>
#include <functional>

class KCPNetServer {
public:

    //Optional context passed to the callbacks
    class KCPContext {
    public:
        explicit KCPContext (uint64_t lKey) {
            mKCPSocket = lKey;
        }
        std::any mObject = nullptr;         // For safe object lifecycles
        void* mUnsafePointer = nullptr;     // Lightweight alternative for unsafe pointers
        uint64_t mValue = 0;                // Generic 64-bit variable
        uint64_t mKCPSocket = 0;
    };

    class KCPServerData {
    public:
        KCPNetServer* mWeakParrent = nullptr;
        ikcpcb* mKCPServer = nullptr;
        kissnet::udp_socket mSocket;
    };

    KCPNetServer(std::string lIP = "", uint16_t lport = 0, std::shared_ptr<KCPContext> pCTX = nullptr);

    virtual ~KCPNetServer();

    void sendData(const char* pData, size_t lSize, KCPContext* pCTX);
    //Method used by the bridge function
    void udpOutputServer(const char *pBuf, int lSize, KCPServerData* lCTX);
    ///Callback handling connecting clients
    std::function<std::shared_ptr<KCPContext>(std::string lIP, uint16_t lPort, std::shared_ptr<KCPContext> &rpCtx)> mClientConnected = nullptr;

    ///Callback for getting data
    std::function<void(const char * pData, size_t lSize, KCPContext* pCTX)> mGotDataServer;

    // delete copy and move constructors and assign operators
    KCPNetServer(KCPNetServer const &) = delete;             // Copy construct
    KCPNetServer(KCPNetServer &&) = delete;                  // Move construct
    KCPNetServer &operator=(KCPNetServer const &) = delete;  // Copy assign
    KCPNetServer &operator=(KCPNetServer &&) = delete;      // Move assign

protected:
    std::shared_ptr<KCPContext> mCTX = nullptr;
private:

    void netWorkerServer();
    void kcpNudgeWorkerServer();

    std::unordered_map<uint64_t, std::unique_ptr<KCPServerData>> mKCPMap;
    kissnet::udp_socket mKissnetSocket;
    bool mNetworkThreadRunning = false;
    bool mNudgeThreadRunning = false;
    bool mNudgeThreadActive = false;
};

class KCPNetClient {
public:

    //Optional context passed to the callbacks
    class KCPContext {
    public:
        std::any mObject = nullptr;         // For safe object lifecycles
        void* mUnsafePointer = nullptr;     // Lightweight alternative for unsafe pointers
        uint64_t mValue = 0;                // Generic 64-bit variable
    };

    KCPNetClient(std::string lIP = "", uint16_t lport = 0, uint32_t lID = 0, std::shared_ptr<KCPContext> pCTX = nullptr);

    virtual ~KCPNetClient();

    void sendData(const char* pData, size_t lSize);

    ///Callback for getting data
    std::function<void()> mGotDataClient;


    //Method used by the bridge function
    void udpOutputClient(const char *pBuf, int lSize);

    // delete copy and move constructors and assign operators
    KCPNetClient(KCPNetClient const &) = delete;             // Copy construct
    KCPNetClient(KCPNetClient &&) = delete;                  // Move construct
    KCPNetClient &operator=(KCPNetClient const &) = delete;  // Copy assign
    KCPNetClient &operator=(KCPNetClient &&) = delete;      // Move assign

protected:
    std::shared_ptr<KCPContext> mCTX = nullptr;
private:

    void netWorkerClient();
    void kcpNudgeWorkerClient();

    ikcpcb *mKCP = nullptr; //The KCP handle for client mode
    kissnet::udp_socket mKissnetSocket;
    bool mNetworkThreadRunning = false;
    bool mNudgeThreadRunning = false;
    bool mNudgeThreadActive = false;
};

#endif //KCP_CPP_KCPNET_H
