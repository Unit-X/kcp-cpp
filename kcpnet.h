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

class KCPSettings {
public:
    bool mNodelay = false; //No delay mode. False: Off / True: On.
    int mInterval = 100; //KCP update interval in ms
    int mResend = 0; //Retransmit when missed mResend number ACK (Default value is 0)
    bool mFlow = false; //Flow control, False: Off / True: On.
    int mMtu = 1472; //Maximum payload in a single UDP datagram
};

//Optional context passed to the callbacks
class KCPContext {
public:
    explicit KCPContext (uint64_t lKey) {
        mKCPSocket = lKey;
    }
    std::any mObject = nullptr;         // For safe object lifecycles
    void* mUnsafePointer = nullptr;     // Lightweight alternative for unsafe pointers
    uint64_t mValue = 0;                // Generic 64-bit variable
    uint64_t mID = 10;                  //KCP ID to be used
    KCPSettings mSettings;
    uint64_t mKCPSocket = 0;
};

class KCPNetServer {
public:

    class KCPServerData {
    public:
        KCPNetServer* mWeakKCPNetServer = nullptr;
        ikcpcb* mKCPServer = nullptr;
        std::shared_ptr<KCPContext> mKCPContext = nullptr;
        kissnet::udp_socket mSocket;
    };

    KCPNetServer(std::string lIP = "", uint16_t lport = 0, std::shared_ptr<KCPContext> pCTX = nullptr);

    virtual ~KCPNetServer();

    int sendData(const char* pData, size_t lSize, KCPContext* pCTX);
    int configureKCP(KCPSettings &rSettings, KCPContext* pCTX);
    //Method used by the bridge function
    void udpOutputServer(const char *pBuf, int lSize, KCPServerData* lCTX);
    ///Callback handling connecting clients
    std::function<std::shared_ptr<KCPContext>(std::string lIP, uint16_t lPort, std::shared_ptr<KCPContext> &rpCtx)> mValidateConnectionCallback = nullptr;

    ///Callback for getting data
    std::function<void(const char * pData, size_t lSize, KCPContext* pCTX)> mGotDataServer;

    // delete copy and move constructors and assign operators
    KCPNetServer(KCPNetServer const &) = delete;             // Copy construct
    KCPNetServer(KCPNetServer &&) = delete;                  // Move construct
    KCPNetServer &operator=(KCPNetServer const &) = delete;  // Copy assign
    KCPNetServer &operator=(KCPNetServer &&) = delete;      // Move assign

    bool mDropAll = false;

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
    KCPNetClient(std::string lIP = "", uint16_t lport = 0, uint32_t lID = 0, std::shared_ptr<KCPContext> pCTX = nullptr);

    virtual ~KCPNetClient();

    int sendData(const char* pData, size_t lSize);
    int configureKCP(KCPSettings &rSettings);

    ///Callback for getting data
    std::function<void(const char * pData, size_t lSize, KCPContext* pCTX)> mGotDataClient;

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
