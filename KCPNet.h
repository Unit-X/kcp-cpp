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

// All payload data is little endian.

#ifndef KCP_CPP_KCPNET_H
#define KCP_CPP_KCPNET_H

#include "ikcp.h"
#include "kissnet/kissnet.hpp"
#include <any>
#include <thread>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <vector>
#include <atomic>

// Time preamble
#define TIME_PREAMBLE_V1 0x000100010ff00ff0

// Time constants server
#define MAX_DELAY_DIFF_MS 20
#define MIN_COLLECTED_TIME_POINTS 5
#define MAX_SAVED_TIME_POINTS 100
#define TIME_PACKETS_BURST_DISTANCE_MS 100
#define TIME_PACKETS_NORMAL_DISTANCE_MS 1000

// Time constants client
// The maximum adjustment + and - in microseconds per second allowed
#define MAX_TIME_DRIFT_PPM 500

// Count the HEART_BEAT every x ms.
#define HEART_BEAT_DISTANCE 500
// Time out after HEART_BEAT_DISTANCE ms * HEART_BEAT_TIME_OUT milliseconds
#define HEART_BEAT_TIME_OUT 10

struct KCPTimePacket{
    uint64_t timePreamble = TIME_PREAMBLE_V1; //Version 1 preamble
    int64_t t1 = 0;
    int64_t t2 = 0;
    int64_t t3 = 0;
    int64_t t4 = 0;
    int64_t correction = 0;
    int64_t correctionActive = 0;
};
static_assert(sizeof(KCPTimePacket) == 56, "KCPTimePacket is not the expected size");

class KCPSettings {
public:
    bool mNoDelay = false;  // No delay mode. False: Off / True: On.
    int  mInterval = 100;   // KCP update interval in ms
    int  mResend = 0;       // Retransmit when missed mResend number ACK (Default value is 0)
    bool mFlow = false;     // Flow control, False: Off / True: On.
    int  mMtu = 1472;       // Maximum payload in a single UDP datagram
    int  mSndWnd = 32;      // Send window size
    int  mRcvWnd = 32;      // Receive window size //The doc says 32 the code says 128
};

// Optional context passed to the callbacks
class KCPContext {
public:
    explicit KCPContext (uint64_t lKey): mKCPSocket(lKey) {

    }
    std::any mObject = nullptr;         // For safe object lifecycles
    void* mUnsafePointer = nullptr;     // Lightweight alternative for unsafe pointers
    uint64_t mValue = 0;                // Generic 64-bit variable
    uint64_t mID = 10;                  //KCP ID to be used
    KCPSettings mSettings;
    uint64_t mKCPSocket = 0;
};

//------------------------------------------------------------------------------------------
//
// KCP Client
//
//------------------------------------------------------------------------------------------

class KCPNetClient {
public:
    explicit KCPNetClient();
    virtual ~KCPNetClient();

    int sendData(const char* pData, size_t lSize);

    int configureKCP(KCPSettings &rSettings,
                     const std::function<void(const char*, size_t, KCPContext*)> &rGotData,
                     const std::function<void(KCPContext*)> &rDisconnect,
                     const std::string& lIP = "",
                     uint16_t lPort = 0,
                     uint32_t lID = 0,
                     std::shared_ptr<KCPContext> pCTX = nullptr);

    int64_t getNetworkTimeus(); //Network time in us

    void udpOutputClient(const char *pBuf, int lSize); //Method used by the bridge function

    // delete copy and move constructors and assign operators
    KCPNetClient(KCPNetClient const &) = delete;             // Copy construct
    KCPNetClient(KCPNetClient &&) = delete;                  // Move construct
    KCPNetClient &operator=(KCPNetClient const &) = delete;  // Copy assign
    KCPNetClient &operator=(KCPNetClient &&) = delete;      // Move assign

protected:
    std::shared_ptr<KCPContext> mCTX = nullptr;

private:
    void netWorkerClient(const std::function<void(const char*, size_t, KCPContext*)> &rDisconnect);
    void kcpNudgeWorkerClient(const std::function<void(KCPContext*)> &rGotData);

    std::mutex mKCPNetMtx;
    ikcpcb *mKCP = nullptr; // The KCP handle for client mode
    kissnet::udp_socket mKissnetSocket;
    bool mNetworkThreadRunning = false;
    bool mNudgeThreadRunning = false;
    bool mNudgeThreadActive = false;
    uint64_t mConnectionTimeOut = HEART_BEAT_TIME_OUT;
    uint64_t mHeartBeatIntervalTrigger = 0;
    bool mFirstTimeDelivery = true;
    int64_t mLastDeliveredTime;
    std::atomic<int64_t> mCurrentCorrectionTarget = 0;
    std::atomic<int64_t> mCurrentCorrection = 0;
    std::atomic<bool> mGotCorrection = false;
};

//------------------------------------------------------------------------------------------
//
// KCP Server
//
//------------------------------------------------------------------------------------------

class KCPNetServer {
public:
    class KCPServerData {
    public:
        virtual ~KCPServerData() {
            ikcp_release(mKCPServer);
        }
        KCPNetServer* mWeakKCPNetServer = nullptr;
        ikcpcb* mKCPServer = nullptr;
        std::shared_ptr<KCPContext> mKCPContext = nullptr;
        kissnet::udp_socket mSocket;
        uint64_t mConnectionTimeOut = HEART_BEAT_TIME_OUT;

        bool mGotStableTime = false;
        bool mClientGotCorrection = false;
        std::vector<std::pair<int64_t, int64_t>> mListOfDelayAndCompensation;
        int64_t mCurrentCorrection = 0;
    };

    explicit KCPNetServer();

    virtual ~KCPNetServer();

    int sendData(const char* pData, size_t lSize, KCPContext* pCTX);

    int configureKCP(const std::function<void(const char*, size_t, KCPContext*)> &rGotData,
                     const std::function<void(KCPContext*)> &rDisconnect,
                     const std::function<std::shared_ptr<KCPContext>(std::string, uint16_t, std::shared_ptr<KCPContext>&)> &rValidate,
                     const std::string& lIP = "",
                     uint16_t lport = 0,
                     std::shared_ptr<KCPContext> pCTX = nullptr);

    // Method used by the bridge function
    void udpOutputServer(const char *pBuf, int lSize, KCPServerData* lCTX) const;

    // delete copy and move constructors and assign operators
    KCPNetServer(KCPNetServer const &) = delete;             // Copy construct
    KCPNetServer(KCPNetServer &&) = delete;                  // Move construct
    KCPNetServer &operator=(KCPNetServer const &) = delete;  // Copy assign
    KCPNetServer &operator=(KCPNetServer &&) = delete;      // Move assign

    bool mDropAll = false;

protected:
    std::shared_ptr<KCPContext> mCTX = nullptr;

private:
    int configureInternal(KCPSettings &rSettings, KCPContext *pCtx);
    void netWorkerServer(const std::function<void(const char*, size_t, KCPContext*)> &rGotData,
                        const std::function<std::shared_ptr<KCPContext>(std::string, uint16_t, std::shared_ptr<KCPContext>&)> &rValidate);
    void kcpNudgeWorkerServer(const std::function<void(KCPContext*)> &rDisconnect);
    void sendTimePacket(KCPServerData &rServerData);

    std::mutex mKCPMapMtx;
    std::unordered_map<uint64_t, std::unique_ptr<KCPServerData>> mKCPMap;
    kissnet::udp_socket mKissnetSocket;
    bool mNetworkThreadRunning = false;
    bool mNudgeThreadRunning = false;
    bool mNudgeThreadActive = false;
    uint64_t mHeartBeatIntervalTrigger = 0;
    uint64_t mSendTimeIntervalTriggerLow = 0;
    uint64_t mSendTimeIntervalTriggerHi = 0;
};

#endif //KCP_CPP_KCPNET_H
