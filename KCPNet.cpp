//
// Created by Anders Cedronius on 2020-09-30.
//

//TODO documentation, unit tests.

#include "KCPNet.h"
#include "KCPLogger.h"
#include <stdexcept>
#include <algorithm>
#include <vector>

#define KCP_MAX_BYTES 4096

//------------------------------------------------------------------------------------------
//
// KCP Client
//
//------------------------------------------------------------------------------------------

int udp_output_client(const char *pBuf, int lSize, ikcpcb *pKCP, void *pCTX) {
    auto *lWeakSelf = (KCPNetClient *) pCTX;
    if (lWeakSelf) {
        lWeakSelf->udpOutputClient(pBuf, lSize);
    } else {
        KCP_LOGGER(true, LOGG_FATAL, "udp_output_client failed getting 'this'")
        return -1; // Throw
    }
    return 0;
}

void KCPNetClient::udpOutputClient(const char *pBuf, int lSize) {
    auto[lSentBytes, lStatus] = mKissnetSocket.send((const std::byte *) pBuf, lSize);
    if (lSentBytes != lSize || lStatus != kissnet::socket_status::valid) {
        KCP_LOGGER(false, LOGG_NOTIFY, "Client failed sending data")
    }
}

KCPNetClient::KCPNetClient() {

}

KCPNetClient::~KCPNetClient() {
    uint32_t lDeadLock;
    // Signal close netWorker and nudge thread
    mKissnetSocket.close(); // End net thread
    mNudgeThreadActive = false; // End nudge thread

    // Join network thread
    if (mNetworkThreadRunning) {
        lDeadLock = 10;
        while (mNetworkThreadRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!--lDeadLock) {
                KCP_LOGGER(true, LOGG_FATAL, "Client network is not ending will terminate anyway")
            }
        }
    }

    // Join nudge thread
    lDeadLock = 10;
    while (mNudgeThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!--lDeadLock) {
            KCP_LOGGER(true, LOGG_FATAL, "Client nudge thread not ending will terminate anyway")
        }
    }
    std::lock_guard<std::mutex> lock(mKCPNetMtx);
    if (mKCP) ikcp_release(mKCP);
    KCP_LOGGER(false, LOGG_NOTIFY, "KCPNetClient Destruct")
}

// Fix in KCP later
int KCPNetClient::sendData(const char *pData, size_t lSize) {
    std::lock_guard<std::mutex> lock(mKCPNetMtx);
    return ikcp_send(mKCP, pData, lSize);
}

int KCPNetClient::configureKCP(KCPSettings &rSettings,
                               const std::function<void(const char *, size_t, KCPContext *)> &rGotData,
                               const std::function<void(KCPContext *)> &rDisconnect,
                               const std::string &lIP, uint16_t lPort,
                               uint32_t lID, std::shared_ptr<KCPContext> pCTX) {
    mCTX = std::move(pCTX);

    if (lIP.empty()) {
        KCP_LOGGER(true, LOGG_FATAL, "IP / HOST must be provided")
        return 1;
    }

    if (!lPort) {
        KCP_LOGGER(true, LOGG_FATAL, "Port must be provided")
        return 1;
    }

    if (!lID) {
        KCP_LOGGER(true, LOGG_FATAL, "KCP ID can't be 0")
        return 1;
    }

    kissnet::udp_socket lCreateSocket(kissnet::endpoint(lIP, lPort));
    mKissnetSocket = std::move(lCreateSocket); // Move ownership to this/me

    mKCP = ikcp_create(lID, this);
    if (!mKCP) {
        KCP_LOGGER(true, LOGG_FATAL, "Failed to create KCP")
        return 1;
    }
    mKCP->output = udp_output_client;
    std::thread([=]() { netWorkerClient(rGotData); }).detach();
    std::thread([=]() { kcpNudgeWorkerClient(rDisconnect); }).detach();
    KCP_LOGGER(false, LOGG_NOTIFY, "KCPNetClient Constructed");


    std::lock_guard<std::mutex> lock(mKCPNetMtx);
    int lResult;
    lResult = ikcp_nodelay(mKCP, rSettings.mNoDelay, rSettings.mInterval, rSettings.mResend, rSettings.mFlow);
    if (lResult) {
        KCP_LOGGER(false, LOGG_ERROR, "ikcp_nodelay client failed.")
        return lResult;
    }
    lResult = ikcp_setmtu(mKCP, rSettings.mMtu);
    if (lResult) {
        KCP_LOGGER(false, LOGG_ERROR, "ikcp_setmtu client failed.")
        return lResult;
    }
    lResult = ikcp_wndsize(mKCP, rSettings.mSndWnd, rSettings.mRcvWnd);
    if (lResult) {
        KCP_LOGGER(false, LOGG_ERROR, "ikcp_wndsize client failed.")
    }
    return lResult;
}

// TODO Create a drift into corrected time
int64_t KCPNetClient::getNetworkTimeus(){
    if (!mGotCorrection) return 0;
    int64_t lLocalTime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    int64_t lRecalculatedTime = lLocalTime - mCurrentCorrection;

    if (!mFirstTimeDelivery) {
        if (mLastDeliveredTime < lRecalculatedTime) {
            mLastDeliveredTime = lRecalculatedTime;
            return lRecalculatedTime;
        } else {
            return mLastDeliveredTime;
        }
    }

    mLastDeliveredTime = lRecalculatedTime;
    mFirstTimeDelivery = false;
    return lRecalculatedTime;
}

void KCPNetClient::kcpNudgeWorkerClient(const std::function<void(KCPContext *)> &rDisconnect) {
    mNudgeThreadRunning = true;
    mNudgeThreadActive = true;
    uint32_t lTimeSleepms = 10;
    uint64_t lTimeBaseus = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

    uint64_t lTimeLast = 0;

    while (mNudgeThreadActive) {
        std::this_thread::sleep_for(std::chrono::milliseconds(lTimeSleepms));
        uint64_t lTimeNowus = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - lTimeBaseus;

        if (mGotCorrection) {
            if (!lTimeLast) {
                lTimeLast = lTimeNowus;
            } else {
                uint64_t lDiffTime = lTimeNowus - lTimeLast;
                lTimeLast = lTimeNowus;
                if (mCurrentCorrectionTarget != mCurrentCorrection) {
                    double lMulFac = lDiffTime / 1000000.0;
                    double lMaxComp =   lMulFac * MAX_TIME_DRIFT_PPM;
                    if (mCurrentCorrectionTarget > mCurrentCorrection) {
                        //KCP_LOGGER(false, LOGG_NOTIFY, "+curr " << mCurrentCorrection << " target " << mCurrentCorrectionTarget << " compensation " << lMaxComp)
                        mCurrentCorrection += lMaxComp;
                        if (mCurrentCorrection > mCurrentCorrectionTarget) {
                            int64_t lTmp = mCurrentCorrectionTarget;
                            mCurrentCorrection = lTmp;
                        }
                    } else {
                        //KCP_LOGGER(false, LOGG_NOTIFY, "-curr " << mCurrentCorrection << " target " << mCurrentCorrectionTarget << " compensation " << lMaxComp)
                        mCurrentCorrection -= lMaxComp;
                        if (mCurrentCorrection < mCurrentCorrectionTarget) {
                            int64_t lTmp = mCurrentCorrectionTarget;
                            mCurrentCorrection = lTmp;
                        }
                    }
                }
            }
        }

        uint64_t lTimeNowms = lTimeNowus / 1000;
        if (lTimeNowms > mHeartBeatIntervalTrigger) {
            mHeartBeatIntervalTrigger += HEART_BEAT_DISTANCE;
            KCP_LOGGER(false, LOGG_NOTIFY, "Heart beat client")
            if (!mConnectionTimeOut && rDisconnect) {
                rDisconnect(mCTX.get());
                mConnectionTimeOut = HEART_BEAT_TIME_OUT;
            }
            mConnectionTimeOut--;
        }
        mKCPNetMtx.lock();
        ikcp_update(mKCP, lTimeNowms);
        lTimeSleepms = ikcp_check(mKCP, lTimeNowms) - lTimeNowms;
        mKCPNetMtx.unlock();
        //KCP_LOGGER(false, LOGG_NOTIFY,"dead client? " << mKCP->dead_link)
        //KCP_LOGGER(false, LOGG_NOTIFY,"k " << lTimeSleep << " " << lTimeNow)
    }
    KCP_LOGGER(false, LOGG_NOTIFY, "kcpNudgeWorkerClient quitting")
    mNudgeThreadRunning = false;
}

void KCPNetClient::netWorkerClient(const std::function<void(const char *, size_t, KCPContext *)> &rGotData) {
    mNetworkThreadRunning = true;
    kissnet::buffer<KCP_MAX_BYTES> receiveBuffer;
    char lBuffer[KCP_MAX_BYTES];
    while (true) {
        auto[received_bytes, status] = mKissnetSocket.recv(receiveBuffer);
        if (!received_bytes || status != kissnet::socket_status::valid) {
            KCP_LOGGER(false, LOGG_NOTIFY, "netWorkerClient quitting")
            break;
        }

        if (*(uint64_t *) receiveBuffer.data() == TIME_PREAMBLE_V1) {
            auto lTimeData = (KCPTimePacket *) receiveBuffer.data();
            if (lTimeData->correctionActive == 1) {
                lTimeData->correctionActive = 2;
                if (!mGotCorrection) {
                    mCurrentCorrection = lTimeData->correction;
                }
                mCurrentCorrectionTarget = lTimeData->correction;
                mGotCorrection = true;
            }
            mKCPNetMtx.lock();
            int64_t lTimeNow = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            lTimeData->t2 = lTimeNow;
            lTimeData->t3 = lTimeNow;
            auto[lSentBytes, lStatus] = mKissnetSocket.send(receiveBuffer, sizeof(KCPTimePacket));
            if (lSentBytes != sizeof(KCPTimePacket) || lStatus != kissnet::socket_status::valid) {
                KCP_LOGGER(false, LOGG_NOTIFY, "Client failed sending data")
            }
            mConnectionTimeOut = HEART_BEAT_TIME_OUT;
            mKCPNetMtx.unlock();
            continue;
        }

        mKCPNetMtx.lock();
        mConnectionTimeOut = HEART_BEAT_TIME_OUT;
        ikcp_input(mKCP, (const char *) receiveBuffer.data(), received_bytes);
        int lRcv = ikcp_recv(mKCP, &lBuffer[0], KCP_MAX_BYTES);
        mKCPNetMtx.unlock();
        if (lRcv > 0 && rGotData) {
            rGotData(&lBuffer[0], lRcv, mCTX.get());
        } // Else deal with code?

    }
    mNetworkThreadRunning = false;
}


//------------------------------------------------------------------------------------------
//
// KCP Server
//
//------------------------------------------------------------------------------------------

int udp_output_server(const char *pBuf, int lSize, ikcpcb *pKCP, void *pCTX) {
    auto *lWeakSelf = (KCPNetServer::KCPServerData *) pCTX;
    if (lWeakSelf) {
        if (lWeakSelf->mWeakKCPNetServer) {
            lWeakSelf->mWeakKCPNetServer->udpOutputServer(pBuf, lSize, lWeakSelf);
        } else {
            KCP_LOGGER(true, LOGG_FATAL, "udp_output_server failed getting 'this'")
        }
    } else {
        KCP_LOGGER(true, LOGG_FATAL, "udp_output_server failed getting KCPServerData")
        return -1; // Throw
    }
    return 0;
}

void KCPNetServer::udpOutputServer(const char *pBuf, int lSize, KCPServerData *lCTX) const {
    if (mDropAll) return;
    auto[lSentBytes, lStatus] = lCTX->mSocket.send((const std::byte *) pBuf, lSize);
    if (lSentBytes != lSize || lStatus != kissnet::socket_status::valid) {
        KCP_LOGGER(false, LOGG_NOTIFY, "Server failed sending data")
    }
}

KCPNetServer::KCPNetServer() {

}

KCPNetServer::~KCPNetServer() {
    uint32_t lDeadLock;
    // Signal close netWorker and nudge thread
    mKissnetSocket.close(); // End net thread
    mNudgeThreadActive = false; // End nudge thread
    // Join network thread
    if (mNetworkThreadRunning) {
        lDeadLock = 10;
        while (mNetworkThreadRunning) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!--lDeadLock) {
                KCP_LOGGER(true, LOGG_FATAL, "Server net thread is not ending will terminate anyway")
            }
        }
    }

    //Join nudge thread
    lDeadLock = 10;
    while (mNudgeThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!--lDeadLock) {
            KCP_LOGGER(true, LOGG_FATAL, "Nudge thread not ending will terminate anyway")
        }
    }

    std::lock_guard<std::mutex> lock(mKCPMapMtx);
    for (const auto &rKCP: mKCPMap) {
        if (rKCP.second->mKCPServer) ikcp_release(rKCP.second->mKCPServer);
    }
    KCP_LOGGER(false, LOGG_NOTIFY, "KCPNetServer Destruct")
}

int KCPNetServer::sendData(const char *pData, size_t lSize, KCPContext *pCTX) {
    std::lock_guard<std::mutex> lock(mKCPMapMtx);
    int lStatus = -1;
    if (mKCPMap.count(pCTX->mKCPSocket)) {
        lStatus = ikcp_send(mKCPMap[pCTX->mKCPSocket]->mKCPServer, pData, lSize);
    } else {
        KCP_LOGGER(false, LOGG_NOTIFY, "KCP Connection is unknown")
    }
    return lStatus;
}

int KCPNetServer::configureKCP(const std::function<void(const char *, size_t, KCPContext *)> &rGotData,
                               const std::function<void(KCPContext *)> &rDisconnect,
                               const std::function<std::shared_ptr<KCPContext>(std::string, uint16_t,
                                                                        std::shared_ptr<KCPContext> &)> &rValidate,
                               const std::string &lIP,
                               uint16_t lPort,
                               std::shared_ptr<KCPContext> pCTX) {
    mCTX = std::move(pCTX);
    if (lIP.empty()) {
        KCP_LOGGER(true, LOGG_FATAL, "IP / HOST must be provided");
        return 1;
    }
    if (!lPort) {
        KCP_LOGGER(true, LOGG_FATAL, "Port must be provided")
        return 1;
    }

    kissnet::udp_socket lCreateSocket(kissnet::endpoint(lIP, lPort));
    mKissnetSocket = std::move(lCreateSocket); //Move ownership to this/me
    mKissnetSocket.bind();

    std::thread([=]() { netWorkerServer(rGotData, rValidate); }).detach();
    std::thread([=]() { kcpNudgeWorkerServer(rDisconnect); }).detach();
    KCP_LOGGER(false, LOGG_NOTIFY, "KCPNetServer Constructed");
    return 0;
}

int KCPNetServer::configureInternal(KCPSettings &rSettings, KCPContext *pCTX) {
    std::lock_guard<std::mutex> lock(mKCPMapMtx);
    int lResult = 0;
    if (mKCPMap.count(pCTX->mKCPSocket)) {
        lResult = ikcp_nodelay(mKCPMap[pCTX->mKCPSocket]->mKCPServer, rSettings.mNoDelay, rSettings.mInterval,
                               rSettings.mResend, rSettings.mFlow);
        if (lResult) {
            KCP_LOGGER(false, LOGG_ERROR, "ikcp_nodelay server failed.")
            return lResult;
        }
        lResult = ikcp_setmtu(mKCPMap[pCTX->mKCPSocket]->mKCPServer, rSettings.mMtu);
        if (lResult) {
            KCP_LOGGER(false, LOGG_ERROR, "ikcp_setmtu server failed.")
            return lResult;
        }
        lResult = ikcp_wndsize(mKCPMap[pCTX->mKCPSocket]->mKCPServer, rSettings.mSndWnd, rSettings.mRcvWnd);
        if (lResult) {
            KCP_LOGGER(false, LOGG_ERROR, "ikcp_wndsize server failed.")
            return lResult;
        }
    } else {
        KCP_LOGGER(false, LOGG_NOTIFY, "KCP Connection is unknown")
    }
    return lResult;
}

// Im in lock here no need to lock again
void KCPNetServer::sendTimePacket(KCPServerData &rServerData) {

    KCPTimePacket lTimePacket;
    lTimePacket.t1 = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    if (rServerData.mGotStableTime) {
        lTimePacket.correctionActive = 1;
        lTimePacket.correction = rServerData.mCurrentCorrection;
    }
    auto[lSentBytes, lStatus] = rServerData.mSocket.send((const std::byte *) &lTimePacket, sizeof(lTimePacket));
    if (lSentBytes != sizeof(lTimePacket) || lStatus != kissnet::socket_status::valid) {
        KCP_LOGGER(false, LOGG_NOTIFY, "Server failed sending timing data")
    }
}

// For now the server is updating all connections every 10ms
void KCPNetServer::kcpNudgeWorkerServer(const std::function<void(KCPContext *)> &rDisconnect) {
    mNudgeThreadRunning = true;
    mNudgeThreadActive = true;
    uint32_t lTimeSleep = 10;
    uint64_t lTimeBase = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    while (mNudgeThreadActive) {
        std::this_thread::sleep_for(std::chrono::milliseconds(lTimeSleep));
        uint64_t lTimeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - lTimeBase;

        bool ltimeOutFlag = false;
        if (lTimeNow > mHeartBeatIntervalTrigger) {
            ltimeOutFlag = true;
            mHeartBeatIntervalTrigger += HEART_BEAT_DISTANCE;
            KCP_LOGGER(false, LOGG_NOTIFY, "Heart beat ")
        }

        bool lSendTimeLowFreq = false;
        if (lTimeNow > mSendTimeIntervalTriggerLow) {
            lSendTimeLowFreq = true;
            mSendTimeIntervalTriggerLow += TIME_PACKETS_NORMAL_DISTANCE_MS;
            //KCP_LOGGER(false, LOGG_NOTIFY,"Send time low")
        }

        bool lSendTimeHiFreq = false;
        if (lTimeNow > mSendTimeIntervalTriggerHi) {
            lSendTimeHiFreq = true;
            mSendTimeIntervalTriggerHi += TIME_PACKETS_BURST_DISTANCE_MS;
            //KCP_LOGGER(false, LOGG_NOTIFY,"Send time hi")
        }

        uint32_t lTimeSleepLowest = UINT32_MAX;
        mKCPMapMtx.lock();
        if (!mKCPMap.empty()) {
            std::vector<uint64_t> lRemoveList;
            //for (const auto &rKCP: mKCPMap) {
            for (auto pKCP = mKCPMap.cbegin(); pKCP != mKCPMap.cend() /* not hoisted */; /* no increment */) {
                // KCP_LOGGER(false, LOGG_NOTIFY,"dead server? " << pKCP->second->mKCPServer->dead_link)
                bool lDeleteThis = false;
                if (ltimeOutFlag) {
                    if (!pKCP->second->mConnectionTimeOut && rDisconnect) {
                        mKCPMapMtx.unlock();
                        rDisconnect(pKCP->second->mKCPContext.get());
                        mKCPMapMtx.lock();
                        lDeleteThis = true;
                    }
                    pKCP->second->mConnectionTimeOut--;
                }

                if (lDeleteThis) {
                    KCP_LOGGER(false, LOGG_NOTIFY, "Removed stale client")
                    mKCPMap.erase(pKCP++);
                } else {
                    ikcp_update(pKCP->second->mKCPServer, lTimeNow);
                    uint32_t lTimeSleepCandidate = ikcp_check(pKCP->second->mKCPServer, lTimeNow) - lTimeNow;
                    if (lTimeSleepCandidate < lTimeSleepLowest) {
                        lTimeSleepLowest = lTimeSleepCandidate;
                    }

                    // Time transfer section

                    if (!pKCP->second->mClientGotCorrection && lSendTimeHiFreq) {
                        // We have not corrected any time. send time packet
                        sendTimePacket(*pKCP->second);
                    } else if (pKCP->second->mClientGotCorrection && lSendTimeLowFreq) {
                        // We have synced send a normal packet
                        sendTimePacket(*pKCP->second);
                    }

                    // ---------------------

                    ++pKCP;
                }
            }
        }
        mKCPMapMtx.unlock();
        if (lTimeSleepLowest != UINT32_MAX) {
            lTimeSleep = lTimeSleepLowest;
        } else {
            lTimeSleep = 10;
        }

    }
    KCP_LOGGER(false, LOGG_NOTIFY, "kcpNudgeWorker quitting")
    mNudgeThreadRunning = false;
}

void KCPNetServer::netWorkerServer(const std::function<void(const char *, size_t, KCPContext *)>& rGotData,
                                   const std::function<std::shared_ptr<KCPContext>(std::string,
                                                                                   uint16_t,
                                                                                   std::shared_ptr<KCPContext> &)> & rValidate) {
    mNetworkThreadRunning = true;
    kissnet::buffer<KCP_MAX_BYTES> receiveBuffer;
    char lBuffer[KCP_MAX_BYTES];
    while (true) {
        auto[received_bytes, status] = mKissnetSocket.recv(receiveBuffer);
        if (!received_bytes || status != kissnet::socket_status::valid) {
            KCP_LOGGER(false, LOGG_NOTIFY, "serverWorker quitting")
            break;
        }
        if (status == kissnet::socket_status::non_blocking_would_have_blocked) {
            KCP_LOGGER(false, LOGG_NOTIFY, "non_blocking_would_have_blocked")
            continue;
        }

        if (mDropAll) continue;
        // Who did send me data? Generate a unique key where (ip:port) a.b.c.d:e becomes a (broken down to uiny8_t) uint64_t 00abcdee
        kissnet::endpoint lFromWho = mKissnetSocket.get_recv_endpoint();
        std::stringstream lS(lFromWho.address);
        int lA, lB, lC, lD; // to store the 4 ints from the ip string
        char lCh; // to temporarily store the '.'
        // or + shift it all together
        lS >> lA >> lCh >> lB >> lCh >> lC >> lCh >> lD;

        // Optimize?
        uint64_t lKey =
                (uint64_t) lA << (uint64_t) 40 | (uint64_t) lB << (uint64_t) 32 | (uint64_t) lC << (uint64_t) 24 |
                (uint64_t) lD << (uint64_t) 16 |
                (uint64_t) lFromWho.port;

        mKCPMapMtx.lock();
        if (!mKCPMap.count(lKey)) {
            KCP_LOGGER(false, LOGG_NOTIFY, "New server connection")
            std::shared_ptr<KCPContext> lx = std::make_shared<KCPContext>(lKey);
            if (mCTX) {
                lx->mUnsafePointer = mCTX->mUnsafePointer;
                lx->mValue = mCTX->mValue;
                lx->mObject = mCTX->mObject;
            }

            if (rValidate) {
                mKCPMapMtx.unlock();
                auto lCTX = rValidate(lFromWho.address, lFromWho.port, lx);
                if (lCTX == nullptr) {
                    KCP_LOGGER(false, LOGG_NOTIFY, "Connection rejected")
                    continue;
                }
                mKCPMapMtx.lock();
            }

            // Create the connection and hand RCP the data
            auto lConnection = std::make_unique<KCPServerData>();
            lConnection->mKCPContext = lx;
            lConnection->mWeakKCPNetServer = this;
            lConnection->mKCPServer = ikcp_create(lx->mID, lConnection.get());
            if (!lConnection->mKCPServer) {
                throw std::runtime_error("Failed creating KCP");
            }
            lConnection->mKCPServer->output = udp_output_server;
            kissnet::udp_socket lCreateSocket(kissnet::endpoint(lFromWho.address, lFromWho.port));
            lConnection->mSocket = std::move(lCreateSocket); //Move ownership to this/me
            mKCPMap[lKey] = std::move(lConnection);
            mKCPMapMtx.unlock();
            configureInternal(lx->mSettings, lx.get());
            if (*(uint64_t *) receiveBuffer.data() == TIME_PREAMBLE_V1) {
                KCP_LOGGER(false, LOGG_NOTIFY, "server got time do nothing")
                continue;
            }
            mKCPMapMtx.lock();
            ikcp_input(mKCPMap[lKey]->mKCPServer, (const char *) receiveBuffer.data(), received_bytes);
            int lRcv = ikcp_recv(mKCPMap[lKey]->mKCPServer, &lBuffer[0], KCP_MAX_BYTES);
            mKCPMapMtx.unlock();
            if (lRcv > 0 && rGotData) {
                rGotData(&lBuffer[0], lRcv, lx.get());
            }
            // The connection is known pass the data to RCP
        } else {
            if (*(uint64_t *) receiveBuffer.data() == TIME_PREAMBLE_V1) {
                auto lTimeData = (KCPTimePacket *) receiveBuffer.data();
                int64_t lTimeNow = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                lTimeData->t4 = lTimeNow;
                int64_t lDelay = lTimeData->t4 - lTimeData->t1;
                int64_t lCompensation = ((lTimeData->t2 - lTimeData->t1) + (lTimeData->t3 - lTimeData->t4)) / 2;

                // Check T2 - T1 compared to T4 - T3 ?
                // The lowest delay has the lowest diff anyway so let's do that later if needed.

                mKCPMap[lKey]->mListOfDelayAndCompensation.emplace_back(std::make_pair(lDelay, lCompensation));
                if (mKCPMap[lKey]->mListOfDelayAndCompensation.size() > MAX_SAVED_TIME_POINTS) {
                    mKCPMap[lKey]->mListOfDelayAndCompensation.erase(
                            mKCPMap[lKey]->mListOfDelayAndCompensation.begin());
                }

                if (mKCPMap[lKey]->mListOfDelayAndCompensation.size() >= MIN_COLLECTED_TIME_POINTS) {
                    std::vector<std::pair<int64_t, int64_t>> lTimePoints(MIN_COLLECTED_TIME_POINTS);
                    std::partial_sort_copy(mKCPMap[lKey]->mListOfDelayAndCompensation.begin(),
                                           mKCPMap[lKey]->mListOfDelayAndCompensation.end(),
                                           lTimePoints.begin(),
                                           lTimePoints.end());
                    auto[lMinDelay, lMinCompensation] = lTimePoints[0];
                    auto[lMaxDelay, lMaxCompensation] = lTimePoints[MIN_COLLECTED_TIME_POINTS - 1];

                    if (((lMaxDelay - lMinDelay) / 1000) < MAX_DELAY_DIFF_MS && !mKCPMap[lKey]->mGotStableTime) {
                        mKCPMap[lKey]->mGotStableTime = true;
                    }

                    if (mKCPMap[lKey]->mGotStableTime) {
                        mKCPMap[lKey]->mCurrentCorrection = lMinCompensation;
                    }

                    if (!mKCPMap[lKey]->mClientGotCorrection && lTimeData->correctionActive == 2) {
                        mKCPMap[lKey]->mClientGotCorrection = true;
                    }
                }

                KCP_LOGGER(false, LOGG_NOTIFY, "server got time. Delay -> " << lDelay
                                                                            << " lCompensation -> " << lCompensation
                                                                            << " StableTime -> "
                                                                            << mKCPMap[lKey]->mGotStableTime
                                                                            << " Compensation -> "
                                                                            << mKCPMap[lKey]->mCurrentCorrection
                )
                mKCPMapMtx.unlock();
                continue;
            }
            mKCPMap[lKey]->mConnectionTimeOut = HEART_BEAT_TIME_OUT;
            ikcp_input(mKCPMap[lKey]->mKCPServer, (const char *) receiveBuffer.data(), received_bytes);
            int lRcv = ikcp_recv(mKCPMap[lKey]->mKCPServer, &lBuffer[0], KCP_MAX_BYTES);
            mKCPMapMtx.unlock();
            if (lRcv > 0 && rGotData) {
                rGotData(&lBuffer[0], lRcv, mKCPMap[lKey]->mKCPContext.get());
            }
        }
    }
    mNetworkThreadRunning = false;
}