//
// Created by Anders Cedronius on 2020-09-30.
//

//TODO Disconnect, locks for thread safety, documentation, unit tests.

#include "kcpnet.h"
#include "kcplogger.h"
#include <stdexcept>

#define MAX_BYTES_RCV 4096

//------------------------------------------------------------------------------------------
//
// KCP Client
//
//------------------------------------------------------------------------------------------

int udp_output_client(const char *pBuf, int lSize, ikcpcb *pKCP, void *pCTX)
{
    KCPNetClient* lWeakSelf = (KCPNetClient*)pCTX;
    if (lWeakSelf) {
        lWeakSelf->udpOutputClient(pBuf, lSize);
    } else {
        KCP_LOGGER(true,LOGG_FATAL,"udp_output_client failed getting 'this'")
        return -1; //Throw
    }
    return 0;
}

void KCPNetClient::udpOutputClient(const char *pBuf, int lSize) {
    auto[lSentBytes, lStatus] = mKissnetSocket.send((const std::byte *) pBuf, lSize);
    if (lSentBytes != lSize || lStatus != kissnet::socket_status::valid) {
        KCP_LOGGER(false, LOGG_NOTIFY,"Client failed sending data")
    }
}

KCPNetClient::KCPNetClient(std::string lIP, uint16_t lPort, uint32_t lID, std::shared_ptr<KCPContext> pCTX) {
    mCTX = std::move(pCTX);

    if (!lIP.size()) {
        KCP_LOGGER(true,LOGG_FATAL,"IP / HOST must be provided")
        throw std::invalid_argument( "IP / HOST must be provided" );
    }

    if (!lPort) {
        KCP_LOGGER(true,LOGG_FATAL,"Port must be provided")
        throw std::invalid_argument( "Port must be provided" );
    }

    if (!lID) {
        KCP_LOGGER(true,LOGG_FATAL,"KCP ID can't be 0")
        throw std::invalid_argument( "KCP ID can't be 0" );
    }

    kissnet::udp_socket lCreateSocket(kissnet::endpoint(lIP, lPort));
    mKissnetSocket = std::move(lCreateSocket); //Move ownership to this/me

    mKCP = ikcp_create(lID, this);
    if (!mKCP) {
        throw std::runtime_error("Failed creating KCP");
    }
    mKCP->output = udp_output_client;

    std::thread(std::bind(&KCPNetClient::netWorkerClient, this)).detach();
    std::thread(std::bind(&KCPNetClient::kcpNudgeWorkerClient, this)).detach();
    KCP_LOGGER(false,LOGG_NOTIFY,"KCPNetClient Constructed")
}

KCPNetClient::~KCPNetClient() {
    uint32_t lDeadLock;
    //Signal close netWorker and nudge thread
    mKissnetSocket.close(); //End net thread
    mNudgeThreadActive = false; //End nudge thread

    //Join network thread
    if (mNetworkThreadRunning) {
        lDeadLock = 10;
        while (mNetworkThreadRunning ) {
            std::this_thread::sleep_for(std::chrono::milliseconds (100));
            if (!--lDeadLock) {
                KCP_LOGGER(true, LOGG_FATAL, "Client network is not ending will terminate anyway")
            }
        }
    }

    //Join nudge thread
    lDeadLock = 10;
    while (mNudgeThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
        if (!--lDeadLock) {
            KCP_LOGGER(true, LOGG_FATAL, "Client nudge thread not ending will terminate anyway")
        }
    }
    if (mKCP) ikcp_release(mKCP);
    KCP_LOGGER(false,LOGG_NOTIFY,"KCPNetClient Destruct")
}

int KCPNetClient::sendData(const char* pData, size_t lSize) {
    return ikcp_send(mKCP, pData, lSize);
}

int KCPNetClient::configureKCP(KCPSettings &rSettings) {
    int lResult = 0;
    lResult = ikcp_nodelay(mKCP, rSettings.mNodelay, rSettings.mInterval, rSettings.mResend, rSettings.mFlow);
    if (lResult) {
        KCP_LOGGER(false,LOGG_ERROR,"ikcp_nodelay client failed.")
        return lResult;
    }
    lResult = ikcp_setmtu(mKCP, rSettings.mMtu);
    if (lResult) {
        KCP_LOGGER(false,LOGG_ERROR,"ikcp_setmtu client failed.")
        return lResult;
    }
    return lResult;
}

void KCPNetClient::kcpNudgeWorkerClient() {
    mNudgeThreadRunning = true;
    mNudgeThreadActive = true;
    uint32_t lTimeSleep = 10;
    uint64_t lTimeBase = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    while (mNudgeThreadActive) {
        std::this_thread::sleep_for(std::chrono::milliseconds(lTimeSleep));
        uint64_t lTimeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - lTimeBase;
        ikcp_update(mKCP, lTimeNow);  //KCP should consider uint64_t as time interface
        lTimeSleep = ikcp_check(mKCP, lTimeNow) - lTimeNow;
        //KCP_LOGGER(false, LOGG_NOTIFY,"k " << lTimeSleep << " " << lTimeNow)
    }
    KCP_LOGGER(false, LOGG_NOTIFY,"kcpNudgeWorkerClient quitting")
    mNudgeThreadRunning = false;
}

void KCPNetClient::netWorkerClient() {
    mNetworkThreadRunning = true;
    kissnet::buffer<MAX_BYTES_RCV> receiveBuffer;
    char lBuffer[MAX_BYTES_RCV];
    while (true) {
        auto[received_bytes, status] = mKissnetSocket.recv(receiveBuffer);
        if (!received_bytes || status != kissnet::socket_status::valid) {
            KCP_LOGGER(false, LOGG_NOTIFY,"netWorkerClient quitting")
            break;
        }
        ikcp_input(mKCP, (const char *)receiveBuffer.data(), received_bytes);
        int lRcv = ikcp_recv(mKCP,&lBuffer[0], MAX_BYTES_RCV);
        if (lRcv>0 && mGotDataClient) {
            mGotDataClient(&lBuffer[0], lRcv, mCTX.get());
        } //Else deal with code?

    }
    mNetworkThreadRunning = false;
}


//------------------------------------------------------------------------------------------
//
// KCP Server
//
//------------------------------------------------------------------------------------------

int udp_output_server(const char *pBuf, int lSize, ikcpcb *pKCP, void *pCTX)
{
   KCPNetServer::KCPServerData* lWeakSelf = (KCPNetServer::KCPServerData*)pCTX;
    if (lWeakSelf) {
        if (lWeakSelf->mWeakKCPNetServer) {
            lWeakSelf->mWeakKCPNetServer->udpOutputServer(pBuf, lSize, lWeakSelf);
        } else {
            KCP_LOGGER(true,LOGG_FATAL,"udp_output_server failed getting 'this'")
        }
    } else {
        KCP_LOGGER(true,LOGG_FATAL,"udp_output_server failed getting KCPServerData")
        return -1; //Throw
    }
    return 0;
}

void KCPNetServer::udpOutputServer(const char *pBuf, int lSize, KCPServerData* lCTX) {
    auto[lSentBytes, lStatus] = lCTX->mSocket.send((const std::byte *) pBuf, lSize);
    if (lSentBytes != lSize || lStatus != kissnet::socket_status::valid) {
        KCP_LOGGER(false, LOGG_NOTIFY,"Server failed sending data")
    }
}

KCPNetServer::KCPNetServer(std::string lIP, uint16_t lPort, std::shared_ptr<KCPContext> pCTX) {
    mCTX = std::move(pCTX);
    if (!lIP.size()) {
        KCP_LOGGER(true,LOGG_FATAL,"IP / HOST must be provided")
        throw std::invalid_argument( "IP / HOST must be provided" );
    }
    if (!lPort) {
        KCP_LOGGER(true,LOGG_FATAL,"Port must be provided")
        throw std::invalid_argument( "Port must be provided" );
    }
    kissnet::udp_socket lCreateSocket(kissnet::endpoint(lIP, lPort));
    mKissnetSocket = std::move(lCreateSocket); //Move ownership to this/me
    mKissnetSocket.bind();
    std::thread(std::bind(&KCPNetServer::netWorkerServer, this)).detach();
    std::thread(std::bind(&KCPNetServer::kcpNudgeWorkerServer, this)).detach();
    KCP_LOGGER(false,LOGG_NOTIFY,"KCPNetServer Constructed")
}

KCPNetServer::~KCPNetServer() {
    uint32_t lDeadLock;
    //Signal close netWorker and nudge thread
    mKissnetSocket.close(); //End net thread
    mNudgeThreadActive = false; //End nudge thread
    //Join network thread
    if (mNetworkThreadRunning) {
        lDeadLock = 10;
        while (mNetworkThreadRunning ) {
            std::this_thread::sleep_for(std::chrono::milliseconds (100));
            if (!--lDeadLock) {
                KCP_LOGGER(true, LOGG_FATAL, "Server net thread is not ending will terminate anyway")
            }
        }
    }

    //Join nudge thread
    lDeadLock = 10;
    while (mNudgeThreadRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
        if (!--lDeadLock) {
            KCP_LOGGER(true, LOGG_FATAL, "Nudge thread not ending will terminate anyway")
        }
    }
    KCP_LOGGER(false,LOGG_NOTIFY,"KCPNetServer Destruct")
}

int KCPNetServer::sendData(const char* pData, size_t lSize, KCPContext* pCTX) {
    int lStatus = -1;
    if (mKCPMap.count(pCTX->mKCPSocket)) {
        lStatus = ikcp_send(mKCPMap[pCTX->mKCPSocket]->mKCPServer, pData, lSize);
    } else {
        KCP_LOGGER(false,LOGG_NOTIFY,"KCP Connection is unknown")
    }
    return lStatus;
}

int KCPNetServer::configureKCP(KCPSettings &rSettings, KCPContext* pCTX) {
    int lResult = 0;
    if (mKCPMap.count(pCTX->mKCPSocket)) {
        lResult = ikcp_nodelay(mKCPMap[pCTX->mKCPSocket]->mKCPServer, rSettings.mNodelay, rSettings.mInterval, rSettings.mResend, rSettings.mFlow);
        if (lResult) {
            KCP_LOGGER(false,LOGG_ERROR,"ikcp_nodelay client failed.")
            return lResult;
        }
        lResult = ikcp_setmtu(mKCPMap[pCTX->mKCPSocket]->mKCPServer, rSettings.mMtu);
        if (lResult) {
            KCP_LOGGER(false,LOGG_ERROR,"ikcp_setmtu client failed.")
            return lResult;
        }
    } else {
        KCP_LOGGER(false,LOGG_NOTIFY,"KCP Connection is unknown")
    }
    return lResult;
}

//For now the server is updating all connections every 10ms
void KCPNetServer::kcpNudgeWorkerServer() {
    mNudgeThreadRunning = true;
    mNudgeThreadActive = true;
    uint32_t lTimeSleep = 10;
    uint64_t lTimeBase = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    while (mNudgeThreadActive) {
        std::this_thread::sleep_for(std::chrono::milliseconds(lTimeSleep));
        uint64_t lTimeNow = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count() - lTimeBase;
        if (mKCPMap.size()) {
            for (const auto &rKCP: mKCPMap) {
                ikcp_update(rKCP.second->mKCPServer, lTimeNow);
            }
        }
    }
    KCP_LOGGER(false, LOGG_NOTIFY,"kcpNudgeWorker quitting")
    mNudgeThreadRunning = false;
}

void KCPNetServer::netWorkerServer() {
    mNetworkThreadRunning = true;
    kissnet::buffer<MAX_BYTES_RCV> receiveBuffer;
    char lBuffer[MAX_BYTES_RCV];
    while (true) {
        auto[received_bytes, status] = mKissnetSocket.recv(receiveBuffer);
        if (!received_bytes || status != kissnet::socket_status::valid) {
            KCP_LOGGER(false, LOGG_NOTIFY,"serverWorker quitting")
            break;
        }

        //Who did send me data? Generate a unique key where (ip:port) a.b.c.d:e becomes a (broken down to uiny8_t) uint64_t 00abcdee
        kissnet::endpoint lFromWho = mKissnetSocket.get_recv_endpoint();
        std::stringstream lS(lFromWho.address);
        int lA, lB, lC, lD; //to store the 4 ints from the ip string
        char lCh; //to temporarily store the '.'
        //or + shift it all together
        lS >> lA >> lCh >> lB >> lCh >> lC >> lCh >> lD;
        uint64_t lKey = (uint64_t) lA << 40 | (uint64_t) lB << 32 | (uint64_t) lC << 24 | (uint64_t) lD << 16 |
                        (uint64_t) lFromWho.port;

        if (!mKCPMap.count(lKey)) {
            KCP_LOGGER(false, LOGG_NOTIFY,"New server connection")
            std::shared_ptr<KCPContext> lx = std::make_shared<KCPContext>(lKey);
            if (mValidateConnectionCallback) {
                auto lCTX = mValidateConnectionCallback(lFromWho.address, lFromWho.port, lx);
                if (lCTX == nullptr) {
                    //The connection was not accepted
                    KCP_LOGGER(false, LOGG_NOTIFY,"Connection rejected")
                    continue;
                }
            }
            //Create the connection and give RCP the data
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
            configureKCP(lx->mSettings, lx.get());
            ikcp_input(mKCPMap[lKey]->mKCPServer,(const char *) receiveBuffer.data(), received_bytes);
            int lRcv = ikcp_recv(mKCPMap[lKey]->mKCPServer, &lBuffer[0], MAX_BYTES_RCV);
            if (lRcv > 0 && mGotDataServer) {
                mGotDataServer(&lBuffer[0], lRcv, lx.get());
            }
            //The connection is known pass the data to RCP
        } else {
            ikcp_input(mKCPMap[lKey]->mKCPServer,(const char *) receiveBuffer.data(), received_bytes);
            int lRcv = ikcp_recv(mKCPMap[lKey]->mKCPServer, &lBuffer[0], MAX_BYTES_RCV);
            if (lRcv > 0 && mGotDataServer) {
                mGotDataServer(&lBuffer[0], lRcv, mKCPMap[lKey]->mKCPContext.get());
            }
        }
    }
    mNetworkThreadRunning = false;
}