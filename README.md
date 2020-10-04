# KCP - CPP

![alt text](kcp_cpp_logo.png)

*Simple C++ wrapper of the [KCP](https://github.com/skywind3000/kcp) protocol*.

![alt text](kcp.svg)

Work in progress

## Build

Requires cmake version >= **3.10** and **C++17**

**Release:**

```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release
```

***Debug:***

```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . --config Debug
```


*Output:*

**(platform specific)kcpnet.(platform specific)**

(Linux/MacOS -> libkcpnet.a)

(Windows -> kcpnet.lib)


## Current build status

[![kcpcpp_ubuntu](https://github.com/Unit-X/kcp-cpp/workflows/kcpcpp_ubuntu/badge.svg)](https://github.com/Unit-X/kcp-cpp/actions?query=workflow%3Akcpcpp_ubuntu)

[![kcpcpp_macos](https://github.com/Unit-X/kcp-cpp/workflows/kcpcpp_macos/badge.svg)](https://github.com/Unit-X/kcp-cpp/actions?query=workflow%3Akcpcpp_macos)

[![kcpcpp_win](https://github.com/Unit-X/kcp-cpp/workflows/kcpcpp_win/badge.svg)](https://github.com/Unit-X/kcp-cpp/actions?query=workflow%3Akcpcpp_win)


## Usage

```cpp

//----------
//Server ---
//----------

//Create the server 
//1. Listening interface
//2. Listening port
//3. Optional context
KCPNetServer lKcpServer ( "127.0.0.1", 8000, nullptr);

//Register callbacks

//Callback when a new client connects
lKcpServer.mValidateConnectionCallback = std::bind(&validateConnection, std::placeholders::_1, std::placeholders::_2,
                                             std::placeholders::_3);

//Callback when receiving data from the client
lKcpServer.mGotDataServer = std::bind(&gotDataServer, std::placeholders::_1, std::placeholders::_2,
                                   std::placeholders::_3);

//Optional callback when client disconnects
lKcpServer.mNoConnectionServer = std::bind(&noConnectionServer, std::placeholders::_1);

//Send data to the client
//1. Pointer to the data
//2. The size of the data
//3. The pointer to the KCPContext you got when accepting the client
lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());



//----------
//Client ---
//----------


//Create the client 
//1. Connect to interface
//2. Connect to port
//3. Connection ID (Must be set identical on the server see -> validateConnection)
//4. Optional context
KCPNetClient lKcpClient ( "127.0.0.1", 8000, 10, nullptr);
    
//Callback when receiving data from the server
lKcpClient.mGotDataClient = std::bind(&gotDataClient, std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3);

//Optional callback when server disconnects
lKcpClient.mNoConnectionClient = std::bind(&noConnectionClient, std::placeholders::_1);

//Send data to the server
//1. Pointer to the data
//2. The size of the data
lKcpClient.sendData((const char*)lData.data(), 4000);


//-------------------------------
//Configuration server/client ---
//-------------------------------

KCPSettings lSettingsClient;
if(client or server.configureKCP(lSettingsClient)) {
    std::cout << "Failed configuring KCP" << std::endl;
}


//Please see KCP documentation for details.
class KCPSettings {
public:
    bool mNodelay = false;  //No delay mode. False: Off / True: On.
    int  mInterval = 100;   //KCP update interval in ms
    int  mResend = 0;       //Retransmit when missed mResend number ACK (Default value is 0)
    bool mFlow = false;     //Flow control, False: Off / True: On.
    int  mMtu = 1472;       //Maximum payload in a single UDP datagram
    int  mSndWnd = 32;      //Send window size
    int  mRcvWnd = 32;      //Receive window size //The doc says 32 the code says 128
};


```

## Using KCP - CPP in your CMake project

* **Step1** 

Add this in your CMake file.

```
#Include kcpnet
include(ExternalProject)
ExternalProject_Add(project_ kcpnet
        GIT_REPOSITORY https://github.com/Unit-X/kcp-cpp
        GIT_SUBMODULES ""
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet
        BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet
        GIT_PROGRESS 1
        BUILD_COMMAND cmake --build ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet --config ${CMAKE_BUILD_TYPE} --target kcpnet
        STEP_TARGETS build
        EXCLUDE_FROM_ALL TRUE
        INSTALL_COMMAND ""
        )
add_library(kcpnet STATIC IMPORTED)
set_property(TARGET kcpnet PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/libkcpnet.a)
add_dependencies(kcpnet project_ kcpnet)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/)
```

* **Step2**

Link your library or executable.

```
target_link_libraries((your target) kcpnet kcp (the rest you want to link)) 
```

* **Step3** 

Add header file to your project.

```
#include "kcpnet.h"
```


## Credits

Anders Cedronius for creating the C++ wrapper

anders.cedronius(at)edgeware.tv


## License

*MIT*

Read *LICENCE* for details
