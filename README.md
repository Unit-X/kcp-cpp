# KCP - CPP

![alt text](kcp_cpp_logo.png)

**Simple C++ wrapper of the [KCP](https://github.com/skywind3000/kcp) protocol.**

[![alt text](kcp.svg)](https://github.com/skywind3000/kcp)

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
//1. Data from server callback
//2. Client disconnect callback
//3. Validate new client callback
//4. Listening interface
//5. Listening port
//6. Optional context
KCPNetServer lKcpServer (gotDataServer,
                         noConnectionServer,
                         validateConnection,
                         "127.0.0.1",
                         8000,
                         nullptr);


//Send data to the client
//1. Pointer to the data
//2. The size of the data
//3. The pointer to the KCPContext you got when accepting the client
lKcpServer.sendData((const char*)lData.data(), 4000, gRetainThis.get());



//----------
//Client ---
//----------

//Create the client 
//1. Got data from server
//2. Lost connection to server
//3. Connect to interface
//4. Connect to port
//5. Connection ID (Must be set identical on the server see -> validateConnection)
//6. Optional context
KCPNetClient lKcpClient (gotDataClient,
                         noConnectionClient,
                         "127.0.0.1",
                         8000,
                         10,
                         nullptr);

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
ExternalProject_Add(project_kcpnet
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
add_library(kcp STATIC IMPORTED)
IF (WIN32)
    set_property(TARGET kcpnet PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/${CMAKE_BUILD_TYPE}/kcpnet.lib)
    set_property(TARGET kcpnet PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/kcp/${CMAKE_BUILD_TYPE}/kcp.lib)
ELSE()
    set_property(TARGET kcpnet PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/libkcpnet.a)
    set_property(TARGET kcp PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/kcp/libkcp.a)
ENDIF()

add_dependencies(kcpnet project_kcpnet)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kcpnet/kcp/)
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
