CMAKE_MINIMUM_REQUIRED(VERSION 2.8.2)
PROJECT(common_test)

#SET(CMAKE_CXX_FLAGS_RELEASE "-Wno-unused-result -Wno-narrowing -DCR_DEBUG_FLAG -O3 -Wall -std=gnu++0x")
#SET(CMAKE_CXX_FLAGS_DEBUG "-Wno-unused-result -Wno-narrowing -Wno-unused-variable -Wall -DCR_DEBUG_FLAG -g -std=gnu++0x")
SET(CMAKE_CXX_FLAGS_RELEASE "-Wno-unused-result -Wno-narrowing -DCR_DEBUG_FLAG -O3 -Wall -std=gnu++0x -std=c++11")
SET(CMAKE_CXX_FLAGS_DEBUG "-Wno-unused-result -Wno-narrowing -Wno-unused-variable -Wall -DCR_DEBUG_FLAG -g -std=gnu++0x -std=c++11")

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS "${CMAKE_SOURCE_DIR}")
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")
SET(SCRIPTS_PATH "${PROJECT_ROOT_PATH}/../scripts/")
#SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--export-all-symbols")
#SET(CMAKE_STATIC_LINKER_FLAGS "-Wl,--export-all-symbols")
SET(LINK_FLAGS ${LINK_FLAGS} "-Wl,-whole-archive")

MESSAGE(STATUS ${PROJECT_ROOT_PATH})

#root
INCLUDE_DIRECTORIES("${PROJECT_ROOT_PATH}")
IF ( "${CMAKE_SIZEOF_VOID_P}" EQUAL "4")
    MESSAGE(STATUS "x32 platform")
    SET(PLATFORM x86)
ELSEIF( "${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
    MESSAGE(STATUS "x64 platform")
    SET(PLATFORM x64)
ENDIF()

# opencv package required


file(GLOB_RECURSE CRYPTO_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "crypto/*.c")

MESSAGE(STATUS ${CRYPTO_SRC})

IF (MSVC)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "ANDORID platform")
    ADD_DEFINITIONS(-DANDROID)
    ADD_DEFINITIONS(-DUSE_GLOG)
    ADD_DEFINITIONS(-DUSE_CAMERA)
    ADD_DEFINITIONS(-DANDROID_CAM_RECORD)
    SET(USE_CAN 1)
    ADD_CUSTOM_TARGET(MINIEYE_VERSION COMMAND ${SCRIPTS_PATH}/version.sh |tee ${PROJECT_ROOT_PATH}/version.h.in)
ELSEIF (UNIX)
ENDIF()

#include
IF (WIN32)
ELSEIF (ANDROID_ABI)
    MESSAGE(STATUS "android platform")
    #include directory
    INCLUDE_DIRECTORIES(
        #"${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/jni/include/"
    )

    LINK_DIRECTORIES(
        #"${PROJECT_ROOT_PATH}/thirdparty/opencv/android/sdk/native/libs/arm64-v8a/"
    )

ELSEIF (UNIX)
    SET(OpenCV_DIR "${PROJECT_ROOT_PATH}/thirdparty/opencv/zynq/share/OpenCV")
    SET(OpenCV_STATIC ON)
    FIND_PACKAGE(OpenCV 3 REQUIRED)

    SET(FPGA_ACCELERATE 1)
    ADD_DEFINITIONS(-DFPGA_ACCELERATE=1)

    INCLUDE_DIRECTORIES(
        #"${PROJECT_ROOT_PATH}/thirdparty/msgpack-2.1.5/include/"
    )

    LINK_DIRECTORIES(
        #"${PROJECT_ROOT_PATH}/thirdparty/nanomsg/zynq/lib/"
    )
ENDIF()

#link library
IF (WIN32)
ELSEIF(ANDROID_ABI)
    #SET(SRC_LIST RingBufDebug.cpp CRingBuf.cpp)
    #ADD_LIBRARY(ringbuf STATIC ${SRC_LIST})

    ADD_LIBRARY(atsha204 SHARED atsha204_app.c ${CRYPTO_SRC})
    ADD_LIBRARY(atsha204-static STATIC atsha204_app.c ${CRYPTO_SRC})

    ADD_EXECUTABLE(atshaTest main.c)
    #ADD_DEPENDENCIES(websocket_client MINIEYE_VERSION)
    TARGET_LINK_LIBRARIES(atshaTest atsha204-static
        #        halio concurrency
        #libmsgpackc.a libwebsockets.a dl
        #ringbuf halio concurrency
        #-Wl,--start-group
        # -Wl,--end-group
           )
ELSE()
    #ADD_EXECUTABLE(audioplayer ./audioplayer_linux.cpp)
    #TARGET_LINK_LIBRARIES(audioplayer
    #    halio
    #    -Wl,--start-group
    #    -Wl,--end-group
    #    )

endif()


