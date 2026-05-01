cmake_minimum_required(VERSION 3.22)
project(open_raw_radar_cli)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPEN_RAW_RADAR_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(RADAR_DRIVER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../ros_workspace/src/radar_driver_cpp)
set(RADAR_INCLUDE_ROOT ${RADAR_DRIVER_ROOT}/include)

if(WIN32)
    set(PLATFORM_WZ_SERIALPORT_SOURCE
        ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/WzSerialportPlus/Windows/WzSerialportPlus.cpp
    )
    set(PLATFORM_WZ_SERIALPORT_INCLUDE
        ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/WzSerialportPlus/Windows
    )
    set(PLATFORM_FTDI_INCLUDE
        ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/FTDI_D2XX/Windows
    )
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(PLATFORM_FTDI_LIB
            ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/FTDI_D2XX/Windows/AMD64/ftd2xx.lib
        )
    else()
        set(PLATFORM_FTDI_LIB
            ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/FTDI_D2XX/Windows/x86/ftd2xx.lib
        )
    endif()
    set(PLATFORM_OUTPUT_OWNER_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/output_owner_windows.cpp
    )
    set(PLATFORM_SYSTEM_LIBS ws2_32 ${PLATFORM_FTDI_LIB})
else()
    set(PLATFORM_WZ_SERIALPORT_SOURCE
        ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/WzSerialportPlus/Linux/WzSerialportPlus.cpp
    )
    set(PLATFORM_WZ_SERIALPORT_INCLUDE
        ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/WzSerialportPlus/Linux
    )
    set(PLATFORM_FTDI_INCLUDE
        ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/FTDI_D2XX/Linux
    )
    set(PLATFORM_OUTPUT_OWNER_SOURCE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/output_owner_linux.cpp
    )
    set(PLATFORM_SYSTEM_LIBS ftd2xx pthread)
endif()

file(GLOB_RECURSE MMWAVE_SINGLECHIP_SRCS
    "${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/*.cpp"
    "${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example/*.c"
)
file(GLOB_RECURSE MMWAVE_LINK_SRCS
    "${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/ti/control/mmwavelink/src/*.c"
)

set(VENDOR_SOURCES
    ${PLATFORM_WZ_SERIALPORT_SOURCE}
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/FTDILib/SourceCode/mmwl_port_ftdi.cpp
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/pevents/pevents.cpp
    ${MMWAVE_SINGLECHIP_SRCS}
    ${MMWAVE_LINK_SRCS}
)

set(OPEN_RAW_RADAR_COMMON_SOURCES
    ${PLATFORM_OUTPUT_OWNER_SOURCE}
    ${VENDOR_SOURCES}
)

add_executable(open_raw_radar_capture
    main.cpp
    src/capture_app.cpp
    src/capture_config.cpp
    src/capture_session.cpp
    ${OPEN_RAW_RADAR_COMMON_SOURCES}
)

add_executable(open_raw_radar_firmware_download
    firmware_download.cpp
    ${OPEN_RAW_RADAR_COMMON_SOURCES}
)

set(OPEN_RAW_RADAR_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RADAR_INCLUDE_ROOT}
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/ti/example/mmWaveLink_SingleChip_NonOS_Example
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/ti/control/mmwavelink/include
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/ti/control/mmwavelink
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/FTDILib/SourceCode
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/mmwaveDFP_2G/firmware
    ${PLATFORM_WZ_SERIALPORT_INCLUDE}
    ${PLATFORM_FTDI_INCLUDE}
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/pevents
    ${RADAR_INCLUDE_ROOT}/radar_driver_cpp/kfifo
)

foreach(target_name open_raw_radar_capture open_raw_radar_firmware_download)
    target_include_directories(${target_name} PRIVATE ${OPEN_RAW_RADAR_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${PLATFORM_SYSTEM_LIBS})
    target_compile_definitions(${target_name} PRIVATE
        OPEN_RAW_RADAR_REPO_ROOT="${OPEN_RAW_RADAR_REPO_ROOT}"
        OPEN_RAW_RADAR_CONFIG_ROOT="${OPEN_RAW_RADAR_REPO_ROOT}/configs"
    )
endforeach()
