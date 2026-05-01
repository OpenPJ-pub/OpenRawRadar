#pragma once
#include "mmw_example_nonos.h"

constexpr uint8_t deviceMap = RL_DEVICE_MAP_CASCADED_1;

inline int AWR2243_firmwareDownload() {
    return MMWL_App_firmwareDownload(deviceMap);
}

inline int AWR2243_init(const std::string& configFilename) {
    return MMWL_App_init(deviceMap, configFilename.c_str(), false);
}

inline int AWR2243_setFrameCfg(const int numFrames) {
    return MMWL_App_setFrameCfg(deviceMap, numFrames);
}

inline int AWR2243_sensorStart() {
    return MMWL_App_startSensor(deviceMap);
}

inline bool AWR2243_isSensorStarted() {
    return MMWL_App_isSensorStarted();
}

inline int AWR2243_waitSensorStop() {
    return MMWL_App_waitSensorStop(deviceMap);
}

inline int AWR2243_sensorStop() {
    return MMWL_App_stopSensor(deviceMap);
}

inline int AWR2243_poweroff() {
    return MMWL_App_poweroff(deviceMap);
}
