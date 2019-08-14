#include "ch_ros/sensor_manager.h"

SensorManager::SensorManager() {}

template <typename msgtype>
void SensorManager::Add(Publisher<msgtype> sensor) {}
