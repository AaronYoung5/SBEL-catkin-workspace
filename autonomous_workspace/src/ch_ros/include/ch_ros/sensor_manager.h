#pragma once

#include "publishers.h"

class SensorManager {
private:
public:
  SensorManager();

  template <typename msgtype> void Add(Publisher<msgtype> sensor);
};
