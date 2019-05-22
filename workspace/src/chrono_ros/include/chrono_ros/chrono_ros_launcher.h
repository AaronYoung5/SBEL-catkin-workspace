#pragma once

// Chrono includes
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/core/ChStream.h>
#include <chrono/utils/ChUtilsInputOutput.h>

#include <chrono_models/vehicle/sedan/Sedan.h>
#include <chrono_vehicle/ChConfigVehicle.h>
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/driver/ChDataDriver.h>
#include <chrono_vehicle/driver/ChIrrGuiDriver.h>
#include <chrono_vehicle/driver/ChRosDriver.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h>

// #include "chrono_ros/chrono_ros_driver.h"

// Namespace declarations
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

class ChronoRosLauncher {
private:
  // Private class variables
  double m_throttle, m_steering, m_braking;
  double m_throttle_delta, m_steering_delta, m_braking_delta;
  double m_throttle_gain, m_steering_gain, m_braking_gain;
  double m_throttle_target, m_steering_target, m_braking_target;

  ChWheeledVehicleIrrApp *m_app;

  Sedan m_veh;

  RigidTerrain *m_terrain;

  // ChIrrGuiDriver *m_driver;
  ChRosDriver *m_driver;

  // Simulation step size and end time
  double m_step_size = 1e-3;
  double m_t_end = 1000;

  // Time interval between two render frames
  double m_render_step_size = 1.0 / 50;

  // Number of simulation steps between miscellaneous events
  int m_render_steps = (int)std::ceil(m_render_step_size / m_step_size);

  // Initialize simulation frame counter and simulation time
  double m_time = 0;
  int m_render_frame = 0;
  int m_step_number = 0;
  ChRealtimeStepTimer m_realtime_timer;

public:
  // Public methods
  ChronoRosLauncher();
  void ChronoLoop();
  void ChronoRun();
  bool ChronoIsRunning() { return m_app->GetDevice()->run(); }

  // Getter methods
  // Returns driver
  // ChIrrGuiDriver GetDriver() { return *m_driver; }
  ChRosDriver GetDriver() { return *m_driver; }
  // Returns vehicle irrlicht app
  ChWheeledVehicleIrrApp GetApp() { return *m_app; }
  // Returns vehicle
  Sedan& GetVehicle() { return m_veh; }
  // Returns terrain
  RigidTerrain GetTerrain() { return *m_terrain; }

  void IncreaseThrottle();
  void DecreaseThrottle();
  void IncreaseBraking();
  void DecreaseBraking();
  void TurnRight();
  void TurnLeft();

  // Advance the state of the driver system by the specified time step.
  void Advance(double step);

private:
  // Private methods
  void InitVehicle();
  void InitTerrain();
  void InitIrrlicht();
  void InitDriver();
};
