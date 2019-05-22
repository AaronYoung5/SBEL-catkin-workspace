// Includes
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/core/ChStream.h>
#include <chrono/utils/ChUtilsInputOutput.h>

#include <chrono_models/vehicle/sedan/Sedan.h>
#include <chrono_vehicle/ChConfigVehicle.h>
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/utils/ChVehiclePath.h>
#include <chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h>

class ChronoRosVehicle {
private:
  // Private class variables
  ChronoRosDriver m_chrono_ros_driver;

  ChWheeledVehicleIrrApp *m_app;

  Sedan m_veh;

  RigidTerrain m_terrain;

  // ChIrrGuiDriver *m_driver;

  // Simulation step size and end time
  double m_step_size = 1e-3;
  double m_t_end = 1000;

  // Time interval between two render frames
  double m_render_step_size = 1.0 / 50;

public:
  // Public methods
  ChronoRosVehicle();
  void ChronoLoop();

  // Getter methods
  // Returns driver
  ChronoRosDriver GetDriver() { return &m_chrono_ros_driver; }
  // Returns vehicle irrlicht app
  ChWheeledVehicleIrrApp GetApp() { return &*m_app;}
  // Returns vehicle
  Sedan GetVehicle() { return &m_veh; }
  // Returns terrain
  RigidTerrain GetTerrain() { return &*m_terrain; }

private:
  // Private methods
  void InitVehicle();
  void InitTerrain();
  void InitIrrlicht();
  void InitOutput();
};
