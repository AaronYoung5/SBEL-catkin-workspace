// Includes
// #include <chrono/core/ChMathematics.h>

#include <chrono_vehicle/driver/ChDataDriver.h>
#include <chrono_vehicle/driver/ChIrrGuiDriver.h>

class ChronoRosDriver {
private:
  // Private class variables
  double m_throttle, m_steering, m_braking;
  double m_throttle_delta, m_steering_delta;

public:
  // Public methods
  ChronoRosDriver();

  // Overrides driver and sets throttle directly
  void SetThrottle(double throttle) { m_throttle = throttle; }
  // Overrides driver and sets steering directly
  void SetSteering(double steering) { m_steering = steering; }
  // Overrides driver and sets braking directly
  void SetBraking(double braking) { m_braking = braking; }

  // Returns throttle position of the driver
  double GetThrottle() { return m_throttle; }
  // Returns steering position of the driver
  double GetSteering() { return m_steering; }
  // Returns braking position of the driver
  double getBraking() { return m_braking; }
private:
  // Private methods
  void increaseThrottle();

};
