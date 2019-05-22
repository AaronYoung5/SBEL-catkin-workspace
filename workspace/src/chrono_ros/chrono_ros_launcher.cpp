#include "chrono_ros/chrono_ros_launcher.h"

// Namespace declarations
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChronoRosLauncher::ChronoRosLauncher()
    : m_throttle(0), m_steering(0), m_braking(0), m_throttle_gain(4.0),
      m_steering_gain(4.0), m_braking_gain(4.0), m_throttle_delta(1.0 / 50),
      m_steering_delta(1.0 / 50), m_braking_delta(1.0 / 50),
      m_throttle_target(0), m_steering_target(0), m_braking_target(0) {
  // Set path to Chrono data directory
  SetChronoDataPath(CHRONO_DATA_DIR);
  // SetDataPath(CHRONO_VEHICLE_DATA_DIR);

  InitVehicle();
  InitTerrain();
  InitIrrlicht();
  InitDriver();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::InitVehicle() {
  // Contact method
  ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
  bool contact_vis = false;

  // Visualization type for the vehicle parts (PRIMITIVES, MESH or NONE)
  VisualizationType chassis_vis_type = VisualizationType::MESH;
  VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
  VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
  VisualizationType wheel_vis_type = VisualizationType::MESH;

  // Collision type for chssis (PRIMITIVES, MESH, or NONE)
  ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE;

  // Type of tire model (RIGID, TMEASY)
  TireModelType tire_model = TireModelType::RIGID;

  // Simulation step sizes
  double tire_step_size = m_step_size;

  // Initial vehicle location and orientation
  ChVector<> initLoc(0, 0, 1.0); // x, y, z
  ChQuaternion<> initRot(1, 0, 0, 0);

  m_veh.SetContactMethod(contact_method);
  m_veh.SetChassisCollisionType(chassis_collision_type);
  m_veh.SetChassisFixed(false);
  m_veh.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  // m_veh.SetPowertrainType(powertrain_model);
  // m_veh.SetDriveType(drive_type);
  m_veh.SetTireType(tire_model);
  m_veh.SetTireStepSize(tire_step_size);
  m_veh.SetVehicleStepSize(m_step_size);
  m_veh.Initialize();

  VisualizationType tire_vis_type =
      VisualizationType::MESH; // : VisualizationType::PRIMITIVES;

  m_veh.SetChassisVisualizationType(chassis_vis_type);
  m_veh.SetSuspensionVisualizationType(suspension_vis_type);
  m_veh.SetSteeringVisualizationType(steering_vis_type);
  m_veh.SetWheelVisualizationType(wheel_vis_type);
  m_veh.SetTireVisualizationType(tire_vis_type);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::InitTerrain() {
  // Rigid terrain
  RigidTerrain::Type terrain_model = RigidTerrain::BOX;
  double terrainHeight = 0;     // terrain height (FLAT terrain only)
  double terrainLength = 100.0; // size in X direction
  double terrainWidth = 100.0;  // size in Y direction

  // Create the terrain
  m_terrain = new RigidTerrain(m_veh.GetSystem());

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (terrain_model) {
  case RigidTerrain::BOX:
    patch = m_terrain->AddPatch(
        ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
        ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200,
                      200);
    break;
  case RigidTerrain::HEIGHT_MAP:
    patch = m_terrain->AddPatch(
        CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
        "test64", 128, 128, 0, 4);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16,
                      16);
    break;
  case RigidTerrain::MESH:
    patch = m_terrain->AddPatch(
        CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100,
                      100);
    break;
  }
  patch->SetContactFrictionCoefficient(0.9f);
  patch->SetContactRestitutionCoefficient(0.01f);
  patch->SetContactMaterialProperties(2e7f, 0.3f);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
  m_terrain->Initialize();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::InitIrrlicht() {
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 0.0, 1.75);

  // Create the vehicle Irrlicht interface
  m_app = new ChWheeledVehicleIrrApp(&m_veh.GetVehicle(),
                                     &m_veh.GetPowertrain(), L"ROS Sedan Demo");
  m_app->SetSkyBox();
  m_app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f),
                          irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
  m_app->SetChaseCamera(trackPoint, 6.0, 0.5);
  m_app->SetTimestep(m_step_size);
  m_app->AssetBindAll();
  m_app->AssetUpdateAll();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::InitDriver() {
  // Create the interactive driver system
  // m_driver = new ChIrrGuiDriver(*m_app);
  m_driver = new ChRosDriver(m_veh.GetVehicle());

  // Set the time response for steering andChronoVehicleLauncher::throttle
  // keyboard inputs
  double steering_time = 1.0; // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0; // time to go from 0 to +1
  double braking_time = 0.3;  // time to go from 0 to +1
  m_driver->SetSteeringDelta(m_render_step_size / steering_time);
  m_driver->SetBrakingDelta(m_render_step_size / braking_time);
  m_driver->SetThrottleDelta(m_render_step_size / throttle_time);

  m_steering_delta = m_render_step_size / steering_time / 10;
  m_throttle_delta = m_render_step_size / throttle_time / 10;

  m_driver->Initialize();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::ChronoRun() {
  if (m_app->GetDevice()->run()) {
    m_time = m_veh.GetSystem()->GetChTime();

    // End simulation
    if (m_time >= m_t_end)
      return;

    // Render scene
    if (m_step_number % m_render_steps == 0) {
      m_app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      m_app->DrawAll();
      m_app->EndScene();

      m_render_frame++;
    }

    // Collect output data from modules (for inter-module communication)
    double m_throttle = m_driver->GetThrottle();
    double m_steering = m_driver->GetSteering();
    double m_braking = m_driver->GetBraking();

    // Update modules (process inputs from other modules)
    // todo the steering,ChronoVehicleLauncher::throttle, brake order is
    // different for these elements
    m_driver->Synchronize(m_time);
    m_terrain->Synchronize(m_time);
    m_veh.Synchronize(m_time, m_steering, m_braking, m_throttle, *m_terrain);
    m_app->Synchronize(m_driver->GetInputModeAsString(), m_steering, m_throttle,
                       m_braking);

    // Advance simulation for one timestep for all modules
    double step = m_realtime_timer.SuggestSimulationStep(m_step_size);
    // this->Advance(step);
    m_driver->Advance(step);
    m_terrain->Advance(step);
    m_veh.Advance(step);
    m_app->Advance(step);

    // Increment frame number
    m_step_number++;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::Advance(double step) {
  // Integrate dynamics, taking as many steps as required to reach the 'step'
  double t = 0;
  while (t < step) {
    double h = std::min<>(m_step_size, step - t);

    double throttle_deriv = m_throttle_gain * (m_throttle_target - m_throttle);
    double steering_deriv = m_steering_gain * (m_steering_target - m_steering);
    double braking_deriv = m_braking_gain * (m_braking_target - m_braking);

    m_throttle += h * throttle_deriv;
    m_steering += h * steering_deriv;
    m_braking += h * braking_deriv;

    // m_driver->SetThrottle(m_throttle + h * throttle_deriv);
    // m_driver->SetSteering(m_steering + h * steering_deriv);
    // m_driver->SetBraking(m_braking + h * braking_deriv);

    m_driver->SetThrottle(m_throttle);
    m_driver->SetSteering(m_steering);
    m_driver->SetBraking(m_braking);

    t += h;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::ChronoLoop() {
  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(m_render_step_size / m_step_size);

  // Initialize simulation frame counter and simulation time
  double time = 0;
  int render_frame = 0;
  int step_number = 0;
  ChRealtimeStepTimer realtime_timer;

  while (m_app->GetDevice()->run()) {
    time = m_veh.GetSystem()->GetChTime();

    // End simulation
    if (time >= m_t_end)
      break;

    // Render scene
    if (step_number % render_steps == 0) {
      m_app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      m_app->DrawAll();
      m_app->EndScene();

      render_frame++;
    }

    // Update modules (process inputs from other modules)
    // todo the steering,ChronoVehicleLauncher::throttle, brake order is
    // different for these elements
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);
    m_veh.Synchronize(time, m_steering, m_braking, m_throttle, *m_terrain);
    m_app->Synchronize(m_driver->GetInputModeAsString(), m_steering, m_throttle,
                       m_braking);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(m_step_size);
    m_driver->Advance(step);
    m_terrain->Advance(step);
    m_veh.Advance(step);
    m_app->Advance(step);
    this->Advance(step);

    // Increment frame number
    step_number++;
  }
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::IncreaseThrottle() {
  m_driver->SpeedUp();
}

// void ChronoRosLauncher::IncreaseThrottle() {
//   m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
//   if (m_throttle_target > 0) {
//     this->DecreaseBraking();
//   }
// }
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::DecreaseThrottle() {
  m_driver->SlowDown();
}

// void ChronoRosLauncher::DecreaseThrottle() {
//   m_throttle_target =
//       ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
//   if (m_throttle_target <= 0) {
//     this->IncreaseBraking();
//   }
// }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::IncreaseBraking() {
  m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::DecreaseBraking() {
  m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::TurnRight() {
  m_driver->TurnRight();
}

// void ChronoRosLauncher::TurnRight() {
//   m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
// }

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void ChronoRosLauncher::TurnLeft() {
  m_driver->TurnLeft();
}

// void ChronoRosLauncher::TurnLeft() {
//   m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
// }
