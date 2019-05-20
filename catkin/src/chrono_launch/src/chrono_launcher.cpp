// ROS includes
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"

// Chrono includes
#include <chrono/core/ChRealtimeStep.h>
#include <chrono/core/ChStream.h>
#include <chrono/utils/ChUtilsInputOutput.h>

#include <chrono_vehicle/ChConfigVehicle.h>
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/driver/ChDataDriver.h>
#include <chrono_vehicle/driver/ChIrrGuiDriver.h>
#include <chrono_vehicle/terrain/RigidTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h>
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono/geometry/ChLineBezier.h"

#include <chrono_models/vehicle/sedan/Sedan.h>

#include <chrono_thirdparty/filesystem/path.h>

// Other includes
#include <signal.h>

// Namespace declarations
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

// Definition declarations
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class ChronoVehicleLauncher {
private:
  // double steering, braking;
  double throttle, steering, braking;
  double throttleDelta, steeringDelta;

  ros::NodeHandle n;
  ros::Subscriber sub;

  ros::Publisher gps_pub;
  ros::Publisher steering_pub;
  ros::Publisher throttle_pub;

  ChWheeledVehicleIrrApp *app;

  Sedan my_veh;

  RigidTerrain *terrain;

  ChIrrGuiDriver *driver;

  // Simulation step size and end time
  double step_size = 1e-3;
  double t_end = 1000;

  // Debug logging
  bool debug_output = false;
  double debug_step_size = 1.0 / 1;

  // POV-Ray output
  bool povray_output = false;

  // Time interval between two render frames
  double render_step_size = 1.0 / 50;

  // Output directories
  // todo:: add vehicle type to change Sedan with
  const std::string out_dir = GetChronoOutputPath() + "Sedan";
  const std::string pov_dir = out_dir + "/POVRAY";
  std::string driver_file = out_dir + "/driver_inputs.txt";
  utils::CSV_writer driver_csv;

  enum DriverMode { DEFAULT, RECORD, PLAYBACK };
  DriverMode driver_mode = DEFAULT;

public:
  ChronoVehicleLauncher();
  void loop();

  void setThrottle(double throttle) { this->throttle = throttle; }
  void setSteering(double steering) { this->steering = steering; }
  void setBraking(double braking) { this->braking = braking; }

  void updateDriver(const std_msgs::Int8::ConstPtr &msg);

private:
  void initVehicle();
  void initTerrain();
  void initIrrlicht();
  void initOutput();
  void initDriver();

  void increaseThrottle() { throttle = clamp(throttle + throttleDelta, -1, 1); }
  void decreaseThrottle() { throttle = clamp(throttle - throttleDelta, -1, 1); }

  void turnRight() { steering = clamp(steering + steeringDelta, -1, 1); }
  void turnLeft() { steering = clamp(steering - steeringDelta, -1, 1); }

  double clamp(double value, double min, double max) {
    return value < max ? value > min ? value : min : max;
  }
};

ChronoVehicleLauncher::ChronoVehicleLauncher() {
  steering = 0;
  braking = 0;

  sub = n.subscribe<std_msgs::Int8>("key_msgs", 1000,
                                    &ChronoVehicleLauncher::updateDriver, this);

  gps_pub = n.advertise<std_msgs::Float32MultiArray>("veh_gps", 1000);
  // sub = n.subscribe<std_msgs::Int8>("key_msgs", 1000,
  // &ChronoVehicleLauncher::callback, this);

  // Set path to Chrono data directory
  SetChronoDataPath(CHRONO_DATA_DIR);

  GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: "
           << CHRONO_VERSION << "\n\n";

  initVehicle();
  initTerrain();
  initIrrlicht();
  initOutput();
  initDriver();
}

void ChronoVehicleLauncher::initVehicle() {
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
  double tire_step_size = step_size;

  // Initial vehicle location and orientation
  ChVector<> initLoc(0, 0, 1.0); // x, y, z
  ChQuaternion<> initRot(1, 0, 0, 0);

  my_veh.SetContactMethod(contact_method);
  my_veh.SetChassisCollisionType(chassis_collision_type);
  my_veh.SetChassisFixed(false);
  my_veh.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  // my_veh.SetPowertrainType(powertrain_model);
  // my_veh.SetDriveType(drive_type);
  my_veh.SetTireType(tire_model);
  my_veh.SetTireStepSize(tire_step_size);
  my_veh.SetVehicleStepSize(step_size);
  my_veh.Initialize();

  VisualizationType tire_vis_type =
      VisualizationType::MESH; // : VisualizationType::PRIMITIVES;

  my_veh.SetChassisVisualizationType(chassis_vis_type);
  my_veh.SetSuspensionVisualizationType(suspension_vis_type);
  my_veh.SetSteeringVisualizationType(steering_vis_type);
  my_veh.SetWheelVisualizationType(wheel_vis_type);
  my_veh.SetTireVisualizationType(tire_vis_type);
}

void ChronoVehicleLauncher::initTerrain() {
  // Rigid terrain
  RigidTerrain::Type terrain_model = RigidTerrain::BOX;
  double terrainHeight = 0;     // terrain height (FLAT terrain only)
  double terrainLength = 100.0; // size in X direction
  double terrainWidth = 100.0;  // size in Y direction

  // Create the terrain
  terrain = new RigidTerrain(my_veh.GetSystem());

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (terrain_model) {
  case RigidTerrain::BOX:
    patch = terrain->AddPatch(
        ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
        ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200,
                      200);
    break;
  case RigidTerrain::HEIGHT_MAP:
    patch = terrain->AddPatch(
        CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
        "test64", 128, 128, 0, 4);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16,
                      16);
    break;
  case RigidTerrain::MESH:
    patch = terrain->AddPatch(
        CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100,
                      100);
    break;
  }
  patch->SetContactFrictionCoefficient(0.9f);
  patch->SetContactRestitutionCoefficient(0.01f);
  patch->SetContactMaterialProperties(2e7f, 0.3f);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
  terrain->Initialize();
}

void ChronoVehicleLauncher::initIrrlicht() {
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 0.0, 1.75);

  // Create the vehicle Irrlicht interface
  app = new ChWheeledVehicleIrrApp(&my_veh.GetVehicle(),
                                   &my_veh.GetPowertrain(), L"Sedan Demo");
  app->SetSkyBox();
  app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f),
                        irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
  app->SetChaseCamera(trackPoint, 6.0, 0.5);
  app->SetTimestep(step_size);
  app->AssetBindAll();
  app->AssetUpdateAll();
}

void ChronoVehicleLauncher::initOutput() {

  if (!filesystem::create_directory(filesystem::path(out_dir))) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return;
  }
  if (povray_output) {
    if (!filesystem::create_directory(filesystem::path(pov_dir))) {
      std::cout << "Error creating directory " << pov_dir << std::endl;
      return;
    }
    terrain->ExportMeshPovray(out_dir);
  }
}

void ChronoVehicleLauncher::initDriver() {

  // Create the interactive driver system
  driver = new ChIrrGuiDriver(*app);

  // Set the time response for steering andChronoVehicleLauncher::throttle
  // keyboard inputs
  double steering_time = 1.0; // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0; // time to go from 0 to +1
  double braking_time = 0.3;  // time to go from 0 to +1
  driver->SetSteeringDelta(render_step_size / steering_time);
  driver->SetThrottleDelta(render_step_size / throttle_time);
  driver->SetBrakingDelta(render_step_size / braking_time);

  this->throttleDelta = render_step_size / throttle_time;
  this->steeringDelta = render_step_size / steering_time;

  // If in playback mode, attach the data file to the driver system and
  // force it to playback the driver inputs.
  if (driver_mode == PLAYBACK) {
    driver->SetInputDataFile(driver_file);
    driver->SetInputMode(ChIrrGuiDriver::DATAFILE);
  }

  driver->Initialize();
}

void ChronoVehicleLauncher::loop() {
  if (debug_output) {
    GetLog() << "\n\n============ System Configuration ============\n";
    my_veh.LogHardpointLocations();
  }

  // output vehicle mass
  std::cout << "VEHICLE MASS: " << my_veh.GetVehicle().GetVehicleMass()
            << std::endl;

  std::string path_file("paths/ISO_double_lane_change.txt");

  // ----------------------
  // Create the Bezier path
  // ----------------------

  // From data file
  auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
  // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(my_veh.GetSystem()->NewBody());
    road->SetBodyFixed(true);

const std::string m_pathName = "my_path";
    auto bezier_curve = path;
    auto num_points = static_cast<unsigned int>(bezier_curve->getNumPoints());
    auto path_asset = std::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(std::make_shared<geometry::ChLineBezier>(bezier_curve));
    path_asset->SetColor(ChColor(0.0f, 0.8f, 0.0f));
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    road->AddAsset(path_asset);
    my_veh.GetSystem()->AddBody(road);

  // Number of simulation steps between miscellaneous events
  int render_steps = (int)std::ceil(render_step_size / step_size);
  int debug_steps = (int)std::ceil(debug_step_size / step_size);

  // Initialize simulation frame counter and simulation time
  double time = 0;
  int render_frame = 0;
  int step_number = 0;
  ChRealtimeStepTimer realtime_timer;

  while (app->GetDevice()->run()) {
    time = my_veh.GetSystem()->GetChTime();

    // End simulation
    if (time >= t_end)
      break;

    // Render scene and output POV-Ray data
    if (step_number % render_steps == 0) {
      app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      app->DrawAll();
      app->EndScene();

      if (povray_output) {
        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(),
                render_frame + 1);
        utils::WriteShapesPovray(my_veh.GetSystem(), filename);
      }

      render_frame++;
    }

    // Debug logging
    if (debug_output && step_number % debug_steps == 0) {
      GetLog() << "\n\n============== System Information ==============\n";
      GetLog() << "Time = " << time << "\n\n";
      my_veh.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
    }

    // Driver output
    if (driver_mode == RECORD) {
      driver_csv << time << steering << ChronoVehicleLauncher::throttle
                 << braking << std::endl;
    }

    // Collect output data from modules (for inter-module communication)
    // double throttle = driver->GetThrottle();
    // double steering = driver->GetSteering();
    // double braking = driver->GetBraking();

    // std::cout << driver->GetThrottle() << std::endl;
    // std::cout << this->throttle << std::endl;

    // std::cout << typeid(my_veh.GetVehicle().GetVehicleCOMPos().x()).name() <<
    // std::endl; std::cout << my_veh.GetVehicle().G

    ChClamp(1, 1, 1);

    // Update modules (process inputs from other modules)
    // todo the steering,ChronoVehicleLauncher::throttle, brake order is
    // different for these elements
    driver->Synchronize(time);
    terrain->Synchronize(time);
    my_veh.Synchronize(time, steering, braking, throttle, *terrain);
    app->Synchronize(driver->GetInputModeAsString(), steering, throttle,
                     braking);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);
    driver->Advance(step);
    terrain->Advance(step);
    my_veh.Advance(step);
    app->Advance(step);

    // Increment frame number
    step_number++;

    // ROS Implementation
    // GPS data publisher
    std_msgs::Float32MultiArray array;
    // Adds x,y,z to array
    array.data.push_back(my_veh.GetVehicle().GetVehicleCOMPos().x());
    array.data.push_back(my_veh.GetVehicle().GetVehicleCOMPos().y());
    array.data.push_back(my_veh.GetVehicle().GetVehicleCOMPos().z());

    // Velocity data publisher
    // std_msgs::

    // Publish array
    gps_pub.publish(array);

    ros::spinOnce();
  }
}

void ChronoVehicleLauncher::updateDriver(const std_msgs::Int8::ConstPtr &msg) {
  switch (msg->data) {
  case KEYCODE_R:
    // ROS_INFO("RIGHT");
    this->turnRight();
    break;
  case KEYCODE_L:
    // ROS_INFO("LEFT");
    this->turnLeft();
    break;
  case KEYCODE_U:
    // ROS_INFO("UP");
    this->increaseThrottle();
    break;
  case KEYCODE_D:
    // ROS_INFO("DOWN");
    this->decreaseThrottle();
    break;
  }
}

void quit(int sig) {
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono_launcher");
  ChronoVehicleLauncher launcher;

  signal(SIGINT, quit);
  launcher.loop();
}
