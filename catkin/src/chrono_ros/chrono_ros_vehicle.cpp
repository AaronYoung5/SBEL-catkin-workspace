// Includes
#include "chrono_ros_vehicle.h"

ChronoRosVehicle::ChronoRosVehicle() {
  // Set path to Chrono data directory
  SetChronoDataPath(CHRONO_DATA_DIR);

  InitVehicle();
  InitTerrain();
  InitIrrlicht();
  InitOutput();
}

void ChronoRosVehicle::ChronoLoop() {}

ChronoRosVehicle::InitVehicle() {
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

  m_veh.SetContactMethod(contact_method);
  m_veh.SetChassisCollisionType(chassis_collision_type);
  m_veh.SetChassisFixed(false);
  m_veh.SetInitPosition(ChCoordsys<>(initLoc, initRot));
  // m_veh.SetPowertrainType(powertrain_model);
  // m_veh.SetDriveType(drive_type);
  m_veh.SetTireType(tire_model);
  m_veh.SetTireStepSize(tire_step_size);
  m_veh.SetVehicleStepSize(step_size);
  m_veh.Initialize();

  VisualizationType tire_vis_type =
      VisualizationType::MESH; // : VisualizationType::PRIMITIVES;

  m_veh.SetChassisVisualizationType(chassis_vis_type);
  m_veh.SetSuspensionVisualizationType(suspension_vis_type);
  m_veh.SetSteeringVisualizationType(steering_vis_type);
  m_veh.SetWheelVisualizationType(wheel_vis_type);
  m_veh.SetTireVisualizationType(tire_vis_type);
}

ChronoRosVehicle::InitTerrain() {
  // Rigid terrain
  RigidTerrain::Type terrain_model = RigidTerrain::BOX;
  double terrainHeight = 0;     // terrain height (FLAT terrain only)
  double terrainLength = 100.0; // size in X direction
  double terrainWidth = 100.0;  // size in Y direction

  // Create the terrain
  terrain = new RigidTerrain(m_veh.GetSystem());

  std::shared_ptr<RigidTerrain::Patch> patch;
  switch (terrain_model) {
  case RigidTerrain::BOX:
    patch = terrain.AddPatch(
        ChCoordsys<>(ChVector<>(0, 0, terrainHeight - 5), QUNIT),
        ChVector<>(terrainLength, terrainWidth, 10));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200,
                      200);
    break;
  case RigidTerrain::HEIGHT_MAP:
    patch = terrain.AddPatch(
        CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
        "test64", 128, 128, 0, 4);
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16,
                      16);
    break;
  case RigidTerrain::MESH:
    patch = terrain.AddPatch(
        CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100,
                      100);
    break;
  }
  patch->SetContactFrictionCoefficient(0.9f);
  patch->SetContactRestitutionCoefficient(0.01f);
  patch->SetContactMaterialProperties(2e7f, 0.3f);
  patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
  terrain.Initialize();
}

ChronoRosVehicle::InitIrrlicht() {
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 0.0, 1.75);

  // Create the vehicle Irrlicht interface
  app = new ChWheeledVehicleIrrApp(&m_veh.GetVehicle(),
                                   &m_veh.GetPowertrain(), L"ROS Sedan Demo");
  app.SetSkyBox();
  app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f),
                        irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
  app.SetChaseCamera(trackPoint, 6.0, 0.5);
  app.SetTimestep(step_size);
  app.AssetBindAll();
  app.AssetUpdateAll();
}

ChronoRosVehicle::InitOutput() {}
