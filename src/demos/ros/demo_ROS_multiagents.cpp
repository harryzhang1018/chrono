// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Harry Zhang
// =============================================================================
//
// Demo to show the use of Chrono::Vehicle with ROS
//
// =============================================================================

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include "chrono/core/ChTypes.h"

#include "chrono/physics/ChInertiaUtils.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChBody.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_models/vehicle/artcar/ARTcar.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/filters/ChFilterRadarVisualizeCluster.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"

#include <chrono>
#include <random>
using namespace chrono;
using namespace chrono::ros;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::vehicle::artcar;
// =============================================================================

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;
double terrainHeight = 0;     // terrain height (FLAT terrain only)
double terrainLength = 300.0;  // size in X direction
double terrainWidth = 300.0;   // size in Y direction

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::PRIMITIVES;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::MESH;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision system
auto collision_system_type = ChCollisionSystem::Type::BULLET;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;
// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::NONE;
// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;
// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// sensor params
unsigned int image_width = 1080;
unsigned int image_height = 720;
float fov = (float)CH_PI / 2.;
int alias_factor = 1;
float lag = 0.0f;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Simulation step size
double step_size = 1e-3;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    // Initial vehicle position
    ChVector3d initLoc1(0.0, 0.0, 0.8);
    ChVector3d initLoc2(-1.0, 0.0, 0.8);
    ChQuaternion<> initRot = QuatFromAngleZ(0.0f);
    // --------------
    // Create systems
    // --------------

    // Chrono system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(150);
    sys.SetMaxPenetrationRecoverySpeed(4.0);

    // Create the terrain
    RigidTerrain terrain(&sys);
    auto patch_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    patch_mat->SetFriction(0.9f);
    patch_mat->SetRestitution(0.01f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, 200, 100);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // define ROS handelers' rate
    auto driver_inputs_rate = 25;
    auto vehicle_state_rate = 25;
    // define ROS topics' name for driver inputs and vehicle state
    auto driver_inputs_topic_name = "~/input/driver_inputs";
    auto vehicle_state_topic_name = "~/output/vehicle/state";

    // Create and initialize the first vehicle
    ARTcar dart_1(&sys);
    dart_1.SetInitPosition(ChCoordsys<>(initLoc1, initRot));
    dart_1.SetChassisFixed(false);
    dart_1.SetTireType(tire_model);
    dart_1.SetTireStepSize(step_size);
    dart_1.SetMaxMotorVoltageRatio(0.09f);
    dart_1.SetStallTorque(0.3f);
    dart_1.Initialize();
    dart_1.SetChassisVisualizationType(VisualizationType::MESH);
    dart_1.SetSuspensionVisualizationType(VisualizationType::MESH);
    dart_1.SetSteeringVisualizationType(VisualizationType::MESH);
    dart_1.SetWheelVisualizationType(VisualizationType::MESH);
    dart_1.SetTireVisualizationType(VisualizationType::MESH);

    // Create the basic driver for the first vehicle
    auto driver_1 = std::make_shared<ChDriver>(dart_1.GetVehicle());
    // Create ROS manager for vehicle one
    auto ros_manager_1 = chrono_types::make_shared<ChROSManager>("dart_1");
    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler_1 = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager_1->RegisterHandler(clock_handler_1);
    // Create a subscriber to the driver inputs
    auto driver_inputs_handler_1 =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_1, driver_inputs_topic_name);
    ros_manager_1->RegisterHandler(driver_inputs_handler_1);
    // Create a publisher for the vehicle state
    auto vehicle_state_handler_1 = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, dart_1.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_1->RegisterHandler(vehicle_state_handler_1);
    // Finally, initialize the ros manager
    ros_manager_1->Initialize();


    // Create and initialize the second vehicle
    ARTcar dart_2(&sys);
    dart_2.SetInitPosition(ChCoordsys<>(initLoc2, initRot));
    dart_2.SetChassisFixed(false);
    dart_2.SetTireType(tire_model);
    dart_2.SetTireStepSize(step_size);
    dart_2.SetMaxMotorVoltageRatio(0.09f);
    dart_2.SetStallTorque(0.3f);
    dart_2.Initialize();
    dart_2.SetChassisVisualizationType(VisualizationType::MESH);
    dart_2.SetSuspensionVisualizationType(VisualizationType::MESH);
    dart_2.SetSteeringVisualizationType(VisualizationType::MESH);
    dart_2.SetWheelVisualizationType(VisualizationType::MESH);
    dart_2.SetTireVisualizationType(VisualizationType::MESH);

    // ----------------------------------------
    // add sensors to second vehicle
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(&sys);
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 200);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});

    // Add camera
    auto cam_pose = chrono::ChFrame<double>({-3.304, 0, 1.0}, QuatFromAngleAxis(0.1, {0, 1.25, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(dart_2.GetChassis()->GetBody(),  // body camera is attached to
                                                         10,                               // update rate in Hz
                                                         cam_pose,                         // offset pose
                                                         image_width,                      // image width
                                                         image_height,                     // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         false);        // use global illumination or not
    cam->SetName(" Camera ");
    cam->SetLag(lag);
    cam->SetCollectionWindow(0.0f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera"));
    //cam->PushFilter(chrono_types::make_shared<ChFilterSave>("./cam1/"));
    sensor_manager->AddSensor(cam);
    sensor_manager->Update();
    // -----------------------------------------

    // Create the basic driver for the first vehicle
    auto driver_2 = std::make_shared<ChDriver>(dart_2.GetVehicle());
    // Create ROS manager for vehicle one
    auto ros_manager_2 = chrono_types::make_shared<ChROSManager>("dart_2");
    // Create a subscriber to the driver inputs
    auto driver_inputs_handler_2 =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_2, driver_inputs_topic_name);
    ros_manager_2->RegisterHandler(driver_inputs_handler_2);
    // Create a publisher for the vehicle state
    auto vehicle_state_handler_2 = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, dart_2.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_2->RegisterHandler(vehicle_state_handler_2);
    // Finally, initialize the ros manager
    ros_manager_2->Initialize();


    // Simulation loop
    double t_end = 300;
    double time = 0;
    dart_1.GetVehicle().EnableRealtime(true);
    dart_2.GetVehicle().EnableRealtime(true);

    // Real start time
    auto realStartTime = std::chrono::steady_clock::now();
    double simStartTime = sys.GetChTime();
    while (time < t_end) {


        // Driver inputs
        DriverInputs driver_inputs_1 = driver_1->GetInputs();
        DriverInputs driver_inputs_2 = driver_2->GetInputs();

        // Update modules (process inputs from other modules)
        driver_1->Synchronize(time);
        driver_2->Synchronize(time);
        dart_1.Synchronize(time, driver_inputs_1, terrain);
        dart_2.Synchronize(time, driver_inputs_2, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules.
        driver_1->Advance(step_size);
        driver_2->Advance(step_size);
        dart_1.Advance(step_size);
        dart_2.Advance(step_size);
        terrain.Advance(step_size);

        // update sensor manager
        sensor_manager->Update();

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);

        double time = sys.GetChTime();
        // Get the current real time
        auto realCurrentTime = std::chrono::steady_clock::now();
        // Calculate the elapsed real time in seconds
        double elapsedRealTime = std::chrono::duration<double>(realCurrentTime - realStartTime).count();

        // Calculate the Real-Time Factor (RTF)
        double RTF = elapsedRealTime / (time - simStartTime);

        // Print or log the RTF
        std::cout << "RTF: " << RTF << std::endl;

        // Update ROS managers
        if (!ros_manager_1->Update(time, step_size) || !ros_manager_2->Update(time, step_size))
            break;

    }

    return 0;
}