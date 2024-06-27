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
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
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
using namespace chrono::irrlicht;
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
VisualizationType wheel_vis_type = VisualizationType::NONE;

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

// Simulation step size
double step_size = 1e-3;
// Time interval between two render frames
double render_step_size = 1.0 / 15;  // FPS = 50
// =============================================================================

int main(int argc, char* argv[]) {
    // Initial vehicle location and orientation
    // float init_x = std::atof(argv[3]);
    // float init_y = std::atof(argv[4]);
    float init_x = 0.0;
    float init_y = 0.0;
    // Initial vehicle position
    ChVector3d initLoc(init_x, init_y, 0.8);
    ChQuaternion<> initRot = QuatFromAngleZ(0.0f);
    ARTcar vehicle;
    vehicle.SetCollisionSystemType(collision_system_type);
    //vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "/vehicle/");
    vehicle.SetContactMethod(contact_method);
    vehicle.SetChassisCollisionType(chassis_collision_type);
    vehicle.SetChassisFixed(false);
    vehicle.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    vehicle.SetTireType(tire_model);
    vehicle.SetTireStepSize(step_size);
    vehicle.SetMaxMotorVoltageRatio(0.09f);
    vehicle.SetStallTorque(0.3f);
    
    vehicle.SetTireRollingResistance(0.05f);
    vehicle.Initialize();
    
    VisualizationType tire_vis_type = VisualizationType::MESH;

    vehicle.SetChassisVisualizationType(chassis_vis_type);
    vehicle.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle.SetSteeringVisualizationType(steering_vis_type);
    vehicle.SetWheelVisualizationType(wheel_vis_type);
    vehicle.SetTireVisualizationType(tire_vis_type);
    // vehicle.GetVehicle().SetWheelCollide(false);
    // Containing system
    auto system = vehicle.GetSystem();
    // system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // --------------------------------
    // Add obstacle objects
    // --------------------------------
    // Add path to follow:
    // Function to read and parse the CSV file
    std::vector<std::tuple<double, double, double, double>> positions;
    std::string directoryPath = "/sbel/Desktop/waypoints_paths/";
    std::string csvFile = directoryPath + argv[1]; // Replace with CSV file path
    std::ifstream file(csvFile);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open CSV file." << std::endl;
        return 1;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y, z, w;
        char delimiter;
        if (ss >> x >> delimiter >> y >> delimiter >> z >> delimiter >> w) {
            positions.emplace_back(x, y, z, w);
        } else {
            std::cerr << "Error: Invalid CSV format in line: " << line << std::endl;
            return 1;
        }
    }
    // Add color to path
    // Define visual material (replace with your material setup)
    auto vis_mat_path = chrono_types::make_shared<chrono::ChVisualMaterial>();
    vis_mat_path->SetDiffuseColor(chrono::ChColor(0.0f, 1.0f, 0.0f));
    // Create ChBodyEasyBox objects at specified positions
    for (const auto& pos : positions) {
        auto box_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.0001, 1000, true, false);
        box_body->SetPos(ChVector3d(std::get<0>(pos), std::get<1>(pos), 0.1));
        box_body->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
        box_body->SetFixed(true);
        vehicle.GetSystem()->Add(box_body);
    }


    // Adding Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChContactMaterial> rockSufaceMaterial = ChContactMaterial::DefaultMaterial(vehicle.GetSystem()->GetContactMethod());
    rockSufaceMaterial->SetFriction(0.7f);
    // Randomly shuffle the positions vector to select n unique positions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(positions.begin(), positions.end(), gen);
    // Uniform distribution from -1 to 1
    std::uniform_real_distribution<> dis(-0.2, 0.2);
    std::random_device sz_rd_lg;
    std::mt19937 gen_sz_lg(sz_rd_lg());
    std::uniform_real_distribution<> rand_size_lg(0.05, 0.25);
    int m = 1; // Number of rocks
    for (int i = 0; i < m; ++i) {
        //double x = 6.0;
        double x = std::get<0>(positions[i]) + dis(gen);
        double y = std::get<1>(positions[i]) + dis(gen);
        //double y = dis(gen);
        // create a rock
        std::string rock_obj_path;
        
        double scale_ratio = rand_size_lg(gen_sz_lg);

        if (int(scale_ratio * 100) % 3 == 0) {
            std::cout<<"use rock 1"<<std::endl;
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        } else if (int(scale_ratio*100) % 3 == 1) {
            std::cout<<"use rock 2"<<std::endl;
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock2.obj");
        } else {
            std::cout<<"use rock 3"<<std::endl;
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        }

        auto rock_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(0.2));  // scale to a different size
        rock_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock_rot = QuatFromAngleX(CH_PI / 2);
        ChVector3d rock_pos = ChVector3d(x, y, 0);
        rock_Body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

        rock_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock_Body->SetInertiaXX(mdensity * principal_I);

        rock_Body->SetFrameRefToAbs(ChFrame<>(ChVector3d(rock_pos), ChQuaternion<>(rock_rot)));
        vehicle.GetSystem()->Add(rock_Body);

        rock_Body->SetFixed(true);
        auto rock_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_mmesh,
                                                                                     false, false, 0.005);
        rock_Body->AddCollisionShape(rock_ct_shape);
        // disable collision to reduce RTF
        rock_Body->EnableCollision(false);

        auto rock_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);

        vehicle.GetSystem()->Add(rock_Body);
        vehicle.GetSystem()->GetCollisionSystem()->BindItem(rock_Body);
        // std::cout<<"done adding all the rocks"<<std::endl;
    }
    // randomly initilize the terrain with friction and elasticity properties.
    RigidTerrain terrain(vehicle.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    //patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle.GetVehicle());

    // Create the vehicle Irrlicht interface
    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("dART Visualization");
    vis->SetChaseCamera(ChVector3d(-1.0, 0.0, 0.5), 3.0, 0.05);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle.GetVehicle());


    // -----------
    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>("dART");

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create a subscriber to the driver inputs
    auto driver_inputs_rate = 25;
    auto driver_inputs_topic_name = "~/driver_inputs";
    auto driver_inputs_handler =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    ros_manager->RegisterHandler(driver_inputs_handler);

    // Create a publisher for the vehicle state
    auto vehicle_state_rate = 25;
    auto vehicle_state_topic_name = "~/state";
    auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, vehicle.GetChassisBody(), vehicle_state_topic_name);
    ros_manager->RegisterHandler(vehicle_state_handler);

    // // Create the publisher for the lidar
    // auto lidar_topic_name = "~/pointcloud";
    // auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
    // ros_manager->RegisterHandler(lidar_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    // Simulation loop
    double t_end = 300;
    double time = 0;
    vehicle.GetVehicle().EnableRealtime(true);
    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    // Initialize simulation frame counters
    int step_number = 0;
    while (vis->Run()) {
        // Render scene and output post-processing data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }
        // Get driver inputs
        driver->SetThrottle(0.7f);
        DriverInputs driver_inputs = driver->GetInputs();
        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver->Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);

        if (!ros_manager->Update(time, step_size))
            break;
        step_number++;
    }

    return 0;
}