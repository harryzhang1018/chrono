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
#include <cmath>  // Include cmath for sin and cos functions

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
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"

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
#include "chrono_sensor/sensors/Sensor.h"

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
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::MESH;
VisualizationType steering_vis_type = VisualizationType::MESH;
VisualizationType wheel_vis_type = VisualizationType::MESH;

// Collision system
auto collision_system_type = ChCollisionSystem::Type::BULLET;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;
// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType chassis_collision_type_leader = CollisionType::NONE;
CollisionType chassis_collision_type_worker = CollisionType::NONE;
// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;
// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// sensor params
unsigned int image_width = 960;
unsigned int image_height = 720;
float fov = (float)CH_PI / 2.;
int alias_factor = 1;
float lag = 0.0f;
CameraLensModelType lens_model = CameraLensModelType::PINHOLE;
// Simulation step size
double step_size = 1e-3;

// Parameters for the falling ball
double radius = 0.12;
double mass = 2;
// Positions for the balls
// std::vector<ChVector3d> positions = {
//     ChVector3d(10, 15, 0),
//     ChVector3d(7, 1, 0),
//     ChVector3d(3, 20, 0)
// };
ChQuaternion<> rot(1, 0, 0, 0);
ChVector3d init_vel(0, 0, 0);
// Struct to hold ball position
struct BallPosition {
    double x;
    double y;
};
// Function to write ball positions to a CSV file
void writePositionsToCSV(const std::vector<BallPosition>& positions, const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        //file << "x,y,z\n"; // CSV header
        for (const auto& pos : positions) {
            file << pos.x << "," << pos.y << "\n";
        }
        file.close();
    } else {
        std::cerr << "Unable to open file";
    }
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Initial vehicle location and orientation
    // float init_x = std::atof(argv[3]);
    // float init_y = std::atof(argv[4]);
    float init_x = 0.0;
    float init_y = 0.0;
    // Initial vehicle position
    ChVector3d initLoc(init_x + 1, init_y + 2, 0.8);
    ChQuaternion<> initRot = QuatFromAngleZ(0.0f);
    // Initial vehicle position
    ChVector3d initLoc_flw(init_x + 1, init_y + 1, 0.8);
    ChQuaternion<> initRot_flw = QuatFromAngleZ(0.0f);
    
    // Create the vehicle leader
    ARTcar vehicle_leader;
    vehicle_leader.SetCollisionSystemType(collision_system_type);
    //vehicle_leader::SetDataPath(std::string(CHRONO_DATA_DIR) + "/vehicle_leader/");
    vehicle_leader.SetContactMethod(contact_method);
    vehicle_leader.SetChassisCollisionType(chassis_collision_type_leader);
    vehicle_leader.SetChassisFixed(false);
    vehicle_leader.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    vehicle_leader.SetTireType(tire_model);
    vehicle_leader.SetTireStepSize(step_size);
    vehicle_leader.SetMaxMotorVoltageRatio(0.08f);
    vehicle_leader.SetStallTorque(0.15f);
    vehicle_leader.SetTireRollingResistance(0.05f);
    vehicle_leader.Initialize();
    VisualizationType tire_vis_type = VisualizationType::MESH;
    vehicle_leader.SetChassisVisualizationType(chassis_vis_type);
    vehicle_leader.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle_leader.SetSteeringVisualizationType(steering_vis_type);
    vehicle_leader.SetWheelVisualizationType(wheel_vis_type);
    vehicle_leader.SetTireVisualizationType(tire_vis_type);
    vehicle_leader.GetVehicle().SetWheelCollide(false);

    // Create the vehicle follower
    ARTcar vehicle_follower(vehicle_leader.GetSystem());
    vehicle_follower.SetCollisionSystemType(collision_system_type);
    //vehicle_leader::SetDataPath(std::string(CHRONO_DATA_DIR) + "/vehicle_leader/");
    vehicle_follower.SetContactMethod(contact_method);
    vehicle_follower.SetChassisCollisionType(chassis_collision_type_worker);
    vehicle_follower.SetChassisFixed(false);
    vehicle_follower.SetInitPosition(ChCoordsys<>(initLoc_flw, initRot_flw));
    vehicle_follower.SetTireType(tire_model);
    vehicle_follower.SetTireStepSize(step_size);
    vehicle_follower.SetMaxMotorVoltageRatio(0.08f);
    vehicle_follower.SetStallTorque(0.15f);
    vehicle_follower.SetTireRollingResistance(0.05f);
    vehicle_follower.Initialize();
    vehicle_follower.SetChassisVisualizationType(chassis_vis_type);
    vehicle_follower.SetSuspensionVisualizationType(suspension_vis_type);
    vehicle_follower.SetSteeringVisualizationType(steering_vis_type);
    vehicle_follower.SetWheelVisualizationType(wheel_vis_type);
    vehicle_follower.SetTireVisualizationType(tire_vis_type);
    vehicle_follower.GetVehicle().SetWheelCollide(true);


    // Containing system
    auto system = vehicle_leader.GetSystem();
    // Create Terrain
    RigidTerrain terrain(vehicle_leader.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 1.2f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    patch_mat->SetRollingFriction(0.001f);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    //patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass_1.jpg"), 20, 20);
    terrain.Initialize();

    // Create some obstacles
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetRollingFriction(0.001f);

    // create boundary box 1
    auto box1 = chrono_types::make_shared<ChBodyEasyBox>(15, 0.5,0.2,100,true,false, material); 
    box1->SetPos(ChVector3d(7.5, 0.0, 0.0));
    box1->SetFixed(true);
    box1->GetVisualModel()->GetShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    system->Add(box1);
    // create boundary box 2
    auto box2 = chrono_types::make_shared<ChBodyEasyBox>(15, 0.5,0.2,100,true,false, material);
    box2->SetPos(ChVector3d(7.5, 25.0, 0.0));
    box2->SetFixed(true);
    box2->GetVisualModel()->GetShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    system->Add(box2);
    // create boundary box 3
    auto box3 = chrono_types::make_shared<ChBodyEasyBox>(0.5, 25,0.2,100,true,false, material);
    box3->SetPos(ChVector3d(0.0, 12.5, 0.0));
    box3->SetFixed(true);
    box3->GetVisualModel()->GetShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    system->Add(box3);
    // create boundary box 4
    auto box4 = chrono_types::make_shared<ChBodyEasyBox>(0.5, 25,0.2,100,true,false, material);
    box4->SetPos(ChVector3d(15.0, 12.5, 0.0));
    box4->SetFixed(true);
    box4->GetVisualModel()->GetShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    system->Add(box4);
    // create rod attaching to the follower vehicle
    auto vehicle_attachment_0 = chrono_types::make_shared<ChBodyEasyBox>(0.05, 0.8, 0.05, 100, true, true, material);
    // auto vehicle_attachment_0 = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/art_attach_1.obj"),true, true,true,100.0, material);
    vehicle_attachment_0->SetPos(vehicle_leader.GetChassisBody()->GetPos() + ChVector3d(0.4, 0, 0));
    vehicle_attachment_0->SetRot(vehicle_leader.GetChassisBody()->GetRot());
    vehicle_attachment_0->GetVisualModel()->GetShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    vehicle_attachment_0->SetFixed(true);
    system->Add(vehicle_attachment_0);

    
    auto vehicle_attachment_1 = chrono_types::make_shared<ChBodyEasyBox>(0.05, 0.8, 0.05, 100, true, true, material);
    // auto vehicle_attachment_1 = chrono_types::make_shared<ChBodyEasyMesh>(GetChronoDataFile("models/art_attach_1.obj"),true, true,true,100.0, material);
    vehicle_attachment_1->SetPos(vehicle_follower.GetChassisBody()->GetPos() + ChVector3d(0.4, 0, 0));
    vehicle_attachment_1->SetRot(vehicle_follower.GetChassisBody()->GetRot());
    vehicle_attachment_1->GetVisualModel()->GetShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
    vehicle_attachment_1->SetFixed(true);
    system->Add(vehicle_attachment_1);
    // Number of balls to create
    int n = 4;
    // Random engine and distributions
    std::default_random_engine rng(std::random_device{}());
    std::uniform_real_distribution<double> dist_x(0, 15);
    std::uniform_real_distribution<double> dist_y(0, 25);
    // Vector to store ball positions
    std::vector<BallPosition> ball_positions;
    for (int i = 0; i < n; ++i) {
        // Generate random position
        ChVector3d pos(dist_x(rng), dist_y(rng), 0);
        //ChVector3d pos(10, 10, radius);        
        std::cout << "Ball " << i << " at " << pos << std::endl;
        auto ball = chrono_types::make_shared<ChBody>();
        ball->SetMass(mass);
        ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
        ball->SetPos(pos);
        ball->SetRot(rot);
        ball->SetPosDt(init_vel);
        ball->SetFixed(false);
        
        auto sphere_coll = chrono_types::make_shared<ChCollisionShapeSphere>(material, radius);
        ball->AddCollisionShape(sphere_coll, ChFrame<>());
        ball->EnableCollision(true);

        auto sphere_vis = chrono_types::make_shared<ChVisualShapeSphere>(radius);
        sphere_vis->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sphere_vis->SetOpacity(1.0f);
        ball->AddVisualShape(sphere_vis);

        system->AddBody(ball);

        // Store position in vector
        ball_positions.push_back({ pos.x(), pos.y() });
    }
    // Write positions to CSV file
    writePositionsToCSV(ball_positions, "/home/harry/ros_ws/src/ball_pos.csv");
    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle_leader.GetVehicle());
    auto driver_follower = std::make_shared<ChDriver>(vehicle_follower.GetVehicle());
    // ----------------------------------------
    // add sensor
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(vehicle_leader.GetSystem());
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 200);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    // Add camera
    auto cam_pose = chrono::ChFrame<double>({-1.75, 0, 12.0}, QuatFromAngleAxis(1.0, {0, 1.25, 0}));
    //auto cam_pose = chrono::ChFrame<double>({-2.32, 0.0, 0.6}, QuatFromAngleAxis(0.2, {0, 1.4, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(box3,  // body camera is attached to
                                                         10,                               // update rate in Hz
                                                         cam_pose,                         // offset pose
                                                         image_width,                      // image width
                                                         image_height,                     // image height
                                                         fov,           // camera's horizontal field of view
                                                         alias_factor,  // supersample factor for antialiasing
                                                         lens_model,    // FOV
                                                         false);        // use global illumination or not
    cam->SetName(" Third Person Camera");
    cam->SetLag(lag);
    cam->SetCollectionWindow(0.0f);
    cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Third Person Camera"));
    //cam->PushFilter(chrono_types::make_shared<ChFilterSave>("./cam1/"));
    sensor_manager->AddSensor(cam);


    // // Add camera
    // auto cam_pose_flw = chrono::ChFrame<double>({.404, 0, 0.2}, QuatFromAngleAxis(0.1, {0, 1.25, 0}));
    // auto cam_flw = chrono_types::make_shared<ChCameraSensor>(vehicle_leader.GetChassis()->GetBody(),  // body camera is attached to
    //                                                      10,                               // update rate in Hz
    //                                                      cam_pose_flw,                         // offset pose
    //                                                      480,                      // image width
    //                                                      320,                     // image height
    //                                                      fov,           // camera's horizontal field of view
    //                                                      alias_factor,  // supersample factor for antialiasing
    //                                                      lens_model,    // FOV
    //                                                      false);        // use global illumination or not
    // cam_flw->SetName(" Camera ");
    // cam_flw->SetLag(lag);
    // cam_flw->SetCollectionWindow(0.0f);
    // cam_flw->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    // cam_flw->PushFilter(chrono_types::make_shared<ChFilterVisualize>(480, 320, "Front Camera"));
    // //cam->PushFilter(chrono_types::make_shared<ChFilterSave>("./cam1/"));
    // sensor_manager->AddSensor(cam_flw);

    // // Create a 2d lidar and add it to the sensor manager
    // auto lidar_pose = chrono::ChFrame<double>({0.32, 0.0, 0.0}, QuatFromAngleAxis(0.0, {0, 1.25, 0}));
    // auto lidar_2d = chrono_types::make_shared<ChLidarSensor>(vehicle_leader.GetChassis()->GetBody(), 10.f, lidar_pose, 360, 1, CH_PI, 0.0, 0.0, 8.0f);
    // lidar_2d->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    // //lidar_2d->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 480, "2D Lidar"));
    // sensor_manager->AddSensor(lidar_2d);

    // -----------
    // Create ROS manager
    auto ros_manager_leader = chrono_types::make_shared<ChROSManager>("leader_vehicle");

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager_leader->RegisterHandler(clock_handler);

    // Create a subscriber to the driver inputs
    auto driver_inputs_rate = 25;
    auto driver_inputs_topic_name = "~/driver_inputs";
    auto driver_inputs_handler =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    ros_manager_leader->RegisterHandler(driver_inputs_handler);
    // // Create the publisher for the lidar
    // auto lidar_2d_topic_name = "~/laser_scan";
    // auto lidar_2d_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar_2d, lidar_2d_topic_name,
    //                                                                      ChROSLidarHandlerMessageType::LASER_SCAN);
    // ros_manager_leader->RegisterHandler(lidar_2d_handler);
    // Create a publisher for the vehicle state
    auto vehicle_state_rate = 25;
    auto vehicle_state_topic_name = "~/state";
    auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, vehicle_leader.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_leader->RegisterHandler(vehicle_state_handler);

    // Finally, initialize the ros manager
    ros_manager_leader->Initialize();

    // create ros manager for follower vehicle
    auto ros_manager_follower = chrono_types::make_shared<ChROSManager>("follower_vehicle");
    // create a subscriber for the vehicle state
    auto driver_inputs_handler_follower = 
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver_follower, driver_inputs_topic_name);
    ros_manager_follower->RegisterHandler(driver_inputs_handler_follower);
    // create a publisher for the vehicle state
    auto vehicle_state_handler_follower = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, vehicle_follower.GetChassisBody(), vehicle_state_topic_name);
    ros_manager_follower->RegisterHandler(vehicle_state_handler_follower);
    // create a publisher for the camera
    // auto camera_update_rate = 10;
    // auto camera_topic_name = "~/camera";
    // auto camera_handler = chrono_types::make_shared<ChROSCameraHandler>(camera_update_rate, cam_flw, camera_topic_name);
    // ros_manager_follower->RegisterHandler(camera_handler);
    // initialize the follower vehicle ros driver
    ros_manager_follower->Initialize();
    // Simulation loop
    double t_end = 300;
    double time = 0;
    vehicle_leader.GetVehicle().EnableRealtime(true);
    while (time < t_end) {
        // put the vehicle attachment to the follower vehicle
        // get heading angle
        auto heading_follow = vehicle_follower.GetChassisBody()->GetRot().GetCardanAnglesZYX()[2];
        ChVector3d disp(0.4*cos(heading_follow), 0.4*sin(heading_follow), 0);
        vehicle_attachment_1->SetPos(vehicle_follower.GetChassisBody()->GetPos() + disp);
        vehicle_attachment_1->SetRot(vehicle_follower.GetChassisBody()->GetRot());
        //vehicle_attachment_1->SetRot(QuatFromAngleZ(CH_PI/2.0));

        auto heading_leader = vehicle_leader.GetChassisBody()->GetRot().GetCardanAnglesZYX()[2];
        ChVector3d disp_0(0.4*cos(heading_leader), 0.4*sin(heading_leader), 0);
        vehicle_attachment_0->SetPos(vehicle_leader.GetChassisBody()->GetPos() + disp_0);
        vehicle_attachment_0->SetRot(vehicle_leader.GetChassisBody()->GetRot());
        //vehicle_attachment_0->SetRot(QuatFromAngleZ(CH_PI/2.0));

        // driver_follower->SetThrottle(1.0);

        // //debug ball position
        // auto pos_ball = ball->GetPos();
        // std::cout << "Ball position: " << pos_ball << std::endl;

        DriverInputs driver_inputs = driver->GetInputs();
        DriverInputs driver_inputs_follower = driver_follower->GetInputs();
        
        // Update modules (process inputs from other modules)
        time = vehicle_leader.GetSystem()->GetChTime();

        driver->Synchronize(time);
        driver_follower->Synchronize(time);

        vehicle_leader.Synchronize(time, driver_inputs, terrain);
        vehicle_follower.Synchronize(time, driver_inputs_follower, terrain);
        terrain.Synchronize(time);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        driver_follower->Advance(step_size);

        vehicle_leader.Advance(step_size);
        vehicle_follower.Advance(step_size);

        terrain.Advance(step_size);

        // update sensor manager
        sensor_manager->Update();

        // Update ROS managers
        if (!ros_manager_leader->Update(time, step_size) || !ros_manager_follower->Update(time, step_size))
            break;
    }

    return 0;
}