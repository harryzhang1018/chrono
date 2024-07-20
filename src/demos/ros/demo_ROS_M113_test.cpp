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
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_models/vehicle/m113/M113.h"

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
#include <std_msgs/msg/float64_multi_array.hpp>
#include "demos/SetChronoSolver.h"

#include <chrono>
#include <random>
using namespace chrono;
using namespace chrono::ros;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================

// Collision system
auto collision_system_type = ChCollisionSystem::Type::BULLET;

// track properties
TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
DoublePinTrackShoeType shoe_topology = DoublePinTrackShoeType::ONE_CONNECTOR;
BrakeType brake_type = BrakeType::SHAFTS;
DrivelineTypeTV driveline_type = DrivelineTypeTV::BDS;
bool use_track_bushings = false;
bool use_suspension_bushings = false;
bool use_track_RSDA = false;
bool create_track = true;

// Contact method (NSC or SMC)
ChContactMethod contact_method = ChContactMethod::SMC;

// Collision type for chassis (HULLS, MESH, or NONE)
CollisionType chassis_collision_type = CollisionType::HULLS;

// Type of tire model (RIGID_MESH, TMEASY)
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
// double step_size = 1e-3;
double step_size = 5e-4;

// Rigid terrain dimensions
double terrainHeight = 0.02f;
double terrainLength = 40.0;  // size in X direction
double terrainWidth = 5.0;   // size in Y direction

// =============================================================================
// =============================================================================
class MyCustomHandler : public ChROSHandler {
public:
    MyCustomHandler(double update_rate,const std::string& topic, chrono::vehicle::ChVehicle& vehicle)
        : ChROSHandler(update_rate), m_topic(topic), m_vehicle(vehicle) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        std::cout << "Creating publisher for topic " << m_topic << " ..." << std::endl;
        m_publisher = interface->GetNode()->create_publisher<std_msgs::msg::Float64MultiArray>(m_topic, 10);
        return true;
    }

    virtual void Tick(double time) override {
        double engine_speed = m_vehicle.GetEngine()->GetMotorSpeed();
        double engine_tq = m_vehicle.GetEngine()->GetOutputMotorshaftTorque();
        //std::cout << "Publishing array [" << engine_speed << ", " << engine_tq << "] ..." << std::endl;
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {engine_speed, engine_tq};
        m_publisher->publish(msg);
    }

private:
    const std::string m_topic;
    chrono::vehicle::ChVehicle& m_vehicle; // Store reference to the vehicle
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_publisher;
};
// =============================================================================
// =============================================================================
void InitializeTerrainParameters(SCMTerrain& terrain) {
    // Seed the random number generator
    std::srand(std::time(0));

    // Generate a random number between 0(Soft) and 2(Hard)
    int randomSelection = std::rand() % 3;
    // int randomSelection = 0;

    switch (randomSelection) {
        case 0:
            terrain.SetSoilParameters(1e7,   // Bekker Kphi
                                      0,     // Bekker Kc
                                      1.1,   // Bekker n exponent
                                      0,     // Mohr cohesive limit (Pa)
                                      20,    // Mohr friction limit (degrees)
                                      0.01,  // Janosi shear coefficient (m)
                                      2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                      3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
            );
            break;
        case 1:
            terrain.SetSoilParameters(2e7,   // Bekker Kphi
                                      0,     // Bekker Kc
                                      1.1,   // Bekker n exponent
                                      0,     // Mohr cohesive limit (Pa)
                                      20,    // Mohr friction limit (degrees)
                                      0.01,  // Janosi shear coefficient (m)
                                      2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                      3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)

            );
            break;
        case 2:
            terrain.SetSoilParameters(4e7,   // Bekker Kphi
                                      0,     // Bekker Kc
                                      1.1,   // Bekker n exponent
                                      0,     // Mohr cohesive limit (Pa)
                                      20,    // Mohr friction limit (degrees)
                                      0.01,  // Janosi shear coefficient (m)
                                      2e8,   // Elastic stiffness (Pa/m), before plastic yield
                                      3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)

            );
            break;
        default:
            // Default case if needed, though it should never be reached
            break;
    }
}

void ReportConstraintViolation(ChSystem& sys, double threshold = 1e-3) {
    Eigen::Index imax = 0;
    double vmax = 0;
    std::string nmax = "";
    for (auto joint : sys.GetLinks()) {
        if (joint->GetConstraintViolation().size() == 0)
            continue;
        Eigen::Index cimax;
        auto cmax = joint->GetConstraintViolation().maxCoeff(&cimax);
        if (cmax > vmax) {
            vmax = cmax;
            imax = cimax;
            nmax = joint->GetNameString();
        }
    }
    if (vmax > threshold)
        cout << vmax << "  in  " << nmax << " [" << imax << "]" << endl;
}

bool ReportTrackFailure(ChTrackedVehicle& veh, double threshold = 1e-2) {
    for (int i = 0; i < 2; i++) {
        auto track = veh.GetTrackAssembly(VehicleSide(i));
        auto nshoes = track->GetNumTrackShoes();
        auto shoe1 = track->GetTrackShoe(0).get();
        for (int j = 1; j < nshoes; j++) {
            auto shoe2 = track->GetTrackShoe(j % (nshoes - 1)).get();
            auto dir = shoe2->GetShoeBody()->TransformDirectionParentToLocal(shoe2->GetTransform().GetPos() -
                                                                             shoe1->GetTransform().GetPos());
            if (std::abs(dir.y()) > threshold) {
                cout << "...Track " << i << " broken between shoes " << j - 1 << " and " << j << endl;
                cout << "time " << veh.GetChTime() << endl;
                cout << "shoe " << j - 1 << " position: " << shoe1->GetTransform().GetPos() << endl;
                cout << "shoe " << j << " position: " << shoe2->GetTransform().GetPos() << endl;
                cout << "Lateral offset: " << dir.y() << endl;
                return true;
            }
            shoe1 = shoe2;
        }
    }
    return false;
}

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;
// =============================================================================

int main(int argc, char* argv[]) {

    // Compatibility checks
    if (use_track_bushings || use_suspension_bushings) {
        if (contact_method == ChContactMethod::NSC) {
            cout << "The NSC iterative solvers cannot be used if bushings are present." << endl;
            return 1;
        }
    }

    if (shoe_type == TrackShoeType::DOUBLE_PIN && shoe_topology == DoublePinTrackShoeType::TWO_CONNECTORS) {
        if (!use_track_bushings) {
            cout << "Double-pin two-connector track shoes must use bushings." << endl;
            return 1;
        }
    }

    // Initial vehicle location and orientation
    // float init_x = std::atof(argv[3]);
    // float init_y = std::atof(argv[4]);
    // float init_x = -5.0;
    float init_x = -20.0;
    float init_y = -30.0;
    ChVector3d initLoc(init_x, init_y, 0.8);
    ChQuaternion<> initRot(1, 0, 0, 0);

    M113 vehicle;
    vehicle.SetContactMethod(contact_method);
    vehicle.SetCollisionSystemType(collision_system_type);
    vehicle.SetTrackShoeType(shoe_type);
    vehicle.SetDoublePinTrackShoeType(shoe_topology);
    vehicle.SetTrackBushings(use_track_bushings);
    vehicle.SetSuspensionBushings(use_suspension_bushings);
    vehicle.SetTrackStiffness(use_track_RSDA);
    vehicle.SetDrivelineType(driveline_type);
    vehicle.SetBrakeType(brake_type);
    vehicle.SetEngineType(EngineModelType::SIMPLE_MAP);
    vehicle.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    vehicle.SetChassisCollisionType(chassis_collision_type);

    vehicle.SetChassisFixed(false);
    vehicle.CreateTrack(create_track);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    vehicle.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    vehicle.Initialize();
    //std::cout<<"hello"<<std::endl;
    
    vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSprocketVisualizationType(VisualizationType::NONE);
    vehicle.SetIdlerVisualizationType(VisualizationType::NONE);
    vehicle.SetSuspensionVisualizationType(VisualizationType::NONE);
    vehicle.SetIdlerWheelVisualizationType(VisualizationType::NONE);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::NONE);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Containing system
    auto system = vehicle.GetSystem();
    system->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));

    SetChronoSolver(*vehicle.GetSystem(), ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    vehicle.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    vehicle.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    std::cout << "SOLVER TYPE:     " << (int)vehicle.GetSystem()->GetSolver()->GetType() << std::endl;
    std::cout << "INTEGRATOR TYPE: " << (int)vehicle.GetSystem()->GetTimestepper()->GetType() << std::endl;
    // system->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // --------------------------------
    // Add obstacle objects
    // --------------------------------
    // Add path to follow:
    // Function to read and parse the CSV file
    std::vector<std::tuple<double, double, double, double>> positions;
    std::string directoryPath = "/sbel/Desktop/waypoints_paths/";
    std::string csvFile = directoryPath + argv[1] + ".csv"; // Replace with CSV file path
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
    vis_mat_path->SetDiffuseColor(chrono::ChColor(1.0f, 0.0f, 0.0f));
    // Create ChBodyEasyBox objects at specified positions
    for (const auto& pos : positions) {
        auto box_body = chrono_types::make_shared<ChBodyEasyBox>(0.025, 0.025, 0.0001, 1000, true, false);
        box_body->SetPos(ChVector3d(std::get<0>(pos), std::get<1>(pos), 0.0));
        // box_body->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
        box_body->SetFixed(true);
        // Set visual material for the box
        auto shape = box_body->GetVisualModel()->GetShapes()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat_path);
        } else {
            shape->GetMaterials()[0] = vis_mat_path;
        }
        vehicle.GetSystem()->Add(box_body);
    }


        // Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChContactMaterial> rockSufaceMaterial = ChContactMaterial::DefaultMaterial(vehicle.GetSystem()->GetContactMethod());
    rockSufaceMaterial->SetFriction(0.7f);
    // Randomly shuffle the positions vector to select n unique positions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(positions.begin(), positions.end(), gen);
    std::uniform_real_distribution<> dis(-1.5, 1.5);

    std::random_device sz_rd;
    std::mt19937 gen_sz(sz_rd());
    std::uniform_real_distribution<> rand_size_sm(1.0, 1.6);
    int n = 3; // Number of big rocks
    for (int i = 0; i < n; ++i) {
        double x = std::get<0>(positions[i]) + dis(gen);
        double y = std::get<1>(positions[i]) + dis(gen);

        // double x = -12.0f;
        // double y = -30.0f;

        // create a rock
        std::string rock_obj_path;
        if (i % 3 == 0) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        } else if (i % 3 == 1) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock2.obj");
        } else {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        }
        double scale_ratio = rand_size_sm(gen_sz);
        auto rock_mmesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        rock_mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
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
        rock_Body->EnableCollision(false);

        auto rock_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);

        // vehicle.GetSystem()->Add(rock_Body);
        //vehicle.GetSystem()->GetCollisionSystem()->BindItem(rock_Body);
        //std::cout<<"done adding all the rocks"<<std::endl;
    }

    // adding small rocks for avoidance purpose:
    std::random_device sz_rd_lg;
    std::mt19937 gen_sz_lg(sz_rd_lg());
    std::uniform_real_distribution<> rand_size_lg(0.2, 0.7);
    std::shuffle(positions.begin(), positions.end(), gen);
    int m = 3; // Number of small rocks
    for (int i = 0; i < m; ++i) {
        // double x = 4.0;
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
        rock_mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
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
        //auto rock_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_mmesh,
        //                                                                             false, false, 0.005);
        //rock_Body->AddCollisionShape(rock_ct_shape);
        // for the purpose of accelerating simulation speed, since we not gonna hit big rocks, so disable collision for that
        rock_Body->EnableCollision(true);

        auto rock_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);

        // vehicle.GetSystem()->Add(rock_Body);
        // vehicle.GetSystem()->GetCollisionSystem()->BindItem(rock_Body);
        //std::cout<<"done adding all the rocks"<<std::endl;
    }

    // SCM Terrain with randomly selected terrain parameters
    SCMTerrain terrain(system);
    InitializeTerrainParameters(terrain);
    double length = 85;
    double width = 85;
    terrain.GetMesh()->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"));
    // std::string terrain_mesh_path = GetChronoDataFile("vehicle/terrain/meshes/track_veh_playground.obj");
    // terrain.Initialize(terrain_mesh_path, 0.02f);
    terrain.Initialize(length,width,0.02f);
    terrain.GetMesh()->SetWireframe(true);

    // // Rigid terrain
    // RigidTerrain terrain(vehicle.GetSystem());
    // ChContactMaterialData minfo;
    // minfo.mu = 0.9f;
    // minfo.cr = 0.2f;
    // minfo.Y = 2e7f;
    // auto patch_mat = minfo.CreateMaterial(contact_method);
    // auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    // patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    // // patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    // terrain.Initialize();

    // Create the basic driver
    auto driver = std::make_shared<ChDriver>(vehicle.GetVehicle());
    driver->Initialize();

    // ----------------------------------------
    // add sensor
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
    Background b;
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/kloppenheim_06_4k.hdr");
    sensor_manager->scene->SetBackground(b);
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 200);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    // Create a lidar and add it to the sensor manager
    chrono::ChFrame<double> offset_pose({1.5, 0, 0}, QuatFromAngleAxis(.0, {0, 1, 0}));
    auto lidar = chrono_types::make_shared<ChLidarSensor>(vehicle.GetChassisBody(), 15.f, offset_pose, 200, 100, CH_PI,
                                                          CH_PI / 12, -CH_PI / 5, 15.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(480, 360, 1, "3D Lidar"));
    sensor_manager->AddSensor(lidar);

    // Add camera
    //// View from behind vehicle
    // auto cam_pose = chrono::ChFrame<double>({-6.304, 0, 1.0}, QuatFromAngleAxis(0.1, {0, 1.25, 0}));
    //// Bird's eye view cam pos.
    auto cam_pose = chrono::ChFrame<double>({-8.0, 0, 5.0}, QuatFromAngleAxis(0.5, {0, 1, 0}));
    auto cam = chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassis()->GetBody(),  // body camera is attached to
                                                         15,                               // update rate in Hz
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
    //cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height, "Camera"));
    cam->PushFilter(chrono_types::make_shared<ChFilterSave>("./cam1/"));
    sensor_manager->AddSensor(cam);
    sensor_manager->Update();

    //------------
    //Read engine power
    auto engine_tq = vehicle.GetVehicle().GetEngine()->GetOutputMotorshaftTorque();
    auto engine_speed = vehicle.GetVehicle().GetEngine()->GetMotorSpeed();
    std::array<double, 2> eg_pw_array = {engine_speed, engine_tq};
    
    // -----------
    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>("m113");

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

    // Create the publisher for the lidar
    auto lidar_topic_name = "~/pointcloud";
    auto lidar_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_topic_name);
    ros_manager->RegisterHandler(lidar_handler);

    // Create custom handler to send engine power message
    auto custom_handler = chrono_types::make_shared<MyCustomHandler>(25.0f,"~/engine_power",vehicle.GetVehicle());
    ros_manager->RegisterHandler(custom_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    // Simulation loop
    double t_end = 300;
    double time = 0;
    vehicle.GetVehicle().EnableRealtime(true);
    while (time < t_end) {
        // Get driver inputs
        // driver->SetThrottle(0.8f);
        // driver->SetSteering(0.0f);
        // driver->SetBraking(0.0f);
        
        DriverInputs driver_inputs = driver->GetInputs();
        //std::cout<<"vehicle engine rmp:"<<vehicle.GetVehicle().GetEngine()->GetMotorSpeed()<<std::endl;
        //std::cout<<"vehicle engine torque:"<<vehicle.GetVehicle().GetEngine()->GetOutputMotorshaftTorque()<<std::endl;        
        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        std::cout<<"time: "<< time <<std::endl;

        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs);
        

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);

        // update sensor manager
        sensor_manager->Update();

        if (!ros_manager->Update(time, step_size))
            break;

        if (ReportTrackFailure(vehicle.GetVehicle(), 0.1)) {
            ReportConstraintViolation(*vehicle.GetSystem());
            break;
        }
    }

    return 0;
}


