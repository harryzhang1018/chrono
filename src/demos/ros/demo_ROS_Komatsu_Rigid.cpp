// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Shunya Shimada, Harry Zhang
// =============================================================================
//
// Main driver function for a tracked vehicle specified through JSON files.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>

#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChMathematics.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeDoublePin.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/ChVehicleOutput.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

#include "chrono/core/ChMathematics.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChNoiseModel.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::collision;
using namespace chrono::irrlicht;
using namespace chrono::ros;

using std::cout;
using std::endl;

// =============================================================================
// Specification of a vehicle model from JSON files
// Available models:
//    M113_SinglePin
//    M113_DoublePin
//    M113_RS_SinglePin
//    Marder

class Vehicle_Model {
  public:
    virtual std::string ModelName() const = 0;
    virtual std::string VehicleJSON() const = 0;
    virtual std::string EngineJSON() const = 0;
    virtual std::string TransmissionJSON() const = 0;
    virtual ChVector<> CameraPoint() const = 0;
    virtual double CameraDistance() const = 0;
};

class M113_SinglePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "M113_SinglePin"; }
    virtual std::string VehicleJSON() const override {
        return "M113/vehicle/M113_Vehicle_SinglePin.json";
        ////return "M113/vehicle/M113_Vehicle_SinglePin_BDS.json";
    }
    virtual std::string EngineJSON() const override {
        ////return "M113/powertrain/M113_EngineSimple.json";
        ////return "M113/powertrain/M113_EngineSimpleMap.json";
        return "M113/powertrain/M113_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        ////return "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";
        return "M113/powertrain/M113_AutomaticTransmissionShafts.json";
    }
    virtual ChVector<> CameraPoint() const override { return ChVector<>(0, 0, 0); }
    virtual double CameraDistance() const override { return 6.0; }
};

class M113_DoublePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "M113_DoublePin"; }
    virtual std::string VehicleJSON() const override {
        return "M113/vehicle/M113_Vehicle_DoublePin.json";
        ////return "M113/vehicle/M113_Vehicle_DoublePin_BDS.json";
    }
    virtual std::string EngineJSON() const override {
        return "M113/powertrain/M113_EngineSimple.json";
        ////return "M113/powertrain/M113_EngineSimpleMap.json";
        ////return "M113/powertrain/M113_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";
        ////return "M113/powertrain/M113_AutomaticTransmissionShafts.json";
    }
    virtual ChVector<> CameraPoint() const override { return ChVector<>(0, 0, 0); }
    virtual double CameraDistance() const override { return 6.0; }
};

class M113_RS_SinglePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "M113_RS_SinglePin"; }
    virtual std::string VehicleJSON() const override {
        ////return "M113_RS/vehicle/M113_Vehicle_SinglePin_Translational_BDS.json";
        return "M113_RS/vehicle/M113_Vehicle_SinglePin_Distance_BDS.json";
    }
    virtual std::string EngineJSON() const override {
        return "M113_RS/powertrain/M113_EngineSimple.json";
        ////return "M113_RS/powertrain/M113_EngineSimpleMap.json";
        ////return "M113_RS/powertrain/M113_EngineShafts.json";
    }
    virtual std::string TransmissionJSON() const override {
        return "M113_RS/powertrain/M113_AutomaticTransmissionSimpleMap.json";
        ////return "M113_RS/powertrain/M113_AutomaticTransmissionShafts.json";
    }
    virtual ChVector<> CameraPoint() const override { return ChVector<>(4, 0, 0); }
    virtual double CameraDistance() const override { return 6.0; }
};

class Marder_SinglePin : public Vehicle_Model {
  public:
    virtual std::string ModelName() const override { return "Marder_SinglePin"; }
    virtual std::string VehicleJSON() const override {
        ////return "Marder/vehicle/marder_sp_joints_shafts.json";
        ////return "Marder/vehicle/marder_sp_bushings_shafts.json";
        return "Marder/vehicle/marder_sp_bushings_simple.json";
    }
    virtual std::string EngineJSON() const override { return "Marder/powertrain/Marder_EngineSimple.json"; }
    virtual std::string TransmissionJSON() const override {
        return "Marder/powertrain/Marder_AutomaticTransmissionSimpleMap.json";
    }
    virtual ChVector<> CameraPoint() const override { return ChVector<>(0, 0, 0); }
    virtual double CameraDistance() const override { return 8.0; }
};

// =============================================================================
// Forward declarations
// =============================================================================

// =============================================================================
// USER SETTINGS
// =============================================================================

// Current vehicle model selection
auto vehicle_model = M113_SinglePin();
////auto vehicle_model = M113_DoublePin();
////auto vehicle_model = M113_RS_SinglePin();
////auto vehicle_model = Marder_SinglePin();

// JSON files for terrain (rigid plane)
std::string rigidterrain_file("terrain/RigidPlane.json");
//std::string rigidterrain_file("terrain/shimada/RigidPlane.json");

// Terrain dimensions for SCM terrain
double terrainLength = 20.0;  // size in X direction
double terrainWidth = 20.0;    // size in Y direction
double delta = 0.05;          // SCM grid spacing

// Initial vehicle position
// ChVector<> initLoc(0, 0, 0.9);  //demo dafault
ChVector<> initLoc(-5, 0, 0.6);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);




// Contact formulation (NSC or SMC)
ChContactMethod contact_method = ChContactMethod::NSC;

// set parallel threads
int nthreads = 40;

// Shared contact material for all meshes
auto mesh_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

// Simulation step size
double step_size_NSC = 1e-3;
double step_size_SMC = 5e-4;

// Solver and integrator types
////ChSolver::Type slvr_type = ChSolver::Type::BARZILAIBORWEIN;
////ChSolver::Type slvr_type = ChSolver::Type::PSOR;
////ChSolver::Type slvr_type = ChSolver::Type::PMINRES;
////ChSolver::Type slvr_type = ChSolver::Type::MINRES;
////ChSolver::Type slvr_type = ChSolver::Type::GMRES;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_LU;
////ChSolver::Type slvr_type = ChSolver::Type::SPARSE_QR;
ChSolver::Type slvr_type = ChSolver::Type::PARDISO_MKL;
////ChSolver::Type slvr_type = ChSolver::Type::MUMPS;

////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT_PROJECTED;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::EULER_IMPLICIT;
////ChTimestepper::Type intgr_type = ChTimestepper::Type::HHT;

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// Time interval between two render frames
double render_step_size = 1.0 / 30;  // FPS = 30
//double render_step_size = 1.0 / 60;  // FPS = 60
//double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Output directories
const std::string out_dir = "_OUTPUT";
const std::string img_dir = out_dir + "/IMG";
const std::string sens_dir = out_dir + "/SENSOR_OUTPUT/";
const std::string lidar_dir = out_dir + "/SENSOR_OUTPUT/";

// Visualization output
bool img_output = false;

// Setting for overturn prevention control
bool ot_controller = false;
bool overturnFlag = false;
bool turningFlag = false;
bool skidFlag = false;
bool testFlag = false;
int directionFlag = 0; //// 0:horizontal, 1:overturned to the right, -1:overturned to the left
int noseFlag = 0; //// 0:horizontal, 1:nose down, -1:nose up
double tempYaw = 0;
double turnYaw = 0;
double otSteer = 0;
double otTimer = 0;

// set overturn angle
double overturnPitch = 56.3;
double overturnRoll = 30.4;

// set safety factor for overturn
double overturnSf = 10;

// set vehicle horizontal threshold
double overturnHorizontal = 2;

// set vehicle skidding speed
double skidYawVelocity = 0.04;
double skidYawAngle = 3.0;

// =============================================================================

void ReportTiming(ChSystem& sys) {
    std::stringstream ss;
    ss.precision(4);
    ss << std::fixed << sys.GetChTime() << " | ";
    ss << sys.GetTimerStep() << " " << sys.GetTimerAdvance() << " " << sys.GetTimerUpdate() << " | ";
    ss << sys.GetTimerJacobian() << " " << sys.GetTimerLSsetup() << " " << sys.GetTimerLSsolve() << " | ";
    ss << sys.GetTimerCollision() << " " << sys.GetTimerCollisionBroad() << " " << sys.GetTimerCollisionNarrow();

    auto LS = std::dynamic_pointer_cast<ChDirectSolverLS>(sys.GetSolver());
    if (LS) {
        ss << " | ";
        ss << LS->GetTimeSetup_Assembly() << " " << LS->GetTimeSetup_SolverCall() << " ";
        ss << LS->GetTimeSolve_Assembly() << " " << LS->GetTimeSolve_SolverCall();
        LS->ResetTimers();
    }
    cout << ss.str() << endl;
}

void ReportConstraintViolation(ChSystem& sys, double threshold = 1e-3) {
    Eigen::Index imax = 0;
    double vmax = 0;
    std::string nmax = "";
    for (auto joint : sys.Get_linklist()) {
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

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the vehicle system
    cout << "VEHICLE: " << vehicle_model.ModelName() << endl;
    TrackedVehicle vehicle(vehicle::GetDataFile(vehicle_model.VehicleJSON()), contact_method);
    // Initialize the vehicle at the specified position.
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
    ////vehicle.GetChassis()->SetFixed(true);

    // Set visualization type for vehicle components
    vehicle.SetChassisVisualizationType(VisualizationType::NONE);
    vehicle.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetRollerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    ChSystem* sys = vehicle.GetSystem();
    sys->SetNumThreads(nthreads,nthreads,1);

    // initialize ROCK mesh
    std::string rock1_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
    double scale_ratio = 0.8;
    auto mesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(rock1_obj_path, false, true);

    mesh->Transform(ChVector<>(0, 0, 0.1), ChMatrix33<>(scale_ratio));  // scale to a different size
    mesh->RepairDuplicateVertexes(1e-9);                      // if meshes are not watertight

    // compute mass inertia from mesh
    double mass;
    ChVector<> cog;
    ChMatrix33<> inertia;
    double density = 8000;
    mesh->ComputeMassProperties(true, mass, cog, inertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

    // Add path to follow:
    // Function to read and parse the CSV file
    std::vector<std::tuple<double, double, double, double>> positions;
    std::string directoryPath = "../data/paths/";
    std::string csvFile = directoryPath + argv[1];; // Replace with CSV file path
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
        auto box_body = chrono_types::make_shared<chrono::ChBodyEasyBox>(0.1, 0.1, 0.0001, 1000, true, false);
        box_body->SetPos(chrono::ChVector<>(std::get<0>(pos), std::get<1>(pos), 0));
        box_body->SetBodyFixed(true);

        // Set visual material for the box
        auto shape = box_body->GetVisualModel()->GetShapes()[0].first;
        if (shape->GetNumMaterials() == 0) {
            shape->AddMaterial(vis_mat_path);
        } else {
            shape->GetMaterials()[0] = vis_mat_path;
        }

        sys->Add(box_body);
    }

    // Randomly shuffle the positions vector to select n unique positions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(positions.begin(), positions.end(), gen);
    int n = std::atoi(argv[2]); // Number of boxes to add
    for (int i = 0; i < n; ++i) {
        double x = std::get<0>(positions[i]);
        double y = std::get<1>(positions[i]);
        ChQuaternion<> rock_rot = Q_from_AngX(CH_C_PI_2);
        auto rock_pos = ChVector<>(x, y, 0);
        auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        rock_Body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));

        rock_Body->SetMass(mass * density);
        rock_Body->SetInertiaXX(density * principal_I);

        rock_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos), ChQuaternion<>(rock_rot)));

        rock_Body->SetBodyFixed(true);
        rock_Body->SetCollide(false);

        auto rock_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        rock_mesh->SetMesh(mesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);

        sys->Add(rock_Body);
    }

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model.EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model.TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);
    auto driver = std::make_shared<ChDriver>(vehicle);
    cout << "  Track assembly templates" << endl;
    cout << "     Sprocket:   " << vehicle.GetTrackAssembly(LEFT)->GetSprocket()->GetTemplateName() << endl;
    cout << "     Brake:      " << vehicle.GetTrackAssembly(LEFT)->GetBrake()->GetTemplateName() << endl;
    cout << "     Idler:      " << vehicle.GetTrackAssembly(LEFT)->GetIdler()->GetTemplateName() << endl;
    cout << "     Suspension: " << vehicle.GetTrackAssembly(LEFT)->GetTrackSuspension(0)->GetTemplateName() << endl;
    cout << "     Track shoe: " << vehicle.GetTrackShoe(LEFT, 0)->GetTemplateName() << endl;
    cout << "  Driveline type:    " << vehicle.GetDriveline()->GetTemplateName() << endl;
    cout << "  Engine type:       " << engine->GetTemplateName() << endl;
    cout << "  Transmission type: " << transmission->GetTemplateName() << endl;
    cout << "  Vehicle mass:      " << vehicle.GetMass() << endl;

    // Create the rigid terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();

    // auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    // vis_mat->SetSpecularColor({.1f, .1f, .1f});
    // vis_mat->SetRoughness(1);
    // vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
    //terrain.GetMesh()->AddMaterial(vis_mat);

    // Add obstacles


    // Compatibility checks
    if (vehicle.HasBushings()) {
        if (contact_method == ChContactMethod::NSC) {
            cout << "The NSC iterative solvers cannot be used if bushings are present." << endl;
            return 1;
        }
    }

    // Create a sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(vehicle.GetSystem());
    float intensity = .5;
    manager->scene->AddPointLight({2, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    // manager->scene->AddPointLight({9, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    // manager->scene->AddPointLight({16, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    // manager->scene->AddPointLight({23, 2.5, 100}, {intensity, intensity, intensity}, 5000);



    // Create a lidar and add it to the sensor manager
    // Create a lidar and add it to the sensor manager
    auto offset_pose = chrono::ChFrame<double>({1.0, 0, -0.2}, Q_from_AngAxis(0, {0, 0, 1}));
    auto lidar = chrono_types::make_shared<ChLidarSensor>(vehicle.GetChassis()->GetBody(),  // body lidar is attached to
                                                          10,                             // scanning rate in Hz
                                                          offset_pose,                    // offset pose
                                                          180,                   // number of horizontal samples
                                                          1,                   // number of vertical channels
                                                          (float)(CH_C_PI),  // horizontal field of view
                                                          (float)0.0f, 
                                                          (float)0.0f,// vertical field of view
                                                          30.0f);
    lidar->SetName("Lidar Sensor 1");
    lidar->SetLag(0.f);
    lidar->SetCollectionWindow(0.0f);

    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(640, 480, "2D Lidar"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    manager->AddSensor(lidar);

    // Create CH:ROS Manager
    // Create ROS manager
    auto ros_manager = chrono_types::make_shared<ChROSManager>();

    // Create a publisher for the simulation clock
    // The clock automatically publishes on every tick and on topic /clock
    auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
    ros_manager->RegisterHandler(clock_handler);

    // Create the publisher for the lidar
    auto lidar_2d_topic_name = "~/output/lidar_2d/data/laser_scan";
    auto lidar_2d_handler = chrono_types::make_shared<ChROSLidarHandler>(lidar, lidar_2d_topic_name, false);  // last parameter indicates whether to use LaserScan or PointCloud2
    ros_manager->RegisterHandler(lidar_2d_handler);

    // Create a subscriber to the driver inputs
    auto driver_inputs_rate = 10;
    auto driver_inputs_topic_name = "~/input/driver_inputs";
    auto driver_inputs_handler =
        chrono_types::make_shared<ChROSDriverInputsHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
    ros_manager->RegisterHandler(driver_inputs_handler);


    // Create a publisher for the vehicle state
    auto vehicle_state_rate = 25;
    auto vehicle_state_topic_name = "~/output/vehicle/state";
    auto vehicle_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
        vehicle_state_rate, vehicle.GetChassisBody(), vehicle_state_topic_name);
    ros_manager->RegisterHandler(vehicle_state_handler);

    // Finally, initialize the ros manager
    ros_manager->Initialize();

// =============================================================================

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("JSON Tracked Vehicle Demo");
    vis->SetChaseCamera(vehicle_model.CameraPoint(), 10, 2);
    //vis->SetChaseCamera(vehicle_model.CameraPoint(), vehicle_model.CameraDistance(), 0.5);
    vis->Initialize();
    //vis->AddCamera(ChVector<>(0, 1.5, -2));
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    // Initialize image output
    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }
    vis->AttachVehicle(&vehicle);

    // Solver and integrator settings
    //double step_size = 1e-3;
    //double step_size = 5e-3;
    double step_size = 1e-2;
    switch (contact_method) {
        case ChContactMethod::NSC:
            cout << "Use NSC" << endl;
            step_size = step_size_NSC;
            break;
        case ChContactMethod::SMC:
            cout << "Use SMC" << endl;
            step_size = step_size_SMC;
            break;
    }

    SetChronoSolver(*vehicle.GetSystem(), slvr_type, intgr_type);
    vehicle.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    vehicle.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    cout << "SOLVER TYPE:     " << (int)slvr_type << endl;
    cout << "INTEGRATOR TYPE: " << (int)intgr_type << endl;

    // ---------------
    // Simulation loop
    // ---------------
    int step_number = 0;
    int render_frame = 0;
    int render_steps = (int)std::ceil(render_step_size / step_size);



    while (vis->Run()) {
        // Initialize simulation frame counter and simulation time
    
        // Render scene
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            //tools::drawColorbar(vis.get(), 0, 0.1, "Sinkage", 30);
            vis->EndScene();

            if (img_output && step_number % render_steps == 0) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
                render_frame++;
            }
        }

        //UserR8BufferPtr camera_data = cam->GetMostRecentBuffer<UserR8BufferPtr>();

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        driver -> SetSteering(0.5);
        driver -> SetThrottle(1.0);
        DriverInputs driver_inputs = driver->GetInputs();


        // Release chassis
        ////if (vehicle.GetChTime() < 1) {
        ////    driver_inputs.m_throttle = 0;
        ////    driver_inputs.m_braking = 0;
        ////    driver_inputs.m_steering = 0;
        ////} else {
        ////    vehicle.GetChassis()->SetFixed(false);
        ////}

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();

        ros_manager->Update(time,step_size);
        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
        vis->Advance(step_size);

        ////ReportTiming(*vehicle.GetSystem());

        if (ReportTrackFailure(vehicle, 0.1)) {
            ReportConstraintViolation(*vehicle.GetSystem());
            break;
        }

        if (!ros_manager->Update(time, step_size))
            break;
        // Increment frame number
        step_number++;
    }

    return 0;
}
