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
// Authors: Radu Serban
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
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;
using namespace chrono::collision;
using namespace chrono::irrlicht;

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
void AddFixedObstacles(ChSystem* system);

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

// Specification of vehicle inputs
enum class DriverMode {
    KEYBOARD,  // interactive (Irrlicht) driver
    DATAFILE,  // inputs from data file
    PATH       // drives in a straight line
};
std::string driver_file("M113/driver/Acceleration.txt");  // used for mode=DATAFILE
//std::string driver_file("M113/driver/AccelerationSim.txt");  // used for mode=DATAFILE
double target_speed = 2;                                   // used for mode=PATH

DriverMode driver_mode = DriverMode::DATAFILE;

// Contact formulation (NSC or SMC)
ChContactMethod contact_method = ChContactMethod::NSC;

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
bool img_output = true;

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

    // Change collision shape for road wheels and idlers (true: cylinder; false: cylshell)
    ////vehicle.GetTrackAssembly(LEFT)->SetWheelCollisionType(false, false, false);
    ////vehicle.GetTrackAssembly(RIGHT)->SetWheelCollisionType(false, false, false);

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);

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

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor contacts involving one of the sprockets.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Render contact normals and/or contact forces.
    vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(vehicle_model.EngineJSON()));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(vehicle_model.TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

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

    /*
    // Create the rigid terrain
    RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();
    */

    // Create the SCM terrain
    // === default ===
    SCMTerrain terrain(vehicle.GetSystem());
    terrain.SetSoilParameters(2e7,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              20,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    /*
    // === sandy soil ===
    SCMTerrain terrain(vehicle.GetSystem());
    terrain.SetSoilParameters(500000,   // Bekker Kphi
                              3000,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              30,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              4e7,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // === clayey soil ===
    SCMTerrain terrain(vehicle.GetSystem());
    terrain.SetSoilParameters(814000,   // Bekker Kphi
                              20680,     // Bekker Kc
                              1.0,   // Bekker n exponent
                              3500,     // Mohr cohesive limit (Pa)
                              11,    // Mohr friction limit (degrees)
                              0.025,  // Janosi shear coefficient (m)
                              7.8e7,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    // === snow soil ===
    SCMTerrain terrain(vehicle.GetSystem());
    terrain.SetSoilParameters(149000,   // Bekker Kphi
                              6160,     // Bekker Kc
                              1.53,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              23,    // Mohr friction limit (degrees)
                              0.042,  // Janosi shear coefficient (m)
                              4e7,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    */
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);


    terrain.Initialize(terrainLength, terrainWidth, delta);
    ////terrain.Initialize(GetChronoDataFile("vehicle/terrain/meshes/sim/SCMSlope.obj"), delta);
    ////terrain.Initialize(GetChronoDataFile("vehicle/terrain/meshes/sim/dem.obj"), delta);

    terrain.GetMesh()->SetWireframe(true);

    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetSpecularColor({.1f, .1f, .1f});
    vis_mat->SetRoughness(1);
    vis_mat->SetKdTexture(GetChronoDataFile("sensor/textures/grass_texture.jpg"));
    terrain.GetMesh()->AddMaterial(vis_mat);

    // Add obstacles
    AddFixedObstacles(sys);

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
    manager->scene->AddPointLight({9, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    manager->scene->AddPointLight({16, 2.5, 100}, {intensity, intensity, intensity}, 5000);
    manager->scene->AddPointLight({23, 2.5, 100}, {intensity, intensity, intensity}, 5000);

/*
    // Create a radar and add it to the sensor manager
    // Update rate in Hz
    float update_rate = 5.f;

    // Number of horizontal and vertical samples
    unsigned int horizontal_samples = 100;
    unsigned int vertical_samples = 100;

    // Field of View
    float horizontal_fov = float(CH_C_PI / 2);  // 20 degree scan
    float vertical_fov = float(CH_C_PI / 3);    // 12 degrees up

    // max detection range
    float max_distance = 100;

    // lag time
    float lag = 0.f;

    // Collection window for the radar
    float collection_time = 1 / update_rate;  // typically 1/update rate

    auto offset_pose = chrono::ChFrame<double>({0, 0, 1}, Q_from_AngZ(0));

    auto radar = chrono_types::make_shared<ChRadarSensor>(vehicle.GetChassisBody(), update_rate, offset_pose, horizontal_samples,
                                                          vertical_samples, horizontal_fov, vertical_fov,
                                                          max_distance);
    radar->SetName("Radar Sensor");
    radar->SetLag(lag);
    radar->SetCollectionWindow(collection_time);

    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZReturn>("Radar XYZ"));
    // radar->PushFilter(chrono_types::make_shared<ChFilterRadarVisualizeCluster>(640, 480, 1, "Radar Clusters"));
    radar->PushFilter(chrono_types::make_shared<ChFilterRadarXYZVisualize>(640, 480, 1, "Radar XYZ Return"));
    manager->AddSensor(radar);
*/

    // Create a imu and add it to the sensor manager

    auto acc = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Accelerometer.json"), vehicle.GetChassisBody(),
                                      ChFrame<>({-2.006, 0, 0.406}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(acc);

    auto gyro = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Gyroscope.json"), vehicle.GetChassisBody(),
                                       ChFrame<>({-2.006, 0, 0.406}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(gyro);

    auto mag = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Magnetometer.json"), vehicle.GetChassisBody(),
                                      ChFrame<>({-2.006, 0, 0.406}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(mag);

    // Create a CSV writer to record the IMU data
    std::string imu_file = sens_dir;
    if (!filesystem::create_directory(filesystem::path(imu_file))) {
        std::cout << "Error creating directory " << imu_file << std::endl;
        return 1;
    }

    imu_file+= "imu.csv";
    utils::CSV_writer imu_csv(" ");
/*
    // Create a gps and add it to the sensor manager

    auto gps = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/GPS.json"), vehicle.GetChassisBody(),
                                      ChFrame<>({0, 0, 0}, Q_from_AngZ(0)));
    // add sensor to the manager
    manager->AddSensor(gps);

    // Create a CSV writer to record the GPS data
    std::string gps_file = sens_dir;
    if (!filesystem::create_directory(filesystem::path(gps_file))) {
        std::cout << "Error creating directory " << gps_file << std::endl;
        return 1;
    }

    gps_file+= "gps.csv";
    utils::CSV_writer gps_csv(" ");
*/
    // Create a lidar and add it to the sensor manager
    auto lidar = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Lidar.json"), vehicle.GetChassisBody(),
                                        ChFrame<>({0, 0, 0.406}, Q_from_AngZ(0)));
    lidar->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(lidar_dir + "lidar/"));
    // add sensor to the manager
    manager->AddSensor(lidar);

    /*
    // Create a camera and add it to the sensor manager
    auto cam = Sensor::CreateFromJSON(GetChronoDataFile("sensor/json/generic/Camera.json"), vehicle.GetChassisBody(),
                                      ChFrame<>({0, 0, 0.406}, Q_from_AngZ(0)));

    // add sensor to the manager
    manager->AddSensor(cam);
    */

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

    // Create the driver system
    std::shared_ptr<ChDriver> driver;
    switch (driver_mode) {
        case DriverMode::KEYBOARD: {
            auto irr_driver = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis);
            double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
            double throttle_time = 1.0;  // time to go from 0 to +1
            double braking_time = 0.3;   // time to go from 0 to +1
            irr_driver->SetSteeringDelta(render_step_size / steering_time);
            irr_driver->SetThrottleDelta(render_step_size / throttle_time);
            irr_driver->SetBrakingDelta(render_step_size / braking_time);
            irr_driver->SetGains(2, 5, 5);
            driver = irr_driver;
            break;
        }
        case DriverMode::DATAFILE: {
            auto data_driver = chrono_types::make_shared<ChDataDriver>(vehicle, vehicle::GetDataFile(driver_file));
            driver = data_driver;
            break;
        }
        case DriverMode::PATH: {
            auto path =
                chrono::vehicle::StraightLinePath(chrono::ChVector<>(0, 0, 0.02), chrono::ChVector<>(500, 0, 0.02), 50);
            auto path_driver = std::make_shared<ChPathFollowerDriver>(vehicle, path, "my_path", target_speed);
            path_driver->GetSteeringController().SetLookAheadDistance(5.0);
            path_driver->GetSteeringController().SetGains(0.5, 0, 0);
            path_driver->GetSpeedController().SetGains(0.4, 0, 0);
            driver = path_driver;
        }
    }

    driver->Initialize();

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

    /*
    // Create a falling rigid bodies
    auto sys = vehicle.GetSystem();
    auto vis_mat2 = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat2->SetAmbientColor({0.f, 0.f, 0.f});
    vis_mat2->SetDiffuseColor({1.0, 0.0, 0.0});
    vis_mat2->SetSpecularColor({.0f, .0f, .0f});
    vis_mat2->SetUseSpecularWorkflow(true);
    vis_mat2->SetRoughness(0.5f);
    vis_mat2->SetClassID(30000);
    vis_mat2->SetInstanceID(20000);
 
    auto sphere_body = chrono_types::make_shared<ChBodyEasySphere>(.5, 1000, true, false);
    sphere_body->SetPos({1, 0, 0});
    sphere_body->SetBodyFixed(true);
    sys->Add(sphere_body);
    {
        auto shape = sphere_body->GetVisualModel()->GetShapes()[0].first;
        if(shape->GetNumMaterials() == 0){
            shape->AddMaterial(vis_mat2);
        }
        else{
            shape->GetMaterials()[0] = vis_mat2;
        }
    }
    */

    /*
    auto obj_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    obj_mat->SetFriction(0.2f);

    for (int bi = 0; bi < 10; bi++) {
        auto sphereBody = chrono_types::make_shared<ChBodyEasySphere>(0.5,      // radius size
                                                                      1000,      // density
                                                                      true,      // visualization?
                                                                      true,      // collision?
                                                                      obj_mat);  // contact material
        sphereBody->SetPos(ChVector<>(3, 0, 0));
        sphereBody->GetVisualShape(0)->SetColor(ChColor(0.3f, 0.3f, 0.6f));
        sys->Add(sphereBody);
    }

    auto obstacle = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 10, 1000, true, false);
    obstacle->SetPos(ChVector<>(5, 0, 0));
    sys->Add(obstacle);
    */

    // ---------------
    // Simulation loop
    // ---------------

    // Preparing calculate vehicle posture angle
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double yawVelocity = 0;
    double rollDeg = 0;
    double pitchDeg = 0;
    double yawDeg = 0;
    double imuCollectionTime = 0.01; //follow .json file

    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    int render_frame = 0;

    UserAccelBufferPtr bufferAcc;
    UserGyroBufferPtr bufferGyro;
    UserMagnetBufferPtr bufferMag;
    UserGPSBufferPtr bufferGPS;

    int num_lidar_updates = 0;

    unsigned int imu_last_launch = 0;
    unsigned int gps_last_launch = 0;

    while (vis->Run()) {
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

        // Get the most recent imu data
        bufferAcc = acc->GetMostRecentBuffer<UserAccelBufferPtr>();
        bufferGyro = gyro->GetMostRecentBuffer<UserGyroBufferPtr>();
        bufferMag = mag->GetMostRecentBuffer<UserMagnetBufferPtr>();

        // Save the imu data to file
        if (bufferAcc->Buffer && bufferGyro->Buffer && bufferMag->Buffer &&
            bufferMag->LaunchedCount > imu_last_launch) {
            AccelData acc_data = bufferAcc->Buffer[0];
            GyroData gyro_data = bufferGyro->Buffer[0];
            MagnetData mag_data = bufferMag->Buffer[0];

            // Calculate vehicle posture angle
            roll = roll + imuCollectionTime * ( gyro_data.Roll + sin(roll)*tan(pitch)*gyro_data.Pitch + cos(roll)*tan(pitch)*gyro_data.Yaw );
            pitch = pitch + imuCollectionTime * ( cos(roll)*gyro_data.Pitch - sin(roll)*gyro_data.Yaw );
            yaw = yaw + imuCollectionTime * ( sin(roll)/cos(pitch)*gyro_data.Pitch + cosf(roll)/cos(pitch)*gyro_data.Yaw );
            yawVelocity = gyro_data.Yaw;
            rollDeg = roll*180/CH_C_PI;
            pitchDeg = pitch*180/CH_C_PI;
            yawDeg = yaw*180/CH_C_PI;

            // Get other data
            double gear = vehicle.GetTransmission()->GetCurrentGear();
            double steer = driver->GetInputs().m_steering;
            //double speed = vehicle.GetSpeed();

            imu_csv << std::fixed << std::setprecision(6);
            imu_csv << vehicle.GetChTime();
            //imu_csv << acc_data.X;
            //imu_csv << acc_data.Y;
            //imu_csv << acc_data.Z;
            imu_csv << gyro_data.Roll;
            imu_csv << gyro_data.Pitch;
            imu_csv << gyro_data.Yaw;
            imu_csv << rollDeg;
            imu_csv << pitchDeg;
            imu_csv << yawDeg;
            //imu_csv << mag_data.X;
            //imu_csv << mag_data.Y;
            //imu_csv << mag_data.Z;
            imu_csv << overturnFlag;
            imu_csv << skidFlag;
            imu_csv << turningFlag;
            imu_csv << gear;
            imu_csv << directionFlag;
            imu_csv << otSteer;
            imu_csv << steer;
            //imu_csv << speed;
            imu_csv << std::endl;
            imu_last_launch = bufferMag->LaunchedCount;

        }

        /*
        // Get the most recent gps data
        bufferGPS = gps->GetMostRecentBuffer<UserGPSBufferPtr>();
        if (bufferGPS->Buffer && bufferGPS->LaunchedCount > gps_last_launch) {
            // Save the gps data to file
            GPSData gps_data = bufferGPS->Buffer[0];
            gps_csv << std::fixed << std::setprecision(6);
            gps_csv << vehicle.GetChTime();
            gps_csv << gps_data.Time;       // Time
            gps_csv << gps_data.Latitude;   // Latitude
            gps_csv << gps_data.Longitude;  // Longitude
            gps_csv << gps_data.Altitude;   // Altitude
            gps_csv << std::endl;
            gps_last_launch = bufferGPS->LaunchedCount;
        }

        */
        UserXYZIBufferPtr lidar_data = lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
        if (lidar_data->Buffer) {
            num_lidar_updates++;
            // std::cout << "Data recieved from lidar. Frame: " << num_lidar_updates << std::endl;
        }

        //UserR8BufferPtr camera_data = cam->GetMostRecentBuffer<UserR8BufferPtr>();

        // Update sensor manager
        // Will render/save/filter automatically
        manager->Update();

        // Collect output data from modules (for inter-module communication)
        DriverInputs driver_inputs = driver->GetInputs();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

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
        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize(time, driver_inputs);

        // Initialize image output
        if (ot_controller) {
            driver->SetSteering(otSteer);
            if (std::abs(overturnSf*rollDeg) > overturnRoll && overturnFlag == false && skidFlag == false  && turningFlag == false) {
                overturnFlag = true;
                skidFlag = false;
                turningFlag = false;
                //
                vehicle.GetTransmission()->SetGear(-1);
                otSteer = 0.0;
                //
                if ((rollDeg-overturnHorizontal) > 0)
                {
                    directionFlag = 1; //// overturned to the right
                } else {
                    directionFlag = -1; //// overturned to the left
                }
                //
            }else if (std::abs(yawVelocity) > skidYawVelocity && std::abs(yawDeg) > skidYawAngle && overturnFlag == false && skidFlag == false && turningFlag == false && testFlag == false) {
                overturnFlag = false;
                skidFlag = true;
                turningFlag = false;
                testFlag = true;
                otTimer = vehicle.GetChTime();
                //
                vehicle.GetTransmission()->SetGear(-1);
                otSteer = 0.0;
                //
                if (yawDeg > 0)
                {
                    directionFlag = 1; //// overturned to the right
                } else {
                    directionFlag = -1; //// overturned to the left
                }
            }else if (std::abs(yawVelocity) > skidYawVelocity && std::abs(yawDeg-turnYaw) > skidYawAngle && overturnFlag == false && skidFlag == false && turningFlag == false && testFlag == true) {
                overturnFlag = false;
                skidFlag = true;
                turningFlag = false;
                otTimer = vehicle.GetChTime();
                //
                vehicle.GetTransmission()->SetGear(-1);
                otSteer = 0.0;
                //
                if ((yawDeg-turnYaw) > 0)
                {
                    directionFlag = 1; //// overturned to the right
                } else {
                    directionFlag = -1; //// overturned to the left
                }
            }else if (overturnFlag == true && std::abs(rollDeg) < overturnHorizontal) {
                overturnFlag = false;
                skidFlag = false;
                turningFlag = true;
                //
                vehicle.GetTransmission()->SetGear(-1);
                tempYaw = yawDeg;
                //
                if (directionFlag == 1)
                {
                    otSteer = -1.0;
                } else {
                    otSteer = 1.0;
                }
            }else if (skidFlag ==true && std::abs(rollDeg) < overturnHorizontal && (vehicle.GetChTime()-otTimer) > 5) {
                overturnFlag = false;
                skidFlag = false;
                turningFlag = true;
                //
                vehicle.GetTransmission()->SetGear(-1);
                tempYaw = yawDeg;
                //
                if (directionFlag == 1)
                {
                    otSteer = -1.0;
                } else {
                    otSteer = 1.0;
                }
            }else if (turningFlag == true) {
                directionFlag = 0;
                if (std::abs(yawDeg-tempYaw) < 10){
                    // nothing
                } else {
                    skidFlag = false;
                    testFlag = true;
                    otSteer = 0.0;
                    vehicle.GetTransmission()->SetGear(1);
                    turnYaw = yawDeg;
                }
            } else {
                // nothing
            }
        } else {
            // nothing
        }

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

        // Increment frame number
        step_number++;
    }

    imu_csv.write_to_file(imu_file);
    //gps_csv.write_to_file(gps_file);

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system) {
    int rock_pos_x = 0;
    double rock_pos_y = 0;
    for (int i = 0; i < 2; i++) {
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

        // set the abs orientation, position and velocity
        auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        //ChQuaternion<> rock_rot = ChQuaternion<>(1, 0, 0, 0);
        ChQuaternion<> rock_rot = Q_from_AngX(CH_C_PI_2);
        ChVector<> rock_pos;

        rock_pos_x = i*3;
        //rock_pos_y = 1.2695*(-1)^i;
        rock_pos_y = 1.2695*std::pow(-1, i);
        rock_pos = ChVector<>(rock_pos_x, rock_pos_y, 0);

        rock_Body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));

        rock_Body->SetMass(mass * density);
        rock_Body->SetInertiaXX(density * principal_I);

        rock_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos), ChQuaternion<>(rock_rot)));
        system->Add(rock_Body);

        rock_Body->SetBodyFixed(true);
        rock_Body->GetCollisionModel()->ClearModel();
        rock_Body->GetCollisionModel()->AddTriangleMesh(mesh_mat, mesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        rock_Body->GetCollisionModel()->BuildModel();
        rock_Body->SetCollide(true);

        auto rock_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        rock_mesh->SetMesh(mesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);
    }
}
