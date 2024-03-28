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
// Demonstration program for M113 vehicle on rigid terrain.
//
// =============================================================================

#include <sstream>

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

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

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"

#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_models/vehicle/m113/M113.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/SetChronoSolver.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::ros;
using namespace chrono::sensor;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
DoublePinTrackShoeType shoe_topology = DoublePinTrackShoeType::ONE_CONNECTOR;
BrakeType brake_type = BrakeType::SHAFTS;
DrivelineTypeTV driveline_type = DrivelineTypeTV::BDS;

bool use_track_bushings = false;
bool use_suspension_bushings = false;
bool use_track_RSDA = false;

bool fix_chassis = false;
bool create_track = true;

// Initial vehicle position
ChVector3d initLoc(-0.0, 0, 0.8);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
////ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
////ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
////ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 200.0;  // size in X direction
double terrainWidth = 200.0;   // size in Y direction


// Contact formulation (NSC or SMC)
ChContactMethod contact_method = ChContactMethod::SMC;

// Simulation step size
double step_size_NSC = 1e-3;
double step_size_SMC = 5e-4;

// Verbose output level (solver and integrator)
bool verbose_solver = false;
bool verbose_integrator = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 0.0);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "M113";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system);
void AddFallingObjects(ChSystem* system);

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
    std::cout << ss.str() << std::endl;
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
        std::cout << vmax << "  in  " << nmax << " [" << imax << "]" << std::endl;
}

bool ReportTrackFailure(ChTrackedVehicle& veh, double threshold = 1e-2) {
    for (int i = 0; i < 2; i++) {
        auto track = veh.GetTrackAssembly(VehicleSide(i));
        auto nshoes = track->GetNumTrackShoes();
        if (nshoes <= 0)
            continue;
        auto shoe1 = track->GetTrackShoe(0).get();
        for (int j = 1; j < nshoes; j++) {
            auto shoe2 = track->GetTrackShoe(j % (nshoes - 1)).get();
            auto dir = shoe2->GetShoeBody()->TransformDirectionParentToLocal(shoe2->GetTransform().GetPos() -
                                                                             shoe1->GetTransform().GetPos());
            if (std::abs(dir.y()) > threshold) {
                std::cout << "...Track " << i << " broken between shoes " << j - 1 << " and " << j << std::endl;
                std::cout << "time " << veh.GetChTime() << std::endl;
                std::cout << "shoe " << j - 1 << " position: " << shoe1->GetTransform().GetPos() << std::endl;
                std::cout << "shoe " << j << " position: " << shoe2->GetTransform().GetPos() << std::endl;
                std::cout << "Lateral offset: " << dir.y() << std::endl;
                return true;
            }
            shoe1 = shoe2;
        }
    }
    return false;
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

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

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    ChCollisionSystem::Type collsys_type = ChCollisionSystem::Type::BULLET;
    CollisionType chassis_collision_type = CollisionType::NONE;

    M113 m113;
    m113.SetContactMethod(contact_method);
    m113.SetCollisionSystemType(collsys_type);
    m113.SetTrackShoeType(shoe_type);
    m113.SetDoublePinTrackShoeType(shoe_topology);
    m113.SetTrackBushings(use_track_bushings);
    m113.SetSuspensionBushings(use_suspension_bushings);
    m113.SetTrackStiffness(use_track_RSDA);
    m113.SetDrivelineType(driveline_type);
    m113.SetBrakeType(brake_type);
    m113.SetEngineType(EngineModelType::SIMPLE_MAP);
    m113.SetTransmissionType(TransmissionModelType::AUTOMATIC_SIMPLE_MAP);
    m113.SetChassisCollisionType(chassis_collision_type);

    m113.SetChassisFixed(fix_chassis);
    m113.CreateTrack(create_track);

    // Control steering type (enable crossdrive capability)
    ////m113.GetDriveline()->SetGyrationMode(true);

    // Change collision shape for road wheels and idlers (true: cylinder; false: cylshell)
    ////m113.SetWheelCollisionType(true, true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    auto& vehicle = m113.GetVehicle();

    // Set visualization type for vehicle components.
    VisualizationType track_vis =
        (shoe_type == TrackShoeType::SINGLE_PIN) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    m113.SetChassisVisualizationType(VisualizationType::NONE);
    m113.SetSprocketVisualizationType(VisualizationType::MESH);
    m113.SetIdlerVisualizationType(track_vis);
    m113.SetSuspensionVisualizationType(track_vis);
    m113.SetIdlerWheelVisualizationType(track_vis);
    m113.SetRoadWheelVisualizationType(track_vis);
    m113.SetTrackShoeVisualizationType(track_vis);

    vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the left idler.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::IDLER_LEFT);

    // Render contact normals and/or contact forces.
    vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

    // Demonstration of user callback for specifying contact between track shoe and
    // idlers and/or road wheels and/or ground.
    // This particular implementation uses a simple SMC-like contact force (normal only).
    class MyCustomContact : public ChTrackCustomContact {
      public:
        virtual bool OverridesIdlerContact() const override { return false; }
        virtual bool OverridesWheelContact() const override { return true; }
        virtual bool OverridesGroundContact() const override { return false; }

        virtual void ComputeIdlerContactForce(const ChCollisionInfo& cinfo,
                                              std::shared_ptr<ChBody> wheelBody,
                                              std::shared_ptr<ChBody> shoeBody,
                                              ChVector3d& forceShoe) override {
            ComputeContactForce(cinfo, wheelBody, shoeBody, forceShoe);
        };

        virtual void ComputeWheelContactForce(const ChCollisionInfo& cinfo,
                                              std::shared_ptr<ChBody> wheelBody,
                                              std::shared_ptr<ChBody> shoeBody,
                                              ChVector3d& forceShoe) override {
            ComputeContactForce(cinfo, wheelBody, shoeBody, forceShoe);
        };

        virtual void ComputeGroundContactForce(const ChCollisionInfo& cinfo,
                                               std::shared_ptr<ChBody> groundBody,
                                               std::shared_ptr<ChBody> shoeBody,
                                               ChVector3d& forceShoe) override {
            ComputeContactForce(cinfo, groundBody, shoeBody, forceShoe);
        };

      private:
        void ComputeContactForce(const ChCollisionInfo& cinfo,
                                 std::shared_ptr<ChBody> other,
                                 std::shared_ptr<ChBody> shoe,
                                 ChVector3d& forceShoe) {
            ////std::cout << other->GetName() << " " << shoe->GetName() << std::endl;

            if (cinfo.distance >= 0) {
                forceShoe = VNULL;
                return;
            }

            // Create a fictitious SMC composite contact material
            // (do not use the shape materials, so that this can work with both an SMC and NSC system)
            ChContactMaterialCompositeSMC mat;
            mat.E_eff = 2e7f;
            mat.cr_eff = 0.2f;

            auto delta = -cinfo.distance;
            auto normal_dir = cinfo.vN;
            auto p1 = cinfo.vpA;
            auto p2 = cinfo.vpB;
            auto objA = cinfo.modelA->GetContactable();
            auto objB = cinfo.modelB->GetContactable();
            auto vel1 = objA->GetContactPointSpeed(p1);
            auto vel2 = objB->GetContactPointSpeed(p2);

            ChVector3d relvel = vel2 - vel1;
            double relvel_n_mag = relvel.Dot(normal_dir);

            double eff_radius = 0.1;
            double eff_mass = objA->GetContactableMass() * objB->GetContactableMass() /
                              (objA->GetContactableMass() + objB->GetContactableMass());
            double Sn = 2 * mat.E_eff * std::sqrt(eff_radius * delta);
            double loge = std::log(mat.cr_eff);
            double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
            double kn = (2.0 / 3) * Sn;
            double gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);

            double forceN = kn * delta - gn * relvel_n_mag;
            forceShoe = (forceN < 0) ? VNULL : forceN * normal_dir;
        }
    };

    // Enable custom contact force calculation for road wheel - track shoe collisions.
    // If enabled, the underlying Chrono contact processing does not compute any forces.
    ////vehicle.EnableCustomContact(chrono_types::make_shared<MyCustomContact>());

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(m113.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 200, 200);
    terrain.Initialize();

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
    vis_mat_path->SetDiffuseColor(chrono::ChColor(0.0f, 1.0f, 0.0f));
    // Create ChBodyEasyBox objects at specified positions
    for (const auto& pos : positions) {
        auto box_body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.0001, 1000, true, false);
        box_body->SetPos(ChVector3d(std::get<0>(pos), std::get<1>(pos), 0.2));
        box_body->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/blue.png"));
        box_body->SetFixed(true);
        m113.GetSystem()->Add(box_body);
    }


    // Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChContactMaterial> rockSufaceMaterial = ChContactMaterial::DefaultMaterial(m113.GetSystem()->GetContactMethod());
    // Randomly shuffle the positions vector to select n unique positions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(positions.begin(), positions.end(), gen);
    // Uniform distribution from -1 to 1
    std::uniform_real_distribution<> dis(-1.0, 1.0);
    int n = std::atoi(argv[2]); // Number of boxes to add
    for (int i = 0; i < n; ++i) {
        double x = std::get<0>(positions[i]) + dis(gen);
        double y = std::get<1>(positions[i]) + dis(gen);
        // ChQuaternion<> rock_rot = Q_from_AngX(CH_C_PI_2);
        // auto rock_pos = ChVector<>(10, 0, 0);
        // auto rock_Body = chrono_types::make_shared<ChBodyAuxRef>();
        // rock_Body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));

        // rock_Body->SetMass(mass * density);
        // rock_Body->SetInertiaXX(density * principal_I);

        // rock_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock_pos), ChQuaternion<>(rock_rot)));

        // rock_Body->SetBodyFixed(true);
        // rock_Body->SetCollide(false);

        // auto rock_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        // rock_mesh->SetMesh(mesh);
        // rock_mesh->SetBackfaceCull(true);
        // rock_Body->AddVisualShape(rock_mesh);

        // sys->Add(rock_Body);


        // create a rock
        std::string rock_obj_path;
        if (i % 3 == 0) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        } else if (i % 3 == 1) {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock2.obj");
        } else {
            rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        }
        double scale_ratio = 0.15;
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
        m113.GetSystem()->Add(rock_Body);

        rock_Body->SetFixed(true);

        // auto rock_ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSufaceMaterial, rock_mmesh,
        //                                                                              false, false, 0.005);
        // rock_Body->AddCollisionShape(rock_ct_shape);
        // rock_Body->EnableCollision(false);

        auto rock_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_mesh->SetMesh(rock_mmesh);
        rock_mesh->SetBackfaceCull(true);
        rock_Body->AddVisualShape(rock_mesh);

        // m113.GetSystem()->Add(rock_Body);
        //m113.GetSystem()->GetCollisionSystem()->BindItem(rock_Body);
        //std::cout<<"done adding all the rocks"<<std::endl;
    }
    

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // and driver system
    // -----------------------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;
    auto driver = std::make_shared<ChDriver>(vehicle);
    
    // ----------------------------------------
    // add sensor
    auto sensor_manager = chrono_types::make_shared<ChSensorManager>(m113.GetSystem());
    sensor_manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 500);
    sensor_manager->scene->SetAmbientLight({0.1f, 0.1f, 0.1f});
    // Create a lidar and add it to the sensor manager
    chrono::ChFrame<double> offset_pose({0, 0, 3}, QuatFromAngleAxis(.0, {0, 1, 0}));
    auto lidar = chrono_types::make_shared<ChLidarSensor>(vehicle.GetChassisBody(), 10.f, offset_pose, 600, 200, CH_PI,
                                                          CH_PI / 12, -CH_PI / 6, 15.0f);
    lidar->PushFilter(chrono_types::make_shared<ChFilterDIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, 1, "3D Lidar"));
    sensor_manager->AddSensor(lidar);

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

    // Finally, initialize the ros manager
    ros_manager->Initialize();

    //--- visualization
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("M113 Vehicle Demo");
            vis_irr->SetChaseCamera(ChVector3d(0, 0, 0), 6.0, 0.5);
            ////vis_irr->SetChaseCameraPosition(vehicle.GetPos() + ChVector3d(0, 2, 0));
            vis_irr->SetChaseCameraMultipliers(1e-4, 10);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();

            vis = vis_irr;

#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("M113 Vehicle Demo");
            vis_vsg->SetChaseCamera(ChVector3d(0, 0, 0), 7.0, 0.5);
            vis_vsg->AttachVehicle(&m113.GetVehicle());
            ////vis_vsg->ShowAllCoGs(0.3);
            vis_vsg->Initialize();

            vis = vis_vsg;

#endif
            break;
        }
    }

    if (vehicle.GetNumTrackShoes(LEFT) > 0)
        std::cout << "Track shoe type: " << vehicle.GetTrackShoe(LEFT, 0)->GetTemplateName() << std::endl;
    std::cout << "Driveline type:  " << vehicle.GetDriveline()->GetTemplateName() << std::endl;
    std::cout << "Engine type: " << m113.GetVehicle().GetEngine()->GetTemplateName() << std::endl;
    std::cout << "Transmission type: " << m113.GetVehicle().GetTransmission()->GetTemplateName() << std::endl;
    std::cout << "Vehicle mass: " << vehicle.GetMass() << std::endl;

    vis->AttachVehicle(&m113.GetVehicle());

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Set up vehicle output
    ////vehicle.SetChassisOutput(true);
    ////vehicle.SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    ////vehicle.SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////vehicle.ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    double step_size = 1e-3;
    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            step_size = step_size_NSC;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            step_size = step_size_SMC;
            break;
    }

    SetChronoSolver(*m113.GetSystem(), ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    m113.GetSystem()->GetSolver()->SetVerbose(verbose_solver);
    m113.GetSystem()->GetTimestepper()->SetVerbose(verbose_integrator);

    std::cout << "SOLVER TYPE:     " << (int)m113.GetSystem()->GetSolver()->GetType() << std::endl;
    std::cout << "INTEGRATOR TYPE: " << (int)m113.GetSystem()->GetTimestepper()->GetType() << std::endl;

    // ---------------
    // Simulation loop
    // ---------------

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    while (vis->Run()) {
        //rendering
        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            // Zero-pad frame numbers in file names for postprocessing
            if (povray_output) {
                std::ostringstream filename;
                filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                chrono::utils::WriteVisualizationAssets(m113.GetSystem(), filename.str());
            }
            if (img_output && step_number > 200) {
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".jpg";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }

        // Current driver inputs
        // driver->SetThrottle(0.0f);
        // driver->SetSteering(0.0f);
        // driver->SetBraking(1.0f);
        DriverInputs driver_inputs = driver->GetInputs();

        //update sensor
        sensor_manager->Update();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver->Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        vis->Advance(step_size);

        ////ReportTiming(*m113.GetSystem());

        if (ReportTrackFailure(vehicle, 0.1)) {
            ReportConstraintViolation(*m113.GetSystem());
            break;
        }

        // Report if the chassis experienced a collision
        if (vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Update ROS managers
        if (!ros_manager->Update(time, step_size))
            break;

        // Increment frame number
        step_number++;
    }

    //vehicle.WriteContacts(out_dir + "/M113_contacts.out");

    return 0;
}
