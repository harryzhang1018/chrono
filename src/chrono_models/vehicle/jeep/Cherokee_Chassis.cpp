//
// Created by Rainer Gericke on 16.04.24.
//

#include "Cherokee_Chassis.h"

namespace chrono {
namespace vehicle {
namespace jeep {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Cherokee_Chassis::m_body_mass = 1663.0;
const ChVector3d Cherokee_Chassis::m_body_inertiaXX(653, 2498, 2704);
const ChVector3d Cherokee_Chassis::m_body_inertiaXY(0, 0, 85);
const ChVector3d Cherokee_Chassis::m_body_COM_loc(-1.147, 0, 0.213);
const ChVector3d Cherokee_Chassis::m_connector_rear_loc(-3.0, 0, -0.25);
const ChCoordsys<> Cherokee_Chassis::m_driverCsys(ChVector3d(-1.5, 0.7, 0.4), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Cherokee_Chassis::Cherokee_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single contact material.
    ChContactMaterialData minfo;
    minfo.mu = 1.0f;
    minfo.cr = 0.1f;
    minfo.Y = 5e5f;
    m_geometry.m_materials.push_back(minfo);

    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXY.x();
    m_body_inertia(0, 2) = m_body_inertiaXY.y();
    m_body_inertia(1, 2) = m_body_inertiaXY.z();
    m_body_inertia(1, 0) = m_body_inertiaXY.x();
    m_body_inertia(2, 0) = m_body_inertiaXY.y();
    m_body_inertia(2, 1) = m_body_inertiaXY.z();

    //// A more appropriate contact shape from primitives
    ChVehicleGeometry::BoxShape box1(ChVector3d(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector3d(2.0, 1.0, 0.2));
    ChVehicleGeometry::BoxShape box2(ChVector3d(0.0, 0.0, 0.3), ChQuaternion<>(1, 0, 0, 0), ChVector3d(1.0, 0.5, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);
    m_geometry.m_vis_boxes.push_back(box2);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "jeep/JeepCherokee.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            ChVehicleGeometry::ConvexHullsShape hull("jeep/JeepCherokee_col.obj", 0);
            m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        case CollisionType::MESH: {
            ChVehicleGeometry::TrimeshShape trimesh(ChVector3d(), "jeep/JeepCherokee_col.obj", 0.005, 0);
            m_geometry.m_coll_meshes.push_back(trimesh);
            break;
        }
        default:
            break;
    }
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono