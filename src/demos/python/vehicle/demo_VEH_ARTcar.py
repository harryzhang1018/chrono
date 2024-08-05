# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono.core as chrono
from pychrono import irrlicht as chronoirr
import pychrono.vehicle as veh
import math
import numpy as np
import time
from path_planner import path_planning, error_state
import argparse


# parser = argparse.ArgumentParser(description="Read two float variables as obstacle position from the command line.")
# parser.add_argument("obs_x", type=float, help="The first float variable")
# parser.add_argument("obs_y", type=float, help="The second float variable")

# args = parser.parse_args()

# print(f"obstacle at : ({args.obs_x},{args.obs_y})")
    
"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

#obstacle_pos = np.loadtxt('/home/harry/chrono_fork/obs_loc.csv',delimiter=',')
obstacle_pos = np.array([[3.945895,8.126506]])
ind = 1
for obs in obstacle_pos:
    obs_x, obs_y = obs[0], obs[1]
    # obs_x, obs_y = 1.997418,9.950001

    # Initial vehicle location and orientation
    initLoc = chrono.ChVector3d(0, 0.0, 0.5)
    # initRot = chrono.ChQuaterniond(1, 0, 0, 0)
    initRot = chrono.QuatFromAngleZ(1.57)

    vis_enable = True

    # Output directories
    out_dir = "./trajectory"
    pov_dir = out_dir + "/POVRAY"
    img_dir = out_dir + "/IMG"


    # Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
    chassis_vis_type = veh.VisualizationType_MESH
    suspension_vis_type = veh.VisualizationType_MESH
    steering_vis_type = veh.VisualizationType_MESH
    wheel_vis_type = veh.VisualizationType_MESH

    # Collision type for chassis (PRIMITIVES, MESH, or NONE)
    chassis_collision_type = veh.CollisionType_NONE

    # Type of tire model (RIGID, TMEASY)
    tire_model = veh.TireModelType_TMEASY

    # Rigid terrain
    # terrain_model = veh.RigidTerrain.BOX
    terrainHeight = 0      # terrain height
    terrainLength = 100.0  # size in X direction
    terrainWidth = 100.0   # size in Y direction

    # Poon chassis tracked by the camera
    trackPoint = chrono.ChVector3d(0.0, 0.0, 0.2)

    # Contact method
    contact_method = chrono.ChContactMethod_NSC
    contact_vis = False

    # Simulation step sizes
    step_size = 1e-3
    tire_step_size = step_size

    # Time interval between two render frames
    render_step_size = 1.0 / 25  # FPS = 25
    contro_step_size = 1.0 / 30 # control frequency is 10 Hz

    ref_path = np.genfromtxt('/home/harry/waypoints_paths/corner_turn.csv',delimiter=',')
    ref_speed = 5.0 # how fast you wanna go

    # helper function to search reference trajectory
    def search_ref_state(current_state,lookahead=1.0):
        x_current = current_state[0]
        y_current = current_state[1]
        dist = np.zeros((1,len(ref_path[:,1])))
        for i in range(len(ref_path[:,1])):
            dist[0][i] = dist[0][i] = (x_current-ref_path[i][0])**2+(y_current-ref_path[i][1])**2
        index = dist.argmin() + int(10 * lookahead)
        ref_state = list(ref_path[index,:])
        return ref_state

    # =============================================================================

    #print ( "Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n")

    # --------------
    # Create systems
    # --------------

    # Create the ARTcar vehicle, set parameters, and initialize
    car = veh.ARTcar()
    car.SetContactMethod(contact_method)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetChassisFixed(False)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.SetTireType(tire_model)
    car.SetTireStepSize(tire_step_size)
    car.SetMaxMotorVoltageRatio(0.3)
    car.SetStallTorque(0.6)
    car.SetTireRollingResistance(0.01)

    car.Initialize()

    tire_vis_type = veh.VisualizationType_MESH  # : VisualizationType::PRIMITIVES

    car.SetChassisVisualizationType(chassis_vis_type)
    car.SetSuspensionVisualizationType(suspension_vis_type)
    car.SetSteeringVisualizationType(steering_vis_type)
    car.SetWheelVisualizationType(wheel_vis_type)
    car.SetTireVisualizationType(tire_vis_type)

    car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(car.GetSystem())
    patch = terrain.AddPatch(patch_mat, 
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
        terrainLength, terrainWidth)
    # patch = terrain.AddPatch(patch_mat, 
    #     chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    #     veh.GetDataFile("terrain/meshes/hallway_ceilingless.obj"))

    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the rigid body as usual (this won't move, it is only for visualization tests)
    body = chrono.ChBody()
    body.SetFixed(True)
    car.GetSystem().Add(body)
    box = chrono.ChVisualShapeBox(0.05, 0.05, 0.01)
    box.SetColor(chrono.ChColor(0.0, 1.0, 0.0))
    for i in range(0, ref_path.shape[0]):
        # form path
        body.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(ref_path[i,0],ref_path[i,1],0), chrono.QUNIT))

    has_obstacle = True
    detect_obstacle = False
    observable_point = [4.0, 9.0]
    # observable_point = [1.0, 9.6]
    if has_obstacle:
        obstacle = [obs_x,obs_y]
        obs_pos = chrono.ChVector3d(obstacle[0], obstacle[1], 0.0)
        box_ob = chrono.ChVisualShapeBox(0.25, 0.25, 0.5)
        box_ob.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
        body.AddVisualShape(box_ob, chrono.ChFramed(obs_pos, chrono.QUNIT))
        

    # -------------------------------------
    # Create the vehicle Irrlicht interface
    # Create the driver system
    # -------------------------------------

    if vis_enable:
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle('dart')
        vis.SetWindowSize(1280, 1024)
        vis.SetChaseCamera(trackPoint, 3.0, 0.5)
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.SetImageOutput(True)
        vis.AttachVehicle(car.GetVehicle())


    driver = veh.ChDriver(car.GetVehicle())
    driver.Initialize()

    driver.Initialize()


    # ---------------
    # Simulation loop
    # ---------------

    # output vehicle mass
    print( "VEHICLE MASS: ",  car.GetVehicle().GetMass())

    # Number of simulation steps between miscellaneous events
    render_steps = math.ceil(render_step_size / step_size)
    control_steps = math.ceil(contro_step_size / step_size)

    # initialize control variables
    kp,ki,kd = 0.3,0.1,0.4
    integral_v_err = 0
    prev_v_err = 0
    steering_gain = [0.02176878 , 0.72672704 , 0.78409284 ,-0.0105355]
    # Initialize simulation frame counter s
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    render_frame = 0
    previous_steering = 0.0
    image_output = True
    vehicle_state = []
    sim_time = 0
    is_replaned = False
    while sim_time < 25 :
        sim_time = car.GetSystem().GetChTime()
        # Render scene and output POV-Ray data
        if vis_enable and (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            # filename = './IMG/img_' + str(render_frame + 1) +'.jpg' 
            # vis.WriteImageToFile(filename)
            # print("done rendering frame ", render_frame + 1)
            vis.EndScene()
            render_frame += 1
        
        if (step_number % control_steps == 0) :
            
            # get vehicle state
            veh_quat = car.GetVehicle().GetRot()
            e0 = veh_quat.e0
            e1 = veh_quat.e1
            e2 = veh_quat.e2
            e3 = veh_quat.e3
            theta = np.arctan2(2*(e0*e3+e1*e2),e0**2+e1**2-e2**2-e3**2)
            state = np.array([car.GetVehicle().GetPos().x, car.GetVehicle().GetPos().y, theta, car.GetVehicle().GetSpeed()])

            # distance from observation point
            dist = np.sqrt((state[0]-observable_point[0])**2+(state[1]-observable_point[1])**2)
            if dist<0.5:
                detect_obstacle = True
            ## perform path planning
            if has_obstacle:
                if detect_obstacle and not is_replaned:
                    print('obstacle detected')
                    ref_path = path_planning(ref_path, obstacle,magnitude=0.9,repulse_distance=3)
                    np.savetxt('new_path.csv', ref_path, delimiter=',',fmt='%f')
                    is_replaned = True
                    print('replanned')
                err = error_state(ref_path,state,lookahead=0.7)
            else:
                err = error_state(ref_path,state,lookahead=0.7)
            
            #use lateral error to control steering--pid controller
            steering = sum([x * y for x, y in zip(err, steering_gain)])
            
            #use longitudinal error to control throttle--pid controller
            v_err = ref_speed - state[3]
            contro_step_size
            P = kp * v_err
            integral_v_err += v_err * contro_step_size
            I = ki * integral_v_err
            D = kd * (v_err - prev_v_err) / contro_step_size
            prev_v_err = v_err
            throttle = max(min((P + I + D), 1), 0.3)

            # prevent steering from changing too fast
            delta_steering = steering -previous_steering
            if abs(delta_steering)>0.2:
                steering  = previous_steering + 0.2*np.sign(delta_steering)
                print('steering limit reached')
            previous_steering = steering
            
            # record vehicle trajectory
            vehicle_state.append([state[0],state[1],state[2],state[3],steering,throttle])
            
        
        # Get driver inputs
        driver.SetSteering(steering)
        driver.SetThrottle(throttle)
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        car.Synchronize(sim_time, driver_inputs, terrain)
        if vis_enable:
            vis.Synchronize(sim_time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)
        if vis_enable:
            vis.Advance(step_size)

        # Increment frame number
        step_number += 1

        # Spin in place for real time to catch up
        #realtime_timer.Spin(step_size)
    np.savetxt(f'./trajectory/traj_{ind}.csv',vehicle_state,delimiter=',',fmt='%f')
    ind += 1

