import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

def path_planning(ref_trajectory, obstacle, magnitude = 0.7, repulse_distance=5,lateral_clearance=0.2):
    def distance(point1, point2):
        """ Calculate Euclidean distance between two points """
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def repulsive_force(point, obstacle, scale=1.0):
        """ Calculate the repulsive force from the obstacle on a waypoint """
        # print(obstacle)
        dist = distance(point, obstacle)
        if dist < repulse_distance:
            force_magnitude = scale * np.exp(-dist/10)
            force_direction = np.array([point[0] - obstacle[0], point[1] - obstacle[1]])
            force_direction = force_direction / np.linalg.norm(force_direction)
            new_point = (point[0] + force_direction[0] * force_magnitude,
                         point[1] + force_direction[1] * force_magnitude)
            return new_point
        return point

    if obstacle is None:
        return ref_trajectory
    # add small perturbation to the obstacle to avoid division by zero
    obstacle = [obstacle[0] + np.random.uniform(-0.01,0.01), obstacle[1]+ np.random.uniform(-0.01,0.01)]
    # Calculate minimum distance from the obstacle to the reference trajectory
    min_distance = min(distance(waypoint[:2], obstacle) for waypoint in ref_trajectory)
    if min_distance < lateral_clearance:
        closest_waypoint = min(ref_trajectory, key=lambda waypoint: distance(waypoint[:2], obstacle))
        direction_to_obstacle = np.array([obstacle[0] - closest_waypoint[0], obstacle[1] - closest_waypoint[1]])
        norm_direction = direction_to_obstacle / np.linalg.norm(direction_to_obstacle)
        obstacle = (closest_waypoint[0] + norm_direction[0] * lateral_clearance,
                    closest_waypoint[1] + norm_direction[1] * lateral_clearance)
    # Modify waypoints if the obstacle is close
    modified_trajectory = []
    index_modified = []
    start_index = 0
    for waypoint in ref_trajectory:
        if distance(waypoint[:2], obstacle) < repulse_distance:
            #print("Obstacle detected")
            modified_point = repulsive_force(waypoint[:2], obstacle,scale = magnitude)
            modified_trajectory.append([modified_point[0], modified_point[1], waypoint[2], waypoint[3]])
            index_modified.append(start_index)
        start_index += 1
    #print('modified index:',index_modified)
    # calculate heading for the modified trajectory
    for i in range(len(modified_trajectory) - 1):
        dx = modified_trajectory[i + 1][0] - modified_trajectory[i][0]
        dy = modified_trajectory[i + 1][1] - modified_trajectory[i][1]
        modified_trajectory[i][2] = np.arctan2(dy, dx)

    if len(modified_trajectory) > 0:  # Update the last heading to be the same as the second last
        modified_trajectory[-1][2] = modified_trajectory[-2][2]
        
    truncate_lenth = 8
    for i in range(truncate_lenth):
        modified_trajectory[i] = modified_trajectory[truncate_lenth]
        modified_trajectory[-1-i] = modified_trajectory[-1-truncate_lenth]
    for i in range(len(modified_trajectory)):
        ref_trajectory[index_modified[i]] = modified_trajectory[i]
        
    result_trajectory = ref_trajectory
    # # Update heading angles and smooth transitions
    # for i in range(len(modified_trajectory) - 1):
    #     dx = modified_trajectory[i + 1][0] - modified_trajectory[i][0]
    #     dy = modified_trajectory[i + 1][1] - modified_trajectory[i][1]
    #     modified_trajectory[i][2] = np.arctan2(dy, dx)

    # if len(modified_trajectory) > 0:  # Update the last heading to be the same as the second last
    #     modified_trajectory[-1][2] = modified_trajectory[-2][2]

    return np.array(result_trajectory)

def error_state(ref_traj,current_state,lookahead = 1.0):
    x_current = current_state[0]
    y_current = current_state[1]
    theta_current = current_state[2]
    v_current = current_state[3]
    #post process theta
    while theta_current<-np.pi:
        theta_current = theta_current+2*np.pi
    while theta_current>np.pi:
        theta_current = theta_current - 2*np.pi

    dist = np.zeros((1,len(ref_traj[:,1])))
    for i in range(len(ref_traj[:,1])):
        dist[0][i] = dist[0][i] = (x_current+np.cos(theta_current)*lookahead-ref_traj[i][0])**2+(y_current+np.sin(theta_current)*lookahead-ref_traj[i][1])**2
    index = dist.argmin()
    ref_state_current = list(ref_traj[index,:])
    err_theta = 0
    ref = ref_state_current[2]
    act = theta_current

    if( (ref>0 and act>0) or (ref<=0 and act <=0)):
        err_theta = ref-act
    elif( ref<=0 and act > 0):
        if(abs(ref-act)<abs(2*np.pi+ref-act)):
            err_theta = -abs(act-ref)
        else:
            err_theta = abs(2*np.pi + ref- act)
    else:
        if(abs(ref-act)<abs(2*np.pi-ref+act)):
            err_theta = abs(act-ref)
        else: 
            err_theta = -abs(2*np.pi-ref+act)
    RotM = np.array([ 
            [np.cos(-theta_current), -np.sin(-theta_current)],
            [np.sin(-theta_current), np.cos(-theta_current)]
        ])

    errM = np.array([[ref_state_current[0]-x_current],[ref_state_current[1]-y_current]])

    errRM = RotM@errM


    error_state = [errRM[0][0],errRM[1][0],err_theta, ref_state_current[3]-v_current]

    return error_state

def test_algo():
    ref_trajectory = np.genfromtxt('/home/harry/transient_based_robotic_navigation/control/corner_turn.csv', delimiter=',')
    plt.plot(ref_trajectory[:,0], ref_trajectory[:,1], label='Reference Path')
    obstacle = [3.800000,7.599841]
    new_path = path_planning(ref_trajectory, obstacle,magnitude=0.9,repulse_distance=3)
    np.savetxt('new_path.csv', new_path, delimiter=',',fmt='%f')
    
    plt.plot(new_path[:,0], new_path[:,1], label='New Path')
    plt.scatter(obstacle[0], obstacle[1], color='red', label='Obstacle')
    plt.legend()
    plt.show()

# test_algo()