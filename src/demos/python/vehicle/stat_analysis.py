import numpy as np
import matplotlib.pyplot as plt

def generate_rand_obs_loc(reference_traj,start_ind=1,end_ind=100):
    lateral_displancement = [0.05,0.1,0.2]
    obs_loc = []
    for i in range(start_ind, end_ind):
        theta = reference_traj[i][2]
        delta_x = [np.sin(theta)*d for d in lateral_displancement]
        delta_y = [np.cos(theta)*d for d in lateral_displancement]
        for j in range(len(delta_x)):
            obs_loc.append([reference_traj[i][0]+delta_x[j],reference_traj[i][1]-delta_y[j]])
        for k in range(len(delta_x)):
            obs_loc.append([reference_traj[i][0]-delta_x[k],reference_traj[i][1]+delta_y[k]])
            
    return obs_loc

ref_path = np.genfromtxt('/home/harry/waypoints_paths/corner_turn.csv',delimiter=',')
obs_loc = generate_rand_obs_loc(ref_path,106,150)
np.savetxt('obs_loc.csv',obs_loc,delimiter=',',fmt='%f')
show_plots = True  
if show_plots:
    plt.plot(ref_path[:,0],ref_path[:,1],label='Reference Path')
    plt.scatter([obs[0] for obs in obs_loc],[obs[1] for obs in obs_loc],label='Obstacle Location')
    plt.legend()
    plt.show()

def check_traj_collision(traj, obs_loc, plot=True):
    # check if any point on trajectory is within 0.3 meters of obstacle
    collision = False
    for point in traj:
        if np.linalg.norm(np.array(point[0:2])-np.array(obs_loc),ord=2) < 0.4:
            collision = True
            break
    if plot:
        plt.plot([point[0] for point in traj],[point[1] for point in traj],label='Trajectory')
        plt.scatter(obs_loc[0],obs_loc[1],label='Obstacle Location')
        #plt.legend()
    return collision
        

folder_traj = '/home/harry/chrono_fork/src/demos/python/vehicle/trajectory/'
obstacle_loc = np.genfromtxt('obs_loc.csv',delimiter=',')

num_collisions = 0
col_ind = []
for i in range(200):
    #print('Checking Trajectory: ',i+1)
    traj = np.genfromtxt(folder_traj+'traj_'+str(i+1)+'.csv',delimiter=',')
    
    col = check_traj_collision(traj,obstacle_loc[i],plot=True)
    if col:
        num_collisions+=1
        col_ind.append(i)
print('Number of Collisions: ',num_collisions)
plt.figure()
#plot all colliding obstacle locations using col_ind
for i in col_ind:
    traj = np.genfromtxt(folder_traj+'traj_'+str(i+1)+'.csv',delimiter=',')
    check_traj_collision(traj,obstacle_loc[i],plot=True)
    plt.legend()
plt.show()
