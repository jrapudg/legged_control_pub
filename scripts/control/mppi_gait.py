import os
import yaml

import numpy as np
from scipy.interpolate import CubicSpline

import mujoco
from mujoco import rollout

import concurrent.futures
import threading
from concurrent.futures import ThreadPoolExecutor

from scipy.spatial.transform import Rotation as R

def batch_world_to_local_velocity(quaternions, world_velocities):
    """
    Transforms a batch of world frame velocities to local body frame using quaternion orientations.
    
    Parameters:
    quaternions (np.array): Array of shape (t, 4) where each quaternion is [q_w, q_x, q_y, q_z]
    world_velocities (np.array): Array of shape (t, 3) where each velocity is [v_x, v_y, v_z]
    
    Returns:
    local_velocities (np.array): Array of shape (t, 3) with local body frame velocities
    """
    # Create a rotation object from the batch of quaternions (scipy expects [q_x, q_y, q_z, q_w])
    rotation = R.from_quat(quaternions[:, [1, 2, 3, 0]])  # Reordering to [q_x, q_y, q_z, q_w]
    
    # Apply the inverse rotation to each velocity vector in the batch
    local_velocities = rotation.inv().apply(world_velocities)
    return local_velocities

def calculate_orientation_quaternion(current_point, goal_point):
    """
    Calculates the orientation quaternion that points from the current point to the goal point
    with counterclockwise rotations around the z-axis (yaw) and y-axis (pitch).
    
    Parameters:
    current_point (np.array): The current position as [x, y, z].
    goal_point (np.array): The goal position as [x, y, z].
    
    Returns:
    quaternion (np.array): The orientation quaternion [q_w, q_x, q_y, q_z].
    """
    
    # Calculate the direction vector from the current point to the goal point
    direction = goal_point - current_point
    length = np.linalg.norm(direction)
    #print("Error: ", length)
    direction_normalized = direction / length
    
    # Calculate yaw (rotation around the z-axis)
    yaw = np.arctan2(direction_normalized[1], direction_normalized[0])
    
    # Calculate pitch (rotation around the y-axis)
    pitch = -np.arctan2(direction_normalized[2], np.sqrt(direction_normalized[0]**2 + direction_normalized[1]**2))
    
    # Convert yaw and pitch to quaternions
    yaw_quat = R.from_euler('z', yaw).as_quat()  # Yaw quaternion
    pitch_quat = R.from_euler('y', pitch).as_quat()  # Pitch quaternion
    
    # Combine the quaternions by multiplying them
    combined_rotation = R.from_quat(yaw_quat) * R.from_quat(pitch_quat)
    quaternion = combined_rotation.as_quat()
    
    # Reorder the quaternion to [q_w, q_x, q_y, q_z]
    quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
    
    return quaternion

class GaitScheduler:
    def __init__(self, gait_path = 'gaits/walking_gait_raibert_0_2.tsv', phase_time = 0):
        # Load the configuration file
        with open(gait_path, 'r') as file:
            gait_array = np.loadtxt(file, delimiter='\t')
        
        # Load model
        self.gait = gait_array
        self.phase_length = gait_array.shape[1]
        self.phase_time = phase_time
        self.indices = np.arange(self.phase_length)
        
    def roll(self):
        self.phase_time += 1
        self.indices = np.roll(self.indices, -1)
    
    def get_current_ref(self):
        return self.gait[:, self.phase_time] 

class MPPI:
    def __init__(self, model_path = os.path.join(os.path.dirname(__file__), "../models/go1/go1_scene_mppi_cf.xml"),
                 #config_path=os.path.join(os.path.dirname(__file__), "configs/mppi_two.yml")) -> None:
                 config_path=os.path.join(os.path.dirname(__file__), "configs/mppi_gait_config_walk_gazebo.yml")) -> None:
        # load the configuration file
        with open(config_path, 'r') as file:
            params = yaml.safe_load(file)

        self.internal_ref = False

        # Load model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = params['dt']
        self.model.opt.enableflags = 1 # to override contact settings
        self.model.opt.o_solref = np.array(params['o_solref'])

        # MPPI controller configuration
        self.temperature = params['lambda']
        self.horizon = params['horizon']
        self.n_samples = params['n_samples']
        self.noise_sigma = np.array(params['noise_sigma'])
        self.num_workers = params['n_workers']
        self.sampling_init = np.array([-0.3,  1.34, -2.83,  
                                       0.3,  1.34, -2.83,  
                                       -0.3,  1.34, -2.83,  
                                       0.3,  1.34, -2.83])
        
        # Cost
        self.Q = np.diag(np.array(params['Q_diag']))
        self.R = np.diag(np.array(params['R_diag']))
        self.x_ref = np.concatenate([np.array(params['q_ref']), np.array(params['v_ref'])])
        self.q_ref = np.array(params['q_ref'])
        self.v_ref = np.array(params['v_ref'])
        
        # Threding
        self.thread_local = threading.local()   
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers, initializer=self.thread_initializer)

        # Get env parameters
        self.act_dim = 12
        self.act_max = np.array([0.863, 4.501, -0.888]*4)
        self.act_min = np.array([-0.863, -0.686, -2.818]*4)
        
        # Gait scheduler
        self.gait_scheduler_in_place = GaitScheduler(gait_path ='gaits/walking_gait_raibert_0.tsv')
        self.gait_scheduler_walk = GaitScheduler(gait_path ='gaits/walking_gait_raibert_0_2.tsv')
        self.gait_scheduler = self.gait_scheduler_walk
        self.joints_ref = None
        #self.x_ref = jnp.concatenate([jnp.array(params['q_ref']), jnp.array(params['v_ref'])])
        
        # Rollouts
        self.h = params['dt']
        self.sample_type = params['sample_type']
        self.n_knots = params['n_knots']
        self.random_generator = np.random.default_rng(params["seed"])
        
        self.rollout_func = self.threaded_rollout
        self.cost_func = self.calculate_total_cost
        self.state_rollouts = np.zeros((self.n_samples, self.horizon, mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_FULLPHYSICS.value)))
            
        self.trajectory = None
        self.reset_planner() 
        self.goal_pos = [[1, 0, 0.27],
                         [2, 1, 0.27],
                         [2, 2, 0.27],
                         [1, 3, 0.27],
                         [0, 3, 0.27], 
                         [-1, 2, 0.27], 
                         [-1, 1, 0.27],
                         [0, 0, 0.27]]
                        
        #self.goal_pos = [[1, 0, 0.57]]
        # self.goal_pos = [[1, 0, 0.4],
        #                  [2, 1, 0.4],
        #                  [2, 2, 0.4],
        #                  [1, 3, 0.4],
        #                  [0, 3, 0.4], 
        #                  [-1, 2, 0.4], 
        #                  [-1, 1, 0.4],
        #                  [0, 0, 0.4]]
        
        self.goal_ori = [[1, 0, 0, 0]]
                         # #[0.819,-0.177,0.427,0.339], 
                         # [0.92388, 0, 0, 0.38268],
                         # [0.7071, 0, 0, 0.7071], 
                         # [0.38268, 0, 0, 0.92388],
                         # [0, 0, 0, 1],
                         # [-0.38268, 0, 0, 0.92388],
                         # [-0.7071, 0, 0, 0.7071],
                         # [-0.92388, 0, 0, 0.38268]]
        # self.goal_pos = [[1, 0, 0.67]]
        
        # self.goal_ori = [[1, 0, 0, 0]]

        self.goal_index = 0
        self.cmd_vel = np.array([0.2, 0])
        self.body_ref = np.concatenate((self.goal_pos[self.goal_index], 
                                        self.goal_ori[self.goal_index],
                                        self.cmd_vel,
                                        np.zeros(4)))

    def next_goal(self):
        if (self.goal_index == len(self.goal_pos) - 1):
            self.gait_scheduler = self.gait_scheduler_in_place
        elif (self.goal_index < len(self.goal_pos) - 1):
            self.goal_index = (self.goal_index + 1)
            self.body_ref[:3] = self.goal_pos[self.goal_index] 
        else:
            pass  
    #np.concatenate((self.goal_pos[self.goal_index], self.goal_ori[self.goal_index], self.cmd_vel, np.zeros(4)))
                
    def reset_planner(self):
        self.trajectory = np.zeros((self.horizon, self.act_dim))
        self.trajectory += self.sampling_init
            
    def generate_noise(self, size):
        return self.random_generator.normal(size=size) * self.noise_sigma
    
    def sample_delta_u(self):
        if self.sample_type == 'normal':
            size = (self.n_samples, self.horizon, self.act_dim)
            return self.generate_noise(size)
        elif self.sample_type == 'cubic':
            indices = np.arange(self.n_knots)*self.horizon//self.n_knots
            size = (self.n_samples, self.n_knots, self.act_dim)
            knot_points = self.generate_noise(size)
            cubic_spline = CubicSpline(indices, knot_points, axis=1)
            return cubic_spline(np.arange(self.horizon))
        
    def perturb_action(self):
        if self.sample_type == 'normal':
            size = (self.n_samples, self.horizon, self.act_dim)
            actions = self.trajectory + self.generate_noise(size)
            actions = np.clip(actions, self.act_min, self.act_max)
            return actions
        
        elif self.sample_type == 'cubic':
            indices_float = np.linspace(0, self.horizon - 1, num=self.n_knots)
            indices = np.round(indices_float).astype(int)
            size = (self.n_samples, self.n_knots, self.act_dim)
            noise = self.generate_noise(size)
            knot_points = self.trajectory[indices] + noise
            #knot_points[:, 0, :] = self.trajectory[0]
            cubic_spline = CubicSpline(indices, knot_points, axis=1)
            actions = cubic_spline(np.arange(self.horizon))
            actions = np.clip(actions, self.act_min, self.act_max)
            return actions
        
    def update(self, obs): 
        actions = self.perturb_action()
        self.obs = obs
        direction = self.body_ref[:3] - obs[:3]
        goal_delta = np.linalg.norm(direction)
        
        if (goal_delta > 0.1) and (self.gait_scheduler == self.gait_scheduler_walk): 
            self.goal_ori = calculate_orientation_quaternion(obs[:3], self.body_ref[:3])
        elif self.gait_scheduler == self.gait_scheduler_in_place:
            self.goal_ori = np.array([1,0,0,0])

        # print("Curernt Position",  obs[:3])
        # print("Desired Position",  self.body_ref[:3])
        # print("Orientation", orientation_goal)
        self.body_ref[3:7] = self.goal_ori
        self.rollout_func(self.state_rollouts, actions, np.repeat(np.array([np.concatenate([[0],obs])]), self.n_samples, axis=0), num_workers=self.num_workers, nstep=self.horizon)
        #states, actions, phase_time, joints_gait, body_ref
        
        if self.internal_ref:
            #print("Updating reference")
            self.joints_ref = self.gait_scheduler.gait[:, self.gait_scheduler.indices[:self.horizon]]
            
        costs_sum = self.cost_func(self.state_rollouts[:,:,1:], actions, 
                                   self.joints_ref, self.body_ref)
        
        self.gait_scheduler.roll()
        # MPPI weights calculation
        ## Scale parameters
        min_cost = np.min(costs_sum)
        max_cost = np.max(costs_sum)
        
        exp_weights = np.exp(-1/self.temperature * ((costs_sum - min_cost)/(max_cost - min_cost)))
        weighted_delta_u = exp_weights.reshape(self.n_samples, 1, 1) * actions
        weighted_delta_u = np.sum(weighted_delta_u, axis=0) / (np.sum(exp_weights) + 1e-10)
        updated_actions = np.clip(weighted_delta_u, self.act_min, self.act_max)
    
        # Pop out first action from the trajectory and repeat last action
        self.trajectory = np.roll(updated_actions, shift=-1, axis=0)
        self.trajectory[-1] = updated_actions[-1]

        # Output first action (MPC)
        action = updated_actions[0] 
        return action
    
    def shutdown(self):
        self.executor.shutdown(wait=True)

    def thread_initializer(self):
        self.thread_local.data = mujoco.MjData(self.model)

    def call_rollout(self, initial_state, ctrl, state):
        rollout.rollout(self.model, self.thread_local.data, skip_checks=True,
                        nroll=state.shape[0], nstep=state.shape[1],
                        initial_state=initial_state, control=ctrl, state=state)

    def threaded_rollout(self, state, ctrl, initial_state, num_workers=32, nstep=5):
        n = len(initial_state) // num_workers
        # Use list comprehension to create chunks for all workers
        chunks = [(initial_state[i*n:(i+1)*n], ctrl[i*n:(i+1)*n], state[i*n:(i+1)*n]) for i in range(num_workers-1)]
        # Add the last chunk, which includes any remainder
        chunks.append((initial_state[(num_workers-1)*n:], ctrl[(num_workers-1)*n:], state[(num_workers-1)*n:]))

        futures = [self.executor.submit(self.call_rollout, *chunk) for chunk in chunks]
        for future in concurrent.futures.as_completed(futures):
            future.result()
    
    def quaternion_distance_np(self, q1, q2):
        dot_products = np.einsum('ij,ij->i', q1, q2)
        return 1 - np.abs(dot_products)

    def quadruped_cost_np(self, x, u, x_ref):
        kp = 50
        kd = 3
        
        # Compute the error terms
        x_error = x - x_ref
        
        q_dist = self.quaternion_distance_np(x[:, 3:7], x_ref[:, 3:7])
        x_error[:, 3] = q_dist
        x_error[:, 4] = q_dist
        x_error[:, 5] = q_dist
        x_error[:, 6] = q_dist

        x_joint = x[:, 7:19]
        v_joint = x[:, 25:]
        u_error = kp * (u - x_joint) - kd * v_joint

        # Compute cost using einsum for precise matrix operations
        # Apply the matrix Q to x_error and R to u_error, sum over appropriate dimensions
        x_error[:, :3] = 0
        x_pos_error = x[:,:3] - x_ref[:,:3]
        L1_norm_pos_cost = np.abs(np.dot(x_pos_error, self.Q[:3,:3])).sum(axis=1)
        #L1_norm_pos_cost = 0

        cost = np.einsum('ij,ik,jk->i', x_error, x_error, self.Q) + np.einsum('ij,ik,jk->i', u_error, u_error, self.R) + L1_norm_pos_cost
        return cost

    def calculate_total_cost(self, states, actions, joints_ref, body_ref):
        num_samples = states.shape[0]
        num_pairs = states.shape[1]

        traj_body_ref = np.repeat(body_ref[np.newaxis,:], num_samples*num_pairs, axis=0)

        # Flatten states and actions to two dimensions, treating all pairs per sample as a batch
        states = states.reshape(-1, states.shape[2])
        actions = actions.reshape(-1, actions.shape[2])

        joints_ref = joints_ref.T
        joints_ref = np.tile(joints_ref, (num_samples, 1, 1))
        joints_ref = joints_ref.reshape(-1, joints_ref.shape[2])
        
        x_ref = np.concatenate([traj_body_ref[:,:7], joints_ref[:,:12], 
                                traj_body_ref[:,7:], joints_ref[:,12:]], 
                                axis=1)
        # Compute batch costs
        rotated_ref = batch_world_to_local_velocity(states[:,3:7], states[:,19:22])
        states[:,19:22] = rotated_ref
        costs = self.quadruped_cost_np(states, actions, x_ref)
        # Sum costs for each sample
        total_costs = costs.reshape(num_samples, num_pairs).sum(axis=1)
        return total_costs
    
    def __del__(self):
        self.shutdown()
    
if __name__ == "__main__":
    mppi = MPPI()