import os
import yaml
#!/usr/bin/env python

import numpy as np
from scipy.interpolate import CubicSpline

import mujoco
from mujoco import rollout

import concurrent.futures
import threading
from concurrent.futures import ThreadPoolExecutor

class MPPI:
    def __init__(self, model_path = os.path.join(os.path.dirname(__file__), "../models/go1/go1_scene_jax_no_collision.xml"),
                 config_path=os.path.join(os.path.dirname(__file__), "configs/mppi.yml")) -> None:
        # load the configuration file
        with open(config_path, 'r') as file:
            params = yaml.safe_load(file)

        # load model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = params['dt']
        self.model.opt.enableflags = 1 # to override contact settings
        self.model.opt.o_solref = np.array(params['o_solref'])

        # mppi controller onfiguration
        self.temperature = params['lambda']
        self.horizon = params['horizon']
        self.n_samples = params['n_samples']
        self.noise_sigma = np.array(params['noise_sigma'])
        self.num_workers = params['n_workers']
        self.sampling_init = np.array([0.073,  1.34 , -2.83 ,  
                                        0.073,  1.34 , -2.83 ,  
                                        0.073,  1.34 , -2.83 ,  
                                        0.073,  1.34 , -2.83 ])
        
        # cost
        self.Q = np.diag(np.array(params['Q_diag']))
        self.R = np.diag(np.array(params['R_diag']))
        self.x_ref = np.concatenate([np.array(params['q_ref']), np.array(params['v_ref'])])

        # threding
        self.thread_local = threading.local()   
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers, initializer=self.thread_initializer)

        # get env parameters
        self.act_dim = 12
        self.act_max = np.array([0.863, 4.501, -0.888]*4)
        self.act_min = np.array([-0.863, -0.686, -2.818]*4)
        
        # rollouts
        self.h = params['dt']
        self.sample_type = params['sample_type']
        self.n_knots = params['n_knots']
        self.random_generator = np.random.default_rng(params["seed"])
        
        self.rollout_func = self.threaded_rollout
        self.cost_func = self.calculate_total_cost
        self.state_rollouts = np.zeros((self.n_samples, self.horizon, mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_FULLPHYSICS.value)))
            
        self.trajectory = None
        self.reset_planner() 
        self.update(self.x_ref)
        self.reset_planner()      
                
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
            knot_points = self.trajectory[indices] + self.generate_noise(size)
            cubic_spline = CubicSpline(indices, knot_points, axis=1)
            actions = cubic_spline(np.arange(self.horizon))
            actions = np.clip(actions, self.act_min, self.act_max)
            return actions
        
    def update(self, obs): 
        actions = self.perturb_action()
        self.rollout_func(self.state_rollouts, actions, np.repeat(np.array([np.concatenate([[0],obs])]), self.n_samples, axis=0), num_workers=self.num_workers, nstep=self.horizon)
        costs_sum = self.cost_func(self.state_rollouts[:,:,1:], actions)
          
        #costs_sum = costs.sum(axis=0)
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
        return 1 - np.abs(np.dot(q1,q2))

    def quadruped_cost_np(self, x, u):
        kp = 30
        kd = 3
        x_error = x - self.x_ref
        q_dist = self.quaternion_distance_np(x[:, 3:7], self.x_ref[3:7])
        x_error[:, 3] = q_dist
        x_error[:, 4] = q_dist
        x_error[:, 5] = q_dist
        x_error[:, 6] = q_dist

        u_error = kp * (u - x[:, 7:19]) - kd * x[:, 25:]

        # Compute cost using einsum for precise matrix operations
        # Apply the matrix Q to x_error and R to u_error, sum over appropriate dimensions
        cost = np.einsum('ij,ik,jk->i', x_error, x_error, self.Q) + np.einsum('ij,ik,jk->i', u_error, u_error, self.R)
        return cost

    def calculate_total_cost(self, states, actions):
        num_samples = states.shape[0]
        num_pairs = states.shape[1]

        # Flatten states and actions to two dimensions, treating all pairs per sample as a batch
        states = states.reshape(-1, states.shape[2])
        actions = actions.reshape(-1, actions.shape[2])
        # Compute batch costs
        costs = self.quadruped_cost_np(states, actions)
        # Sum costs for each sample
        total_costs = costs.reshape(num_samples, num_pairs).sum(axis=1)

        return total_costs
    
    def __del__(self):
        self.shutdown()

if __name__ == "__main__":
    mppi = MPPI()
    import timeit 
    update_time = timeit.timeit(lambda: mppi.update(np.zeros(37)), number=1000)/1000
    print("update frequency: ", 1/update_time)