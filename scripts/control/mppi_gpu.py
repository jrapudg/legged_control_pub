import os
import yaml

import numpy as np
from scipy.interpolate import CubicSpline

import jax
import jax.numpy as jnp

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
        self.noise_sigma = jnp.array(params['noise_sigma'])
        self.num_workers = params['n_workers']
        self.sampling_init = jnp.array([0.073,  1.34 , -2.83 ,  
                                        0.073,  1.34 , -2.83 ,  
                                        0.073,  1.34 , -2.83 ,  
                                        0.073,  1.34 , -2.83 ])
        
        # Cost
        self.Q = jnp.diag(jnp.array(params['Q_diag']))
        self.R = jnp.diag(jnp.array(params['R_diag']))
        self.x_ref = jnp.concatenate([jnp.array(params['q_ref']), jnp.array(params['v_ref'])])

        # threding
        self.thread_local = threading.local()   
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers, initializer=self.thread_initializer)

        # Get env parameters
        self.act_dim = 12
        self.act_max = [0.863, 4.501, -0.888]*4
        self.act_min = [-0.863, -0.686, -2.818]*4
        
        # Rollouts
        self.h = params['dt']
        self.sample_type = params['sample_type']
        self.n_knots = params['n_knots']
        self.random_generator = np.random.default_rng(params["seed"])
        
        self.rollout_func = self.threaded_rollout
        self.cost_func = jax.jit(jax.vmap(jax.vmap(self.quadruped_cost, in_axes=(0, 0)), in_axes=(1, 1))) #, device=gpu_device)
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
            indices_float = jnp.linspace(0, self.horizon - 1, num=self.n_knots)
            indices = jnp.round(indices_float).astype(int)
            size = (self.n_samples, self.n_knots, self.act_dim)
            knot_points = self.trajectory[indices] + self.generate_noise(size)
            cubic_spline = CubicSpline(indices, knot_points, axis=1)
            actions = cubic_spline(np.arange(self.horizon))
            actions = np.clip(actions, self.act_min, self.act_max)
            return actions
        
    def update(self, obs): 
        actions = self.perturb_action()
        self.rollout_func(self.state_rollouts, actions, np.repeat([np.concatenate([[0],obs])], self.n_samples, axis=0), num_workers=self.num_workers, nstep=self.horizon)
        costs = self.cost_func(self.state_rollouts[:,:,1:], actions)
          
        costs_sum = costs.sum(axis=0)
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


        with concurrent.futures.ThreadPoolExecutor(
            max_workers=num_workers, initializer=self.thread_initializer) as executor:
            futures = []
            for chunk in chunks:
                futures.append(executor.submit(self.call_rollout, *chunk))
            for future in concurrent.futures.as_completed(futures):
                future.result()
    
    def quaternion_distance(self, q1, q2):
        return 1 - jnp.abs(jnp.dot(q1,q2))
    
    def quadruped_cost(self, x, u):
        kp = 30
        kd = 3
        # Compute the error terms
        x_error = x - self.x_ref
        # Assuming quaternion_distance is a function you've defined elsewhere
        x_error = x_error.at[3:7].set(self.quaternion_distance(x[3:7], self.x_ref[3:7]))
        u_error = kp*(u - x[7:19]) - kd*x[25:]

        # Compute the cost
        cost = jnp.dot(x_error, jnp.dot(self.Q, x_error)) + jnp.dot(u_error, jnp.dot(self.R, u_error))
        return cost
    
    def __del__(self):
        self.shutdown()
    
if __name__ == "__main__":
    mppi = MPPI()
    import timeit 
    update_time = timeit.timeit(lambda: mppi.update(jnp.zeros(37)), number=1000)/1000
    print("update frequency: ", 1/update_time)