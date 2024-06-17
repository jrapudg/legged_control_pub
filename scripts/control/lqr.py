import mujoco
import numpy as np
import os
import yaml
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are, inv
from estimation.quat_utils import log_map, exp_map

class LQR:
    def __init__(self, model_path = os.path.join(os.path.dirname(__file__), "../models/go1/task_filter_mocap.xml"),
                 config_path=os.path.join(os.path.dirname(__file__), "configs/lqr.yml")) -> None:
        # load the configuration file
        with open(config_path, 'r') as file:
            params = yaml.safe_load(file)

        self.model = mujoco.MjModel.from_xml_path(model_path)
        # self.model.opt.enableflags = 1
        self.model.opt.o_solref = np.array([0.005, 1])

        self.data = mujoco.MjData(self.model)
        self.reset()

        self.nx = self.model.nv*2
        self.nu = self.model.nu
        self.A = np.zeros((self.nx, self.nx))
        self.B = np.zeros((self.nx, self.nu))
        self.Q = np.diag(params['Q_diag'])
        self.R = np.diag(params['R_diag'])

        self.compute_feedback_policy()
        self.linearize_dynamics()
        print("condition number of A: ", np.linalg.cond(self.A))
        print("condition number of B: ", np.linalg.cond(self.B))
        self.state_desired = self.get_state()
        return None
    
    def get_state(self):
        return np.concatenate([self.data.qpos, self.data.qvel], axis=0)
    
    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        return None
    
    def linearize_dynamics(self, epsilon=1e-6, centered=True):
        """
        Linearize the dynamics of the system
        """
        mujoco.mjd_transitionFD(self.model, self.data, epsilon, centered, self.A, self.B, None, None)
        return None
    
    def compute_feedback_policy(self):
        self.linearize_dynamics()
        self.P = solve_discrete_are(self.A, self.B, self.Q, self.R)
        self.K = inv(self.R + self.B.T @ self.P @ self.B) @ (self.B.T @ self.P @ self.A)
        return None

    def update(self, state_estimate):
        # mujoco.mj_setState(self.model, self.data, state_estimate, spec=mujoco.mjtState.mjSTATE_PHYSICS)
        # mujoco.mj_forward(self.model, self.data)
        ctrl = -self.K @ self.state_difference(state_estimate, self.state_desired)
        return ctrl + self.data.ctrl
    
    def state_difference(self, state1, state2):
        state1 = np.concatenate([state1[:3], log_map(state1[3:7]), state1[7:]], axis=0)
        state2 = np.concatenate([state2[:3], log_map(state2[3:7]), state2[7:]], axis=0)
        return state1 - state2

if __name__ == "__main__":
    lqr = LQR()
    # plt.imshow(lqr.K)
    # plt.colorbar()
    # plt.show()

