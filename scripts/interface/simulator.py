import mujoco
import os
import mujoco_viewer
import matplotlib.pyplot as plt
import numpy as np
#from estimation.ekf import EKF
#from control.lqr import LQR
import tqdm

class Simulator:
    """
    A class representing a simulator for controlling and estimating the state of a system.
    
    Attributes:
        filter (object): The filter used for state estimation.
        agent (object): The agent used for control.
        model_path (str): The path to the XML model file.
        T (int): The number of time steps.
        dt (float): The time step size.
        viewer (bool): Flag indicating whether to enable the viewer.
        gravity (bool): Flag indicating whether to enable gravity.
        model (object): The MuJoCo model.
        data (object): The MuJoCo data.
        qpos (ndarray): The position trajectory.
        qvel (ndarray): The velocity trajectory.
        finite_diff_qvel (ndarray): The finite difference of velocity.
        ctrl (ndarray): The control trajectory.
        sensordata (ndarray): The sensor data trajectory.
        noisy_sensordata (ndarray): The noisy sensor data trajectory.
        time (ndarray): The time trajectory.
        state_estimate (ndarray): The estimated state trajectory.
        viewer (object): The MuJoCo viewer.
    """
    def __init__(self, filter=None, agent=None,
                 model_path = os.path.join(os.path.dirname(__file__), "../models/go1/task_simulate.xml"),
                T = 200, dt = 0.002, viewer = True, gravity = True,
                # stiff=False
                timeconst=0.04, dampingratio=1.0
                ):
        # filter
        self.filter = filter
        self.agent = agent

        # model
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = dt
        # TODO: load these settings from yml file
        # if stiff:
        self.model.opt.enableflags = 1 # to override contact settings
        # self.model.opt.o_solref = np.array([0.1, 1.0])
        self.model.opt.o_solref = np.array([timeconst, dampingratio])
        # self.filter.model.opt.enableflags = 1
        # self.filter.model.opt.o_solref = np.array([0.005, 1])
        # self.agent.model.opt.enableflags = 1
        # self.agent.model.opt.o_solref = np.array([0.005, 1])
        # self.agent.compute_feedback_policy()

        # self.filter.model.opt.timestep = dt
        # print(self.model.opt.o_solimp)
        # print(self.model.opt.o_solref)
        # data
        self.data = mujoco.MjData(self.model)
        
        self.T = T
        # rollout
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos = self.model.key_qpos[0]
        self.data.qvel = self.model.key_qvel[0]
        self.data.ctrl = self.model.key_ctrl[0]
        # self.data.qpos[2] += 0.15 # shift height up
        # self.data.qvel[0] += 0.5 # forward velocity

        
        self.lying_joint_angles = np.array([-0.37961966, 1.1736176, -2.78803563, 0.39082235, 1.13189518, -2.75953436, -0.39512175, 1.1023442, -2.76518631, 0.36369368, 1.12129807, -2.74447632])

        # turn off gravity
        if not gravity:
            self.model.opt.gravity[:] = 0
            self.filter.model.opt.gravity[:] = 0

        # viewer
        if viewer:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        else:
            self.viewer = None

        # trajectories
        self.qpos = np.zeros((self.model.nq, self.T))
        self.qvel = np.zeros((self.model.nv, self.T))
        self.finite_diff_qvel = np.zeros((self.model.nv, self.T-1))
        self.ctrl = np.zeros((self.model.nu, self.T))
        self.sensordata = np.zeros((self.model.nsensordata, self.T))
        self.noisy_sensordata = np.zeros((self.model.nsensordata, self.T))
        self.time = np.zeros(self.T)
        self.state_estimate = np.zeros((self.model.nq + self.model.nv, self.T))

    def get_sensor(self):
        return self.data.sensordata

    def step(self, ctrl=None):
        self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)
        return self.data.qpos, self.data.qvel
    
    def store_trajectory(self, t):
        self.qpos[:, t] = self.data.qpos
        # self.qpos[:, t] = self.filter.data.qpos
        self.qvel[:, t] = self.data.qvel
        # self.qvel[:, t] = self.filter.data.qvel
        self.ctrl[:, t] = self.data.ctrl
        self.sensordata[:, t] = self.data.sensordata #+ np.random.normal(scale=1.0e-3, size=self.model.nsensordata)
        # self.noisy_sensordata[:, t] = self.filter.data.sensordata #self.sensordata[:, t] #+ np.random.normal(scale=1.0e-3, size=self.model.nsensordata)
        self.time[t] = self.data.time
        # self.time[t] = self.filter.data.time
        if self.filter is not None:
            self.state_estimate[:, t] = self.filter.get_state()
        if t > 0:
            self.finite_diff_qvel[:, t-1] = self.state_difference(self.qpos[:, t-1], self.qpos[:, t])
        return None
    
    def state_difference(self, pos1, pos2):
        # computes the finite difference between two states
        vel = np.zeros(self.model.nv)
        mujoco.mj_differentiatePos(self.model, vel, self.model.opt.timestep, pos1, pos2)
        return vel
    
    def run(self):
        # set initial state
        # mujoco.mj_resetData(self.filter.model, self.filter.data)
        # n_updates = int(self.model.opt.timestep/self.filter.model.opt.timestep)

        # for t in range(self.T-1):
        tqdm_range = tqdm.tqdm(range(self.T-1))
        for t in tqdm_range:
            mujoco.mj_forward(self.model, self.data)
            
            self.store_trajectory(t)
            
            self.ctrl[:, t] = self.data.ctrl
            # self.filter.update(ctrl=self.ctrl[:, t], sensor=self.sensordata[:, t])
            if self.filter is not None:
                self.filter.update(self.ctrl[:, t], self.sensordata[:, t])

            if self.agent is not None:
                if t % 1 == 0:
                    action = self.agent.update(np.concatenate([self.data.qpos, self.data.qvel], axis=0))
                self.data.ctrl = action

            mujoco.mj_step(self.model, self.data)
            
            error = np.linalg.norm(np.array(self.agent.body_ref[:3]) - np.array(self.data.qpos[:3]))
            if error < 0.2:
                self.agent.next_goal()

            if self.viewer is not None and self.viewer.is_alive:
                self.viewer.render()
            else:
                pass
        
        # store last state
        self.store_trajectory(self.T-1)
        
        if self.viewer is not None:
            self.viewer.close()
        return None

    def plot_trajectory(self):
        # position
        plt.figure()
        
        plt.plot(self.time, self.qpos[0, :], label="x (sim)", ls="--", color="blue")
        plt.plot(self.time, self.state_estimate[0, :], label="x (est)", color="blue")

        plt.plot(self.time, self.qpos[1, :], label="y (sim)", ls="--", color="orange")
        plt.plot(self.time, self.state_estimate[1, :], label="y (est)", color="orange")

        plt.plot(self.time, self.qpos[2, :], label="z (sim)", ls="--", color="magenta")
        plt.plot(self.time, self.state_estimate[2, :], label="z (est)", color="magenta")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")

        # orientation plot
        fig = plt.figure()

        plt.plot(self.time, self.qpos[3, :], label="q0 (sim)", ls="--", color="blue")
        plt.plot(self.time, self.state_estimate[3, :], label="q0 (est)", color="blue")

        plt.plot(self.time, self.qpos[4, :], label="q1 (sim)", ls="--", color="orange")
        plt.plot(self.time, self.state_estimate[4, :], label="q1 (est)", color="orange")

        plt.plot(self.time, self.qpos[5, :], label="q2 (sim)", ls="--", color="magenta")
        plt.plot(self.time, self.state_estimate[5, :], label="q2 (est)", color="magenta")

        plt.plot(self.time, self.qpos[6, :], label="q3 (sim)", ls="--", color="green")
        plt.plot(self.time, self.state_estimate[6, :], label="q3 (est)", color="green")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Orientation")

        # plot sensor
        fig = plt.figure()
        plt.plot(self.time, self.sensordata[0, :], label="pos x", color="blue")
        plt.plot(self.time, self.noisy_sensordata[0, :], label="pos x (noisy)", ls="--", color="blue")
        plt.plot(self.time, self.sensordata[1, :], label="pos y", color="orange")
        plt.plot(self.time, self.noisy_sensordata[1, :], label="pos y (noisy)", ls="--", color="orange")
        plt.plot(self.time, self.sensordata[2, :], label="pos z", color="magenta")
        plt.plot(self.time, self.noisy_sensordata[2, :], label="pos z (noisy)", ls="--", color="magenta")
        plt.plot(self.time, self.sensordata[3, :], label="q0", color="blue")
        plt.plot(self.time, self.noisy_sensordata[3, :], label="q0 (noisy)", ls="--", color="blue")
        plt.plot(self.time, self.sensordata[4, :], label="q1", color="orange")
        plt.plot(self.time, self.noisy_sensordata[4, :], label="q1 (noisy)", ls="--", color="orange")
        plt.plot(self.time, self.sensordata[5, :], label="q2", color="magenta")
        plt.plot(self.time, self.noisy_sensordata[5, :], label="q2 (noisy)", ls="--", color="magenta")
        plt.plot(self.time, self.sensordata[6, :], label="q3", color="green")
        plt.plot(self.time, self.noisy_sensordata[6, :], label="q3 (noisy)", ls="--", color="green")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Sensor reading")

        # plot velocity
        fig = plt.figure()
        plt.plot(self.time, self.qvel[0, :], label="vx", color="blue", ls="--")
        plt.plot(self.time, self.state_estimate[19, :], label="vx (est)", color="blue", ls="-")
        plt.plot(self.time[1:], self.finite_diff_qvel[0, :], label="vx (finite diff)", color="blue", ls=":")

        plt.plot(self.time, self.qvel[1, :], label="vy", color="orange", ls="--")
        plt.plot(self.time, self.state_estimate[20, :], label="vy (est)", color="orange", ls="-")
        plt.plot(self.time[1:], self.finite_diff_qvel[1, :], label="vy (finite diff)", color="orange", ls=":")

        plt.plot(self.time, self.qvel[2, :], label="vz", color="magenta", ls="--")
        plt.plot(self.time, self.state_estimate[21, :], label="vz (est)", color="magenta", ls="-")
        plt.plot(self.time[1:], self.finite_diff_qvel[2, :], label="vz (finite diff)", color="magenta", ls=":")
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")

        # plt angular velocity
        fig = plt.figure()
        plt.plot(self.time, self.qvel[3, :], label="wx", color="blue", ls="--")
        plt.plot(self.time, self.state_estimate[22, :], label="wx (est)", color="blue", ls="-")
        plt.plot(self.time[1:], self.finite_diff_qvel[3, :], label="wx (finite diff)", color="blue", ls=":")
        plt.plot(self.time, self.qvel[4, :], label="wy", color="orange", ls="--")
        plt.plot(self.time, self.state_estimate[23, :], label="wy (est)", color="orange", ls="-")
        plt.plot(self.time[1:], self.finite_diff_qvel[4, :], label="wy (finite diff)", color="orange", ls=":")
        plt.plot(self.time, self.qvel[5, :], label="wz", color="magenta", ls="--")
        plt.plot(self.time, self.state_estimate[24, :], label="wz (est)", color="magenta", ls="-")
        plt.plot(self.time[1:], self.finite_diff_qvel[5, :], label="wz (finite diff)", color="magenta", ls=":")
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")



        # plot controls
        fig = plt.figure()
        plt.plot(self.time[:], self.ctrl[0, :], label="FR_thigh", color="blue")
        plt.plot(self.time[:], self.ctrl[1, :], label="FR_hip", color="orange")
        plt.plot(self.time[:], self.ctrl[2, :], label="FR_knee", color="magenta")
        plt.plot(self.time[:], self.ctrl[3, :], label="FL_thigh", color="green")
        plt.plot(self.time[:], self.ctrl[4, :], label="FL_hip", color="red")
        plt.plot(self.time[:], self.ctrl[5, :], label="FL_knee", color="purple")
        plt.plot(self.time[:], self.ctrl[6, :], label="RR_thigh", color="black")
        plt.plot(self.time[:], self.ctrl[7, :], label="RR_hip", color="blue")
        plt.plot(self.time[:], self.ctrl[8, :], label="RR_knee", color="orange")
        plt.plot(self.time[:], self.ctrl[9, :], label="RL_thigh", color="magenta")
        plt.plot(self.time[:], self.ctrl[10, :], label="RL_hip", color="green")
        plt.plot(self.time[:], self.ctrl[11, :], label="RL_knee", color="red")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Control (angles)")
        plt.show()
        return None

    def compute_errors(self):
        # computes the root mean square error between the true state and the estimated state

        # position error
        pos_error = np.linalg.norm(self.qpos[:3, :] - self.state_estimate[:3, :], axis=0)
        print("Position RMSE: ", np.sqrt(np.mean(pos_error**2)))

        # orientation error
        q_error = np.linalg.norm(self.qpos[3:7, :] - self.state_estimate[3:7, :], axis=0)
        print("Orientation RMSE: ", np.sqrt(np.mean(q_error**2)))

        # velocity error
        vel_error = np.linalg.norm(self.qvel[:3, :] - self.state_estimate[19:22, :], axis=0)
        print("Velocity RMSE: ", np.sqrt(np.mean(vel_error**2)))

        # angular velocity error
        w_error = np.linalg.norm(self.qvel[3:6, :] - self.state_estimate[22:25, :], axis=0)
        print("Angular Velocity RMSE: ", np.sqrt(np.mean(w_error**2)))

        return None

    def get_state(self):
        return np.concatenate([self.data.qpos, self.data.qvel], axis=0)
    
if __name__ == "__main__":
    model_path = os.path.join(os.path.dirname(__file__), "../models/go1/task_simulate.xml")
    # model_path = os.path.join(os.path.dirname(__file__), "../models/quadrotor/task.xml")
    #filter = EKF(model_path=model_path)
    # agent = LQR(model_path=model_path)

    simulator = Simulator(filter=None, T = 300, dt=0.002, viewer=True, gravity=True, model_path=model_path)
    simulator.run()
    simulator.compute_errors()
    simulator.plot_trajectory()



