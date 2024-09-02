import numpy as np
from interface.simulator import Simulator
#from control.mppi_gait_mujoco import MPPI
from control.mppi_gait import MPPI

import os

def main():
    T = 10000
    # model_path = os.path.join(os.path.dirname(__file__), "../models/go1/go1_scene_jax_no_collision.xml")
    model_path = os.path.join(os.path.dirname(__file__), "models/go1/go1_scene_mppi.xml")
    task = 'walk_octagon_80'
    #ctrl_model_path = os.path.join(os.path.dirname(__file__), "models/go1/go1_scene_mppi_cf.xml")
    # agent = MPPI(model_path=model_path)
    #agent = MPPI()
    #print(agent.body_ref)
    agent = MPPI(task=task)
    agent.internal_ref = True
    simulator = Simulator(agent=agent, viewer=True, T=T, dt=0.0125, timeconst=0.02,
                          model_path=model_path)
    # simulator = Simulator(agent=None, viewer=True, T=T)
    simulator.run()
    # simulator.plot_trajectory()
    pass

if __name__ == "__main__":
    main()