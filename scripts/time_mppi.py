from control.mppi_gait import MPPI
import timeit
import os
import numpy as np
import threading
from concurrent.futures import ThreadPoolExecutor

def main():
    # control model path
    ctrl_model_path = os.path.join(os.path.dirname(__file__), "models/go1/go1_scene_mppi_cf.xml")
    
    # cpu name
    cpu_name = os.popen("cat /proc/cpuinfo | grep 'model name'").readline().strip()
    print(f"CPU name: {cpu_name}")

    # get number of cores available
    num_cores = os.cpu_count()
    print(f"Number of cores available: {num_cores}")

    update_frequency= np.zeros(num_cores)
    for i in range(1, num_cores+1):
        mppi = MPPI(model_path=ctrl_model_path)
        mppi.internal_ref = True
        mppi.num_workers = i
        mppi.thread_local = threading.local()
        mppi.executor =  ThreadPoolExecutor(max_workers=i, initializer=mppi.thread_initializer)
        update_time = timeit.timeit(lambda: mppi.update(np.zeros(37)), number=100)/100
        update_frequency[i-1] = 1/update_time
        print(f"update frequency for {i} cores: ", update_frequency[i-1])

    import matplotlib.pyplot as plt
    plt.plot(range(1, num_cores+1), update_frequency, 'o-')
    plt.title(cpu_name)
    plt.xlabel("Number of cores")
    plt.ylabel("Update frequency (Hz)")
    plt.savefig(f"results/mppi_update_frequency_{cpu_name}.png")
    plt.show()

if __name__ == "__main__":
    main()