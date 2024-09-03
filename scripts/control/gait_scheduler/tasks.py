def task_walk_straight():
    goal_pos = [[0, 0, 0.27],
                [1, 0, 0.27],
                [1, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0],
                         [0.5, 0.0],
                         [0.0, 0.0]]

    goal_thresh =  [0.1,
                    0.2,
                    0.2]
        
    desired_gait = ['in_place',
                    'trot',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_octagon():
    goal_pos = [[0, 0, 0.27], #1
                [1, 0, 0.27], #2
                [2, 1, 0.27], #3
                [2, 2, 0.27], #4
                [1, 3, 0.27], #5
                [0, 3, 0.27], #6
                [-1, 2, 0.27],#7 
                [-1, 1, 0.27],#8
                [0, 0, 0.27], #9
                [0, 0, 0.27]] #10
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel = [[0.0, 0.0], #1
               [0.2, 0.0], #2
               [0.2, 0.0], #3
               [0.2, 0.0], #4
               [0.2, 0.0], #5
               [0.2, 0.0], #6
               [0.2, 0.0], #7 
               [0.2, 0.0], #8
               [0.2, 0.0], #9
               [0.0, 0.0]] #10

    goal_thresh =  [0.1, #1
                    0.2, #2
                    0.2, #3
                    0.2, #4
                    0.2, #5
                    0.2, #6
                    0.2, #7
                    0.2, #8
                    0.2, #9
                    0.2] #10
        
    desired_gait = ['in_place',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path
    
def task_box():
    goal_pos = [[0.8, 0, 0.7], #1
                 [1, 0, 0.65],  #2
                 [1, 0.4, 0.62],#3
                 [2, 1, 0.27],  #4
                 [2, 2, 0.27],  #5
                 [1, 3, 0.27],  #6
                 [0, 3, 0.27],  #7
                 [-1, 2, 0.27], #8
                 [-1, 1, 0.27], #9
                 [0, 0, 0.27],  #10
                 [0, 0, 0.27]]  #11
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel = [[0.5, 0.0], #1
               [0.5, 0.0], #2
               [0.5, 0.0], #3
               [0.5, 0.0], #4
               [0.5, 0.0], #5
               [0.5, 0.0], #6
               [0.5, 0.0], #7
               [0.5, 0.0], #8
               [0.5, 0.0], #9
               [0.2, 0.0], #10
               [0.0, 0.0]] #11

    goal_thresh =  [0.2,
                     0.2,
                     0.2,
                     0.2,
                     0.2,
                     0.2,
                     0.2, 
                     0.2, 
                     0.2,
                     0.2,
                     0.2]
        
    desired_gait = ['trot',    #1
                     'trot',    #2
                     'trot',    #3
                     'trot',    #4
                     'trot',    #5
                     'trot',    #6
                     'trot',    #7 
                     'trot',    #8 
                     'trot',    #9
                     'trot',    #10
                     'in_place']#11
    
    model_path = 'models/go1/go1_scene_mppi_pyr_box.xml'
    config_path = 'control/configs/mppi_gait_config_box.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_stairs():
    goal_pos = [[0.8, 0.0, 0.27],
                 [3.1, 0.0, 1.67],
                 [3.3, 0.0, 1.73],
                 [3.3, 0.0, 1.73]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.2, 0.0], #1
                 [0.2, 0.0], #2
                 [0.2, 0.0], #3
                 [0.0, 0.0]] #4

    goal_thresh =  [0.2,
                     0.2,
                     0.2,
                     0.2]
        
    desired_gait = ['walk',    #1
                     'walk',    #2
                     'walk',    #3
                     'in_place'] #4
    
    model_path = 'models/go1/go1_scene_mppi_stairs.xml'
    config_path = 'control/configs/mppi_gait_config_stairs.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_stand():
    goal_pos = [[0, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0]]

    goal_thresh =  [0.2]
        
    desired_gait = ['in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_stand_80():
    goal_pos = [[0, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0]]

    goal_thresh =  [0.2]
        
    desired_gait = ['in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_straight_80():
    goal_pos = [[0, 0, 0.27],
                [1, 0, 0.27],
                [1, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0],
                [0.2, 0.0],
                [0.0, 0.0]]

    goal_thresh =  [0.1,
                    0.2,
                    0.2]
        
    desired_gait = ['in_place',
                    'walk',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_octagon_80():
    goal_pos = [[0, 0, 0.27], #1
                [1, 0, 0.27], #2
                [2, 1, 0.27], #3
                [2, 2, 0.27], #4
                [1, 3, 0.27], #5
                [0, 3, 0.27], #6
                [-1, 2, 0.27],#7 
                [-1, 1, 0.27],#8
                [0, 0, 0.27], #9
                [0, 0, 0.27]] #10
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel = [[0.0, 0.0], #1
               [0.1, 0.0], #2
               [0.1, 0.0], #3
               [0.1, 0.0], #4
               [0.1, 0.0], #5
               [0.1, 0.0], #6
               [0.1, 0.0], #7 
               [0.1, 0.0], #8
               [0.1, 0.0], #9
               [0.0, 0.0]] #10

    goal_thresh =  [0.1, #1
                    0.2, #2
                    0.2, #3
                    0.2, #4
                    0.2, #5
                    0.2, #6
                    0.2, #7
                    0.2, #8
                    0.2, #9
                    0.2] #10
        
    desired_gait = ['in_place',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_stand_100():
    goal_pos = [[0, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0]]

    goal_thresh =  [0.2]
        
    desired_gait = ['in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw_100.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_straight_100():
    goal_pos = [[0, 0, 0.27],
                [1, 0, 0.27],
                [1, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0],
                [0.2, 0.0],
                [0.0, 0.0]]

    goal_thresh =  [0.1,
                    0.2,
                    0.2]
        
    desired_gait = ['in_place',
                    'walk',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw_100.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_octagon_100():
    goal_pos = [[0, 0, 0.27], #1
                [1, 0, 0.27], #2
                [2, 1, 0.27], #3
                [2, 2, 0.27], #4
                [1, 3, 0.27], #5
                [0, 3, 0.27], #6
                [-1, 2, 0.27],#7 
                [-1, 1, 0.27],#8
                [0, 0, 0.27], #9
                [0, 0, 0.27]] #10
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel = [[0.0, 0.0], #1
               [0.1, 0.0], #2
               [0.1, 0.0], #3
               [0.1, 0.0], #4
               [0.1, 0.0], #5
               [0.1, 0.0], #6
               [0.1, 0.0], #7 
               [0.1, 0.0], #8
               [0.1, 0.0], #9
               [0.0, 0.0]] #10

    goal_thresh =  [0.1, #1
                    0.2, #2
                    0.2, #3
                    0.2, #4
                    0.2, #5
                    0.2, #6
                    0.2, #7
                    0.2, #8
                    0.2, #9
                    0.2] #10
        
    desired_gait = ['in_place',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw_100.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_straight_gz():
    goal_pos = [[-3.0, 0.5, 0.27],
                [-2.0, 0.5, 0.27],
                [-1.0, 0.5, 0.27],
                [-1.0, 0.5, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0],
                [0.5, 0.0],
                [0.5, 0.0],
                [0.0, 0.0]]

    goal_thresh =  [0.1,
                    0.2,
                    0.2,
                    0.2]
        
    desired_gait = ['in_place',
                    'trot',
                    'trot',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_gazebo.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_stand_gz():
    goal_pos = [[-2.0, 0.5, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0]]

    goal_thresh =  [0.2]
        
    desired_gait = ['in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_gazebo.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_stand_hw():
    goal_pos = [[-2.96, 0.55, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0]]

    goal_thresh =  [0.2]
        
    desired_gait = ['in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_straight_hw():
    goal_pos = [[0, 0, 0.27],
                [1, 0, 0.27],
                [1, 0, 0.27]]
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel =  [[0.0, 0.0],
                [0.5, 0.0],
                [0.0, 0.0]]

    goal_thresh =  [0.1,
                    0.2,
                    0.2]
        
    desired_gait = ['in_place',
                    'trot',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_hw.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path

def task_walk_octagon_gz():
    goal_pos = [[0, 0, 0.27], #1
                [1, 0, 0.27], #2
                [2, 1, 0.27], #3
                [2, 2, 0.27], #4
                [1, 3, 0.27], #5
                [0, 3, 0.27], #6
                [-1, 2, 0.27],#7 
                [-1, 1, 0.27],#8
                [0, 0, 0.27], #9
                [0, 0, 0.27]] #10
        
    default_orientation = [[1, 0, 0, 0]]

    cmd_vel = [[0.0, 0.0], #1
               [0.1, 0.0], #2
               [0.1, 0.0], #3
               [0.1, 0.0], #4
               [0.1, 0.0], #5
               [0.1, 0.0], #6
               [0.1, 0.0], #7 
               [0.1, 0.0], #8
               [0.1, 0.0], #9
               [0.0, 0.0]] #10

    goal_thresh =  [0.1, #1
                    0.2, #2
                    0.2, #3
                    0.2, #4
                    0.2, #5
                    0.2, #6
                    0.2, #7
                    0.2, #8
                    0.2, #9
                    0.2] #10
        
    desired_gait = ['in_place',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'walk',
                    'in_place']
    
    model_path = 'models/go1/go1_scene_mppi_cf.xml'
    config_path = 'control/configs/mppi_gait_config_walk_gazebo.yml'
    return goal_pos, default_orientation, cmd_vel, goal_thresh, desired_gait, model_path, config_path
 

def get_task(task):
    if task == 'walk_straight':
        return task_walk_straight()
    elif task == 'walk_octagon':
        return task_walk_octagon()
    elif task == 'stand':
        return task_stand()
    elif task == 'box':
        return task_box()
    elif task == 'stairs':
        return task_stairs()
    elif task == 'stand_80':
        return task_stand_80()
    elif task == 'walk_straight_80':
        return task_walk_straight_80()
    elif task == 'walk_octagon_80':
        return task_walk_octagon_80()
    elif task == 'stand_100':
        return task_stand_100()
    elif task == 'walk_straight_100':
        return task_walk_straight_100()
    elif task == 'walk_octagon_100':
        return task_walk_octagon_100()
    elif task == 'walk_straight_gz':
        return task_walk_straight_gz()
    elif task == 'stand_gz':
        return task_stand_gz()
    elif task == 'walk_octagon_gz':
        return task_walk_octagon_gz()
    elif task == 'stand_hw':
        return task_stand_hw()
    elif task == 'walk_straight_hw':
        return task_walk_straight_hw()