import os
import numpy as np
import gym
import gym_minigrid
import pickle
import matplotlib.pyplot as plt
import imageio
import random

MF = 0 # Move Forward
TL = 1 # Turn Left
TR = 2 # Turn Right
PK = 3 # Pickup Key
UD = 4 # Unlock Door

# Definitions of boolean variables
key_collected   = False
door_unlocked   = False

#def step_cost(action):
def step_cost(_parent_node, _agent_dir, _child_nodes, _child_node_type, _key_pos, _door_pos, _goal_pos, _state_cost_map, _direct_path_check):

    global key_collected
    global door_unlocked

    if not key_collected and not _direct_path_check:
        for i in range(len(_child_nodes)):
            if _child_node_type[i] != 'wall':

                if np.all(_child_nodes[i] == _parent_node + _agent_dir):
                    _state_cost_map[_child_nodes[i][1], _child_nodes[i][0]] = _state_cost_map[_parent_node[1], _parent_node[0]] + np.linalg.norm(_key_pos - _child_nodes[i])**2

                else:
                    _state_cost_map[_child_nodes[i][1], _child_nodes[i][0]] = _state_cost_map[_parent_node[1], _parent_node[0]] + np.linalg.norm(_key_pos - _child_nodes[i])**2 + 0.5 # Adding cost of 0.5 for turn

    if key_collected and not door_unlocked and not _direct_path_check:
        for i in range(len(_child_nodes)):
            if _child_node_type[i] != 'wall':

                if np.all(_child_nodes[i] == _parent_node + _agent_dir):
                    _state_cost_map[_child_nodes[i][1], _child_nodes[i][0]] = _state_cost_map[_parent_node[1], _parent_node[0]] + np.linalg.norm(_door_pos - _child_nodes[i])**2

                else:
                    _state_cost_map[_child_nodes[i][1], _child_nodes[i][0]] = _state_cost_map[_parent_node[1], _parent_node[0]] + np.linalg.norm(_door_pos - _child_nodes[i])**2 + 0.5 # Adding cost of 0.5 for turn

    if door_unlocked or _direct_path_check:
        for i in range(len(_child_nodes)):
            if _child_node_type[i] != 'wall':
                if np.all(_child_nodes[i] == _parent_node + _agent_dir):
                    _state_cost_map[_child_nodes[i][1], _child_nodes[i][0]] = _state_cost_map[_parent_node[1], _parent_node[0]] + np.linalg.norm(_goal_pos - _child_nodes[i])**2

                else:
                    _state_cost_map[_child_nodes[i][1], _child_nodes[i][0]] = _state_cost_map[_parent_node[1], _parent_node[0]] + np.linalg.norm(_goal_pos - _child_nodes[i])**2 + 0.5 # Adding cost of 0.5 for turn

    return _state_cost_map # the cost of action

def step(env, action):
    '''
    Take Action
    ----------------------------------
    actions:
        0 # Move forward (MF)
        1 # Turn left (TL)
        2 # Turn right (TR)
        3 # Pickup the key (PK)
        4 # Unlock the door (UD)
    '''
    actions = {
        0: env.actions.forward,
        1: env.actions.left,
        2: env.actions.right,
        3: env.actions.pickup,
        4: env.actions.toggle
        }

    _, _, done, _ = env.step(actions[action])
    #return step_cost(action), done
    return done

def generate_random_env(seed, task):
    '''
    Generate a random environment for testing
    -----------------------------------------
    seed:
        A Positive Integer,
        the same seed always produces the same environment
    task:
        'MiniGrid-DoorKey-5x5-v0'
        'MiniGrid-DoorKey-6x6-v0'
        'MiniGrid-DoorKey-8x8-v0'
    '''
    if seed < 0:
        seed = np.random.randint(50)
    env = gym.make(task)
    env.seed(seed)
    env.reset()
    return env

def load_env(path):
    '''
    Load Environments
    ---------------------------------------------
    Returns:
        gym-environment, info
    '''
    with open(path, 'rb') as f:
        env = pickle.load(f)

    info = {
        'height': env.height,
        'width': env.width,
        'init_agent_pos': env.agent_pos,
        'init_agent_dir': env.dir_vec
        }

    for i in range(env.height):
        for j in range(env.width):
            if isinstance(env.grid.get(j, i),
                          gym_minigrid.minigrid.Key):
                info['key_pos'] = np.array([j, i])
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Door):
                info['door_pos'] = np.array([j, i])
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Goal):
                info['goal_pos'] = np.array([j, i])

    return env, info

def load_random_env(env_folder):
    '''
    Load a random DoorKey environment
    ---------------------------------------------
    Returns:
        gym-environment, info
    '''
    env_list = [os.path.join(env_folder, env_file) for env_file in os.listdir(env_folder)]
    env_path = random.choice(env_list)
    with open(env_path, 'rb') as f:
        env = pickle.load(f)

    info = {
        'height': env.height,
        'width': env.width,
        'init_agent_pos': env.agent_pos,
        'init_agent_dir': env.dir_vec,
        'door_pos': [],
        'door_open': [],
        }

    for i in range(env.height):
        for j in range(env.width):
            if isinstance(env.grid.get(j, i),
                          gym_minigrid.minigrid.Key):
                info['key_pos'] = np.array([j, i])
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Door):
                info['door_pos'].append(np.array([j, i]))
                if env.grid.get(j, i).is_open:
                    info['door_open'].append(True)
                else:
                    info['door_open'].append(False)
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Goal):
                info['goal_pos'] = np.array([j, i])

    return env, info, env_path

def save_env(env, path):
    with open(path, 'wb') as f:
        pickle.dump(env, f)

def plot_env(env):
    '''
    Plot current environment
    ----------------------------------
    '''
    img = env.render('rgb_array', tile_size=32)
    plt.figure()
    plt.imshow(img)
    plt.show()

def draw_gif_from_seq(seq, env, path='./gif/doorkey.gif'):
    '''
    Save gif with a given action sequence
    ----------------------------------------
    seq:
        Action sequence, e.g [0,0,0,0] or [MF, MF, MF, MF]

    env:
        The doorkey environment
    '''
    with imageio.get_writer(path, mode='I', duration=0.8) as writer:
        img = env.render('rgb_array', tile_size=32)
        writer.append_data(img)
        i = 0
        for act in seq:
            i = i + 1
            img = env.render('rgb_array', tile_size=32)
            step(env, act)
            writer.append_data(img)
    print('GIF is written to {}'.format(path))
    return

def get_child_nodes(_parent_node):

    # Returns child nodes of the given parent node in the order N,W,E,S
    _child_nodes    = []
    top_node        = _parent_node + [0, -1]
    left_node       = _parent_node + [-1, 0]
    right_node      = _parent_node + [1, 0]
    bottom_node     = _parent_node + [0, 1]

    _child_nodes.append(top_node)
    _child_nodes.append(left_node)
    _child_nodes.append(right_node)
    _child_nodes.append(bottom_node)

    return _child_nodes

def policy(_agent_pos, _agent_dir, _child_nodes, _key_pos, _door_pos, _goal_pos, _state_cost_map, _direct_path_check):

    global key_collected
    global door_unlocked

    # Takes all the child nodes, finds the minimum value function and returns index of min. value function as well as actions to reach it
    a = np.zeros(( len(_child_nodes) ))
    b = np.zeros(( len(_child_nodes) ))

    for i in range(len(_child_nodes)):
        a[i] = _child_nodes[i][0]
        b[i] = _child_nodes[i][1]

    a = a.astype(np.int64)
    b = b.astype(np.int64)

    _index  = np.argmin(_state_cost_map[b,a]) # Index of Minimum cost function

    _next_step = np.array([a[_index], b[_index]]) # Get indices of next step

    # Evaluate sequence of actions to reach next step

    if np.all(_next_step == _agent_pos + _agent_dir): # Next step and agent direction are in line
        _seq = [MF]

    elif np.all(_next_step == _agent_pos + -1*_agent_dir): # Next step is opposite to agent direction
        _seq        = [TR,TR,MF]
        _agent_dir  = -1*_agent_dir

    elif np.all(_next_step == _agent_pos + np.flip(_agent_dir)): # Next step is either left or right of agent direction
        if _next_step[0] == _agent_pos[0]:
            _seq        = [TR,MF]
            _agent_dir  = np.flip(_agent_dir)
        else:
            _seq        = [TL,MF]
            _agent_dir  = np.flip(_agent_dir)

    elif np.all(_next_step == _agent_pos + -1*np.flip(_agent_dir)): # Next step is either left or right of agent direction
        if _next_step[0] == _agent_pos[0]:
            _seq        = [TL,MF]
            _agent_dir  = -1*np.flip(_agent_dir)
        else:
            _seq = [TR,MF]
            _agent_dir  = -1*np.flip(_agent_dir)

    if np.all(_next_step == _key_pos) and not key_collected:
        _seq[-1]        = PK
        _next_step      = _agent_pos
        key_collected   = True

    if np.all(_next_step == _door_pos) and not door_unlocked and not _direct_path_check:
        _seq[-1]        = UD
        door_unlocked   = True

    if np.all(_next_step == _goal_pos):
        key_collected   = False
        door_unlocked   = False

    return _next_step, _agent_dir, _seq
