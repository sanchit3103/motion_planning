import numpy as np
import gym
import math
from utils import *
from part_a import *
from part_b import *

MF = 0 # Move Forward
TL = 1 # Turn Left
TR = 2 # Turn Right
PK = 3 # Pickup Key
UD = 4 # Unlock Door

def partA():
    env_path = './envs/doorkey-8x8-shortcut.env'
    env, info = load_env(env_path) # load an environment
    seq = doorkey_problem(env, info) # find the optimal action sequence
    draw_gif_from_seq(seq, load_env(env_path)[0]) # draw a GIF & save

def partB():
    env_folder = './envs/random_envs'
    env, info, env_path = load_random_env(env_folder)
    seq = doorkey_problem_part_b(env, info)
    draw_gif_from_seq(seq, load_env(env_path)[0])

if __name__ == '__main__':
    partA()
    partB()

'''
You are required to find the optimal path in
    doorkey-5x5-normal.env
    doorkey-6x6-normal.env
    doorkey-8x8-normal.env

    doorkey-6x6-direct.env
    doorkey-8x8-direct.env

    doorkey-6x6-shortcut.env
    doorkey-8x8-shortcut.env

Feel Free to modify this fuction
'''
