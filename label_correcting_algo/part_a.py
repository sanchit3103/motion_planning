import numpy as np
import gym
import math
from utils import *

def doorkey_problem(env, info):

    # Common Variable Definitions
    state_Cost_Map              = math.inf * np.ones((8, 8))
    next_Step                   = []
    iteration                   = 0

    # Variable Definitions for direct path
    direct_Path                 = []
    direct_Path_Cells           = []
    optimal_Seq_Direct          = []
    value_Function_Direct       = 0
    direct_Path_Check           = True

    # Variable Definitions for indirect path
    indirect_Path               = []
    optimal_Seq_Indirect        = []
    value_Function_Indirect     = 0

    # Extracting all the required details from the environment and its info
    agent_Pos           = env.agent_pos
    agent_Dir           = env.dir_vec
    key_Pos             = info['key_pos']
    door_Pos            = info['door_pos']
    goal_Pos            = info['goal_pos']

    # Giving zero label to the start position
    state_Cost_Map[agent_Pos[1], agent_Pos[0]]   = 0

    print(info)
    plot_env(env)

    # Loop to check the Direct Path
    while not np.all(agent_Pos == goal_Pos):

        # Obtain Child nodes and their types
        child_Nodes     = get_child_nodes(agent_Pos)
        child_Node_Type = []
        for i in range(len(child_Nodes)):
            cell = env.grid.get(child_Nodes[i][0], child_Nodes[i][1])
            if cell != None:
                child_Node_Type.append(cell.type)
            else:
                child_Node_Type.append(cell)

        state_Cost_Map                      = step_cost(agent_Pos, agent_Dir, child_Nodes, child_Node_Type, key_Pos, door_Pos, goal_Pos, state_Cost_Map, direct_Path_Check)
        next_Step, agent_Dir, action_Seq    = policy(agent_Pos, agent_Dir, child_Nodes, key_Pos, door_Pos, goal_Pos, state_Cost_Map, direct_Path_Check)
        agent_Pos                           = next_Step
        direct_Path.append(next_Step)

        for i in range(len(action_Seq)):
            optimal_Seq_Direct.append(action_Seq[i])

        if np.all(next_Step == door_Pos):
            optimal_Seq_Direct.append(MF)

        iteration = iteration + 1

        if iteration > 25:
            break

    # Calculation of Value Function for Direct Path
    for i in range(len(direct_Path)):
        value_Function_Direct   = value_Function_Direct + state_Cost_Map[direct_Path[i][1], direct_Path[i][0]]
        cell                    = env.grid.get(direct_Path[i][0], direct_Path[i][1])
        if cell != None:
            direct_Path_Cells.append(cell.type)
        else:
            direct_Path_Cells.append(cell)

    # Variable changes in transition from Direct path loop to Indirect Path Loop
    direct_Path_Check   = False
    state_Cost_Map      = math.inf * np.ones((8, 8))
    iteration           = 0
    agent_Pos           = env.agent_pos
    agent_Dir           = env.dir_vec

    # Giving zero label to the start position
    state_Cost_Map[agent_Pos[1], agent_Pos[0]]   = 0

    # Loop to check the Indirect Path
    while not np.all(agent_Pos == goal_Pos):

        # Obtain Child nodes and their types
        child_Nodes     = get_child_nodes(agent_Pos)
        child_Node_Type = []
        for i in range(len(child_Nodes)):
            cell = env.grid.get(child_Nodes[i][0], child_Nodes[i][1])
            if cell != None:
                child_Node_Type.append(cell.type)
            else:
                child_Node_Type.append(cell)

        state_Cost_Map                      = step_cost(agent_Pos, agent_Dir, child_Nodes, child_Node_Type, key_Pos, door_Pos, goal_Pos, state_Cost_Map, direct_Path_Check)
        next_Step, agent_Dir, action_Seq    = policy(agent_Pos, agent_Dir, child_Nodes, key_Pos, door_Pos, goal_Pos, state_Cost_Map, direct_Path_Check)
        agent_Pos                           = next_Step
        indirect_Path.append(next_Step)

        for i in range(len(action_Seq)):
            optimal_Seq_Indirect.append(action_Seq[i])

        if np.all(next_Step == door_Pos):
            optimal_Seq_Indirect.append(MF)

        iteration = iteration + 1

        if iteration > 25:
            break

    # Calculation of Value Function for Indirect Path
    for i in range(len(indirect_Path)):
        value_Function_Indirect  = value_Function_Indirect + state_Cost_Map[indirect_Path[i][1], indirect_Path[i][0]]

    # Selection of Optimal Control Policy between Direct and Indirect Path
    if (value_Function_Indirect < value_Function_Direct) or ('door' in direct_Path_Cells):
        optimal_Seq = optimal_Seq_Indirect

    else:
        optimal_Seq = optimal_Seq_Direct

    return optimal_Seq
