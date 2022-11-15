import numpy as np
import gym
import math
from utils import *

def doorkey_problem_part_b(env, info):

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
    value_Function_Indirect     = []

    # Extracting all the required details from the environment and its info
    agent_Pos           = env.agent_pos
    agent_Dir           = env.dir_vec
    key_Pos             = info['key_pos']
    door_Pos            = info['door_pos']
    door_Status         = info['door_open']
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

        state_Cost_Map                      = step_cost(agent_Pos, agent_Dir, child_Nodes, child_Node_Type, key_Pos, door_Pos[0], goal_Pos, state_Cost_Map, direct_Path_Check) # Sending any door position as it is not required here
        next_Step, agent_Dir, action_Seq    = policy(agent_Pos, agent_Dir, child_Nodes, key_Pos, door_Pos[0], goal_Pos, state_Cost_Map, direct_Path_Check)
        agent_Pos                           = next_Step
        direct_Path.append(next_Step)

        for i in range(len(action_Seq)):
            optimal_Seq_Direct.append(action_Seq[i])

        iteration = iteration + 1

        if iteration > 25:
            break

    # Calculation of Value Function for Direct Path
    for i in range(len(direct_Path)):
        value_Function_Direct   = value_Function_Direct + state_Cost_Map[direct_Path[i][1], direct_Path[i][0]]
        cell                    = env.grid.get(direct_Path[i][0], direct_Path[i][1])
        if cell != None:
            direct_Path_Cells.append(cell.type)
            if cell.type == 'door' and np.all(direct_Path[i] == door_Pos[0]) and not door_Status[0]:
                value_Function_Direct = value_Function_Direct + math.inf

            elif cell.type == 'door' and np.all(direct_Path[i] == door_Pos[1]) and not door_Status[1]:
                value_Function_Direct = value_Function_Direct + math.inf
        else:
            direct_Path_Cells.append(cell)

    # Variable changes in transition from Direct path loop to Indirect Path Loop
    direct_Path_Check   = False

    # Loop to check the Indirect Path
    for j in range(len(door_Pos)):

        # Intializations inside the loop
        temp_Path           = []
        temp_Sequence       = []
        temp_Value_Function = 0

        state_Cost_Map      = math.inf * np.ones((8, 8))
        iteration           = 0
        agent_Pos           = env.agent_pos
        agent_Dir           = env.dir_vec

        # Giving zero label to the start position
        state_Cost_Map[agent_Pos[1], agent_Pos[0]]   = 0

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

            state_Cost_Map                      = step_cost(agent_Pos, agent_Dir, child_Nodes, child_Node_Type, key_Pos, door_Pos[j], goal_Pos, state_Cost_Map, direct_Path_Check)
            next_Step, agent_Dir, action_Seq    = policy(agent_Pos, agent_Dir, child_Nodes, key_Pos, door_Pos[j], goal_Pos, state_Cost_Map, direct_Path_Check)
            agent_Pos                           = next_Step
            temp_Path.append(next_Step)

            for i in range(len(action_Seq)):
                temp_Sequence.append(action_Seq[i])

            if np.all(next_Step == door_Pos[j]):
                temp_Sequence.append(MF)

            iteration = iteration + 1

            if iteration > 25:
                break

        # Calculation of Value Function for Indirect Path
        for i in range(len(temp_Path)):
            temp_Value_Function  = temp_Value_Function + state_Cost_Map[temp_Path[i][1], temp_Path[i][0]]

        if door_Status[j]:
            temp_Sequence.remove(UD)

        # Store in original arrays
        indirect_Path.append(temp_Path)
        optimal_Seq_Indirect.append(temp_Sequence)
        value_Function_Indirect.append(temp_Value_Function)

    index = np.argmin(value_Function_Indirect)

    # Selection of Optimal Control Policy between Direct and Indirect Path
    if value_Function_Direct == math.inf or value_Function_Indirect[index] < value_Function_Direct:
        optimal_Seq = optimal_Seq_Indirect[index]

    else:
        optimal_Seq = optimal_Seq_Direct

    return optimal_Seq
