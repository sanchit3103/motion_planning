import numpy as np
import math
import time
import random

class motionPlanner():

    def __init__(self, _envmap, _robot_start_pos, _target_pos):

        # Initializations of all parameters and variables
        self.envmap                 = _envmap
        self.OPEN                   = []
        self.CLOSED                 = []
        self.PATH                   = []
        self.InitialPath            = []
        self.FinalPath              = []
        self.Previous_Target_Pos    = []
        self.Straight_Path_Cost     = 1
        self.Diagonal_Path_Cost     = np.sqrt(2)
        self.N                      = 1000
        self.reusePath              = False
        self.computePath            = True
        self.counter                = 0
        self.Numofdirs              = 8
        self.dX                     = [-1, -1, -1, 0, 0, 1, 1, 1]
        self.dY                     = [-1,  0,  1, -1, 1, -1, 0, 1]
        self.Labels                 = np.ones((self.envmap.shape))*np.inf
        self.Heuristic              = np.zeros((self.envmap.shape))

        # Set label of start position as zero
        self.Labels[_robot_start_pos[0], _robot_start_pos[1]] = 0

        # Store the start position of the robot in FinalPath list
        self.FinalPath.append(_robot_start_pos)

        pass

    def robotplanner(self, _envmap, _robot_pos, _target_pos):

        new_robot_pos = np.copy(_robot_pos)

        # Block to reuse the path calculated by A-star Algorithm up to certain number of nodes for larger maps
        if self.reusePath == True:
            self.PATH.pop(0)
            new_Position        = self.PATH[1]
            self.counter        = self.counter + 1
            if len(self.PATH) < 3: # Condition to prevent the length of Path list from going below 3
                self.reusePath      = False
                self.computePath    = True

        # A-star Algorithm Block
        if self.computePath == True:

            # Condition to check if the target position has changed from the previous iteration
            if not np.all( self.Previous_Target_Pos == _target_pos ) and len(self.Previous_Target_Pos) != 0:
                self.replanPath(_robot_pos)

            self.OPEN.append(_robot_pos.tolist())

            iteration = 0

            while _target_pos.tolist() not in self.CLOSED:

                node_to_expand = self.nodeToExpand(_robot_pos.tolist())

                self.OPEN.remove(node_to_expand)

                self.CLOSED.append(node_to_expand)

                child_Nodes = self.getChildNodes(node_to_expand)

                self.calculateHeuristics(_target_pos, child_Nodes)

                for node in child_Nodes:
                    if node not in self.CLOSED:
                        self.updateLabel(node_to_expand, node)
                        if node not in self.OPEN:
                            self.OPEN.append(node)

                # Condition to break the while loop once certain number of nodes have been expanded - to be used only for larger maps
                if len(self.CLOSED) == self.N:
                    break

            self.backtrackPath()

            new_Position        = self.PATH[1]
            #self.reusePath      = True
            #self.computePath    = False

        # Temp code - remove this for submission
        if len(self.FinalPath) == 1:
            self.InitialPath = self.PATH

        # Block to check if the desired number of nodes have been used while reusing the path calculated by A-star algorithm - Set the bool variables accordingly
        if self.counter == 10:
            self.reusePath      = False
            self.computePath    = True
            self.counter        = 0

        # Store the new position of the Robot in the form of the array
        new_robot_pos[0]    = new_Position[0]
        new_robot_pos[1]    = new_Position[1]

        # Store current target position for comparing in next iteration
        self.Previous_Target_Pos = _target_pos

        # Store the actual path taken by the robot in the FinalPath list
        self.FinalPath.append(new_robot_pos)

        return new_robot_pos

    def getChildNodes(self, _current_pos):

        # Input = Current Robot position
        # Output = Child nodes of the current robot position in all 8 directions

        _child_nodes    = []

        for i in range(self.Numofdirs):
            #childnode   = _current_pos + [ self.dX[i], self.dY[i] ]
            childnode   = [ _current_pos[0] + self.dX[i], _current_pos[1] + self.dY[i] ]

            if (childnode[0] >= 0 and childnode[0] < self.envmap.shape[0] and childnode[1] >= 0 and childnode[1] < self.envmap.shape[1]): # Check if the node calculated is within the boundary of the map
                if not self.envmap[ childnode[0], childnode[1] ]: # Checks for obstacles here
                    _child_nodes.append(childnode)

        return _child_nodes

    def updateLabel(self, _current_pos, _child_node):

        # Input = Current robot position and child node of the current robot position
        # Updates the label of the given child node according to the label of current robot position

        straight_dir = [ [-1,0], [0,-1], [0,1], [1,0] ]

        if not self.envmap[ _child_node[0], _child_node[1] ]: # Checks for obstacles here
            dir = [ _current_pos[0] - _child_node[0], _current_pos[1] - _child_node[1] ]
            if np.all( dir == straight_dir[0] ) or np.all( dir == straight_dir[1] ) or np.all( dir == straight_dir[2] ) or np.all( dir == straight_dir[3] ):
                if self.Labels[ _child_node[0], _child_node[1] ] > self.Labels[ _current_pos[0], _current_pos[1] ] + self.Straight_Path_Cost:
                    self.Labels[ _child_node[0], _child_node[1] ] = round( self.Labels[ _current_pos[0], _current_pos[1] ] + self.Straight_Path_Cost, 2 )

            else:
                if self.Labels[ _child_node[0], _child_node[1] ] > self.Labels[ _current_pos[0], _current_pos[1] ] + self.Diagonal_Path_Cost:
                    self.Labels[ _child_node[0], _child_node[1] ] = round( self.Labels[ _current_pos[0], _current_pos[1] ] + self.Diagonal_Path_Cost, 2 )

        pass

    def calculateHeuristics(self, _target_pos, _child_nodes):

        # Calculates heuristics of all child nodes as L2 distance from the target position
        # Input = Target posiion and all child nodes of a particular node

        for node in _child_nodes:
            self.Heuristic[node[0],node[1]] = round( np.linalg.norm( node - _target_pos ), 2 )

        pass

    def nodeToExpand(self, _current_pos):

        # Output = Next node to expand
        # This function finds the node with minimum f = g + h value from the Open list
        # If the f value for more than 1 nodes is equal to min of f value, it then compares their g (label) value and outputs the node with minimum g value
        # If the g (label) value is also equal, then it outputs any node randomly

        f_values    = []
        g_values    = []
        indices     = []

        # Calculate gi + hi for all nodes in Open list
        for i in range(len(self.OPEN)):
            fvalue = self.Labels[ self.OPEN[i][0], self.OPEN[i][1] ] + self.Heuristic[ self.OPEN[i][0], self.OPEN[i][1] ]
            gvalue = self.Labels[ self.OPEN[i][0], self.OPEN[i][1] ]
            f_values.append(fvalue)
            g_values.append(gvalue)

        # Find the minimum f-value and store its indices
        indices = np.where(f_values == np.min(f_values))[0]

        # Check if there are more than one indices with minimum value and check their g values
        if len(indices) > 1:
            temp = []
            for i in range(len(indices)):
                temp.append(g_values[indices[i]])

            # Find indices of minimum g values
            g_values_indices = np.where(g_values == np.min(temp))[0]

            # Check if there are more than one node with same g values
            if len(g_values_indices) > 1:
                indice = random.choice(g_values_indices)
                _node_to_expand = self.OPEN[indice]
            else:
                indice = g_values_indices[0]
                _node_to_expand = self.OPEN[indice]

        else:
            indice = indices[0]
            _node_to_expand = self.OPEN[indice]

        return _node_to_expand

    def replanPath(self, _current_pos):

        # Reinitialize all parameters
        self.OPEN                   = []
        self.CLOSED                 = []
        self.Labels                 = np.ones((self.envmap.shape))*np.inf
        self.Heuristic              = np.zeros((self.envmap.shape))

        # Set label of start position as zero
        self.Labels[_current_pos[0], _current_pos[1]] = 0

        pass

    def backtrackPath(self):

        self.PATH = []

        # Reverse the CLOSED list as the nodes are backtracked from the target position
        self.CLOSED.reverse()

        # Store the first node of CLOSED list, append it to Path list and pop it from CLOSED list
        temp_node = self.CLOSED[0]

        self.PATH.append(temp_node)

        self.CLOSED.pop(0)

        # Loop over the CLOSED list to check for adjacent nodes and store them in the Path list
        for node in self.CLOSED:
            if abs(temp_node[0] - node[0]) <= 1 and abs(temp_node[1] - node[1]) <= 1:
                self.PATH.append(node)
                temp_node = node

        # Reverse the Path list - to make it from start position to end position
        self.PATH.reverse()

    def updateHeuristics(self, _new_robot_pos):

        for node in self.CLOSED:
            self.Heuristic[ node[0], node[1] ] = self.Labels[ _new_robot_pos[0], _new_robot_pos[1] ] + self.Heuristic[ _new_robot_pos[0], _new_robot_pos[1] ] - self.Labels[ node[0], node[1] ]

        self.CLOSED = []

        pass
