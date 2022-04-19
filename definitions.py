#code inspired from https://github.com/jbnerd/Path_Planning_Agent/blob/master/Uninformed_Search/Definitions_BFS.py
import copy

class State(object):
    """
    State (in state space) abstraction
    """
    def __init__(self, agent_position, dirt_positions):
        self.agent_position = agent_position
        self.dirt_positions = dirt_positions
    def __str__(self):
        message = "Agent position: "+ str(self.agent_position) + "\nDirt positions: " + str(self.dirt_positions)
        return message
    def __eq__(self, other):
        return self.agent_position == other.agent_position and self.dirt_positions == other.dirt_positions
    def __hash__(self):
        return hash((tuple(self.agent_position), tuple([tuple(x) for x in self.dirt_positions])))
    
class Node(object):
    """
    Search tree node abtraction 
    """
    def __init__(self, state, parent, action, path_cost):
        """As specified in p. 78 of the course book (http://aima.cs.berkeley.edu/), requires:
            - state: state in state space.
            - parent: node in the search tree generating this node.
            - action: action applied to the parent node to generate the node.
            - path_cost: from the initial node to the node.
        """
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __str__(self):
        if self.path_cost ==0:
            message = "Parent is root node."
        else:
            message = "Parent node: " + str(self.parent) + "\nAction: " + str(self.action) + "\nPath cost:" + str(self.path_cost)
        return message


    def compute_child_nodes(self, grid_size, obstacles_positions):
        """Implementation of the successor function

        Args:
            grid_size (int)
            obstacles_positions ( (n_obstacles,2) array-like)
        """
        child_nodes = []
        #we assume our vacumm cleaner only sucks when is located on a dirty location
        if self.state.dirt_positions and self.state.agent_position in self.state.dirt_positions:
            index = self.state.dirt_positions.index(self.state.agent_position)
            new_dirt_positions = copy.deepcopy(self.state.dirt_positions)
            new_dirt_positions = new_dirt_positions[:index]+new_dirt_positions[index+1:]
            position_copy = copy.deepcopy(self.state.agent_position)
            new_state = State(position_copy, new_dirt_positions)
            new_node = Node(new_state, self, "suck", self.path_cost+1)
            child_nodes.append(new_node)
        else:

            #up
            new_position = self.state.agent_position.copy()
            new_position[0]-=1
            if new_position[0] >=0 and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "up", self.path_cost+1))

            #down
            new_position = self.state.agent_position.copy()
            new_position[0]+=1
            if new_position[0] <grid_size and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "down", self.path_cost+1))

            #left
            new_position = self.state.agent_position.copy()
            new_position[1]-=1
            if new_position[1] >=0 and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "left", self.path_cost+1))

            #right
            new_position = self.state.agent_position.copy()
            new_position[1]+=1
            if new_position[1] < grid_size and new_position not in obstacles_positions:
                new_state = State(new_position, self.state.dirt_positions)
                child_nodes.append(Node(new_state, self, "right", self.path_cost+1))
        return child_nodes



        

