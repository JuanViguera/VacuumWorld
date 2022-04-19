
def manhattan_distance(node):
    """Computes Manhattan distance between the vacuum cleaner and the first dirt position in the list.

    Args:
        node (Node)
    """
    return abs(node.state.agent_position[0]-node.state.dirt_positions[0][0]) + abs(node.state.agent_position[1]-node.state.dirt_positions[0][1]) 