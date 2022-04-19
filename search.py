#inspired from https://github.com/jbnerd/Path_Planning_Agent/blob/master/Uninformed_Search/BFS.py
from collections import deque
import copy
from definitions import State, Node
import bisect

def write_solution(node):
    """Returns the sequence of actions and the cost once a solution has been found.

    Args:
        node (Node): Solution node.
    """
    action_list = []
    cost = node.path_cost
    while (node.parent is not None):
        action_list.append(node.action)
        node=node.parent
    action_list.reverse()

    return action_list, cost

def bfs(initial_state, grid_size, obstacles_positions):
    """Performs breadth-first search on the vacuum cleaner world.

    Args:
        initial_state (State)
        grid_size (int)
        obstacles_positions (n_obstaclesx2 array-like)
    """
    solution_checks = 0
    solutions = []
    costs = []
    solutions_checks = []
    node = Node(initial_state, None, None, path_cost = 0)
    if not initial_state.dirt_positions:
        print("Problem solved!")
        return [], 0
    else:
        frontier = deque()
        explored = set()
        frontier_check = set()

        frontier.append(node)
        frontier_check.add(node.state)

        while(True):
            if not frontier:
                return solutions, costs, solutions_checks
            else:
                node = frontier.popleft()
                frontier_check.discard(node.state)
                explored.add(node.state)
                child_nodes = node.compute_child_nodes(grid_size, obstacles_positions)

                for child_node in child_nodes:
                    if child_node.state not in explored and child_node.state not in frontier_check:
                        if not child_node.state.dirt_positions:
                            print("Found a solution!")
                            solution, cost = write_solution(child_node)
                            print(solution, cost)
                            print(f"Solution checks performed: {solution_checks}.")
                            solutions.append(solution)
                            costs.append(cost)
                            solutions_checks.append(solution_checks)

                        else:
                            frontier.append(child_node)
                            frontier_check.add(child_node.state)
                            solution_checks += 1

def a_star(initial_state, grid_size, obstacles_positions, heuristic):
    """Performs A* search on the vacuum cleaner world.

    Args:
        initial_state (State)
        grid_size (int)
        obstacles_positions (n_obstaclesx2 array-like)
    """
    solution_checks = 0
    solutions = []
    costs = []
    solutions_checks = []
    node = Node(initial_state, None, None, path_cost = 0)
    if not initial_state.dirt_positions:
        print("Problem solved!")
        return [], 0
    else:
        frontier = deque()
        frontier_f_values = deque()
        explored = set()
        frontier_check = set()

        frontier.append(node)
        f = heuristic(node)
        frontier_f_values.append(f)
        frontier_check.add(node.state)

        while(True):
            if not frontier:
                return solutions, costs, solutions_checks
            else:
                node = frontier.popleft()
                frontier_f_values.popleft()
                frontier_check.discard(node.state)
                explored.add(node.state)
                child_nodes = node.compute_child_nodes(grid_size, obstacles_positions)

                for child_node in child_nodes:
                    if child_node.state not in explored and child_node.state not in frontier_check:
                        if not child_node.state.dirt_positions:
                            print("Found a solution!")
                            solution, cost = write_solution(child_node)
                            print(solution, cost)
                            print(f"Solution checks performed: {solution_checks}.")
                            solutions.append(solution)
                            costs.append(cost)
                            solutions_checks.append(solution_checks)
                        else:
                            f = child_node.path_cost + heuristic(child_node)
                            index = bisect.bisect_left(frontier_f_values,f)
                            frontier.insert(index,child_node)
                            frontier_f_values.insert(index,f) 
                            frontier_check.add(child_node.state)
                            solution_checks += 1