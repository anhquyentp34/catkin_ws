import rospy
import heapq
from algorithms.neighbors import find_neighbors

def euclidean_distance_squared(index, goal_index, width):
    """Squared Heuristic Function for A Star algorithm"""
    index_x, index_y = divmod(index, width)
    goal_x, goal_y = divmod(goal_index, width)

    distance_squared = (index_x - goal_x) ** 2 + (index_y - goal_y) ** 2
    return distance_squared

def is_valid_grid(index, costmap):
    """Check if the grid is valid (not in the costmap)"""
    """return not costmap[index]"""
    return not costmap[index]  

def astar(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
    open_list = []
    heapq.heappush(open_list, (0 + euclidean_distance_squared(start_index, goal_index, width), 0, start_index))

    closed_set = set()
    parents = {}
    g_costs = {start_index: 0}

    shortest_path = []

    path_found = False
    rospy.loginfo('A Star: Done with initialization')

    while open_list:
        _, current_g_cost, current_node = heapq.heappop(open_list)

        if current_node in closed_set:
            continue

        closed_set.add(current_node)

        if current_node == goal_index:
            path_found = True
            break

        neighbors = find_neighbors(current_node, width, height, costmap, resolution)

        for neighbor_index, step_cost in neighbors:
            if neighbor_index in closed_set or not is_valid_grid(neighbor_index, costmap):
                continue

            g_cost = current_g_cost + step_cost
            h_cost = euclidean_distance_squared(neighbor_index, goal_index, width)
            f_cost = g_cost + h_cost

            if neighbor_index in g_costs and g_cost >= g_costs[neighbor_index]:
                continue

            g_costs[neighbor_index] = g_cost
            parents[neighbor_index] = current_node

            heapq.heappush(open_list, (f_cost, g_cost, neighbor_index))

    rospy.loginfo('AStar: Done traversing nodes in open_list')

    if not path_found:
        rospy.logwarn('AStar: No path found!')
        return shortest_path

    node = goal_index
    shortest_path.append(goal_index)
    while node != start_index:
        shortest_path.append(node)
        node = parents[node]

    shortest_path = shortest_path[::-1]
    rospy.loginfo('AStar: Done reconstructing path')

    return shortest_path, None
