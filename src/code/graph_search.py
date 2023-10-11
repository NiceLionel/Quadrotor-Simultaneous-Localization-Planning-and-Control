from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World
from collections import defaultdict
from .occupancy_map import OccupancyMap # Recommended.

def cal_cost(goal_index, current_index):
    return np.linalg.norm((np.array(goal_index) - np.array(current_index)))

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """
    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    # print('start',start_index,start)
    goal_index = tuple(occ_map.metric_to_index(goal))
    # print('goal',goal_index,goal)
    x_res = resolution[0]
    y_res = resolution[1]
    z_res = resolution[2]
    # initialize queues
    open_list = []
    neighbor_index = set()
    closed_list_index = set()
    # initialize the cost of every node to infinity
    node_cost = defaultdict(lambda: float("inf"))
    node_cost[start_index] = 0
    # parent matrix (dictionary)
    parent_node = {start_index: None}
    # print(node_cost)
    step_length = [(0, 0, -1), (0, 0, 1), (0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0),(0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
                   (1, 0, -1), (1, -1, 0), (1, 1, 0), (1, 0, 1), (-1, 0, -1), (-1, -1, 0), (-1, 1, 0), (-1, 0, 1),
                   (-1, -1, -1), (-1, -1, 1), (-1, 1, -1), (-1, 1, 1), (1, -1, -1), (1, -1, 1), (1, 1, -1), (1, 1, 1)]
    # Dijkstra's Algorithm
    if astar == False:
        # Put the start node into heap queue
        heappush(open_list, (0, start_index, start))
        # print(open_list[0])
        while len(open_list)>0 and open_list[0][0] < 10000:
            # find the node with the lowest cost currently
            cost, idx, point_metric = heappop(open_list)
            # add the lowest cost node into the closed list
            closed_list_index.add(idx)
            # the goal is reached already
            if idx == goal_index:
                break
            for step in step_length:
                neigh_idx = (idx[0] + step[0], idx[1] + step[1], idx[2] + step[2])
                neigh_metric = [point_metric[0] + x_res * step[0], point_metric[1] + y_res * step[1],
                                point_metric[2] + z_res * step[2]]
                # add more cost to the node away from goal_pos
                if neigh_idx not in closed_list_index:
                    # check if it counters any obstacle and update the cost of the node
                    if not occ_map.is_occupied_metric(neigh_metric) and not occ_map.is_occupied_index(neigh_idx):
                        cost_move = node_cost[idx] + cal_cost(idx, neigh_idx)
                        total_cost = cost_move
                        if cost_move < node_cost[neigh_idx]:
                            # update the cost of son node
                            node_cost[neigh_idx] = cost_move
                            # store the parent node and its corresponding son node
                            parent_node[neigh_idx] = (idx, point_metric)
                        # add the nodes in the neighbor to the open list
                        if neigh_idx not in neighbor_index:
                            heappush(open_list, (total_cost, neigh_idx, neigh_metric))
                            neighbor_index.add(neigh_idx)
    # A* algorithm
    else:
        heappush(open_list, (cal_cost(goal_index, start_index),(start_index[0], start_index[1], start_index[2]), (start[0], start[1], start[2])))
        # graph search
        while len(open_list)>0 and open_list[0][0] < 10000:
            # find the current lowest cost node
            u, idx, point_metric = heappop(open_list)
            # add removed node to the closed heap
            closed_list_index.add(idx)
            # the goal is reached already
            if idx == goal_index:
                break
            for step in step_length:
                neigh_idx = (idx[0] + step[0], idx[1] + step[1], idx[2] + step[2])
                neigh_metric = (point_metric[0] + x_res * step[0], point_metric[1] + y_res * step[1],
                                point_metric[2] + z_res * step[2])
                # check if the neighbor node in the closed list
                if neigh_idx not in closed_list_index:
                    if not occ_map.is_occupied_metric(neigh_metric) and not occ_map.is_occupied_index(neigh_idx):
                        # compute total cost
                        cost_move = node_cost[idx] + cal_cost(idx, neigh_idx)
                        total_cost = cost_move + cal_cost(goal_index, neigh_idx)
                        if cost_move < node_cost[neigh_idx]:
                            # store the index and the position of the son node
                            parent_node[neigh_idx] = (idx, point_metric)
                            # update the cost of son node
                            node_cost[neigh_idx] = cost_move
                        # put the neighbor node to the open list
                        if neigh_idx not in neighbor_index:
                            heappush(open_list, (total_cost, neigh_idx, neigh_metric))
                            neighbor_index.add(neigh_idx)

    # return the result
    if goal_index in closed_list_index:
        print("Path has been found")
        path_list = np.array(goal)
        son_node, son_metric = parent_node[goal_index]
        path_list = np.vstack((path_list, np.array(son_metric)))
        while True:
            son_node, son_metric = parent_node[son_node]
            path_list = np.vstack((path_list, np.array(son_metric)))
            if list(son_node) == list(start_index):
                break
        path_list = np.flip(path_list, 0)
    else:
        print("No path has been found")
        return None, len(closed_list_index)
    # print(path_list)
    return path_list, len(closed_list_index)
    # Return a tuple (path, nodes_expanded)