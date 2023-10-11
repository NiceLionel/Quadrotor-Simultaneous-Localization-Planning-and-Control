import math
import numpy as np
from scipy.sparse.linalg import lsmr
from .graph_search import graph_search


class WorldTraj(object):
    """

    """

    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must choose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.2, 0.2, 0.2])
        self.margin = 0.5

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path, _ = graph_search(world, self.resolution, self.margin, start, goal, astar=True)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        # self.points = np.zeros((1,3)) # shape=(n_pts,3)

        # Finally, you must compute a trajectory through the waypoints similar
        # to your task in the first project. One possibility is to use the
        # WaypointTraj object you already wrote in the first project. However,
        # you probably need to improve it using techniques we have learned this
        # semester.

        # STUDENT CODE HERE
        self.points = self.path
        self.final_points = self.path[-1]
        self.number_point = self.points.shape[0]

    def update(self, time):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x = np.zeros((3,))
        x_dot = np.zeros((3,))
        x_ddot = np.zeros((3,))
        x_dddot = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE

        direction = np.zeros((self.number_point - 1, 3))
        distance = np.zeros((self.number_point - 1, 3))
        t = np.zeros((self.number_point, 1))
        # determine the direction and the distance array
        for i in range(self.number_point - 1):
            direction[i, :] = (self.points[i + 1] - self.points[i]) / np.linalg.norm(self.points[i + 1] - self.points[i])
            distance[i, :] = self.points[i + 1] - self.points[i]
            t[i + 1] = t[i] + np.linalg.norm(distance[i, :]) / 3

        # determine which time interval is in at the current time
        for i in range(self.number_point - 1):
            if time > t[-1]:
                x = self.points[-1]
                break
            elif time == 0:
                x = self.points[0]
                break
            elif t[i] <= time < t[i + 1]:
                tau = 1 / (t[i + 1] - t[i])
                if i == 0:
                    init_a = direction[i, :]
                    end_a = direction[i + 1, :]
                elif i == self.number_point - 2:
                    init_a = direction[i, :]
                    end_a = np.zeros((1, 3))
                else:
                    init_a = direction[i, :]
                    end_a = direction[i + 1, :]
                # construct the A and b matrix to solve for the coefficient matrix
                A = np.array([[0, 0, 0, 0, 0, 0, 0, 1],
                              [1, 1, 1, 1, 1, 1, 1, 1],
                              [0, 0, 0, 0, 0, 0, 1, 0],
                              [7, 6, 5, 4, 3, 2, 1, 0],
                              [0, 0, 0, 0, 0, 2, 0, 0],
                              [42, 30, 20, 12, 6, 2, 0, 0],
                              [0, 0, 0, 0, 6, 0, 0, 0],
                              [210, 120, 60, 24, 6, 0, 0, 0]]).astype('float64')
                b = np.vstack((self.points[i], self.points[i + 1], 0 * init_a, 0 * end_a, 0 * init_a, 0 * end_a, np.zeros((1, 3)), np.zeros((1, 3)))).astype('float64')
                coeff_x = lsmr(A, b[:, 0])[0]
                coeff_y = lsmr(A, b[:, 1])[0]
                coeff_z = lsmr(A, b[:, 2])[0]
                coeff_matrix = np.hstack((coeff_x.reshape(8, 1), coeff_y.reshape(8, 1), coeff_z.reshape(8, 1)))
                interval_num = ((time - t[i]) * tau)[0]
                print('tn', interval_num)
                x = (np.array([interval_num ** 7, interval_num ** 6, interval_num ** 5, interval_num ** 4, interval_num ** 3, interval_num ** 2, interval_num, 1], dtype=float) @ coeff_matrix).reshape(3, 1)
                x_dot = (np.array([7 * interval_num ** 6, 6 * interval_num ** 5, 5 * interval_num ** 4, 4 * interval_num ** 3, 3 * interval_num ** 2, 2 * interval_num, 1, 0], dtype=float) @ coeff_matrix).reshape(3, 1)
                x_ddot = (np.array([42 * interval_num ** 5, 30 * interval_num ** 4, 20 * interval_num ** 3, 12 * interval_num ** 2, 6 * interval_num, 2, 0, 0], dtype=float) @ coeff_matrix).reshape(3, 1)
                x_dddot = (np.array([210 * interval_num ** 4, 120 * interval_num ** 3, 60 * interval_num ** 2, 24 * interval_num, 6, 0, 0, 0], dtype=float) @ coeff_matrix).reshape(3, 1)
                x_ddddot = (np.array([840 * interval_num ** 3, 360 * interval_num ** 2, 120 * interval_num, 24, 0, 0, 0, 0], dtype=float) @ coeff_matrix).reshape(3, 1)

        flat_output = {'x': x, 'x_dot': x_dot, 'x_ddot': x_ddot, 'x_dddot': x_dddot, 'x_ddddot': x_ddddot,
                       'yaw': yaw, 'yaw_dot': yaw_dot}

        return flat_output