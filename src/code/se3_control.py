import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2


        self.gamma = self.k_drag / self.k_thrust


        ## v =2.2
        self.kr = np.diag([300, 300, 40])
        self.kw = np.diag([25, 25, 15])
        self.kd = np.diag([5, 5, 5])
        self.kp = np.diag([13, 13, 13])



    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # Geometric Nonlinear Controller
        # desired position
        des_x = flat_output["x"]
        # desired velocity
        des_x_dot = flat_output["x_dot"]
        # desired acceleration
        des_x_ddot = flat_output["x_ddot"]
        # desired jerk
        des_x_dddot = flat_output["x_dddot"]
        # desired snap
        des_x_ddddot = flat_output["x_ddddot"]
        # yaw angle
        yaw = flat_output["yaw"]
        # yaw rate
        yaw_dot = flat_output["yaw_dot"]
        # current position
        cur_x = state["x"]
        # current velocity
        cur_x_dot = state["v"]
        # current quaternion
        i, j, k, w = state["q"]
        # current angular velocity
        p, q, r = state["w"]

        # determine F_des
        x_ddot_control = flat_output["x_ddot"] - self.kd @ (cur_x_dot - des_x_dot) - self.kp @ (cur_x - des_x)
        F_des = self.mass * x_ddot_control.reshape(3, 1) + np.array([0, 0, self.mass * self.g]).reshape(3, 1)
        # determine teh rotation matrix
        R = Rotation.from_quat(state["q"]).as_matrix()
        # determine u1
        b3 = R @ np.array([0, 0, 1]).T
        u1 = b3.T @ F_des
        # determine R_des
        b3_des = F_des / np.linalg.norm(F_des)
        a_psi = np.array([np.cos(yaw), np.sin(yaw), 0]).reshape(3, 1)

        b2_des = np.cross(b3_des, a_psi, axis=0) / np.linalg.norm(np.cross(b3_des, a_psi, axis=0))
        R_des = np.concatenate((np.cross(b2_des, b3_des, axis=0), b2_des, b3_des), axis=1)
        # calculate the orientation error
        error_R_ = 1 / 2 * (R_des.T @ R - R.T @ R_des)
        error_R = np.array([error_R_[2, 1], -error_R_[2, 0], error_R_[1, 0]]).reshape(3, 1)
        # determine u2
        w = np.array([p, q, r]).reshape(3, 1)
        des_w = np.zeros((3, 1))
        error_w = w - des_w
        u2 = self.inertia @ (-self.kr @ error_R - self.kw @ error_w)
        # calculate the input matrix
        input_M = np.concatenate((u1.reshape(1, 1), u2), axis=0)
        A = np.array([[1, 1, 1, 1],
                      [0, self.arm_length, 0, -self.arm_length],
                      [-self.arm_length, 0, self.arm_length, 0],
                      [self.gamma, -self.gamma, self.gamma, -self.gamma]])
        F, _, _, _ = np.linalg.lstsq(A, input_M)

        cmd_motor_speeds = np.sqrt(F / self.k_thrust) * np.sign(F)
        cmd_thrust = u1
        cmd_moment = u2
        cmd_q = Rotation.from_matrix(R_des).as_quat()

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        return control_input



