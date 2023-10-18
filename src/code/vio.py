#%% Imports

import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
from scipy.spatial.transform import Rotation


#%% Functions

def nominal_state_update(nominal_state, w_m, a_m, dt):
    """
    function to perform the nominal state update

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                    all elements are 3x1 vectors except for q which is a Rotation object
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :return: new tuple containing the updated state
    """
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    new_p = p + v * dt + 1 / 2 * (Rotation.as_matrix(q) @ (a_m - a_b) + g) * dt ** 2
    new_q = q * Rotation.from_rotvec(((w_m - w_b) * dt).reshape(3, ))
    new_v = v + (Rotation.as_matrix(q) @ (a_m - a_b) + g) * dt

    return new_p, new_v, new_q, a_b, w_b, g


def error_covariance_update(nominal_state, error_state_covariance, w_m, a_m, dt,
                            accelerometer_noise_density, gyroscope_noise_density,
                            accelerometer_random_walk, gyroscope_random_walk):
    """
    Function to update the error state covariance matrix

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param w_m: 3x1 vector - measured angular velocity in radians per second
    :param a_m: 3x1 vector - measured linear acceleration in meters per second squared
    :param dt: duration of time interval since last update in seconds
    :param accelerometer_noise_density: standard deviation of accelerometer noise
    :param gyroscope_noise_density: standard deviation of gyro noise
    :param accelerometer_random_walk: accelerometer random walk rate
    :param gyroscope_random_walk: gyro random walk rate
    :return:
    """

    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    R = Rotation.as_matrix(q)
    a_hat = np.array([[0, -(a_m - a_b)[2], (a_m - a_b)[1]], [(a_m - a_b)[2], 0, -(a_m - a_b)[0]],
                      [-(a_m - a_b)[1], (a_m - a_b)[0], 0]], dtype=object)
    Fx = np.eye(18)
    Fx[0:3, 3:6] = np.eye(3) * dt
    Fx[3:6, 6:9] = -R @ a_hat * dt
    Fx[3:6, 9:12] = -R * dt
    Fx[3:6, 15:18] = np.eye(3) * dt
    RT = Rotation.from_rotvec(((w_m - w_b) * dt).reshape(3, )).as_matrix()
    Fx[6:9, 6:9] = RT.T
    Fx[6:9, 12:15] = -np.eye(3) * dt

    Fi = np.zeros((18, 12))
    for i in range(3, 15):
        Fi[i, i - 3] = 1

    Qi = np.eye(12)
    Vi = (accelerometer_noise_density * dt) ** 2 * np.eye(3)
    thetai = (gyroscope_noise_density * dt) ** 2 * np.eye(3)
    Ai = accelerometer_random_walk ** 2 * dt * np.eye(3)
    omegai = gyroscope_random_walk ** 2 * dt * np.eye(3)
    Qi[:3, :3] = Vi
    Qi[3:6, 3:6] = thetai
    Qi[6:9, 6:9] = Ai
    Qi[9:12, 9:12] = omegai
    # return an 18x18 covariance matrix
    return Fx @ error_state_covariance @ Fx.T + Fi @ Qi @ Fi.T


def measurement_update_step(nominal_state, error_state_covariance, uv, Pw, error_threshold, Q):
    """
    Function to update the nominal state and the error state covariance matrix based on a single
    observed image measurement uv, which is a projection of Pw.

    :param nominal_state: State tuple (p, v, q, a_b, w_b, g)
                        all elements are 3x1 vectors except for q which is a Rotation object
    :param error_state_covariance: 18x18 initial error state covariance matrix
    :param uv: 2x1 vector of image measurements
    :param Pw: 3x1 vector world coordinate
    :param error_threshold: inlier threshold
    :param Q: 2x2 image covariance matrix
    :return: new_state_tuple, new error state covariance matrix
    """
    
    # Unpack nominal_state tuple
    p, v, q, a_b, w_b, g = nominal_state

    # Compute the innovation next state, next error_state covariance
    R = q.as_matrix()
    Pc = R.T @ (Pw - p)
    Pc_image = Pc[0:2] / Pc[2]
    innovation = uv - Pc_image
    # print('Pcxy',Pc_xy)
    # print('Pc',Pc)
    # update according to the observation
    if np.linalg.norm(innovation) < error_threshold:
        # construct Ht
        Ht = np.zeros((2, 18))
        Pc = Pc.flatten()
        # print('Pcf',Pc)
        uv = uv.flatten()
        Pc_x = Pc_image[0][0]
        Pc_y = Pc_image[1][0]
        # print('Pc_x',Pc_x,type(Pc_x))

        diff_Pc_delta_p = - R.T
        diff_zt_delta_theta = (1 / (Pc[2]) * np.array([[1, 0, -Pc_x], [0, 1, -Pc_y]])) @ np.array(
            [[0, -Pc[2], Pc[1]], [Pc[2], 0, -Pc[0]], [-Pc[1], Pc[0], 0]])
        diff_zt_delta_p = (1 / (Pc[2]) * np.array([[1, 0, -Pc_x], [0, 1, -Pc_y]])) @ diff_Pc_delta_p
        Ht[:, 0:3] = diff_zt_delta_p
        Ht[:, 6:9] = diff_zt_delta_theta

        # update the error state covariance
        Kt = error_state_covariance @ Ht.T @ np.linalg.inv(Ht @ error_state_covariance @ Ht.T + Q)
        error_state_covariance = (np.eye(18) - Kt @ Ht) @ error_state_covariance @ (
                    np.eye(18) - Kt @ Ht).T + Kt @ Q @ Kt.T

        # update the new nominal state
        delta_x = Kt @ innovation
        delta_p = delta_x[0:3]
        p = p + delta_p
        delta_v = delta_x[3:6]
        v = v + delta_v
        delta_q = delta_x[6:9]
        q = q * Rotation.from_rotvec(delta_q.flatten())
        delta_ab = delta_x[9:12]
        a_b = a_b + delta_ab
        delta_wb = delta_x[12:15]
        w_b = w_b + delta_wb
        delta_g = delta_x[15:18]
        g = g + delta_g
    return (p, v, q, a_b, w_b, g), error_state_covariance, innovation














