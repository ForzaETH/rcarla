#!/bin/python3

import numpy as np

def vehicle_dynamics_st_update_pacejka(state, accel, steer_angle_vel, p, dt):
    start = state
    # Local params
    v_b = 3  # m/s
    v_s = 1  # m/s
    v_min = v_b - 2 * v_s  # m/s
    g = 9.81  # m/s^2

    # Lateral tire slip angles
    start_vx = start['velocity'] * np.cos(start['slip_angle'])
    start_vy = start['velocity'] * np.sin(start['slip_angle'])
    if start['velocity'] >= v_min:
        alpha_f = np.arctan2(
            (-start_vy - p['l_f'] * start['angular_velocity']), start_vx) + start['steer_angle']
        alpha_r = np.arctan2(
            (-start_vy + p['l_r'] * start['angular_velocity']), start_vx)
    else:
        alpha_f = 0
        alpha_r = 0

    # Compute vertical tire forces (load transfer due to acceleration)
    F_zf = p['m'] * (-accel * p['h_cg'] + g * p['l_r']) / (p['l_f'] + p['l_r'])
    F_zr = p['m'] * (accel * p['h_cg'] + g * p['l_f']) / (p['l_f'] + p['l_r'])

    # Combined lateral slip forces according to Pacejka
    F_yf = p['mu'] * p['Df'] * F_zf * np.sin(
        p['Cf'] * np.arctan(
            p['Bf'] * alpha_f - p['Ef'] * (p['Bf'] * alpha_f - np.arctan(
                p['Bf'] * alpha_f))))
    F_yr = p['mu'] * p['Dr'] * F_zr * np.sin(
        p['Cr'] * np.arctan(
            p['Br'] * alpha_r - p['Er'] * (p['Br'] * alpha_r - np.arctan(
                p['Br'] * alpha_r))))

    # now all-wheel drive model is based on https://arxiv.org/pdf/2404.08362
    F_m = accel * p['m']
    rear_tire_split = 1
    assert 0 <= rear_tire_split <= 1, "Rear tire split must be between 0 and 1"
    F_xr = rear_tire_split * F_m
    F_xf = (1 - rear_tire_split) * F_m

    # Compute first derivatives of state
    x_dot = start_vx * np.cos(start['theta']) - \
        start_vy * np.sin(start['theta'])
    y_dot = start_vx * np.sin(start['theta']) + \
        start_vy * np.cos(start['theta'])
    vx_dot = (1 / p['m']) * (
        F_xr
        + F_xf * np.cos(start['steer_angle'])
        - F_yf * np.sin(start['steer_angle'])
        + start_vy * start['angular_velocity'] * p['m']
    )
    vy_dot = (1 / p['m']) * (
        F_yr
        + F_xf * np.sin(start['steer_angle'])
        + F_yf * np.cos(start['steer_angle'])
        - start_vx * start['angular_velocity'] * p['m']
    )
    steer_angle_dot = steer_angle_vel
    theta_dot = start['angular_velocity']
    theta_ddot = (1 / p['I_z']) * (
        F_yf * p['l_f'] * np.cos(start['steer_angle'])
        + F_xf * p['l_f'] * np.sin(start['steer_angle'])
        - F_yr * p['l_r']
    )

    end_vx = start_vx + vx_dot * dt
    end_vy = start_vy + vy_dot * dt

    # Update state
    end = dict()
    end['x'] = start['x'] + x_dot * dt
    end['y'] = start['y'] + y_dot * dt
    end['theta'] = start['theta'] + theta_dot * dt
    end['velocity'] = np.sqrt(end_vx ** 2 + end_vy ** 2)
    end['steer_angle'] = start['steer_angle'] + steer_angle_dot * dt
    end['angular_velocity'] = start['angular_velocity'] + theta_ddot * dt
    end['slip_angle'] = np.arctan2(end_vy, end_vx)

    # Mix with kinematic model at low speeds (assuming update_k is also njit-compiled)
    kin_end = vehicle_dynamics_update_k(
        start, accel, steer_angle_vel, p, dt)

    # Weights for mixing
    w_std = 0.5 * (1 + np.tanh((start['velocity'] - v_b) / v_s))
    w_kin = 1 - w_std
    if start['velocity'] < v_min:
        w_std = 0
        w_kin = 1

    # Mix states
    for key in start.keys():
        end[key] = w_std * end[key] + w_kin * kin_end[key]

    # mix derivatives of velocity
    vx_dot = w_std * vx_dot + w_kin * accel
    vy_dot = w_std * vy_dot + w_kin * 0

    return end, np.array([vx_dot, vy_dot])

def vehicle_dynamics_update_k(start, accel, steer_angle_vel, p, dt):
    # Compute first derivatives of state
    x_dot = start['velocity'] * np.cos(start['theta'])
    y_dot = start['velocity'] * np.sin(start['theta'])
    v_dot = accel
    theta_dot = start['velocity'] * np.tan(start['steer_angle']) / p['wheelbase']

    # Update state
    end = dict()
    end['x'] = start['x'] + x_dot * dt
    end['y'] = start['y'] + y_dot * dt
    end['steer_angle'] = start['steer_angle'] + steer_angle_vel * dt
    end['velocity'] = start['velocity'] + v_dot * dt
    end['theta'] = start['theta'] + theta_dot * dt
    end['angular_velocity'] = theta_dot
    end['slip_angle'] = 0

    return end
