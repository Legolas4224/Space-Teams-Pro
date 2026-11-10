# This module is designed to just point the shuttle towards the target to minimise excess fuel use.

import numpy as np
import numpy.typing as npt
import spaceteams as st
import copy



"""
Get angular velocity from two quaternions representing the orientation of 
an object at time (t0) and (t0 + dt). Assumes scalar-last quaternions.
"""
def AngVelFromQuats(q1: npt.NDArray, q2: npt.NDArray, dt: float):
    return (2.0 / dt) * np.array([
        q1[3]*q2[0] - q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1],
        q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] - q1[2]*q2[0],
        q1[3]*q2[2] - q1[0]*q2[1] + q1[1]*q2[0] - q1[2]*q2[3]])





def point_to_target(shuttle, phobosFrame, shuttleFrame, target) :

    locVelAcc = shuttle.getLocVelAcc()
    loc = locVelAcc.loc_WRT_ExprIn(phobosFrame)
    vel = locVelAcc.vel_WRT_ExprIn(phobosFrame)
    rot = shuttle.getRotation().Quat_WRT(phobosFrame)
    angvel = shuttle.getAngVelocity().WRT_ExprIn(phobosFrame)

    # Rotational control
    targetState: st.frames.FramedLocVelAcc = target.getLocVelAcc()
    targetLoc: npt.NDArray = targetState.loc_WRT_ExprIn(phobosFrame)
    targetVel: npt.NDArray = targetState.vel_WRT_ExprIn(phobosFrame)

    goalDirection: npt.NDArray = (targetLoc - loc) / np.linalg.norm(targetLoc - loc)
    # Gains
    accel_gain: float = 1.0
    angvel_gain: float = 0.2
    kp_pitch = 2.0
    kp_yaw = 2.0
    kp_roll = 2.0

    # Gains (add near your other gains)
    kd_roll = 2.0
    kd_pitch = 2.0
    kd_yaw = 2.0


    # Align shuttle x-axis to target and shuttle y-axis to Phobos y-axis
    x_hat = copy.deepcopy(goalDirection)
    y_hat = np.array([0.0, -1.0, 0.0])
    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / np.linalg.norm(z_hat)
    T_shuttlePhobos = np.vstack((x_hat, y_hat, z_hat))
    q_shuttlePhobos = st.math.DCM_to_Quat(T_shuttlePhobos)

    desired_AngVel = np.zeros(3)
    if angvel_gain != 0.0:
        desired_AngVel = AngVelFromQuats(rot, q_shuttlePhobos, 1.0 / angvel_gain)
    desired_AngVel = st.frames.FramedAngVel(desired_AngVel, phobosFrame).WRT(phobosFrame).ExprIn(shuttleFrame)

    desired_AngVel_x = desired_AngVel[0]
    desired_AngVel_y = desired_AngVel[1]
    desired_AngVel_z = desired_AngVel[2]

    # get current angular velocity values
    current_AngVel = shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame)
    current_AngVel_x: float = current_AngVel[0]
    current_AngVel_y: float = current_AngVel[1]
    current_AngVel_z: float = current_AngVel[2]

    # calculate angular velocity error
    AngVel_error_x = desired_AngVel_x - current_AngVel_x
    AngVel_error_y = desired_AngVel_y - current_AngVel_y
    AngVel_error_z = desired_AngVel_z - current_AngVel_z




    strength_torque = shuttle.GetParam(st.VarType.double, ["Thrusters", "TorqueStrength_Nm"])

    deadzone = 0.08

    if AngVel_error_x < deadzone :
        torque_x = kp_roll * AngVel_error_x * strength_torque
        d_torque_x = -kd_roll * current_AngVel_x * strength_torque

    else : 
        d_torque_x = 0
        torque_x = 0

    if AngVel_error_y < deadzone :
        torque_y = kp_pitch * AngVel_error_y * strength_torque
        d_torque_y = -kd_pitch * current_AngVel_y * strength_torque

    else : 
        d_torque_y = 0
        torque_y = 0

    if AngVel_error_z < deadzone :
        torque_z = kp_yaw * AngVel_error_z * strength_torque
        d_torque_z = -kd_yaw * current_AngVel_z * strength_torque

    else : 
        d_torque_z = 0
        torque_z = 0
    
    ########################
    # P term torque
    
    
    #torque_y = kp_pitch * AngVel_error_y * strength_torque
    #torque_z = kp_yaw * AngVel_error_z * strength_torque

    #######################
    # D term torque

    #d_torque_x = kd_roll * AngVel_error_x * strength_torque
    #d_torque_y = kd_pitch * AngVel_error_y * strength_torque
    #d_torque_z = kd_yaw * AngVel_error_z * strength_torque



    #######################
    # D term torque (damps rotation to stop overshoot)

    
    #d_torque_y = -kd_pitch * current_AngVel_y * strength_torque
    #d_torque_z = -kd_yaw * current_AngVel_z * strength_torque

    torque_Nm = np.array([torque_x + d_torque_x,
                          torque_y + d_torque_y,
                          torque_z + d_torque_z])

    #torque_Nm = np.array([torque_x, torque_y, torque_z])
    angularAcceleration_rad_s2 = np.linalg.inv(shuttle.getInertiaTensor(shuttleFrame)) @ torque_Nm
    return angularAcceleration_rad_s2