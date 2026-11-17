#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3, 
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


#########################################################
# EXAMPLE USER CODE (NOTE: RUNNING A LOOP IS NECESSARY) #
#########################################################

import copy
from PIL import Image

###############################
# Get initial simulation time #
###############################

t0_sim: st.timestamp = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"])
t0: datetime.datetime = t0_sim.as_datetime()

###################
# Make a log file #
###################
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "controller_log.txt")
log_file = open(LOG_PATH, "w", buffering=1)   # line-buffered, fresh file each run

def status(msg: str, key: str = "Status") -> None:
    print(msg, flush=True)
    st.OnScreenLogMessage(msg, key, st.Severity.Info)
    log_file.write(msg + "\n")

"""
Get angular velocity from two quaternions representing the orientation of 
an object at time (t0) and (t0 + dt). Assumes scalar-last quaternions.
"""
def AngVelFromQuats(q1: npt.NDArray, q2: npt.NDArray, dt: float):
    return (2.0 / dt) * np.array([
        q1[3]*q2[0] - q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1],
        q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] - q1[2]*q2[0],
        q1[3]*q2[2] - q1[0]*q2[1] + q1[1]*q2[0] - q1[2]*q2[3]])

####################################
# Classes and Function Definitions # 
####################################

"""
Conventions used everywhere:
  * Positions and velocities are in the Phobos frame, in metres and metres/second.
  * "Radar item" = one 3x3 array from GetNearestDebrisLocations():
        item[0, :]  rock centre RELATIVE to the shuttle (m)
        item[1, 0]  distance to its surface     item[1, 1]  range rate (m/s)
        item[1, 2]  diameter (m)                item[2, 0]  time of measurement (s since sim start)
"""

import numpy as np
from collections import deque

# =====================================================================================
# 1. TRACKING - the radar forgets everything between readings, so we keep a list of rocks
# =====================================================================================

class Rock:
    def __init__(self, position, diameter, time):
        pass

    def add_sighting(self, position, diameter, time):
        pass

    def update_velocity(self):
        pass

    def predicted_position(self, time):
        pass


class Tracker:
    def __init__(self, gate=15.0, max_age=5.0):
        pass

    def update(self, radar_items, shuttle_position, now):
        pass

    def find_match(self, position, diameter, time):
        """The closest existing rock that is a similar size and within the gate, else None."""
        pass

# =====================================================================================
# 2. AVOIDANCE - choose the velocity to fly
# =====================================================================================

def clearance(my_velocity, rel_pos, rock_vel, keep_out, horizon=15.0):
    """
    If we fly at my_velocity, how close do we get to the worst rock in the next `horizon` seconds,
    after subtracting its keep-out radius? Positive = safe, negative = collision course.
      rel_pos  (n,3)  where each rock is, relative to us
      rock_vel (n,3)  how fast each rock is moving
      keep_out (n,)   radius we want to stay outside (rock radius + safety margin)
    """
    pass

def two_sideways_directions(direction):
    """Two unit vectors that are perpendicular to `direction` and to each other."""
    pass

class Avoider:
    def __init__(self, margin=6.0, unknown_extra=4.0, horizon=50.0):
        pass

    def choose(self, wanted_velocity, rocks, shuttle_position, now):
        """Return the velocity to fly: the wanted one if it is safe, else the smallest safe change."""
        pass

# =====================================================================================
# 3. FUEL - only fire the thrusters when the velocity error is worth a burn
# =====================================================================================

class BurnGovernor:
    def __init__(self, gain=5.0, start=0.3, stop=0.1):
        pass

    def acceleration(self, wanted_velocity, current_velocity):
        pass

# =====================================================================================
# 4. GOAL - the target is a SPHERE, so we only steer when our path would miss it
# =====================================================================================

class GoalPlanner:
    """
    Decides the velocity we WANT (before avoidance). Instead of constantly steering at the exact
    centre of the target, it lets us coast while our current path already passes well inside the
    goal sphere, and only steers when the path drifts towards the edge. Less steering = less fuel.
    """

    def __init__(self, cruise_speed, goal_radius=10.0, start_fraction=0.7, stop_fraction=0.3):
        pass
        
    def miss_distance(self, shuttle_position, shuttle_velocity, target_position):
        """How far from the target centre we would pass if we kept our current velocity."""
        pass

    def wanted_velocity(self, shuttle_position, shuttle_velocity, target_position):
        pass

####################################
#        Main Code Start           # 
####################################

tracker  = Tracker()             
avoider  = Avoider()
governor = BurnGovernor(gain=500000.0)

##################
# Start the loop #
##################

iteration = 0
imageNumber = 0
exit_flag: bool = False
while not exit_flag:

    ################################
    # Wait between loop iterations #
    ################################
    
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

    ###################################
    # Get elapsed time from sim start #
    ###################################

    t_now: datetime.datetime = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    t_from_start: float = (t_now - t0).total_seconds()

    #####################################
    # Example for getting vehicle state #
    #####################################

    locVelAcc = shuttle.getLocVelAcc()
    loc = locVelAcc.loc_WRT_ExprIn(phobosFrame)
    vel = locVelAcc.vel_WRT_ExprIn(phobosFrame)
    rot = shuttle.getRotation().Quat_WRT(phobosFrame)
    angvel = shuttle.getAngVelocity().WRT_ExprIn(phobosFrame)

    #####################################################
    # Example for getting additional vehicle properties #
    #####################################################

    mass = shuttle.getMass()
    inertiaBodyFrame = shuttle.getInertiaTensor(shuttleFrame)
    damage: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
    fuelUsed: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])

    ##########################################
    # Get the target's location and velocity #
    ##########################################

    targetState: st.frames.FramedLocVelAcc = target.getLocVelAcc()
    targetLoc: npt.NDArray = targetState.loc_WRT_ExprIn(phobosFrame)
    targetVel: npt.NDArray = targetState.vel_WRT_ExprIn(phobosFrame)

    ##############################
    # Calculate desired vecocity #
    ##############################

    goalDirection: npt.NDArray = (targetLoc - loc) / np.linalg.norm(targetLoc - loc)
    goalSpeed_m_s: float = 17.0
    goalVelocity: npt.NDArray = goalSpeed_m_s * goalDirection

    ####################################################################
    # Update tracker module with RF data and calculate a safe velocity #
    ####################################################################

    tracker.update(GetNearestDebrisLocations(50), loc, t_from_start)
    safe_velocity = avoider.choose(goalVelocity, tracker.rocks, loc, t_from_start)
    accel = governor.acceleration(safe_velocity, vel)                # Phobos frame
    accel = st.frames.FramedVector(accel, phobosFrame).ExprIn(shuttleFrame)

    angularAcceleration_rad_s2 =  np.array([0,0,0]) # no rotation at the moment
    #accel= np.array([0,0,10000])
    SetCommand(accel, angularAcceleration_rad_s2) # forward desired accelerations to the sim shuttle

    st.OnScreenLogMessage(f"Current Velocity: {np.linalg.norm(vel)}\nDesired Velocity: {safe_velocity}\nFuel Used: {fuelUsed} kg", "Velocity", st.Severity.Info)

    iteration += 1

st.leave_sim()
