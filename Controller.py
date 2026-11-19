# ruff: noqa: E402, E501
#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import datetime
import os
import sys
import time
import traceback
from collections import deque

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
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(
        st.VarType.doubleM3x3, ["Radar", "SortedDebrisData"]
    )
    nearest = allDebrisInfo[: min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(
        st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame
    )


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


#########################################################
# EXAMPLE USER CODE (NOTE: RUNNING A LOOP IS NECESSARY) #
#########################################################

###############################
# Get initial simulation time #
###############################

t0_sim: st.timestamp = st.SimGlobals.GetSimEntity().GetParam(
    st.VarType.timestamp, ["Time", "StartDateTAI"]
)
t0: datetime.datetime = t0_sim.as_datetime()

###################
# Make a log file #
###################
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "controller_log.txt")
log_file = open(LOG_PATH, "w", buffering=1)  # noqa: SIM115  # line-buffered, fresh file each run


def status(msg: str, key: str = "Status") -> None:
    print(msg, flush=True)
    st.OnScreenLogMessage(msg, key, st.Severity.Info)
    log_file.write(msg + "\n")


"""
Get angular velocity from two quaternions representing the orientation of
an object at time (t0) and (t0 + dt). Assumes scalar-last quaternions.
"""


def AngVelFromQuats(q1: npt.NDArray, q2: npt.NDArray, dt: float):
    return (2.0 / dt) * np.array(
        [
            q1[3] * q2[0] - q1[0] * q2[3] - q1[1] * q2[2] + q1[2] * q2[1],
            q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3] - q1[2] * q2[0],
            q1[3] * q2[2] - q1[0] * q2[1] + q1[1] * q2[0] - q1[2] * q2[3],
        ]
    )


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


# =====================================================================================
# 1. TRACKING - the radar forgets everything between readings, so we keep a list of rocks
# =====================================================================================


class Rock:
    def __init__(self, position, diameter, time):
        self.position = position  # latest ABSOLUTE position
        self.diameter = diameter
        self.velocity = None  # unknown until we have watched it for a while
        self.last_seen = time
        self.history = deque(
            maxlen=30
        )  # recent (time, position) samples; capped so memory can't grow
        self.history.append((time, position))

    def add_sighting(self, position, diameter, time):
        if time <= self.last_seen:  # the radar repeated an old reading: ignore it
            return
        self.position = position
        self.diameter = 0.8 * self.diameter + 0.2 * diameter  # smooth the noisy diameter
        self.last_seen = time
        self.history.append((time, position))
        self.update_velocity()

    def update_velocity(self):
        times = np.array([t for t, p in self.history])
        points = np.array([p for t, p in self.history])
        if len(times) < 3 or times[-1] - times[0] < 1.0:
            return  # too little data: a velocity now would be mostly noise
        # Fit a straight line through the positions. Its slope (per axis) is the velocity.
        self.velocity = np.polyfit(times - times[0], points, 1)[0]

    def predicted_position(self, time):
        if self.velocity is None:
            return self.position
        return self.position + self.velocity * (time - self.last_seen)


class Tracker:
    def __init__(self, gate=5.0, max_age=5.0):
        self.rocks = []
        self.gate = gate  # sightings closer than this (m) count as the same rock
        self.max_age = max_age  # forget rocks not seen for this long (s)

    def update(self, radar_items, shuttle_position, now, target_position):
        TRACKER_DEBUG = True

        CONE_ANGLE: float = np.radians(50.0)
        CORE_BUBBLE: float = 50 # (m)
        target_vec = target_position - shuttle_position

        ROCKS_FROM_RADAR = len(radar_items)
        ETC = f"tv: {target_vec}\n"
        if TRACKER_DEBUG:
            for r in radar_items:
                debris_pos = r[0]
                dot = np.dot(target_vec, debris_pos)
                ang = np.acos(dot / (np.linalg.norm(target_vec) * np.linalg.norm(debris_pos)))
                ETC += f"({debris_pos}, {dot}, {np.degrees(ang)})\n"

        def debris_in_cone(debris) -> bool:
            # debris_pos already relative
            debris_pos = debris[0]
            if debris[1][0] < CORE_BUBBLE:
                return True
            dot = np.dot(target_vec, debris_pos)
            ang = np.acos(dot / (np.linalg.norm(target_vec) * np.linalg.norm(debris_pos)))
            return dot > 0.0 and ang < CONE_ANGLE

        radar_items = [r for r in radar_items if debris_in_cone(r)]
        ROCKS_IN_FRUSTUM = len(radar_items)

        #for item in radar_items:
        #    # The radar gives positions relative to the shuttle. Add the shuttle's own
        #    # position so that all sightings share one fixed frame.
        #    position = shuttle_position + item[0, :]
        #    diameter, time = item[1, 2], item[2, 0]
        #    rock = self.find_match(position, diameter, time)
        #    if rock is None:
        #        self.rocks.append(Rock(position, diameter, time))  # new rock
        #    else:
        #        rock.add_sighting(position, diameter, time)

        # replace the "for item in radar_items" loop with:
        sightings = [(shuttle_position + item[0, :], item[1, 2], item[2, 0])
                    for item in radar_items]

        pairs = []
        for i, (pos, d, t) in enumerate(sightings):
            for j, rock in enumerate(self.rocks):
                if abs(rock.diameter - d) > 0.25 * rock.diameter:
                    continue
                dist = np.linalg.norm(rock.predicted_position(t) - pos)
                if dist < self.gate:
                    pairs.append((dist, i, j))

        pairs.sort(key=lambda p: p[0])          # globally closest first, order-independent
        used_s, used_r = set(), set()
        for dist, i, j in pairs:
            if i in used_s or j in used_r:
                continue
            self.rocks[j].add_sighting(*sightings[i])
            used_s.add(i)
            used_r.add(j)

        for i, s in enumerate(sightings):       # new rocks only after matching is done
            if i not in used_s:
                self.rocks.append(Rock(*s))


        self.rocks = [r for r in self.rocks if now - r.last_seen < self.max_age]
        if TRACKER_DEBUG:
            return (ROCKS_FROM_RADAR, ROCKS_IN_FRUSTUM, ETC)

    def find_match(self, position, diameter, time):
        """The closest existing rock that is a similar size and within the gate, else None."""
        best, best_distance = None, self.gate
        for rock in self.rocks:
            if abs(rock.diameter - diameter) > 0.25 * rock.diameter:
                continue  # sizes too different
            distance = np.linalg.norm(rock.predicted_position(time) - position)
            if distance < best_distance:
                best, best_distance = rock, distance
        return best


# =====================================================================================
# 2. AVOIDANCE - choose the velocity to fly
# =====================================================================================


def clearance(my_velocity, rel_pos, rock_vel, keep_out, horizon=50.0):
    """
    If we fly at my_velocity, how close do we get to the worst rock in the next `horizon` seconds,
    after subtracting its keep-out radius? Positive = safe, negative = collision course.
      rel_pos  (n,3)  where each rock is, relative to us
      rock_vel (n,3)  how fast each rock is moving
      keep_out (n,)   radius we want to stay outside (rock radius + safety margin)
    """
    rel_vel = rock_vel - my_velocity  # how each rock moves as seen from us
    speed_squared = (rel_vel**2).sum(axis=1) + 1e-9
    # time when the rock is closest to us (from minimising |rel_pos + t * rel_vel|)
    t = -(rel_pos * rel_vel).sum(axis=1) / speed_squared
    t = np.clip(t, 0.0, horizon)  # only look from now until the horizon
    closest = np.linalg.norm(rel_pos + t[:, None] * rel_vel, axis=1)
    # return (closest - keep_out).min()
    return np.all((closest-keep_out) > 0)


def two_sideways_directions(direction):
    """Two unit vectors that are perpendicular to `direction` and to each other."""
    helper = np.array([0.0, 0.0, 1.0]) if abs(direction[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    side1 = np.cross(direction, helper)
    side1 /= np.linalg.norm(side1)
    side2 = np.cross(direction, side1)
    return side1, side2


class Avoider:
    def __init__(self, margin=6.0, unknown_extra=4.0, horizon=50.0):
        self.margin = margin  # metres of clearance beyond each rock's surface
        self.unknown_extra = (
            unknown_extra  # extra margin for rocks whose velocity we don't know yet
        )
        self.horizon = horizon
        self.last_dodge = None  # remembered so we don't flip-flop between dodges
        self.dodge_counter = None

    def choose(self, wanted_velocity, rocks, shuttle_position, now):
        """Return the velocity to fly: the wanted one if it is safe, else the smallest safe change."""
        def dodge_msg(msg) -> None:
            
            st.OnScreenLogMessage(f"Dodge info: {msg}", "Dodge", st.Severity.Info)

        wanted_velocity = np.asarray(wanted_velocity, float)
        speed = np.linalg.norm(wanted_velocity)
        if speed < 1e-3 or len(rocks) == 0:
            dodge_msg("no_dodge (no rocks)")
            return wanted_velocity

        # Describe every rock as plain arrays.
        rel_pos = np.array([r.predicted_position(now) - shuttle_position for r in rocks])
        rock_vel = np.array([r.velocity if r.velocity is not None else np.zeros(3) for r in rocks])
        keep_out = np.array(
            [
                r.diameter / 2 + self.margin + (self.unknown_extra if r.velocity is None else 0.0)
                for r in rocks
            ]
        )

        def score(velocity):
            # this now returns whether there will be:
            #   a collision with any rock, at this velocity
            s = clearance(velocity, rel_pos, rock_vel, keep_out, self.horizon)
            return 1 if s else 0

        # 1. Nothing in the way? Fly exactly as wanted (no thruster use at all).
        if score(wanted_velocity) > 0:
            if self.dodge_counter > 0:
                self.dodge_counter -= 1
                return self.last_dodge
            else:
                self.last_dodge = None
                dodge_msg(f"nothing in way, req_vel:{wanted_velocity}")
                return wanted_velocity

        # 2. Still dodging? Keep the same dodge while it still works (stops flip-flopping).
        if self.last_dodge is not None and score(self.last_dodge) > 0:
            dodge_msg(f"still dodging last: {str(self.last_dodge)}")
            self.dodge_counter = 20
            return self.last_dodge

        # 3. Try sideways dodges, SMALLEST first: a small velocity change costs the least fuel.
        side1, side2 = two_sideways_directions(wanted_velocity / speed)
        tried = [wanted_velocity]
        for size in (0.05, 0.1, 0.2, 0.35, 0.5, 0.75):  # as a fraction of our speed
            for angle in np.linspace(0, 2 * np.pi, 8, endpoint=False):
                sideways = np.cos(angle) * side1 + np.sin(angle) * side2
                candidate = wanted_velocity + size * speed * sideways
                if score(candidate) > 0:
                    self.last_dodge = candidate
                    dodge_msg(f"sideways dodge: {str(candidate)}")
                    self.dodge_counter = 20
                    return candidate
                tried.append(candidate)

        # 4. No sideways dodge works: try slowing down.
        for fraction in (0.5, 0.25, 0.0):
            candidate = wanted_velocity * fraction
            if score(candidate) > 0:
                self.last_dodge = candidate
                dodge_msg(f"trying to slow down: {str(self.last_dodge)}")
                self.dodge_counter = 20
                return candidate
            tried.append(candidate)

        # 4.5. Try speeding up!
        for fraction in (1.15, 1.25, 1.5):
            candidate = wanted_velocity * fraction
            if score(candidate) > 0:
                self.last_dodge = candidate
                dodge_msg(f"trying to speed up: {str(self.last_dodge)}")
                self.dodge_counter = 20
                return candidate
            tried.append(candidate)

        # 5. Nothing is fully safe: take the least-bad option we looked at.
        self.last_dodge = max(tried, key=score)
        self.dodge_counter = 20

        # Report Changes
        dodge_msg(f"nothing fully safe! : {str(self.last_dodge)}")

        return self.last_dodge


# =====================================================================================
# 3. FUEL - only fire the thrusters when the velocity error is worth a burn
# =====================================================================================


class BurnGovernor:
    def __init__(self, gain=5.0, start=0.3, stop=0.1):
        self.gain = gain  # acceleration (m/s^2) per m/s of velocity error
        self.start = start  # start burning when the error is bigger than this (m/s)
        self.stop = stop  # stop burning once the error is smaller than this (m/s)
        self.burning = False

    def acceleration(self, wanted_velocity, current_velocity):
        error = np.asarray(wanted_velocity, float) - np.asarray(current_velocity, float)
        size = np.linalg.norm(error)
        if self.burning and size < self.stop:
            self.burning = False
        elif not self.burning and size > self.start:
            self.burning = True
        return self.gain * error if self.burning else np.zeros(3)


# =====================================================================================
# 4. GOAL - the target is a SPHERE, so we only steer when our path would miss it
# =====================================================================================


class GoalPlanner:
    """
    Decides the velocity we WANT (before avoidance). Instead of constantly steering at the exact
    centre of the target, it lets us coast while our current path already passes well inside the
    goal sphere, and only steers when the path drifts towards the edge. Less steering = less fuel.
    """

    def __init__(self, cruise_speed, goal_radius=15.0, start_fraction=0.7, stop_fraction=0.3):
        self.cruise_speed = cruise_speed
        self.goal_radius = goal_radius
        self.start = (
            start_fraction * goal_radius
        )  # start steering if we would miss the centre by more than this
        self.stop = stop_fraction * goal_radius  # ...and keep steering until the miss is under this
        self.correcting = True

    def miss_distance(self, shuttle_position, shuttle_velocity, target_position):
        """How far from the target centre we would pass if we kept our current velocity."""
        speed = np.linalg.norm(shuttle_velocity)
        if speed < 0.98 * self.cruise_speed:
            return np.inf  # not at cruise speed yet: keep accelerating
        heading = shuttle_velocity / speed
        to_target = target_position - shuttle_position
        along = to_target @ heading  # how far ahead the target is along our heading
        if along <= 0:
            return np.inf  # we are moving away from it
        return np.linalg.norm(to_target - along * heading)  # sideways distance at closest approach

    def wanted_velocity(self, shuttle_position, shuttle_velocity, target_position):
        miss = self.miss_distance(shuttle_position, shuttle_velocity, target_position)
        if self.correcting and miss < self.stop:
            self.correcting = False  # on a good path: coast
        elif not self.correcting and miss > self.start:
            self.correcting = True  # drifting off: steer again
        if not self.correcting:
            return np.asarray(shuttle_velocity, float)  # wanting what we have = no thruster use
        to_target = target_position - shuttle_position
        return self.cruise_speed * to_target / np.linalg.norm(to_target)


def point_at_target(): 
    # points the x axis of the shuttle towards the target. 
    # This is basically just the code from the original beeline example, but with an added d term to avoid overshoot
    import copy
    # Gains
    angvel_gain: float = 0.2
    kp_pitch = 2.0
    kp_yaw = 2.0
    kp_roll = 2.0
    
    kd_pitch = 5.0
    kd_yaw = 5.0
    kd_roll = 5.0

    # Translational control
    targetState: st.frames.FramedLocVelAcc = target.getLocVelAcc()
    targetLoc: npt.NDArray = targetState.loc_WRT_ExprIn(phobosFrame)
    targetVel: npt.NDArray = targetState.vel_WRT_ExprIn(phobosFrame)

    goalDirection: npt.NDArray = (targetLoc - loc) / np.linalg.norm(targetLoc - loc)

    ## Rotational control
    ## Align shuttle x-axis to target and shuttle y-axis to Phobos y-axis
    #x_hat = copy.deepcopy(goalDirection)
    #y_hat = np.array([0.0, -1.0, 0.0])
    #z_hat = np.cross(x_hat, y_hat)
    #z_hat = z_hat / np.linalg.norm(z_hat)
    #T_shuttlePhobos = np.vstack((x_hat, y_hat, z_hat))
    #q_shuttlePhobos = st.math.DCM_to_Quat(T_shuttlePhobos)

    # Rotational control
    # Align shuttle z-axis to target and keep shuttle y-axis as close as possible to Phobos -y
    z_hat = copy.deepcopy(goalDirection)
    up_ref = np.array([0.0, -1.0, 0.0])

    x_hat = np.cross(up_ref, z_hat)            # y × z = x (right-handed)
    x_norm = np.linalg.norm(x_hat)
    if x_norm < 1e-6:                          # target is along the reference axis, so pick any perpendicular
        x_hat = np.cross(np.array([1.0, 0.0, 0.0]), z_hat)
        x_norm = np.linalg.norm(x_hat)
    x_hat = x_hat / x_norm
    y_hat = np.cross(z_hat, x_hat)             # z × x = y, exactly perpendicular to both

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

    # Clamp to avoid excessive burns
    if abs(AngVel_error_x) < 0.02 :
        AngVel_error_x = 0
    if abs(AngVel_error_y) < 0.02 :
        AngVel_error_y = 0
    if abs(AngVel_error_z) < 0.02 :
        AngVel_error_z = 0

    st.OnScreenLogMessage(f"AngVel_error_x = {AngVel_error_x}, AngVel_error_y = {AngVel_error_y}, AngVel_error_z = {AngVel_error_z}", "Attitude", st.Severity.Info,)
    

    strength_torque = shuttle.GetParam(st.VarType.double, ["Thrusters", "TorqueStrength_Nm"])
    torque_x = (kp_roll * AngVel_error_x - kd_roll * current_AngVel_x) * strength_torque 
    torque_y = (kp_pitch * AngVel_error_y - kd_pitch * current_AngVel_y) * strength_torque
    torque_z = (kp_yaw * AngVel_error_z * - kd_yaw * current_AngVel_z) * strength_torque

    torque_Nm = np.array([torque_x, torque_y, torque_z])
    angularAcceleration_rad_s2 = np.linalg.inv(shuttle.getInertiaTensor(shuttleFrame)) @ torque_Nm
    return angularAcceleration_rad_s2


####################################
#        Main Code Start           #
####################################

tracker = Tracker()
avoider = Avoider()
governor = BurnGovernor(gain=50.0)
planner = GoalPlanner(cruise_speed=8.0)

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
    # Calculate desired velocity #
    ##############################

    goalDirection: npt.NDArray = (targetLoc - loc) / np.linalg.norm(targetLoc - loc)
    goalSpeed_m_s: float = 8.0
    goalVelocity = planner.wanted_velocity(loc, vel, targetLoc)

    ####################################################################
    # Update tracker module with RF data and calculate a safe velocity #
    ####################################################################

    tracker.update(GetNearestDebrisLocations(500), loc, t_from_start, targetLoc)
    safe_velocity = avoider.choose(goalVelocity, tracker.rocks, loc, t_from_start)
    accel = governor.acceleration(safe_velocity, vel)  # Phobos frame
    accel = st.frames.FramedVector(accel, phobosFrame).ExprIn(shuttleFrame)

    angularAcceleration_rad_s2 = np.array(
        [0, 0, 0]
    )  #point_to_target(shuttle, phobosFrame, shuttleFrame, target)
    angularAcceleration_rad_s2 = point_at_target()

    SetCommand(
        accel, angularAcceleration_rad_s2
    )  # forward desired accelerations to the sim shuttle

    st.OnScreenLogMessage(
        f"Current Speed: {np.linalg.norm(vel)}\nDesired Velocity: {safe_velocity}\nFuel Used: {fuelUsed} kg",
        "Velocity",
        st.Severity.Info,
    )

    iteration += 1

st.leave_sim()
