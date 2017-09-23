"""
Navigation classes and functions.

This module contains the classes and functions for the basic navigation of the 
copter, mostly in GUIDED mode. Some of the functions are referenced and modified
from the navigation functions written by the Dronekit group.

Reference:
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
    http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html

Classes:
    Takeoff(threading.Thread): copter takeoff sequence.
    
Functions:
    set_mode: set vehicle flight mode.
    arm_and_takeoff: takeoff to a target altitude.
    wait_for_disarm: wait for the vehicle to disarm.
    eclipse_compensate: compensate earth's eclipse effect.
    get_location_metres: calculate coordinate based on distance vector.
    get_position_error: calculate position vector between two coordinates.
    get_distance_metres: calculate distance between two coordiantes.
    get_bearing: calculate bearing between two coordinates.
    goto: move the vehicle to a location.
    send_ned_velocity: send velocity control in NED frame.
    send_global_velocity: send velocity control in WGS84 frame.
    goto_position_target_global_int: goto a location in WGS84 frame.
    goto_position_target_local_ned: goto a location in body NED frame.
    
Copyright:
    Copyright 2017 Quan Yuan, Adaptive Networks and Control Lab,
    Research Center of Smart Networks and Systems,
    School of Information Science and Engineering,
    Fudan University.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
"""

import time
import math
import threading

from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobal
from dronekit import LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions

import comm
import util
import shared


class Takeoff(threading.Thread):
    """
    Perform preflight check, wait `LIFT` command, arm and takeoff.
    
    Class `Takeoff` is a subclass of `threading.Thread`, it calls
    `wait_home_origin`, `preflight_check`, `wait_lift_cmd` and `arm_and_takeoff`
    sequentially to perform takeoff of the copter. 
    
    Because it requires a high-level command link with the GCS (`gcs.py`), 
    an `xbee.XBee` or `xbee.Zigbee` object (either a real one or simulated) 
    is need. See the module `xbee`, `comm.py` and the datasheets for more details.
    
    The copter is a `dronekit.Vehicle` object. Command the copter by using the 
    Dronekit-API. Other functions can also be added accordingly.
    
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
        des_alt(float): takeoff altitude, validated in the main script.
        loiter_time(int): seconds to wait after `des_alt` is reached.   
    """
    
    def __init__(self, vehicle, xbee, des_alt, loiter_time=3):
        threading.Thread.__init__(self)
        self._vehicle = vehicle
        self._xbee = xbee
        self._dalt = des_alt
        self._delay = loiter_time
        
    def run(self):
        """
        Exicute the takeoff sequence.
        """
        if not _wait_home_origin(self._xbee): return
        if not _preflight_check(self._vehicle, self._xbee): return
        if _wait_lift_cmd(self._xbee):
            shared.status['airborne'] = arm_and_takeoff(self._vehicle, self._dalt, self._delay)
            

def _wait_home_origin(xbee):
    """
    Wait for the HOME_ORIGIN sent by the GCS.
    
    `HOME_ORIGIN` is not the internal `home` set by the flight controller, it is
    a high-level reference coordinate, used in the application algorithms.
    Note that this is a blocking wait, so the program is stuck indefinately unless
    stopped by `EXIT` command or killed by KeyboardInterrupt.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        
    Returns:
        bool: True if success, False if killed by `EXIT` command. 
    """
    util.log_info("Waiting HOME_ORIGIN.")
    wait_count = 0
    while not shared.home_origin:
        time.sleep(1)
        wait_count = wait_count + 1
        
        if shared.status['command'] == 'EXIT':
            comm.xbee_broadcast(xbee, "IFO,%s abort takeoff." % shared.AGENT_ID)
            util.log_info("'EXIT' received. Abort takeoff.")
            return False
        
        if wait_count >= 10:
            wait_count = 0
            comm.xbee_broadcast(xbee, "IFO,%s awaiting HOME_ORIGIN." % shared.AGENT_ID)

    return True

def _preflight_check(vehicle, xbee):
    """
    Perform preflight check, validate home coordinates and satellites fix.
    
    This is a blocking check, so the program is stuck indefinately unless
    stopped by `EXIT` command or killed by KeyboardInterrupt.
    
    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
        
    Returns:
        bool: True if success, False if killed by `EXIT` command. 
    """
    util.log_info("Waiting for home location.")
    while not vehicle.home_location:
        if shared.status['command'] == 'EXIT':
            comm.xbee_broadcast(xbee, "IFO,%s abort takeoff." % shared.AGENT_ID)
            util.log_info("'EXIT' received. Abort takeoff.")
            return False
        
        time.sleep(2)
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        comm.xbee_broadcast(xbee, "IFO,%s getting home fix." % shared.AGENT_ID)
            
    # We have a home location now.
    comm.xbee_broadcast(xbee, 'IFO,%s home: %s.' % (shared.AGENT_ID, vehicle.home_location))
    util.log_info('Home location: %s' % vehicle.home_location)
    
    # Check satellite condition to ensure 3D-fix first
    while vehicle.gps_0.fix_type < 3:
        if shared.status['command'] == 'EXIT':
            comm.xbee_broadcast(xbee, "IFO,%s abort takeoff." % shared.AGENT_ID)
            util.log_info("'EXIT' received. Abort takeoff.")
            return False
        
        comm.xbee_broadcast(xbee, "IFO,%s GNSS No 3D-fix." % shared.AGENT_ID)
        util.log_warning("GNSS No 3D Fix.")
        time.sleep(3)

    # APM:Copter parameter: GPS_HDOP_GOOD
    # The value is mutiplied by 100 into a integer, default good HDOP is below 140
    while vehicle.gps_0.eph > 140 or vehicle.gps_0.satellites_visible < 9 :
        if shared.status['command'] == 'EXIT':
            comm.xbee_broadcast(xbee, "IFO,%s abort takeoff." % shared.AGENT_ID)
            util.log_info("'EXIT' received. Abort takeoff.")
            return False
            
        util.log_info(
            "HDOP: %.2f NumSat: %s" % 
            (vehicle.gps_0.eph/100.0, vehicle.gps_0.satellites_visible))
            
        comm.xbee_broadcast(
            xbee,"IFO,%s HDOP: %.2f NumSat: %s" % 
            (shared.AGENT_ID, vehicle.gps_0.eph/100.0, vehicle.gps_0.satellites_visible))
            
        time.sleep(3)    
    # --END of while-- Preflight check passed.  
    
    comm.xbee_broadcast(xbee, "IFO,%s Preflight check passed." % shared.AGENT_ID)
    util.log_info(
        "Preflight check passed. HDOP: %.2f NumSats: %s" % 
        (vehicle.gps_0.eph/100.0, vehicle.gps_0.satellites_visible))
        
    util.log_info("Local time %s" % shared.timestamp)
    return True

def _wait_lift_cmd(xbee):
    """
    Wait for takeoff clearance from the GCS.
    
    This is a blocking check, so the program is stuck indefinately unless
    received `LIFT` clearance, stopped by `EXIT` command or killed by 
    KeyboardInterrupt.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        
    Returns:
        bool: True if success, False if killed by `EXIT` command. 
    """
    shared.status['command'] = 'STDBY'
    util.log_info("%s Standby, awaiting 'LIFT'." % shared.AGENT_ID)
    
    wait_count = 0
    while True:
        time.sleep(.1)
        wait_count = wait_count + 1
        
        if shared.status['command'] == 'LIFT':
            comm.xbee_broadcast(xbee, "IFO,%s cleared for takeoff." % shared.AGENT_ID)
            util.log_info("'LIFT' received! Taking off!")
            return True
            
        elif shared.status['command'] == 'EXIT':
            comm.xbee_broadcast(xbee, "IFO,%s abort takeoff." % shared.AGENT_ID)
            util.log_info("'EXIT' received. Abort takeoff.")
            return False
            
        elif wait_count >= 100:
            wait_count = 0
            comm.xbee_broadcast(xbee,"IFO,%s standby. Alt: %.2f m." % (shared.AGENT_ID, shared.des_alt))
            

def set_mode(vehicle, mode):
    """
    Set the vehicle's flight modes. 200ms period state validation.
    
    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        mode(str): flight mode string, supported by the firmware.
        
    Returns:
        bool: True if success, False if failed. 
              Failure will set shared.status['abort'].
    """
    util.log_info("Setting %s." % mode)
    shared.status['manual_mode'] = mode
    vehicle.mode = VehicleMode(mode)
    
    wait_count = 0   
    while True:
        time.sleep(.2)
        wait_count = wait_count + 1
        
        if vehicle.mode.name == mode :
            return True
            
        elif wait_count >= 45:
            util.log_warning("Unable to set %s. Assume link lost." % mode)
            shared.status['abort'] = True
            return False
            
        elif wait_count % 15 == 0 :
            util.log_warning("Retry setting %s" % mode)
            vehicle.mode = VehicleMode(mode)  # resend command

def arm_and_takeoff(vehicle, target_alt, loiter_time):
    """
    Arm the vehicle and takeoff to target altitude.
    
    This is a blocking check, so the program is stuck indefinately unless
    received `LIFT` clearance, stopped by `EXIT` command or killed by 
    KeyboardInterrupt.
    
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        target_alt(float): takeoff altitude, validated in the main script.
        loiter_time(int): seconds to wait after `des_alt` is reached.
        
    Returns:
        bool: True if success, False if killed by `EXIT` command. 
    """
    util.log_info("Checking if armable.")
    
    wait_count = 0
    while not vehicle.is_armable:
        time.sleep(.2)
        wait_count = wait_count + 1
        
        if wait_count % 25 == 0:
            util.log_warning("Vehicle not armable.")
        
        if wait_count >= 100:
            util.log_warning("Unable to arm. Abort.")
            shared.status['abort'] = True
            return False

    util.log_info("Switching to GUIDED and Arming.")
    set_mode(vehicle, "GUIDED")

    util.log_debug("Arming...")
    vehicle.armed  = True
    time.sleep(3)
    
    wait_count = 0
    while True:
        time.sleep(.5)
        wait_count = wait_count + 1
        
        if vehicle.armed :
            util.log_info("Armed.")
            break
            
        elif wait_count % 10 == 0:
            util.log_warning('Retry arming.')
            
        if wait_count >= 20:
            util.log_warning("Arming failed. Abort.")
            shared.status['abort'] = True
            return False

    vehicle.simple_takeoff(target_alt) 

    # Wait until the vehicle reaches a safe altitude (95%), or otherwise the command 
    # after Vehicle.simple_takeoff will execute immediately, causing unexpected results.
    while True:
        util.log_debug("Altitude: %s" % vehicle.location.global_relative_frame.alt)    
        if vehicle.location.global_relative_frame.alt >= target_alt - shared.ALT_TOLERANCE:
            util.log_info("Target altitude reached: %s m." % vehicle.location.global_relative_frame.alt)
            break

        time.sleep(.5)
    
    time.sleep(loiter_time)
    return True

def wait_for_disarm(vehicle, timeout=180):
    """
    Wait for the vehicle to disarm and log every 2 sec.
    
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        timeout(int): seconds before invoking forced exit. Default 30 seconds.
    """
    wait_count = 0
    sleep_period = 1
    log_period = 30
    
    timeout_limit = timeout/sleep_period
    log_count = log_period/sleep_period
    
    while vehicle.armed:
        if wait_count % log_count == 0: # 10sec period logging
            util.log_info("Waiting for the copter to disarm.")
        
        if wait_count >= timeout_limit:
            util.log_warning("Wait timeout! Exit script now!")
            break
            
        time.sleep(sleep_period)
        wait_count = wait_count + 1

def eclipse_compensate(aLocation):
    """
    Return the center radius and latitude ring radius to compensate eclipse effect.
    
    Args:
        aLocation(dronekit.LocationXXYY): a location coordinate.
    
    Returns:
        list: center radius and latitude ring radius
    """
    EQUATOR_RADIUS  = 6378137.0 # equator radius, or "spherical" earth
    POLAR_RADIUS    = 6356725.0 # ploar radius
    EP_DIFF = EQUATOR_RADIUS - POLAR_RADIUS # rad-diff between equator and pole
    
    # assuming linear slope from equator to pole
    r_center = POLAR_RADIUS + EP_DIFF * (1.0 - abs(aLocation.lat)/90) # the ring thru earth center
    r_level  = r_center * math.cos(math.radians(aLocation.lat)) # the ring thru latitude level
    
    return [r_center, r_level]

def get_location_metres(original_location, dNorth, dEast):
    """
    Calculate a coordinate by a vector related to a known coordinate.
    
    Returns a `Location` object containing the latitude/longitude `dNorth` 
    and `dEast` metres from the specified `original_location`. The returned 
    `Location` has the same `alt` value as `original_location`.
    
    Added the compensation of the eclipse effect to the earth, increasing accuracy.
    This is a approximation, and would get sloppy and inaccurate when closing to poles.
    
    Args:
        original_location(dronekit.LocationXXYY): a location coordinate.
        dNorth(float): meters to the north.
        dEast(float): meters to the east.
        
    Returns:
        dronekit.LocationXXYY: calculated coordinate of the same type.
    """   
    [r_center, r_level] = eclipse_compensate(original_location)
    
    # coordinate offsets in radians
    dLat = dNorth / r_center
    dLon = dEast  / r_level
    
    # new position in decimal degrees
    newlat = original_location.lat + math.degrees(dLat)
    newlon = original_location.lon + math.degrees(dLon)
    
    # return according to the input coordinate Class
    if isinstance(original_location,LocationGlobal):
        targetlocation = LocationGlobal(newlat, newlon,original_location.alt)
        
    elif isinstance(original_location,LocationGlobalRelative):
        targetlocation = LocationGlobalRelative(newlat, newlon,original_location.alt)
        
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation
    
def get_position_error(aLocation1, aLocation2):
    """
    Calculate the positional error vector between two coordinates in NED frame.
    
    Added the compensation of the eclipse effect to the earth, increasing accuracy.
    This is a approximation, and would get sloppy and inaccurate when closing to poles.
    
    Args:
        aLocation1(dronekit.LocationXXYY): the subtrahend coordinate.
        aLocation2(dronekit.LocationXXYY): the minuend coordinate.
        
    Returns:
        list: error vector as (aLocation2-aLocation1) in NED frame.
    """
    if type(aLocation1) is not type(aLocation2):
        raise Exception("Input location type mismatch")

    [r_center, r_level] = eclipse_compensate(aLocation1)
    
    dNorth = math.radians(aLocation2.lat - aLocation1.lat) * r_center
    dEast  = math.radians(aLocation2.lon - aLocation1.lon) * r_level
    dDown  = aLocation2.alt - aLocation1.alt
    
    return [dNorth, dEast, dDown]
    
def get_distance_metres(aLocation1, aLocation2):
    """
    Calculate the ground distance between two coordinates of the same type.
    
    Added the compensation of the eclipse effect to the earth, increasing accuracy.
    This is a approximation, and would get sloppy and inaccurate when closing to poles.
    
    Args:
        aLocation1(dronekit.LocationXXYY): the subtrahend coordinate.
        aLocation2(dronekit.LocationXXYY): the minuend coordinate.
        
    Returns:
        float: distance in meters between aLocation1 and aLocation2.
    """
    [dNorth, dEast, dDown] = get_position_error(aLocation1, aLocation2)
    
    return math.sqrt((dNorth*dNorth) + (dEast*dEast))

def get_bearing(aLocation1, aLocation2):
    """
    Calculate the bearing between two coordinates of the same type.
    
    Added the compensation of the eclipse effect to the earth, increasing accuracy.
    This is a approximation, and would get sloppy and inaccurate when closing to poles.
    
    Args:
        aLocation1(dronekit.LocationXXYY): the referenced coordinate.
        aLocation2(dronekit.LocationXXYY): the targeted coordinate.
        
    Returns:
        float: bearing in degrees pointing from aLocation1 to aLocation2.
    """
    [off_y, off_x] = get_position_error(aLocation1, aLocation2)
    
    # bearing is clockwise, offset_y is north, offset_x is east
    bearing = 90.00 + math.degrees(math.atan2(-off_y, off_x))
    if bearing < 0: bearing += 360.00
    
    return bearing
    
def goto(vehicle, dNorth, dEast):
    """
    Move the vehicle to a position [dNorth, dEast] meters of the current position.

    The method takes a function pointer argument with a single `dronekit.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.Vehicle.simple_goto().

    The method checks the distance to target every 500ms. Use `shared.WP_RADIUS`
    instead to determine if the waypoint is reached and trigger a brake.
    
    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        dNorth(float): meters to the north.
        dEast(float): meters to the east.
    """
    goto_function = vehicle.simple_goto # can be changed
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    goto_function(targetLocation)

    #Stop action if we are no longer in guided mode.
    while vehicle.mode.name == "GUIDED": 
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        #print "Distance to target: ", remainingDistance
        if remainingDistance <= shared.WP_RADIUS: #Just below target, in case of undershoot.
            #print "Reached target"
            break;

        time.sleep(0.5)

"""
Functions that move the vehicle by specifying the velocity components in each direction.

The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.

Functions:
    send_ned_velocity: set velocity components using SET_POSITION_TARGET_LOCAL_NED command
    send_global_velocity: set velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""
def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    
    The sending rate is boosted to 10Hz, and uses xrange() generator. [x, y, z]
    corresponds to the NED frame order.
    
    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        velocity_x(float): desired velocity in x direction.
        velocity_y(float): desired velocity in y direction.
        velocity_z(float): desired velocity in z direction.
        duration(float): seconds for the velocity control.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 10 Hz cycle
    for x in xrange(0, int(10*duration)):
        vehicle.send_mavlink(msg)
        time.sleep(.10)

def send_global_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    
    The sending rate is boosted to 10Hz, and uses xrange() generator. [x, y, z]
    corresponds to the NED frame order.
    
    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        velocity_x(float): desired velocity in x direction.
        velocity_y(float): desired velocity in y direction.
        velocity_z(float): desired velocity in z direction.
        duration(float): seconds for the velocity control.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 10 Hz cycle
    for x in xrange(0, int(10*duration)):
        vehicle.send_mavlink(msg)
        time.sleep(.10)
    
"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

Functions:
    goto_position_target_global_int: 
        set position using SET_POSITION_TARGET_GLOBAL_INT command in 
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
    goto_position_target_local_ned:
        set position using SET_POSITION_TARGET_LOCAL_NED command in 
        MAV_FRAME_BODY_NED frame
    goto - A convenience function that can use Vehicle.simple_goto (default) or 
    goto_position_target_global_int to travel to a specific position in meters 
    North and East from the current location. 
    This method reports distance to the destination.
"""

def goto_position_target_global_int(vehicle, aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        aLocation(dronekit.LocationGlobal): the target location in global frame
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)

def goto_position_target_local_ned(vehicle, north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    
    Args:
        vehicle(dronekit.Vehicle): the vehicle to be controlled.
        north(float): meters to the north in the MAV_FRAME_BODY_NED frame.
        east(float): meters to the east in the MAV_FRAME_BODY_NED frame.
        down(float):meters to the ground in the MAV_FRAME_BODY_NED frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    