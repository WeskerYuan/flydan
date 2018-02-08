#! /usr/bin/python
"""
Software-in-the-loop simulation script for the multi quadcopter flocking control.

This is the main script for the multi quadcopter flocking control (SITL). 
The script runs under the dronekit-sitl environment.
A high-level XBee module should be connected for the inter communication between
the drones and the ground control station if specified the hardware ports.
Otherwise, a ZeroMQ publisher-subscriber network is set to simulate the 
communication.

The XBee module runs in API2, escaped character mode. By the time written, an 
XBee Pro S1 module is used (with the DIJI Mesh firmware). See the official site
of DIJI and the datasheets for more details. Simulated XBee modules uses the 
same interface as the real ones.

The dronekit API package supports Python 2.7 for now. Preferably, Ubuntu is
the better choice of onboard Linux OS as it is uses `apt` to get distributed 
packages, which is easy to setup and very convenient.

See reference [1] for more details about the algorithm.

Reference:
    DIJI Xbee: https://docs.digi.com/display/WirelessConnectivityKit/XBee+API+mode
    python-xbee: https://github.com/nioinnovation/python-xbee
    DKPY-API Reference: http://python.dronekit.io/automodule.html
    Dronekit-SITL: http://python.dronekit.io/develop/sitl_setup.html?highlight=sitl
    
    [1] Q. Yuan, J. Zhan and X. Li, Outdoor flocking of quadcopter drones with
        decentralized model predictive control, ISA Transactions, 2017.

Environment:
    Computer and OS: Raspberry Model 3B with Ubuntu MATE 16.04LTS.
    Wireless module: XBee Pro S1 with DIJI Mesh firmware.
    Python packages: dronekit, dronekit-sitl, xbee, numpy
    
Attibutes:
    start_loc(dict): starting location coordinates related to agent_id.
    comm_port_list(dict): SITL TCP ports related to agent_id.

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

import re
import sys
import time
import math
import serial
import logging
import argparse
import threading

from dronekit_sitl import SITL

from src import nav
from src import mas
from src import comm
from src import util
from src import shared

def _add_listeners(vehicle):
    """
    Add listeners to monitor vehicle status.
    
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
    """
    @vehicle.on_attribute('mode')
    def mode_listener(self,name, msg):
        util.log_info("Mode switched to %s" % msg.name)
        
        if msg.name != shared.status['manual_mode']:    # manual override
            if msg.name == 'RTL' or msg.name == 'LAND':
                util.log_warning("External %s detected. Abort." % msg.name)
                shared.status['abort'] = True
    
    @vehicle.on_attribute('gps_0')
    def gps_listener(self,name, msg): # monitor satellites
        if not shared.status['thread_flag'] & shared.NSATS_TOO_LOW:
            if msg.satellites_visible < 6:
                util.log_warning("Satellites dropped below 5!")
                shared.status['thread_flag'] |= shared.NSATS_TOO_LOW
                
        elif msg.satellites_visible >= 10:
            util.log_info("Satellites recovered to %d." % msg.satellites_visible)
            shared.status['thread_flag'] &= ~shared.NSATS_TOO_LOW
    
    @vehicle.on_message('SYSTEM_TIME')
    def time_listener(self,name, msg): # log timestamp
        format = '%Y-%m-%d %H:%M:%S'
        val = time.localtime(msg.time_unix_usec/1000000)
        shared.timestamp = time.strftime(format, val)

def _parse_arguments():
    """
    Parse the arguments to the main script and validate the inputs.
    
    Returns:
        argparse.ArgumentParser: the argument structure.
    """
    parser = argparse.ArgumentParser(
        fromfile_prefix_chars='@',
        formatter_class = argparse.ArgumentDefaultsHelpFormatter,
        description = 'Arguments for the SITL simulation.'
        )

    parser.add_argument('-id', type=str, default='FF', metavar='AgentID', required=True,
                        help="AGENT_ID, must be a 2-digit integer.")
    parser.add_argument('-alt', type=float, default=15.0, metavar='',
                        help='Takeoff altitude, within [10.0, 100.0] (m).')
    parser.add_argument('-xbee', type=str, default=None, metavar='',
                        help="XBee module's device path. If not provided, use ZeroMQ.")
    parser.add_argument('-pix', type=str, default='fw/ac3.5.2_port5760', metavar='',
                        help="Pixhawk's device path. Can be SITL firmware.")
    parser.add_argument('-algorithm', '-a', type=str, default='MPC', metavar='',
                        choices=['Vicsek','MPC'],
                        help="Algorithm used for main script.")
    parser.add_argument('-character', '-c', type=str, default='follower', metavar='',
                        choices=['square','passive','follower'],
                        help="Whether this agent is leader or follower?")
    parser.add_argument('-n', type=int, default=5, metavar='',
                        help="Total agent count.")   
    parser.add_argument('-level', '-l', type=str, default='info', metavar='',
                        choices=['warning','debug','info'],
                        help="Logging level: ['warning','debug','info']") 
                        
    args = parser.parse_args()

    # get correct parameters
    if args.alt < 10.0 or args.alt > 100.0:
        raise Exception('-alt should between [10.0, 100.0]')
    if not args.id.isdigit() or len(args.id) != 2:
        raise Exception('-id shoud be a 2-digit integer')
    
    return args

def _choose_algorithm(vehicle, xbee, neighbors):
    """
    Choose which algorithm thread to be instantiated.
    
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
        neighbors(dict): the dictionary containing neighbors data.
        
    Returns:
        mas.Object: different thread instance based on the parameters.
    """
    if shared.AGENT_CHARACTER == 'square':
        return mas.SquareRoute(vehicle, xbee)
        
    elif shared.AGENT_CHARACTER == 'passive':
        return mas.PassiveLeader(vehicle, xbee)
        
    elif shared.CURRENT_ALGORITHM == 'Vicsek':
        return mas.Vicsek(vehicle, xbee, neighbors)
        
    elif shared.CURRENT_ALGORITHM == 'MPC':
        return mas.Decentralized(vehicle, xbee, neighbors)

# starting location GNSS coordinates. Modify accordingly.
# Format: latitude,longitude,MSL altitude, heading
# 'FFF' is reserved for not IDed ones. Not available when using pyzmq.
start_loc = {
    'A01': '31.2991103,121.4953190,9,340',
    'A02': '31.2989222,121.4954363,9,340',
    'A03': '31.2988302,121.4953633,9,340',
    'A04': '31.2988857,121.4954170,9,340',
    'A05': '31.2989833,121.4955480,9,340',
    'FFF': '31.3012010,121.4981920,9,340'
}

# port list for SITL communications
comm_port_list = {
    'A01': 5789,
    'A02': 6789,
    'A03': 7789,
    'A04': 8789,
    'A05': 9789,
    'GCS': 1789
}

def main():
    """
    The Main function of this script.
    """
    args = _parse_arguments()

    util.log_init("sitl_A%s_%s.txt" % (args.id, util.get_latest_log("latest_sitl.txt")), util.log_level[args.level])

    shared.AGENT_ID = 'A%s' % args.id
    shared.AGENT_COUNT = args.n
    shared.CURRENT_ALGORITHM = args.algorithm
    shared.AGENT_CHARACTER = args.character
    shared.des_alt = args.alt
    
    util.log_info("AGENT_ID = %s" % shared.AGENT_ID)
    util.log_info("Algorithm: %s" % shared.CURRENT_ALGORITHM)
    util.log_info("Agent type: %s" % shared.AGENT_CHARACTER)

    print "Start simulator (SITL)"
    sitl = SITL(args.pix)    # initialize SITL with firmware path

    if shared.AGENT_ID in start_loc:
        sitl_args = ['--home=%s' % start_loc[shared.AGENT_ID]]
    else:
        sitl_args = ['--home=%s' % start_loc['FFF']]
        
    # Pre-recorded coordinates.
    #sitl_args = ['-I0', '--model', 'quad', '--home=31.301201,121.498192,9,353']	
    sitl.launch(sitl_args, await_ready=True, restart=True)

    # Connect to the vehicle. (Spawn an instance of Vehicle named "vehicle")
    # connection port is coded in the file name of the firmware like "ac3.4.5_port5760"
    # use regular expression to search the string and extract port number
    port = re.search(r'port\d{4}', args.pix)
    port = re.search(r'\d{4}', port.group()).group()

    print "Connecting to copter on: TCP: 127.0.0.1:%s" % port
    copter = nav.connect('tcp:127.0.0.1:%s' % port, wait_ready=True, rate=20)
    util.log_info("Copter connected. Firmware: %s" % copter.version)
    
    if not args.xbee: # simulate XBee using ZeroMQ
        [pub, sub] = comm.zmq_init(comm_port_list[shared.AGENT_ID], comm_port_list)
        subscriber_thread = comm.Subscriber(shared.AGENT_ID, sub)
        subscriber_thread.start()
        xbee = pub # make xbee the publisher
        util.log_info("ZeroMQ initialzied.") 
       
    else: # use actual xbee ports
        ser = serial.Serial(args.xbee, 57600)
        xbee = comm.xbee_init(ser)
        util.log_info("Xbee initialzed.")

    info = "IFO,%s connected with firmware %s" % (shared.AGENT_ID, copter.version)
    comm.xbee_broadcast(xbee, info)

    _add_listeners(copter)

    takeoff_thread = nav.Takeoff(copter, xbee, shared.des_alt, 3)
    purge_thread = comm.Purge(shared.neighbors)
    broadcast_thread = comm.Broadcast(shared.AGENT_ID, copter, xbee)
    flocking_thread = _choose_algorithm(copter, xbee, shared.neighbors)

    takeoff_thread.start()
    takeoff_thread.join() # wait until takeoff procedure completed

    if shared.status['airborne']: # only execute the threads when airborne
        util.log_info("Copter is airborne, starting threads.")
        broadcast_thread.start()
        purge_thread.start()
        flocking_thread.start()

    # main loop
    while True:
        try: time.sleep(.2)
        except KeyboardInterrupt: break
        
        if shared.status['airborne']:
            # echo exiting status
            if shared.status['exiting']:
                info = "IFO,%s %s-ing." % (shared.AGENT_ID,shared.status['command'])
                comm.xbee_broadcast(xbee, info)
                util.log_info(info)

            # if an rtl or land command is received, kill flocking and set the `exiting` flag
            elif shared.status['command'] == 'RTL' or shared.status['command'] == 'LAND':
                shared.status['thread_flag'] |= shared.FLOCKING_FLAG
                nav.set_mode(copter, shared.status['command'])
                shared.status['exiting'] = True

        if not flocking_thread.is_alive(): # break the loop if finished
            break

    nav.wait_for_disarm(copter) # wait for disarm
    comm.xbee_broadcast(xbee, 'IFO,%s terminated.' % shared.AGENT_ID)

    # clean up
    purge_thread.stop()
    while purge_thread.is_alive(): 
        util.log_info('Waiting for purge to shutdown')  
        purge_thread.join(3)
    util.log_info('Purge killed.')

    broadcast_thread.stop()
    while broadcast_thread.is_alive(): 
        util.log_info('Waiting for broadcast to shutdown')  
        broadcast_thread.join(3)
    util.log_info('Broadcast killed.')

    copter.close()
    util.log_info("Copter shutdown.")

    if args.xbee:
        xbee.halt()
        ser.close()
        util.log_info("Xbee and serial closed.")
    else:
        subscriber_thread.stop()
        while subscriber_thread.is_alive(): 
            util.log_info('Waiting for Subscriber to shutdown')  
            subscriber_thread.join(3)
        util.log_info('Subscriber killed.')

    sitl.stop()
    util.log_info("SITL shutdown.")

if __name__ == '__main__':
    main()