#! /usr/bin/python
"""
The ground control station script for the quadcopter flocking control experiment.

The script runs on a commercial laptop with Linux OS. A high-level XBee module 
should be connected for the inter communication between the drones and the 
ground control station.

The XBee module runs in API2, escaped character mode. By the time written, an 
XBee Pro S1 module is used (with the DIJI Mesh firmware). See the official site
of DIJI and the datasheets for more details.

A GNSS module is needed for the GCS to grab its location coordinates. Python
`gps` package is used. `gpsd` package is used for a backend daemon of the GNSS 
module data. For some USB-ttl modules that are identified as an ttyACMx device,
there may have some issue about the data stream. See the reference for details. 
Preferably, use a virtual USB com (like FT232 or CP2104 adapter) to connect the
GNSS module.

Be very carefull about the connection order of the XBee and the GNSS module when
both of them are identified as ttyUSBx devices.

The dronekit API package supports Python 2.7 for now. Preferably, Ubuntu is
the better choice as it uses `apt` to get distributed packages.

See reference [1] for more details about the algorithm.

Classes:
    GNSS(threading.Thread): stream the GNSS data.

Reference:
    DIJI Xbee: https://docs.digi.com/display/WirelessConnectivityKit/XBee+API+mode
    python-xbee: https://github.com/nioinnovation/python-xbee
    DKPY-API Reference: http://python.dronekit.io/automodule.html
    gpsd: http://catb.org/gpsd/
    gpsd Example: http://www.danmandle.com/blog/getting-gpsd-to-work-with-python/
    
    ttyACMx problem:
        https://www.raspberrypi.org/forums/viewtopic.php?t=8886&p=104848
        http://stackoverflow.com/questions/29333424/gpsd-not-getting-a-good-fix
        http://raspberrypi.stackexchange.com/questions/29547/cant-get-gps-to-automatically-work-after-reboot
        
Environment:
    Computer and OS: A laptop with Ubuntu 16.04LTS.
    Wireless module: XBee Pro S1 with DIJI Mesh firmware.
    GNSS module: any module that compatible with NMEA protocol via UART or USB.
    Python packages: dronekit, xbee, gps

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

import os
import sys
import time
import math
import serial
import logging
import argparse
import threading

import fcntl
import termios

import gps
from struct import pack
from struct import unpack
from dronekit import LocationGlobal
from dronekit import LocationGlobalRelative

from src import mas
from src import comm
from src import util
from src import shared


class GNSS(threading.Thread):
    """
    Stream the GNSS data using the gps daemon (gpsd).
    
    Args:
        gpsd(gps.gps): the gps object monitoring gps daemon data.
    """
    def __init__(self, gpsd):
        threading.Thread.__init__(self)
        self._gpsd     = gpsd
        self._stopflag = False
        
    def run(self):
        """
        Run the thread and streaming GNSS data.
        """
        self._gpsd.stream(flags=gps.WATCH_ENABLE)

        while not self._stopflag:
            time.sleep(.1)    # 10Hz polling
            val = self._gpsd.next()
            # if there is no GNSS module and this method is called, the thread
            # may stuck here.
            # TODO(Q.Yuan): try to fix or walkaround this bug.
            
        self._gpsd.stream(flags=gps.WATCH_DISABLE)

    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True


def _set_home_origin(gpsd):
    """
    Set the high-level HOME_ORIGIN, manually invoked by a key input.
    
    Args:
        gpsd(gps.gps): the gps object monitoring gps daemon data.
    
    Returns:
        bool: True if success, False if failed.
    """
    if (gpsd.valid & gps.LATLON_SET) and (gpsd.satellites_used >= MIN_SAT_NUM):  
        util.log_info("Setting HOME_ORIGIN.")

        shared.home_origin = LocationGlobalRelative( 
            gpsd.fix.latitude, 
            gpsd.fix.longitude,
            gpsd.fix.altitude
            )
        
        util.log_info("HOME_ORIGIN: %s" % shared.home_origin)
        return True
        
    elif (gpsd.valid & gps.LATLON_SET):
        util.log_info("<ORG> Satellites low at %d" % gpsd.satellites_used)
        
    else:
        util.log_info("<ORG> No valid coordinates!")
    
    return False

def _broadcast_rendezvous(gpsd, xbee):
    """
    Broadcast the rendezvous coordinates to the drones.
    
    Args:
        gpsd(gps.gps): the gps object monitoring gps daemon data.
        xbee(xbee.Zigbee): the XBee communication interface.
        
    Returns:
        bool: True if success, False if failed.
    """
    if (gpsd.valid & gps.LATLON_SET) and (gpsd.satellites_used >= MIN_SAT_NUM):         
        # 'TAR' delimiter is only for flocking algorithms. 
        # For other applications, the frames may vary.
        package = pack( '=3s2d5f',
                        'TAR',  # 'TAR' = 'target'
                        gpsd.fix.latitude,     # latitude
                        gpsd.fix.longitude,    # longitude
                        gpsd.fix.altitude,     # msl_altitude
                        shared.des_alt, # relative altitude, default 15.0 m
                        0, 0, 0 )   # only report coordinates, no velocity
        # constant 39bytes transmision, similar to normal agents.

        util.log_info("Sending rendezvous coordinates.")
        comm.xbee_broadcast(xbee, package)
        return True
        
    elif (gpsd.valid & gps.LATLON_SET):
        util.log_info("<TAR> Satellites low at %d" % gpsd.satellites_used)
        
    else:
        util.log_info("<TAR> No valid coordinates!")
    
    return False

def _broadcast_origin(xbee):
    """
    Broadcast HOME_ORIGIN to the drones.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        
    Returns:
        bool: True if success, False if failed.
    """
    if not shared.home_origin:
        util.log_warning("HOME_ORIGIN invalid!")
        return False

    package = pack( '=3s2d5f',
                    'ORG',  # 'TAR' = 'target'
                    shared.home_origin.lat,       # latitude
                    shared.home_origin.lon,       # longitude
                    shared.home_origin.alt,       # msl_altitude
                    shared.des_alt, # relative altitude, default 15.0 m
                    0, 0, 0 )       # only report coordinates, no velocity
                    
    util.log_info("Broadcasting HOME_ORIGIN %s" % shared.home_origin)
    comm.xbee_broadcast(xbee, package)
    return True
    
def _update_parameter(xbee, path):
    """
    Update algorithm parameters to the drones.
    
    Parameters are stored in a text file with a delimiter, `=` and `,` separation.
    Broadcast is not reception-guaranteed, so the drones shall echo an ACK.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        path(str): path to a text file storing the parameters.
    """
    fp = open(path, 'r')
    
    param = fp.read().split()
    for i in xrange(0, len(param)):
        param[i] = dict(item.split('=') for item in param[i].split(','))
    param = dict(zip(list(param[i]['ID'] for i in range(0,len(param))), param))
    
    fp.close()

    if shared.CURRENT_ALGORITHM == 'MPC':
        util.log_info("Updating MPC param.")
        package = pack (
            '=3s3f2i',
            param['MPC']['ID'],
            float(param['MPC']['Ts']),
            float(param['MPC']['Vmax']),
            float(param['MPC']['D0']),
            int(param['MPC']['Hp']),
            int(param['MPC']['Hu'])
        )
        
        shared.param_mpc = mas.ParamMPC(unpack('=3s3f2i', package))
        util.log_info("New MPC param sent: %s" % shared.param_mpc)
        comm.xbee_broadcast(xbee, 'PRM,%s' % package) # identifier: `PRM`
        
    elif shared.CURRENT_ALGORITHM == 'Vicsek': pass # reserved
    else: util.log_warning("Err: shared.CURRENT_ALGORITHM is None!")

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

    parser.add_argument('-algorithm', '-a', type=str, default='MPC', metavar='',
                        choices=['Vicsek','MPC'], required=True,
                        help="Algorithm used for main script.")
    parser.add_argument('-xbee', type=str, default='/dev/ttyUSB1', metavar='',
                        help="Xbee module's device path.")
    parser.add_argument('-gnss', '-g', type=str, default='/dev/ttyUSB0', metavar='',
                        help="GNSS module's device path.")
    parser.add_argument('-param', '-p', type=str, default='doc/parameters.txt', metavar='',
                        help="Text file path of the parameters.")
    parser.add_argument('-level', '-l', type=str, default='info', metavar='',
                        choices=['warning','debug','info'],
                        help="Logging level: ['warning','debug','info']")

    args = parser.parse_args()
    
    return args

def main():
    """
    The Main function of this script.
    """
    args = _parse_arguments()
   
    util.log_init("gcs_%s.txt" % util.get_latest_log("latest_gcs.txt"), util.log_level[args.level])

    shared.AGENT_ID = 'GCS'
    shared.CURRENT_ALGORITHM = args.algorithm
    util.log_info("AGENT_ID = %s" % shared.AGENT_ID)
    util.log_info("Algorithm: %s" % shared.CURRENT_ALGORITHM)
    util.log_info("Agent type: Ground station.")
    
    fparam = args.param

    ser = serial.Serial(args.xbee, 230400)
    xbee = comm.xbee_init(ser)
    util.log_info("Xbee initialized.")

    MIN_SAT_NUM = 7 # minimum satellites number required
    gpsd = gps.gps() # in some cases, the gpsd need to be flushed and reset
    gps_thread = GNSS(gpsd)
    gps_thread.start()
    util.log_info('GNSS running.')

    # This dictionary should keep the <offset> consistant with <cmd_list>.
    # It is useful for multi-to-one mapping. All the keys are lower-cased, 
    # the first key is the native one, others are considered alias, and are 
    # only for conveinience usage.
    key_dict = {  
      # <key>  <offset>
        'p':    -4,     # set parameters
        's':    -3,     # set HOME ORIGIN
        'o':    -2,     # send origin
        't':    -1,     # send rendezvous
        'r':     0,     # RTL command
        'l':     1,     # land command
        'b':     2,     # lift command
        'e':     3,     # exit command
        'c':     4,     # clear rendezvous     
    }

    cmd_list = [
     # <command string>  <key>  <description>
       # -----------------Positive   Index-----------------
       ['CTR,RTL ',   'r', 'Exit algorithms at run and RTL'],       # 0
       ['CTR,LAND',   'l', 'Exit algorithms at run and LAND'],      # 1
       ['CTR,LIFT',   'b', 'Initiate takeoff.'],                    # 2
       ['CTR,EXIT',   'e', 'Exit takeoff when on the ground.'],     # 3
       ['CLR_RDV ',   'c', 'Clear rendezvous coordinates'],         # 4
       # -----------------Negative   Index-----------------
       ['"None"  ',   'p', 'Update parameters'],                    # -4
       ['"None"  ',   's', 'Set HOME ORIGIN coordinates'],          # -3
       ['ORG,<pack>', 'o', 'Send HOME ORIGIN coordinates'],         # -2
       ['TAR,<pack>', 't', 'Send rendezvous coordinates'],          # -1
    ]

    # get keyboard file descrpter from stdin and save current terminal attribute
    # then turn into cbreak style, without terminal echo
    # See: https://docs.python.org/2/faq/library.html?highlight=read
    keyboard = sys.stdin

    old_attr = termios.tcgetattr(keyboard)
    new_attr = termios.tcgetattr(keyboard)
    new_attr[3] = new_attr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(keyboard, termios.TCSANOW, new_attr)

    old_flags = fcntl.fcntl(keyboard, fcntl.F_GETFL)
    fcntl.fcntl(keyboard, fcntl.F_SETFL, old_flags)# | os.O_NONBLOCK)

    key = None
    while True: # main loop
        try:
            time.sleep(.05)

            key = keyboard.read(1) # read only one key
            
            if key in key_dict: # valid key
                if key_dict[key] >= 0: # positive value: send command
                    util.log_info("Sending command: '%s'" % cmd_list[key_dict[key]][0].strip(' '))
                    comm.xbee_broadcast(xbee, cmd_list[key_dict[key]][0].strip(' '))
                    
                else: # negative value: send coordinates or parameters
                    # TODO(Q.Yuan): try to get automated broadcasting
                    if key_dict[key] == -1:
                        _broadcast_rendezvous(gpsd, xbee)
                    elif key_dict[key] == -2:
                        _broadcast_origin(xbee)
                    elif key_dict[key] == -3:
                        _set_home_origin(gpsd)
                    elif key_dict[key] == -4:
                        _update_parameter(xbee, fparam)
            
            else: # not a valid key, print help
                print "\n---- Command List ----\nCommand:\tKey:\tDescription"
                for idx in range(0, len(cmd_list)):
                    print '%s\t%s\t%s' % (cmd_list[idx][0], cmd_list[idx][1], cmd_list[idx][2])
                    
        except IOError: pass
        except KeyboardInterrupt: break

    # clean up
    gps_thread.stop()
    while gps_thread.is_alive(): 
        util.log_info('Waiting for GNSS to shutdown')  
        gps_thread.join(3)
    util.log_info('GNSS shutdown.')

    xbee.halt()
    ser.close()
    util.log_info("Xbee and serial closed.")

    # restore previous terminal attribute
    termios.tcsetattr(keyboard, termios.TCSAFLUSH, old_attr)
    fcntl.fcntl(keyboard, fcntl.F_SETFL, old_flags)

if __name__ == '__main__':
    main()