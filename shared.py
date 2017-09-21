"""
Attributes shared across the files.

This module contains many attributes that are globally shared among different 
files. They are either constants or pre-defined objects, depending on the case.
Some of the attributes are poped or instantiated at program start, and are 
considered constant throughout the runtime. See the source code comments for 
detailed descriptions.

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

from dronekit import LocationGlobal
from dronekit import LocationGlobalRelative

timestamp = ' ' # Local time in <struct_time>, converted from UNIX epoch

AGENT_COUNT = 5 # total agent count
AGENT_ID = 'FFF' # FFF is the reserved initial value
AGENT_CHARACTER = '' # if this agent is leader or follower
CURRENT_ALGORITHM = None # str: current algorithm type

des_alt = 0.0 # Desired altitude
# For onboard.py, this is the takeoff altitude.
# For gcs.py, this is the groundstation's 'altitude above ground' that
# would be passed thru the broadcast comm-link.

neighbors = {} # Neighborhood dictionary

status = { # control status dictionary
    'manual_mode'   : ''    , # monitors mode change by the python script
    'command'       : ''    , # logs the external command
    'abort'         : False , # emergency abort flag, external RC control
    'exiting'       : False , # exiting flag, normal control
    'thread_flag'   : 0x0000, # flag register to kill threads
    'airborne'      : False , # airborne or not
    'new_param'     : False   # if there is new parameters arrived
}

# thread_flag mask:
BROADCAST_FLAG  = (1 << 0)
PURGE_FLAG      = (1 << 1)
FLOCKING_FLAG   = (1 << 2)
NSATS_TOO_LOW   = (1 << 3)
SUBSCRIBER_FLAG = (1 << 4)

# payload length for the Xbee datapack
# the info_list components: (agentID is also the Key)
# ----------------------------------
# item:         Python-type:   C-type   length(B)    offset:
# ID            string         char[]      3           0
# lat           float          double      8           1
# lon           float          double      8           2
# msl_alt       float          float       4           3
# rel_alt       float          float       4           4
# vx            float          float       4           5
# vy            float          float       4           6
# vz            float          float       4           7
# -----------------------------------
# Example: 
#  Key :    ID   latitude     longitude   msl_alt rel_alt   vx      vy     vz
# 'A12': ['A12', '31.3011976','121.4981926','21.6','12.59','-0.57','-0.04','0.17']
PAYLOAD_LENGTH = 39

rendezvous  = None   # class `comm.WrappedData`, target point for convergence.
home_origin = None   # class `dronekit.LocationGlobalRelative`, reference origin.

# physical parameters in meters
WP_RADIUS = 1.0
ALT_TOLERANCE = 0.5

# flocking parameters, default is none
param_mpc = None
