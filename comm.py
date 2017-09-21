"""
Communication classes and functions.

This module contains the classes and functions for the communication between the
vehicles, or between the vehicles and the GCS. Generally, XBee modules are used
for the vehicles to talk to each other. For software-in-the-loop simulation,
`pyzmq` is used to establish a publish-subscrib model.

Reference:
    DIJI Xbee: https://docs.digi.com/display/XBeeZigBeeMeshKit/Frame+structure
    python-xbee: https://github.com/nioinnovation/python-xbee

Classes:
    Broadcast(threading.Thread): broadcast datapacks across the PAN.
    Purge(threading.Thread): clean the neighborhood periodically.
    Subscriber(threading.Thread): subscriber thread by `pyzmq` for SITL.  
    WrappedData(object): a data structure that packs the information into binary.
    
Functions:
    zmq_init(pub_port, sub_port_list): initialize ZeroMQ pub-sub.
    dispatch_init(ser): initialize a dispatcher.
    xbee_init(ser): initialize an XBee interface.
    xbee_broadcast(xbee, data): broadcast some data via the XBee interface.
    
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

import sys
import time
import math
import serial
import threading
import zmq

from struct import pack
from struct import unpack
from xbee import XBee
from xbee import ZigBee
from xbee.helpers.dispatch import Dispatch
from dronekit import LocationGlobal
from dronekit import LocationGlobalRelative
from zmq.error import Again

import mas
import nav
import util
import shared

class Broadcast(threading.Thread):
    """
    Broadcast `my` data to `my neighbors` periodically via the XBee interface.
    
    Because it requires a high-level command link with the GCS (`gcs.py`), 
    an `xbee.XBee` or `xbee.Zigbee` object (either a real one or simulated) 
    is need.
    
    The copter is a `dronekit.Vehicle` object. The status of the copter is needed.
    
    Args:
        agent_id(str): `my` agent id
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
    """
    
    def __init__(self, agent_id, vehicle, xbee):
        threading.Thread.__init__(self)
        self._vehicle  = vehicle
        self._xbee     = xbee
        self._agent_id = agent_id
        self._stopflag = False
        
    def run(self):
        """
        Broadcast the data at 16Hz.
        
        Pack all the data into a binary pack using `struct.pack`. The payload will
        have a constant 39 byte length. Coordinates should be Ctype-double to 
        guarantee precision. Altitude and velocity should work okay with float.
        
        The data list: 
            (ID is also the Key in <neighbors>)
            ----------------------------------
            item:         Python-type:   C-type   length(B)    offset:
            agent_id      string         char[]      3           0
            lat           float          double      8           1
            lon           float          double      8           2
            msl_alt       float          float       4           3
            rel_alt       float          float       4           4
            vx            float          float       4           5
            vy            float          float       4           6
            vz            float          float       4           7
            -----------------------------------
        """
        util.log_info("Broadcast Started.")
        while not self._stopflag:
            time.sleep(.0625)    # 16Hz
            if self._vehicle.armed: # Only broadcast while vehicle is armed
                package = pack( '=3s2d5f',
                                self._agent_id, 
                                self._vehicle.location.global_frame.lat, 
                                self._vehicle.location.global_frame.lon, 
                                self._vehicle.location.global_frame.alt, 
                                self._vehicle.location.global_relative_frame.alt,
                                self._vehicle.velocity[0], 
                                self._vehicle.velocity[1], 
                                self._vehicle.velocity[2] )
                
                xbee_broadcast(self._xbee, package)
                
            if shared.status['thread_flag'] & shared.BROADCAST_FLAG:
                self._stopflag = True
                util.log_info("Broadcast killed by flag.")

    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True
            

class Purge(threading.Thread):
    """
    Clear a dictionary periodically.
    
    Args:
        table(dict): the dictionary to be purged.
    """
    
    def __init__(self, table):
        threading.Thread.__init__(self)
        self.table = table
        self._stopflag = False
        
    def run(self):
        """
        Run the thread and purge periodically.
        """
        util.log_info("Purge Started.")
        count = 0
        
        while not self._stopflag:
            time.sleep(.2)
            count = count + 1
            
            count = count % 15  # Clear neighbors table each 3s
            if count == 0:
                self.table.clear()
            
            if shared.status['thread_flag'] & shared.PURGE_FLAG:
                util.log_info("Purge killed by flag.")
                self._stopflag = True
    
    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True


class WrappedData(object):
    """
    Wrap a list of data components into an data pack object.    
    """
    
    def __init__(self, info_list):
        """
        Input a list of data components.
        
        The info_list components: 
            (ID is also the Key in <neighbors>)
            ----------------------------------
            item:         Python-type:   C-type   length(B)    offset:
            ID            string         char[]      3           0
            lat           float          double      8           1
            lon           float          double      8           2
            msl_alt       float          float       4           3
            rel_alt       float          float       4           4
            vx            float          float       4           5
            vy            float          float       4           6
            vz            float          float       4           7
            -----------------------------------
            
        Example: 
            Key :    ID   latitude     longitude   msl_alt rel_alt   vx      vy     vz
            'A12': ['A12', '31.3011976','121.4981926','21.6','12.59','-0.57','-0.04','0.17']
        """
        self.__ID      = info_list[0]
        self.__lat     = float(info_list[1]) # These two are double
        self.__lon     = float(info_list[2])
        self.__msl_alt = round(float(info_list[3]),8) # round-up floats
        self.__rel_alt = round(float(info_list[4]),8)
        self.__vx      = round(float(info_list[5]),8)
        self.__vy      = round(float(info_list[6]),8)
        self.__vz      = round(float(info_list[7]),8)
    
    @property
    def ID(self):
        """
        str: Pack ID.
        """
        return self.__ID
        
    @property
    def velocity(self):
        """
        list: Velocity vector in NED.
        """
        return [self.__vx, self.__vy, self.__vz]
    
    @property
    def location_global(self):
        """
        dronekit.LocationGlobal: Global NED coordinate. Altitude is MSL altitude.
        """
        return LocationGlobal(self.__lat, self.__lon, self.__msl_alt)
    
    @property
    def location_global_relative(self):
        """
        dronekit.LocationGlobalRelative: Global NED coordinate. Altitude is home-relative.
        """
        return LocationGlobalRelative(self.__lat, self.__lon, self.__rel_alt)
    
    def __str__(self):
        return "WrappedData: %s,%f,%f,%.2f,%.2f,%.2f,%.2f,%.2f" % (
                    self.__ID, 
                    self.__lat, 
                    self.__lon, 
                    self.__msl_alt, 
                    self.__rel_alt,
                    self.__vx, 
                    self.__vy, 
                    self.__vz
                )


class Subscriber(threading.Thread):
    """
    A ZeroMQ subscriber thread that monitors the publisher periodically.
    
    The parameters are for the Decentralized Model Predictive Control algorithm
    only.
    
    Args:
        agent_id(str): `my` agent id
        sub(zmq.Context.socket): a subscriber listening the publisher.
    """
    
    def __init__(self, agent_id, sub):
        threading.Thread.__init__(self)
        self.sub      = sub
        self.agent_id = agent_id
        self._stopflag = False
        
    def run(self):
        """
        Run the subscriber and listen to the publisher every 5ms.
        """
        util.log_info("Subscriber started for %s." % self.agent_id)
        while not self._stopflag:
            time.sleep(.005)
            try:
                [tag, payload] = self.sub.recv_multipart(flags=1)
                if tag == 'XBEE':
                    # package to be consistent with <rx_data_handler>
                    packet = {'rf_data': payload}
                    rx_data_handler('sub_rx', packet)
            except Again:   # zmq.error Exception
                pass
                
            if shared.status['thread_flag'] & shared.SUBSCRIBER_FLAG:
                self._stopflag = True
                util.log_info("Subscriber killed by flag.")

    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True
        

def zmq_init(pub_port, sub_port_list):
    """
    Initialize the ZeroMQ publisher and subscriber.
    
    `My` publisher publishes `my` data to the neighbors. `My` subscriber listen
    to the ports of other neighbors. `sub_port_list` stores all the possible 
    neighbors' TCP ports.
    
    The data packs are wrapped as an XBee interface, compatable with the XBee
    transmission and reception functions in this module.
    
    Args:
        pub_port(str/int): TCP port for the publisher.
        sub_port_list(list): TCP port list for the subscriber to listen to.
    
    Returns:
        list: `my` publisher and `my` subscriber (i.e. listener).
    """
    pub = zmq.Context().socket(zmq.PUB)
    pub.bind('tcp://*:%s' % pub_port)
    
    sub = zmq.Context().socket(zmq.SUB)
    for port in sub_port_list:
        if sub_port_list[port] != pub_port:
            sub.connect('tcp://127.0.0.1:%s' % sub_port_list[port])
        time.sleep(0.05)
    
    sub.setsockopt(zmq.SUBSCRIBE, 'XBEE')
    
    return [pub, sub]

def rx_data_handler(name, packet):
    """
    Callback function for incoming XBee transmission.
    
    Args:
        name(str): name to identify the callback function in a dispatcher.
        packet(dict): package received from the XBee API.
    """
    # Split datapack header and payload -- Small misc packs.
    if packet['rf_data'][0:3] == 'CTR': # control command pack
        recv_pack = packet['rf_data'].split(',')
        shared.status['command'] = recv_pack[1]
        util.log_info("CMD: %s" % recv_pack[1])        
    
    elif packet['rf_data'][0:3] == 'IFO': # information string pack
        recv_pack = packet['rf_data'].split(',')
        util.log_info("IFO: %s" % recv_pack[1])
    
    elif packet['rf_data'][0:3] == 'PRM': # parameter pack
        recv_pack = packet['rf_data'].split(',')
        if recv_pack[1][0:3] == 'MPC':  # currently only DMPC is available
            shared.param_mpc = mas.ParamMPC(unpack('=3s3f2i', recv_pack[1]))
            util.log_info("New MPC param received: %s" % shared.param_mpc)
            shared.status['new_param'] = True
        
    elif packet['rf_data'] == 'CLR_RDV': # clear rendezvous command, no comma split.
        shared.rendezvous = None
        util.log_info("'CLR_RDV' received, clear rendezvous")
    
    # guarantee packet integrity (payload length) -- Actual agent data pack
    elif len(packet['rf_data']) == shared.PAYLOAD_LENGTH:
        #from binascii import hexlify # logging binary bytes
        #util.log_debug("DATAPACK: %s" % hexlify(packet['rf_data']))
        
        # unpack data into lists and wrap into local datapacks
        recv_pack = WrappedData(unpack('=3s2d5f', packet['rf_data']))
        util.log_debug("%s" % recv_pack)
        
        if recv_pack.ID == 'TAR': # target point pack
            shared.rendezvous = recv_pack
            util.log_debug("Rendezvous coordinate received.")
            
        elif recv_pack.ID == 'ORG': # HOME_ORIGIN pack
            shared.home_origin = recv_pack.location_global_relative
            util.log_info("HOME_ORIGIN set: %s." % shared.home_origin)
            
        elif recv_pack.ID[0] == 'A': # normal neighbor begins with 'Axx'
            shared.neighbors[recv_pack.ID] = recv_pack

def dispatch_init(ser):
    """
    Initialize a dispatcher and register rx_data_handler.
    
    Args:
        ser(serial.Serial): a serial interface object.
        
    Returns:
        xbee.helpers.dispatch.Dispatch: an XBee dispatcher object.
    """
    # --------------------------------------------------------------------		
    # When a Dispatch is created with a serial port, it will automatically
    # create an XBee object on your behalf for accessing the device.
    # If you wish, you may explicitly provide your own XBee:
    #
    #  xbee = XBee(ser)
    #  dispatch = Dispatch(xbee=xbee)
    #
    # Functionally, these are the same.   --Paul Malmsten
    # --------------------------------------------------------------------
    # Register the packet handlers with the dispatch:
    # The string name allows one to distinguish between mutiple registrations
    # for a single callback function.
    # The second argument is the callback function name.
    # The third argument is a function which determines whether to call its
    # associated callback when a packet arrives. It should return a boolean.
    # --------------------------------------------------------------------	
    # Spawn a dispatch instance and associate it with packet id 'rx'
    dispatcher = Dispatch(ser)
    dispatcher.register(
        "rx_data", 
        rx_data_handler,
        lambda packet: packet['id']=='rx'
    )
    return dispatcher

def xbee_init(ser):
    """
    Initialize an XBee communication interface running in API mode.
    
    Args:
        ser(serial.Serial): a serial interface object.
        
    Returns:
        xbee.ZigBee: an XBee interface running in Zigbee compatable API.
    """
    dispatcher = dispatch_init(ser)
    return ZigBee(ser, callback=dispatcher.dispatch, escaped=True)

def xbee_broadcast(xbee, data):
    """
    Broadcast a payload through the XBee interface.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        data(str): data string or raw binary.
    """
    # if xbee is ZeroMQ socket type, initialize the publisher. SITL only.
    if isinstance(xbee, zmq.sugar.socket.Socket):
        xbee.send_multipart(['XBEE',data])
        return
    
    # else this is a real xbee
    xbee.send(
        'tx',
        frame_id = '\x00',                                   # frameID
        dest_addr_long = '\x00\x00\x00\x00\x00\x00\xFF\xFF', # 64-bit address - broadcast
        dest_addr = '\xFF\xFE',                              # 16-bit address - non-identified
        options = b'\x01',                                   # disable ACK
        data = data
	)
