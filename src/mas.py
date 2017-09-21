"""
Multi-agent system control algorithm module.

This module contains the classes and functions for the multi-agent system control
at high-level. There are two main flocking control algorithms. 
The COLLMOT's self-propelling algorithm at 2014 and the RCSNS' decentralized 
model predictive control at 2016. There are also some control threads for the 
leader copter in the drone flock.

Reference:
    [1] J. Zhan, X. Li, Decentralized focking protocol of multi-agent 
        systems with predictive mechanisms, CCC2011, 5995-6000.
    [2] Q. Yuan, J. Zhan and X. Li, Outdoor flocking of quadcopter drones with
        decentralized model predictive control, ISA Transactions, 2017.
    [3] G.Vasarhelyi, Cs.Viragh, G.Somorjai, et. al. Outdoor Flocking and 
        formation flight with autonomous aerial robots, IROS2014

Classes:
    ParamMPC(object): a data structure that packs the DMPC algorithm parameters.
    Decentralized(threading.Thread): the Zhan's DMPC CCC2011 algorithm.
    Vicsek(threading.Thread): the Vicsek IROS2014 algorithm.
    SquareRoute(threading.Thread): the autonomous square shaped route.
    PassiveLeader(threading.Thread): the passive leader controlled by GCS.

Functions:
    get_shape_factor: calculate the shape factors in Vicsek algorithm.
    bump_function(x, R, d): soft bump function.
    array_pow(a, k): calculate the k-power of a square matrix. 
    
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
import logging
import threading
import numpy as np

from copy import copy
from timeit import default_timer
from numpy import copy as np_copy
from numpy import dot as np_dot
from dronekit import LocationGlobal
from dronekit import LocationGlobalRelative

import nav
import comm
import util
import shared

def get_shape_factor(my_location, neighbors, R0, shape='grid'):
    """
    Get the shape vector X_shape and average coordinate X_CoM in the COLLMOT IROS2014 program.
    
    Reference:
        [3] G.Vasarhelyi, Cs.Viragh, G.Somorjai, et. al. Outdoor Flocking and 
        formation flight with autonomous aerial robots, IROS2014
        
    Args:
        my_location(dronekit.LocationXXYY): `my` location coordinate.
        neighbors(dict): a dictionary containing the data of `my neighbors`.
        R0(float): desired inter-agent distance.
        shape(str): a string describing the shape (grid/line/ring).
    
    Returns:
        list: shape vector X_shape, average position X_CoM, bump pass-band width Rs.
    """
    # g(N) is a heuristic function defining the radius of the smallest circle
    # that can contain N unit circles. 
    # Ref: http://hydra.nat.uni-magdeburg.de/packing/cci/cci.html#cci1
    g = [1,2,2.15470053840000,2.41421356240000,2.70130161670000,3,3,3.30476487100000,
         3.61312592980000,3.81302563140000,3.92380440020000,4.02960193010000,4.23606797750000,
         4.32842855490000,4.52135696470000,4.61542559490000,4.79203374830000,4.86370330520000,
         4.86370330520000,5.12232073700000,5.25231747500000,5.43971895910000,5.54520422260000,
         5.65166109180000,5.75282433090000,5.82817653690000,5.90639784740000,6.01493809740000,
         6.13859790400000,6.19774107090000]
    
    X_CoM = copy(my_location) # copy `my` location since it's mutable.
    agent_count = len(neighbors) + 1 # agent count ('me' included)
   
    # Do for each neighbor of 'me', calculate X_CoM
    # This is a approximation, since the coordinates are in decimal degrees
    # and are not converted into NED frame in meteres. The calculation ignored
    # the eclipse effect of the earth, and directly averages the coordinates.
    for idx in neighbors:
        if isinstance(X_CoM,LocationGlobal):
            X_CoM.lat += neighbors[idx].location_global.lat
            X_CoM.lon += neighbors[idx].location_global.lon
            X_CoM.alt += neighbors[idx].location_global.alt
            
        elif isinstance(X_CoM,LocationGlobalRelative):
            X_CoM.lat += neighbors[idx].location_global_relative.lat
            X_CoM.lon += neighbors[idx].location_global_relative.lon
            X_CoM.alt += neighbors[idx].location_global_relative.alt
            
        else:
            raise Exception("Invalid Location object passed.")

    X_CoM.lat /= agent_count # average position (`me` included)
    X_CoM.lon /= agent_count
    X_CoM.alt /= agent_count
    
    if shape == 'grid':
        Rs = 0.5*R0*(g[shared.AGENT_COUNT-1] - 1)
        X_shape = copy(X_CoM)

    elif shape == 'line': pass # reserved
    elif shape == 'ring': pass # reserved
    else: pass
    
    return [X_shape, X_CoM, Rs]

def bump_function(x, R, d):
    """
    A smooth transitional function to bump the inter-agent distance.
    
    Args:
        x(float): input inter-distance.
        R(float): pass-band width.
        d(float): transition-band width (R+d will be the stop-band).
        
    Returns:
        float: bump weight between [0, 1]
    """
    if (x < 0) or (R < 0) or (d < 0):
        raise Exception('bump_function:Input below zero!')
    else:
        if (0 <= x) and (x <= R):
            fx = 0
        elif (R < x) and (x <= (R + d)):
            fx = 0.5*(math.sin(math.pi/d*(x-R)-math.pi/2)+1)
        else:
            fx = 1
    return fx

def array_pow(a, k):
    """
    Calculate the matrix power as a^k.
    
    Args:
        a(numpy.array): a square matrix to be mutiplied.
        k(int): the power index.
    """
    retval = np_copy(a)
    for i in xrange(1, k):
        retval = np_dot(retval, a)
    return retval
    
def _log_and_broadcast(xbee, info):
    """
    Log some information at `INFO` level and broadcast via XBee.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        info(str): the information string.
    """
    comm.xbee_broadcast(xbee, info)
    util.log_info(info)

def _check_terminate_condition(vehicle, name_string):
    """
    Check several termination conditions and set the stopflag to a thread.
    
    Priority:
        shared.status['abort'] == True: emergency, highest level.
        shared.status['thread_flag'] has FLOCKING_FLAG: thread killing command.
        vehicle.mode != 'GUIDED': grabbed manual control.
        
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        name_string(str): a string specifies the thread name.
    
    Returns:
        bool: True if condition is met, False if no-stop.
    """
    status = False
    exit_number = 0
    if shared.status['abort']:
        util.log_warning("Abort detected!")
        status = True
        exit_number = 1
        
    elif shared.status['thread_flag'] & shared.FLOCKING_FLAG:
        util.log_info("FLOCKING_FLAG detected.")
        status = True
        exit_number = 2
        
    elif vehicle.mode != 'GUIDED':
        util.log_warning("Mode switched out from 'GUIDED'!")
        status = True
        exit_number = 3
        
    if status: util.log_info("Stopping %s" % name_string)
    
    return [status, exit_number]
    
def _check_satellite_low(xbee, is_on_hold):
    """
    Check if satellites are low and set the is_on_hold flag.
    
    Args:
        xbee(xbee.Zigbee): the XBee communication interface.
        is_on_hold(bool): a flag telling if the thread is already on hold.
    
    Returns:
        bool: True if low sats, False if cleared.
    """
    if shared.status['thread_flag'] & shared.NSATS_TOO_LOW:
        if not is_on_hold: _log_and_broadcast(xbee, "IFO,%s low sats hold." % shared.AGENT_ID)
        tiime.sleep(0.5)    
        return True
    
    else: return False


class ParamMPC(object):
    """
    Wrap a list of parameters into a data pack object.
    
    The parameters are for the Decentralized Model Predictive Control algorithm
    only.
    
    Reference:
        [1] J. Zhan, X. Li, Decentralized focking protocol of multi-agent 
            systems with predictive mechanisms, CCC2011, 5995-6000.
        [2] Q. Yuan, J. Zhan and X. Li, Outdoor flocking of quadcopter drones with
            decentralized model predictive control, ISA Transactions, 2017.
    """
    
    def __init__(self, paramlist):
        """
        Input a list of DMPC parameters.
        
        The info_list components: 
            (ID should always be 'MPC')
            ----------------------------------
            item:   Python-type:   C-type   length(B)    offset:
            ID      string         char[]     3           0
            Ts      float          float      4           1
            Vmax    float          float      4           2
            D0      float          float      4           3
            Hp      int            int        4           4
            Hu      int            int        4           5
            -----------------------------------
            
        Example: 
            ID      Ts    Vmax  D0     Hp Hu
            ['MPC', 0.20, 3.50, 12.00, 2, 1]
        """
        self.__ID   = paramlist[0]
        self.__Ts   = round(float(paramlist[1]),8)
        self.__Vmax = round(float(paramlist[2]),8)
        self.__D0   = round(float(paramlist[3]),8)
        self.__Hp   = int(paramlist[4])
        self.__Hu   = int(paramlist[5])
    
    @property
    def ID(self):
        """
        str: Parameter identifier.
        """
        return self.__ID
        
    @property
    def Ts(self):
        """
        float: Sampling period.
        """
        return self.__Ts
    
    @property
    def Vmax(self):
        """
        float: Maximum moving velocity.
        """
        return self.__Vmax
    
    @property
    def D0(self):
        """
        float: Inter-agent distance.
        """
        return self.__D0
        
    @property
    def Hp(self):
        """
        int: Prediction horizon.
        """
        return self.__Hp
    
    @property
    def Hu(self):
        """
        int: Control horizon.
        """
        return self.__Hu    
    
    def __str__(self):
        return "ParamMPC: %s,%.2f,%.2f,%.2f,%d,%d" % (
                self.__ID,
                self.__Ts,
                self.__Vmax,
                self.__D0,
                self.__Hp,
                self.__Hu
                )


class Decentralized(threading.Thread):
    """
    The Zhan's CCC2011 flocking algorithm as a thread.
    
    Reference:
        [1] J. Zhan, X. Li, Decentralized focking protocol of multi-agent 
            systems with predictive mechanisms, CCC2011, 5995-6000.
        [2] Q. Yuan, J. Zhan and X. Li, Outdoor flocking of quadcopter drones with
            decentralized model predictive control, ISA Transactions, 2017.
            
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
        neighbors(dict): a dictionary containing all the states of the neighbors.
    """
    
    def __init__(self, vehicle, xbee, neighbors):
        threading.Thread.__init__(self)
        self._vehicle   = vehicle
        self._xbee      = xbee
        self._neighbors = neighbors
        self._stopflag  = False
    
    def run(self):
        """
        Run the ZHan' DMPC CCC2011 algorithm thread.
        """
        # ============== Zhan's MPC Algorithm -- Default Parameters ===============
        D0 = 15.0       # Pair potential lattice constant (Unit: m)
        Rc = 10 * D0    # Communication range, used accordingly (1.2 is optimal)
        C  = 0.2        # Coeeficient in velocity difference equation
        Lambda = 1e-3   # Lambda in the MPC iteration
        Hp = 2          # prediction horizon
        Hu = 1          # control horizon
        Ts = 0.20       # sampling interval
        Vmax = 4.5      # maximum flocking speed
                
        loop_count = 1 # for calculate loop time
        average_time = 0
        
        is_on_hold = False   # hold position flag for low Nsats
        
        while not self._stopflag:
            time.sleep(0.001)
            
            [self._stopflag, exit_number] = _check_terminate_condition(self._vehicle, 'Decentralized')

            is_on_hold = _check_satellite_low(self._xbee, is_on_hold)
            if is_on_hold: continue
            
            # parameter update
            if shared.status['new_param'] and shared.param_mpc is not None:
                Ts   = shared.param_mpc.Ts
                Vmax = shared.param_mpc.Vmax
                D0   = shared.param_mpc.D0
                Hp   = shared.param_mpc.Hp
                Hu   = shared.param_mpc.Hu
                
                util.log_info("New MPC param accepted.")
                shared.status['new_param'] = False
                shared.param_mpc = None
            
            loop_begin_time = default_timer()   # Flocking algorithm begins
            
            friends    = copy(self._neighbors)
            my_location = self._vehicle.location.global_relative_frame
            my_velocity = self._vehicle.velocity
            
            # -------- Tailor the neighbors if we use Rc = 1.2 d0 ----------
            # comment the block if not used
            # not_my_friend = dict()
            # for j in friends:
                # if nav.get_distance_metres(my_location, friends[j].location_global_relative) > Rc:
                    # not_my_friend[j] = j

            # for j in not_my_friend:
                # del friends[j]
            
            # ------------- <Step I.> Extract local information ----------------
            Ni  = len(friends)   # agent-i have <Ni> neighbors
            Ni1 = Ni + 1         # account for agent-i itself 
            
            # if there is no neighbor, hold position and pass on
            if Ni == 0: continue
            
            # generate local information matrices
            xi_p = np.zeros( (Ni1, 2) )
            xi_v = np.zeros( (Ni1, 2) )
            
            # first entry is agent-i itself
            xi_p[0] = nav.get_position_error(shared.home_origin, my_location)[0:2]
            xi_v[0] = my_velocity[0:2]
            
            # from 1 to Ni, is agent-i's friends
            idx = 1
            for j in friends:
                xi_p[idx] = nav.get_position_error(shared.home_origin, friends[j].location_global_relative)[0:2]
                xi_v[idx] = (friends[j].velocity)[0:2]
                idx = idx + 1
            
            # stack <xi_p> and <xi_v> into a large array
            xi = np.vstack( (xi_p, xi_v) )
            
            # All the coordinates are non-linear, so an absolute origin
            # is needed to convert the GNSS coordinates into plannar form.
            # This is done thru nav.get_position_error
            # From this line on, there's no GNSS coordinates.
            
            # ------------- <Step II.> Generate related matrices --------------
            # constrcut the block matrices A and B
            I = np.eye(Ni1)
            Z = np.zeros( (Ni1, Ni1) )
            A = np.vstack( (np.hstack( (I, Ts*I) ),  np.hstack( (Z, I) )) )
            B = np.vstack( (Z, Ts*I) )

            # predict state using transition matrix A
            X = np.zeros( (2*Ni1*(Hp+1), 2) )
            X[0:2*Ni1] = xi
            for k in xrange(1, Hp+1):
                X[k*2*Ni1 : (k+1)*2*Ni1] = A.dot( X[(k-1)*2*Ni1 : k*2*Ni1] )

            # P_Xi matrix    
            PX = np.zeros( (2*Ni1*Hp, 2*Ni1) )
            for k in xrange(1, Hp+1):                    
                PX[(k-1)*2*Ni1 : k*2*Ni1] = array_pow(A, k)
            
            # P_Ui matrix
            PU = np.zeros( (2*Ni1*Hp, Ni1*Hu) )
            for k in xrange(1, Hu):
                for l in xrange(k, Hp+1):
                    PU[(l-1)*2*Ni1 : l*2*Ni1, (k-1)*Ni1 : k*Ni1] = array_pow(A, l-k).dot(B)
            
            for l in xrange(Hu, Hp+1):
                for k in xrange(0, l-Hu+1):
                    PU[(l-1)*2*Ni1 : l*2*Ni1, (Hu-1)*Ni1 : Hu*Ni1] = \
                        array_pow(A, l-k).dot(B) + \
                        PU[(l-1)*2*Ni1 : l*2*Ni1, (Hu-1)*Ni1 : Hu*Ni1]

            # E & S matrices
            E = np.zeros( (Ni1*Hp, 2*Ni1*Hp) )
            S = np.zeros( (Ni1*Hp, 2) )
            for k in xrange(1, Hp+1):
                # E matrix and e_(Ni+1)
                E[k*Ni1-1, (2*k-1)*Ni1] = C*(1/Ni1-1)
                E[k*Ni1-1 : k*Ni1, (2*k-1)*Ni1+1 : 2*k*Ni1] = C*(1/Ni1)*np.ones((1,Ni))
                
                # '-1' vector & Identity block
                E[(k-1)*Ni1 : k*Ni1-1, (k-1)*2*Ni1 : (k-1)*2*Ni1+1] = -np.ones((Ni,1))
                E[(k-1)*Ni1 : k*Ni1-1, (k-1)*2*Ni1+1 : (2*k-1)*Ni1] = np.eye(Ni)
                
                # S matrix
                for l in xrange(2, Ni1+1):
                    e_ji = X[k*2*Ni1+l-1, : ] - X[k*2*Ni1, : ]
                    S[(k-1)*Ni1+l-2, : ] = D0/np.linalg.norm(e_ji)*e_ji
            
            # ------------- <Step III.> Do the MPC and update states -----------
            # MPC input
            R = Lambda * np.eye(Hu*Ni1)
            U = - np.linalg.solve( E.dot(PU).T.dot(E).dot(PU) + R,
                                E.dot(PU).T.dot(E.dot(PX).dot(xi) - S) )
            # update state
            W = np.zeros((Ni1, Ni1*Hu))
            W[:, 0:Ni1] = np.eye(Ni1)
            u = W.dot(U)            
            
            # update velocity control to the Pixhawk (2-dimensional)
            des_velocity = xi_v[0] + Ts*u[0]
            des_speed    = np.linalg.norm(des_velocity)
            
            # normalize to Vmax if V exceeded.
            if des_speed > Vmax: des_velocity = (Vmax/des_speed) * des_velocity
  
            # calculate algorithm loop time (this method only runs at valid control loop)
            loop_end_time = default_timer() # Time marker ends
            elapsed_time = (loop_end_time - loop_begin_time) * 1000000  # convert into us
            inv_count    = 1.0/loop_count   # incremental averaging method
            average_time = (1.0 - inv_count) * average_time + inv_count * elapsed_time
            loop_count   = loop_count + 1
            
            # Send thru Mavlink
            des_velocity = list(des_velocity)
            nav.send_ned_velocity(self._vehicle, 
                                  des_velocity[0],
                                  des_velocity[1],
                                  0, # Vz is ignored
                                  Ts )
        # End of While #             
        _log_and_broadcast(self._xbee, "IFO,%s DMPC ended with number %d." % (shared.AGENT_ID, exit_number))
        util.log_info("Average loop time: %d us" % average_time)
    # End of run() #
    
    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True


class Vicsek(threading.Thread):
    """
    The Vicsek IROS2014 flocking algorithm as a thread.
    
    Reference:
        [3] G.Vasarhelyi, Cs.Viragh, G.Somorjai, et. al. Outdoor Flocking and 
        formation flight with autonomous aerial robots, IROS2014
    
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
        neighbors(dict): a dictionary containing all the states of the neighbors.
    """
    def __init__(self, vehicle, xbee, neighbors):
        threading.Thread.__init__(self)
        self._vehicle   = vehicle
        self._xbee      = xbee
        self._neighbors = neighbors
        self._stopflag  = False
        
    def run(self):
        """
        Run the Vicsek IROS 2014 algorithm thread.
        """
        # ============== Vicsek Algorithm -- Parameters ===============
        R0 = 12.0       # Pair potential lattice constant (Unit: m)
        Dp = 1.0        # Pair potential spring constant (Unit: s^-2)
        Cf = 18.0       # Viscous friction coefficient, or damp factor (Unit: m^2/s)
        Cs = 5.0        # Viscous friction coefficient for the shill agent (Unit: s^-1)
        Vf = 1.0        # Flocking speed for the MAS to maintain (Unit: m/s)
        V0 = 2.0        # Maximum tracking velocity
        Vmax = 3.0      # Maximum navigating velocity (the copter's total max V)

        Alpha = 0.6     # TRG coefficient (0 <= Alpha <= 1)
        Beta = 0.8      # COM coefficient (0 <= Beta <= 1)
        Rp = 0.5 * R0   # Upper threshold for coping with over-excitation (Short-range)
        Rs1 = 0.8 * R0  # Upper threshold incase division by 0 (Mid-range)
        Rs2 = 0.3 * R0  # Constant slope factor around R0

        Tau = 0.10      # Characteristc time for reaching velocity
        Del_t = 0.10    # Feed-forward estimation time
        Ts = Del_t      # Let sampling period equal to del_t

        Rw = 100.0      # Radius for wall (Unit: m)
        Rt = 1.0        # Radius for target (Unit: m)
        Dw = 5.0        # Transition bandwith for wall (Unit: m)
        Ds = 2.0        # Transition bandwith for shape (Unit: m)
        Dt = 1.0        # Transition bandwith for target point (Unit: m)
        
        loop_count = 1 # for calculate loop time
        average_time = 0
        
        is_on_hold = False # a flag marking if the copter is holding position
        no_rendezvous = False # a flag marking if there's a rendezvous point
        
        while not self._stopflag:
            time.sleep(0.001)           
            
            [self._stopflag, exit_number] = _check_terminate_condition(self._vehicle, 'Vicsek')
            
            is_on_hold = _check_satellite_low(self._xbee, is_on_hold)
            if is_on_hold: continue
            
            loop_begin_time = default_timer()   # Flocking algorithm begins
            # -------------- <Part I> Repulsion and Viscous -------------- #
            friends    = copy(self._neighbors)
            my_location = self._vehicle.location.global_relative_frame
            my_velocity = self._vehicle.velocity
            # attributes from Vehicle returns a newly spwaned object, no need to
            # `copy`. But the dictionary is mutable so it should be copied.
            
            if shared.rendezvous is None: # hold position if no rendezvous
                if not no_rendezvous:
                    _log_and_broadcast(self._xbee, "IFO,%s no rendezvous." % shared.AGENT_ID)
                    no_rendezvous = True
                
                time.sleep(0.5)
                continue
            
            else: no_rendezvous = False
            
            # Self-propelling term
            my_speed = util.vec_norm2(my_velocity)
            V_spp = util.vec_scal(my_velocity, Vf/my_speed)
            
            # potential term and slipping term
            sum_potential = [0.0, 0.0, 0.0]
            sum_slipping  = [0.0, 0.0, 0.0]
            
            # go thru each neighbor
            for j in friends:
                # location error (form: [dNorth, dEast, dDown], unit: meters)
                # distance, metres (get_distance_metres is also ok)
                X_ij = nav.get_position_error(my_location, friends[j].location_global_relative)
                D_ij = util.vec_norm2(X_ij[0:2]) # only measure ground distance, ignore altitude
                
                # velocity error (form: [dVx, dVy, dVz], unit: m/s)
                V_ij  = util.vec_sub(my_velocity, friends[j].velocity)
                
                # i. short-range repulsion, when D_ij < R0
                if D_ij < R0:
                    temp_vector = util.vec_scal(X_ij, min(Rp, R0-D_ij)/D_ij)
                    sum_potential = util.vec_add(sum_potential, temp_vector)
                
                # ii. mid-range viscous friction
                temp_vector = util.vec_scal(V_ij, 1.0/pow(max(Rs1, D_ij-R0+Rs2), 2) )
                sum_slipping = util.vec_add(sum_slipping, temp_vector)
            # End of for j in friends
            
            # ---- collective potential acceleration ----
            A_potential = util.vec_scal(sum_potential, -Dp)
            A_slipping  = util.vec_scal(sum_slipping ,  Cf)
            
            # ------------ <Part II> Global Positional Constraint -------------- 
            # alias for the rendezvous point (a comm.WrappedData object)
            X_trg = shared.rendezvous.location_global_relative 
            
            # i. Flocking: Shill agent wall
            X_tmp = nav.get_position_error(my_location, X_trg)
            D_tmp = util.vec_norm2(X_tmp[0:2])
            # normally the X_trg cannot equal to my_location, but there is still room
            # to add algorithm twitches to improve robustness.
            
            # The force of the wall only applies when agents are out of the area
            # A_wall = Cs * bump(D_tmp, Rw, Dw) * (Vf/D_tmp*X_tmp - V_i)
            temp_vector = util.vec_scal(X_tmp, Vf/D_tmp)
            temp_vector = util.vec_sub(my_velocity, temp_vector) 
            A_wall = util.vec_scal(temp_vector, Cs*bump_function(D_tmp, Rw, Dw))
            
            # ii. Formation: Shape shift and CoM
            [X_shp, X_CoM, Rs] = get_shape_factor(my_location, friends, R0, 'grid')
            
            X_tmp = nav.get_position_error(my_location, X_shp)
            D_tmp = util.vec_norm2(X_tmp[0:2])
            
            if D_tmp: # deals with divide-by-zero
                V_shp = util.vec_scal(X_tmp, Beta*V0*bump_function(D_tmp,Rs,Ds)/D_tmp)
            else:
                V_shp = util.vec_scal(X_tmp, 0)
            
            # rendezvous target vector
            X_tmp = nav.get_position_error(X_CoM, X_trg)
            D_tmp = util.vec_norm2(X_tmp[0:2])
            V_trg = util.vec_scal(X_tmp, Alpha*V0*bump_function(D_tmp,Rt,Dt)/D_tmp)
            # normally the X_trg cannot equal to X_CoM, but there is still room
            # to add algorithm twitches to improve robustness
            
            # tracking vector (normalize if saturated)
            V_trk = util.vec_add(V_shp, V_trg)
            S_trk = util.vec_norm2(V_trk)
            if S_trk > V0: V_trk = util.vec_scal(V_trk, V0/S_trk)
            
            # ------------ <Part III> Full Dynamic ------------
            # V(t+1) = V(t) + 1/Tau*(V_spp + V_trk - V(t))*Del_t + (A_pot + A_slip + A_wall)*del_t
            acceleration = util.vec_add(A_wall, util.vec_add(A_potential,A_slipping))
            velocity     = util.vec_add(V_spp, util.vec_sub(my_velocity, V_trk))
            
            acceleration = util.vec_scal(acceleration, Del_t)
            velocity     = util.vec_scal(velocity, Del_t/Tau)
            
            inc_velocity = util.vec_add(acceleration, velocity)
            des_velocity = util.vec_add(my_velocity, inc_velocity)
            des_speed    = util.vec_norm2(des_velocity)
            
            if des_speed > Vmax: des_velocity = util.vec_scal(des_velocity, Vmax/des_speed)
            
            loop_end_time = default_timer() # Time marker ends
            elapsed_time = (loop_end_time - loop_begin_time) * 1000000  # convert into us
            inv_count    = 1.0/loop_count   # incremental averaging method
            average_time = (1.0 - inv_count) * average_time + inv_count * elapsed_time
            loop_count   = loop_count + 1
            
            # Send thru Mavlink
            nav.send_ned_velocity(  self._vehicle, 
                                    des_velocity[0],
                                    des_velocity[1],
                                    0, # Vz is ignored
                                    Ts )
        # End of While # 
        _log_and_broadcast(self._xbee, "IFO,%s Vicsek ended with number %d." % (shared.AGENT_ID, exit_number))
        util.log_info("Average loop time: %d us" % average_time)
    # End of run() #
    
    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True


class SquareRoute(threading.Thread):
    """
    Make the copter run in a square shaped route.
  
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
    """
    def __init__(self, vehicle, xbee):
        threading.Thread.__init__(self)
        self._vehicle  = vehicle
        self._xbee     = xbee
        self._stopflag = False
        
    def run(self):
        """
        Run `SquareRoute` thread.
        """
        info = "IFO,Leader %s engaging <SquareRoute>." % shared.AGENT_ID
        comm.xbee_broadcast(self._xbee, info)
        util.log_info(info)
        
        EDGE_LEN = 20.0 # square edge length
        WP_GSPEED = 1.0 # groundspeed for navigating
        edge_count = 0 # a counter holding which edge the copter is on
        old_gspeed = self._vehicle.groundspeed # log original gspeed
        
        # flying direction vector
        dir = [
            # dNorth, dEast
            [EDGE_LEN, 0 , 'North'],  # North
            [0, EDGE_LEN , 'East' ],  # East
            [-EDGE_LEN, 0, 'South'],  # South
            [0, -EDGE_LEN, 'West' ]   # West
        ]
        
        # waypoints
        wp = [
            # WP, description
            [None, 'N-W'], 
            [None, 'N-E'],
            [None, 'S-E'],
            [None, 'S-W']
        ]
        
        # extract square-shaped waypoints (hard waypoints to counter drifting)
        wp[-1][0] = self._vehicle.location.global_relative_frame
        wp[ 0][0] = nav.get_location_metres(wp[-1][0], dir[0][0], dir[0][1])
        wp[ 1][0] = nav.get_location_metres(wp[ 0][0], dir[1][0], dir[1][1])
        wp[ 2][0] = nav.get_location_metres(wp[ 1][0], dir[2][0], dir[2][1])
        
        is_on_hold = False   # hold position flag for low Nsats
        
        while not self._stopflag:       
            [self._stopflag, exit_number] = _check_terminate_condition(self._vehicle, 'SquareRoute')
            
            is_on_hold = _check_satellite_low(self._xbee, is_on_hold)
            if is_on_hold: continue

            # hold position if RC_CH6 is at low-level
            if self._vehicle.channels['6'] < 1500:
                time.sleep(1)
                continue
                
            # loiter for 3 second and moving along an edge utilizing
            # internal function <simple_goto>
            time.sleep(3)
            _log_and_broadcast(self._xbee, "IFO,Leader-%s moving towards %s" % (shared.AGENT_ID, dir[edge_count][2]))
            self._vehicle.simple_goto(wp[edge_count][0], groundspeed = WP_GSPEED)
            
            # check distance until reached
            while not self._stopflag:
                [self._stopflag, exit_number] = _check_terminate_condition(self._vehicle, 'SquareRoute')
                
                is_on_hold = _check_satellite_low(self._xbee, is_on_hold)
                if is_on_hold: continue
                
                time.sleep(0.2)
                
                current = self._vehicle.location.global_relative_frame
                distance = nav.get_distance_metres(current, wp[edge_count][0])
                
                if distance <= shared.WP_RADIUS:
                    _log_and_broadcast(self._xbee, "IFO,Leader-%s %s corner reached" % (shared.AGENT_ID, wp[edge_count][1]))
                    edge_count = (edge_count + 1) % 4
                    break
                    
                elif self._vehicle.channels['6'] < 1500:
                    nav.send_ned_velocity(self._vehicle, 0,0,0, 0.5) # brake
                    break
        # End of While #
        _log_and_broadcast(self._xbee, "IFO,%s SquareRoute ended with number %d." % (shared.AGENT_ID, exit_number))

        self._vehicle.groundspeed = old_gspeed #restore vehicle gspeed
    # End of run() #
    
    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True
        

class PassiveLeader(threading.Thread):
    """
    Make the copter follow the moving command from GCS as `passive` instead of `autonomous`.
  
    Args:
        vehicle(dronekit.Vehicle): the copter to be controlled.
        xbee(xbee.Zigbee): the XBee communication interface.
    """
    def __init__(self, vehicle, xbee):
        threading.Thread.__init__(self)
        self._vehicle  = vehicle
        self._xbee     = xbee
        self._stopflag = False
        
    def run(self):
        """
        Run `PassiveLeader` thread.
        """
        _log_and_broadcast(self._xbee, "IFO,Leader %s engaging <PassiveLeader>." % shared.AGENT_ID)
        
        WP_GSPEED = 1.0   # groundspeed for navigating
        old_gspeed = self._vehicle.groundspeed   # log original gspeed
        
        is_on_hold = False   # hold position flag for low Nsats
        while not self._stopflag:               
            [self._stopflag, exit_number] = _check_terminate_condition(self._vehicle, 'PassiveLeader')
            
            is_on_hold = _check_satellite_low(self._xbee, is_on_hold)
            if is_on_hold: continue
                
            # ******************** PassiveLeader Route Begins ********************
            # hold position if already reached rendezvous (or no rendezvous)
            if shared.rendezvous is None:
                time.sleep(0.5)
                continue
                
            else:
                cached_rendezvous = copy(shared.rendezvous.location_global_relative)
                cached_rendezvous.alt = self._vehicle.location.global_relative_frame.alt
                current = self._vehicle.location.global_relative_frame
                distance = nav.get_distance_metres(current, cached_rendezvous)
                if distance <= shared.WP_RADIUS:
                    time.sleep(0.5)
                    continue
                
            # loiter for 1 second and moving to the target utilizing
            # internal function <simple_goto>
            time.sleep(1)
            _log_and_broadcast(self._xbee, "IFO,Leader %s moving towards %s" % (shared.AGENT_ID, cached_rendezvous))
            self._vehicle.simple_goto(cached_rendezvous, groundspeed = WP_GSPEED)  
            
            while not self._stopflag:   # check distance until reached
                [self._stopflag, exit_number] = _check_terminate_condition(self._vehicle, 'PassiveLeader')
                
                is_on_hold = _check_satellite_low(self._xbee, is_on_hold)
                if is_on_hold: continue
                
                time.sleep(0.2)

                current = self._vehicle.location.global_relative_frame
                distance = nav.get_distance_metres(current, cached_rendezvous)
                print 'distance = %.2f' % distance 
                if distance <= shared.WP_RADIUS:
                    _log_and_broadcast(self._xbee, "IFO,Leader %s reached %s" % (shared.AGENT_ID, cached_rendezvous))
                    break
                    
                elif shared.rendezvous is None:
                    _log_and_broadcast(self._xbee, "IFO,Leader %s holding position due to target loss." % shared.AGENT_ID)
                    nav.send_ned_velocity(self._vehicle, 0,0,0, 0.5) # brake
                    break
        # End of While #
        _log_and_broadcast(self._xbee, "IFO,%s PassiveLeader ended with number %d." % (shared.AGENT_ID, exit_number))
        
        self._vehicle.groundspeed = old_gspeed #restore vehicle gspeed
    # End of def run() #
    
    def stop(self):
        """
        Set a flag to exit the thread.
        """
        self._stopflag = True

