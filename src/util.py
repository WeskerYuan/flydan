"""
Utility functions.

This module holds several useful functions for easy vector calculations and data
logging. Most of the functions can be directly used without any dependencies.

Attributes:
    log_info: alias for function `logging.info`.
    log_debug: alias for function `logging.debug`.
    log_warning: alias for function `logging.warning`.
    log_shutdown: alias for function `logging.shutdown`.
    
Functions:
    log_init(path, loglevel): initialize a logging.
    get_latest_log(path): acquire the last log ID and create a new one.
    vec_norm2(vector): calculate the 2-norm of a vector.
    vec_add(v1, v2): calculate (v1+v2).
    vec_sub(v1, v2): calculate (v2-v1).
    vec_scal(vector, a): calculate a*vector.

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
import logging

log_info = logging.info
log_debug = logging.debug
log_warning = logging.warning
log_shutdown = logging.shutdown

log_level = {
    'warning':logging.WARNING, 
    'debug':logging.DEBUG, 
    'info':logging.INFO
    }

def log_init(fname, loglevel):
    """
    Initialize the data logging to `write only` and `stream to console`.
    
    Args:
        fname(str): the file name for the data log.
        loglevel(int): logging level as defined in module:<logging>.
    
    Returns:
        object(`logging.RootLogger`): a root looger object as is instantiated.
    """
    if not os.path.isdir('log'):
        os.system("mkdir log")
    
    path = "log/%s" % fname
    
    logging.basicConfig(
        level=loglevel,
        format = '%(asctime)s %(filename)-8s %(levelname)-8s %(message)s',
        filename = path,
        filemode = 'w',
        datefmt = '%H:%M:%S'
        )
        
    formatter = logging.Formatter('%(filename)-8s %(levelname)-8s %(message)s')
    console = logging.StreamHandler()
    console.setLevel(loglevel)
    console.setFormatter(formatter)
    
    logger = logging.getLogger()
    logger.addHandler(console)
	
    logger.info('Logging file established: %s' % path)
    for delay in xrange(0, 10):
        time.sleep(0.2)
    
    return logger

def get_latest_log(fname):
    """
    Get the latest log number and establish a new one with 1 increment in the ID.
    
    Args:
        fname(str): the file name for a text file containing the last logging ID.
        
    Returns:
        int: logging ID for the log this time.   
    """
    if not os.path.isdir('log'):
        os.system("mkdir log")
    
    path = "log/%s" % fname
    
    if not os.path.isfile(path):
        latest = open(path, 'w')
        latest.write('0')
        latest.close()
        log_id = 0
        
    else:
        latest = open(path, 'r')
        log_id = int(latest.read()) + 1
        latest.close()
        latest = open(path, 'w')
        latest.write(str(log_id))
        latest.close()
        
    return log_id
    
def vec_norm2(vector):
    """
    Return the Euclidian 2-module of a vector.
    
    Args:
        vector(int/float/list): the vector to be evaluated.
        
    Returns:
        float: the Euclidian 2-module of vector.   
    """
    if type(vector) is list: 
        retval = 0 
        for idx in xrange(0, len(vector)):
            retval = retval + vector[idx]*vector[idx]
            
        retval = math.sqrt(retval)
        
    elif type(vector) is int or type(vector) is float:
        retval = float(vector)
        
    else:
        raise Exception('Input invalid.')
        
    return retval
    
def vec_scal(vector, a=1.0):
    """
    Scale the vector with a.
    
    Args:
        vector(int/float/list): the vector to be scaled.
        a(float): the scaling factor, default 1.0.
        
    Returns:
        float/list: the scaled vector as a*vector.
    """
    if type(vector) is list: 
        retval = [] 
        for idx in xrange(0, len(vector)):
            retval.insert(idx, a*vector[idx])
            
    elif type(vector) is int or type(vector) is float:
        retval = float(a*vector)
        
    else:
        raise Exception('Input invalid.')
        
    return retval
    
def vec_sub(v1, v2):
    """
    Return the difference of two vectors as (v2-v1).
    
    Args:
        v1(int/float/list): the subtrahend vector.
        v2(int/float/list): the minuend vector.
        
    Returns:
        float/list: the difference vector of (v2-v1).  
    """
    if type(v1) is list and type(v2) is list:
        if len(v1) == len(v2):
            retval = []
            for idx in xrange(0, len(v1)):
                retval.insert(idx, v2[idx] - v1[idx])
                
        else:
            raise Exception('Dimension mismatch!')
            
    elif ((type(v1) is int or type(v1) is float) and 
          (type(v2) is int or type(v2) is float)):
        retval = float(v2-v1)
    
    else:
        raise Exception('Type incorrect!')
        
    return retval
    
def vec_add(v1, v2):
    """
    Return the sum of two vectors as (v1+v2).
    
    Args:
        v1(int/float/list): the first vector.
        v2(int/float/list): the second vector.
        
    Returns:
        float/list: the sum vector of (v1+v2).
    """
    if type(v1) is list and type(v2) is list:
        if len(v1) == len(v2):
            retval = []
            for idx in xrange(0, len(v1)):
                retval.insert(idx, v1[idx] + v2[idx])
                
        else:
            raise Exception('Dimension mismatch!')
            
    elif ((type(v1) is int or type(v1) is float) and 
          (type(v2) is int or type(v2) is float)):
        retval = float(v1+v2)
    
    else:
        raise Exception('Type incorrect!')
        
    return retval
        