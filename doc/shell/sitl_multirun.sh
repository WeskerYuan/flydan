#!/bin/bash

# This script is used to spawn multiple threads for the dronekit-sitl.
# Each thread shall have its unique firmware(wiz its TCP ports) and each thread
# could have its XBee module or use the ZeroMQ pub-sub model for communication.
# need superuser clearance if using actual USB devices.
python sitl.py -id 01 -alt 10 -c passive -pix fw/ac3.5.2_port5760  &
python sitl.py -id 02 -alt 11 -pix fw/ac3.5.2_port6760  &
python sitl.py -id 03 -alt 12 -pix fw/ac3.5.2_port7760  &
python sitl.py -id 04 -alt 13 -pix fw/ac3.5.2_port8760  &
python sitl.py -id 05 -alt 14 -pix fw/ac3.5.2_port9760

# Copyright:
#   Copyright 2017 Quan Yuan, Adaptive Networks and Control Lab,
#   Research Center of Smart Networks and Systems,
#   School of Information Science and Engineering,
#   Fudan University.
    
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
    
        # http://www.apache.org/licenses/LICENSE-2.0
    
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.