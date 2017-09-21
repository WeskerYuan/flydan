# This script is used to enable run-on-boot function for Ubuntu Linux

# Edit /lib/systemd/system/getty@.service and add auto login function by
# inserting `-a <USERNAME>` into the following sentence:
#   [Service]
#   ExecStart=-/sbin/agetty -a <USERNAME> --noclear %I $TERM

# Put this script under /etc/profile.d and it will auto-run on boot. 

cd /home/can/flydan
echo "can" | sudo -S python /home/can/flydan/onboard.py -id 01 -alt 25
# modify the directory and the password accordingly.
# for each drone, modify the arguments at discretion.

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