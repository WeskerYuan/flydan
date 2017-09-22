# Restart gpsd service.
# The gpsd package is poorly coded. It automatically grabs the "/dev/ttyUSB0" 
# as the default device. When using other USB ports or ACM devices, it often fails.
# Currently the way to walkaround is modify '/etc/default/gpsd'
# and change GPSD_OPTIONS="/dev/ttyXXX" (by default ttyACM0, if using
# USB virtual COM GNSS modules; if using USB-TTL adapters, it should be ttyUSBx)
# Run this script under super-user clearnce.
# Referance: https://www.raspberrypi.org/forums/viewtopic.php?t=8886&p=104848
#            http://stackoverflow.com/questions/29333424/gpsd-not-getting-a-good-fix
#            http://raspberrypi.stackexchange.com/questions/29547/cant-get-gps-to-automatically-work-after-reboot

echo "can" | sudo -S killall gpsd
echo "can" | sudo -S rm /var/run/gpsd.sock
echo "can" | sudo -S /etc/init.d/gpsd restart
# switch the password to your account's password.

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