## Flydan Project
### Introduction
Welcome to the Flydan project! "Flydan" is a multi-copter drone test platform originally designed by [Quan Yuan](https://github.com/WeskerYuan/) from the Adaptive Networks and Control Lab ([CAN Lab](www.can.fudan.edu.cn)), Fudan University.

The "Flydan" drones take the [Pixhawk](pixhawk.org) and the [ArduPilot](www.ardupilot.org) stack as their low-level flight  controller and use [dronekit-python](python.dronekit.io) as the high-level application control. So far there is no modification at the Pixhawk and the ArduPilot level, so this project is purely written in Python running on a Linux companion computer (e.g. Raspberry Pi).

The "Flydan" platform aims at realizing a real-world **multi-agent system (MAS)** by implementing and testing the MAS algorithms on the platform (e.g. flocking/formation control/...). The "Flydan" drones use [XBee](https://www.digi.com/products/xbee-rf-solutions/2-4-ghz-modules) modules to establish a high-level communication network between the drones and the ground control station. We have successfuly tested some flocking algorithms in outdoor environment and realized our own [Decentralized Model Predictive Control](https://doi.org/10.1016/j.isatra.2017.07.005) flocking. 

#### Reference
1. Quan Yuan, Jingyuan Zhan and Xiang Li, Outdoor flocking of quadcopter drones with decentralized model predictive control, ISA Transactions, 2017, http://dx.doi.org/10.1016/j.isatra.2017.07.005.

#### Copyright
2017 Quan Yuan, Adaptive Networks and Control Lab ([CAN](www.can.fudan.edu.cn)),
Research Center of Smart Networks and Systems (RCSNS),
School of Information Science and Engineering,
Fudan University ([FDU](www.fudan.edu.cn)).

#### License
Flydan Project is made available under the permissive open source Apache 2.0 License.

---

### System requirement
#### Drone
1. A multi-copter drone using Pixhawk as its flight controller.
2. A mini onboard Linux companion computer. (e.g. Raspberry Pi)
3. An XBee module with a USB adapter. (e.g. XBee S1, XBee S2C, etc.)

    __Note:__ Zigbee's are not recommended as they are relatively slow and have small data throughput volume. Zigbee modules tend to get stuck often. The XBee Pro S1 with DIJI Mesh firmware is tested to be working very well. A new hardware upgrade by DIJI unifies XBee and Zigbee to "S2C" version, which are now compatible across all the DIJI product lines.

4. (Optional) A USB-TTL adapter for debugging (e.g. FT232, CP2102/CP2104, do not use PL2303)

#### Ground control station
1. A Linux laptop. (Virtual machines are okay)
2. A GNSS module (e.g. u-blox M8N).
3. An XBee module with a USB adapter. (e.g. XBee S1, XBee S2C, etc.)

#### Software-in-the-loop (SITL) simulation
1. A Linux laptop. (Virtual machines are okay, but must be x86 compatible)
2. The [Dronekit-SITL](http://python.dronekit.io/develop/sitl_setup.html) (Other native SITLs are also possible)
3. ArduPilot source code and EEPROM binaries for the Dronekit-SITL

#### Packages
1. Linux packages: python-pip python-dev python-serial python-gps gpsd gpsd-clients
2. Python packages: dronekit dronekit-sitl xbee numpy pyzmq

---

### How to use
#### Field test and script execution
Assume one has already configured the onboard companion computer to auto-login on boot, and auto-run the `onboard.py` script with certain arguments. A basic running script command may look like this:
```shell
python onboard.py -id 01 -alt 25 -a MPC -xbee /dev/ttyUSB0 -pix /dev/ttyAMA0
```
Note some of the arguments have their default values and can be omitted. Use `--help` or refer to the source code docstrings for detailed script arguments.
    
For a safe field test, one shall find a clear test field, clear away unnecessary personnel and follow these steps:
1. Turn on the RC transmitter.
2. Connect battery to the copter, wait for the Pixhawk to perform pre-flight check.
3. Wait until satellites number fit the requirement, preferably over 10 satellites or HDOP <= 1.4 .
4. (Optional) One may try to arm the copter in Loiter mode. If it is possible to arm, than the GNSS signal is good enough.
5. Connect power cable to the RPi and the script will auto-run on boot.
6. Turn on the laptop, plug in the GNSS modules and XBee and run `gcs.py`. Some specific steps are necessary for a correct recognition of the GNSS module, see the GCS section of this file for more details.
7. Follow the takeoff sequence in the script. Set HOME_ORIGIN, send it, and then takeoff in GUIDED mode after all the copters echo "standby".
8. (__CRITICAL!__) Make sure to move the throttle stick to 50%!
    When there's mode change or in emergency takeover, a 0% throttle has the risk of letting the copter free-fall!
9. (Optional) Broadcast the rendezvous coordinates if needed.
10. Send `LAND` command or `RTL` command through the GCS to call the birds back.
    Alternatively, one may us the RC transmitter switch to manually override the flight mode, the script will detect the external signal and respond to it by killing the high-level control thread. As mentioned in step 8, make sure the throttle stick is around 50%!

#### GCS startup steps
The program uses `gpsd` for monitoring the GNSS data at background, and we found out some compatibility issue with the gpsd when plugging in new devices. Follow these steps to test if it runs correctly.
1. Turn on the computer
2. Connect the linux computer with the Xbee and GNSS module. Connect the GNSS module first because gpsd will grab it automatically. Connect the XBee module afterwards. Double check the USB ports use `lsusb` if needed.
3. (Optional) Reset gpsd service: (1) First use `cgps` to see if the GNSS data are correctly fed. If not, `sudo sh doc/shell/gpsd_reset.sh` (2) After reset, use `cgps` to check if the data flow is correct.
4. Run the `gcs.py` script by `sudo python gcs.py -a MPC`. Use `--help` or refer to the source code docstrings for detailed script arguments.
5. The console will start prompting some information. Input keys accordingly to execute certain operations.
    Keylist:
    * `'s'`: set current location as HOME_ORIGIN.
    * `'o'`: broadcast HOME_ORIGIN to all the drones in the same network.
    * `'e'`: exit command, cancel takeoff.
    * `'b'/'l'/'r'`: takeoff/land/return-to-launch command.
    * `'t'`: send current location as the rendezvous point to all drones.
    * `'p'`: update algorithm parameter to the drones.
    * `ENTER`: show the keylist.

#### SITL simulation
The `fw` folder contains some pre-compiled firmwares and the `default_eeprom.bin` for a flocking test simulation with five drones. Execute the multi-drone simulation test by:
1. `sh doc/shell/sitl_multirun.sh`.
2. Open another terminal, execute the `gcs-sitl.py` by `python gcs-sitl.py -a MPC`.
3. Open the third terminal, execute QGroundControl and connect the localhost TCP ports `5762/6762/7762/8762/9762`.

The SITL simulation in this project uses [ZeroMQ](zeromq.org/) to establish a publisher-subscriber model and simulates the XBee network. Alternatively, one can plug in real XBee modules and specify the USB ports.

__Note:__ The dronekit-sitl will throw a `OSError: [Errno 13] Permission denied` exception if the binaries is uploaded in Windows and pulled from github in Linux. Same will occur if using zip files.

---

### Compile the ArduPilot firmware and prepare the EEPROM
For SITL simulation with multiple drones, the firmware should be compiled with different TCP ports. Also, the parameters of the ArduPilot firmware should be edited to pass the pre-flight checks.

#### Building the firmware
1. Install git and clone the repository.
```shell
>> git clone https://github.com/ardupilot/ardupilot
>> git submodule update --recursive
```
2. Checkout to the desired branch (List available tags by `git tag`).
```shell
>> git checkout Copter-3.5.2
```
3. (__CRITICAL!__) Modify the TCP ports accordingly. Once built/compiled, the TCP ports for SITLare hard-coded in the firmware. Each simulated copter has its own firmware binary, coded with its assigned TCP ports.

a. Open the file `ardupilot/libraries/AP_HAL_SITL/SITL_cmdline.cpp` and locate:
```c++
const int BASE_PORT = 5760;
const int RCIN_PORT = 5501;
const int RCOUT_PORT = 5502;
const int FG_VIEW_PORT = 5503;

const int SIM_IN_PORT = 9003;
const int SIM_OUT_PORT = 9002;
const int IRLOCK_PORT = 9005;
```
b. Mmodify these ports if necessary without overlapping.

4. (__CRITICAL!__) Modify MAVlink system ID

a. Open the file `ardupilot/ArduCopter/config.h` and locate:
```c++
#ifndef MAV_SYSTEM_ID
   # define MAV_SYSTEM_ID          1
#endif 
```
b. Modify the macro for different firmware compilation. This macro `MAV_SYSTEM_ID` is copied into parameter `SYSID_THISMAV` in the firmware as the default value. Make sure each built firmware has a unique system ID so that the GCS can recognize them as different vehicles.

5. Configure for WAF building.
```shell
>> cd ardupilot
>> ./waf clean
>> ./waf distclean
>> ./waf configure
>> ./waf configure --board=sitl
>> ./waf --targets=bin/arducopter 
```
Use `>> ./waf list` to see available targets.
6. After the build completed, locate the firmware binaries in `ardupilot/build/sitl/bin/` and copy the compiled firmware to the folder containing the firmwares and rename the file with suffix "portABCD" where "ABCD" is the four digit TCP port (e.g. "ac3.5.2_port6760"). This naming rule is coded in the script with regular expressions. One may change to another naming convention and the ports should be consistent with the firmware before building.
7. Prepare the default eeprom binaries and put it under the same folder with the firmwares. See next section for details.

#### Changing SITL parameters and prepare the EEPROM
During SITL, one may want to load the parameters from an actual copter, or modify certain "default" ones.
Because the SITL uses simulated vehicles, not actual boards, the parameters are saved from an actual quadcopter in the Mission Planner, the accelerometers and magnetometers will be 'inconsistent' since the offsets and scaling factors compensate the real-world errors while there are no errors in the SITL. A real-world calibration is viewed as "imperfect" in the SITL. 

Before loding the parameters, manually edit the parameter file and make sure the 'compass' and 'accelerometers' are consistent, i.e.:
`INS_ACCOFFS_X   0.078671` <== equal in value ==> `INS_ACC2OFFS_X   0.078671`
For all the connected accelerometers and compasses and all their axes, do the same.

(__CRITICAL!__) For making the `default_eeprom.bin` for SITL (as will be described next) with multiple copters, delete `SYSID_THISMAV` to prevent from it affecting other drones (use the `SYSID_THISMAV` value built inside the firmwares) as all the firmwares under the same folder share one `default_eeprom.bin`, as is hard-coded in `dronekit-sitl`. Also, disable the battery and current sensor to prevent battery warning in the SITL.

Dronekit-SITL will load `default_eeprom.bin` each time it's launched. Changes made between simulations are discarded. Natually, one may want to preserve the parameters in the EEPROM. In the ArduPilot SITL, this is done through MAVProxy, and the parameters are 
saved in the simulated `eeprom.bin`, located in the temp directory `<FILESYSTEM>/tmp/`. Dronekit-SITL is also compatible with this method.

1. Reboot your Linux system and clear the tmp files. In Ubuntu, the tmp files are automatically cleared on boot.
2. Run the dronekit-sitl in terminal and specify the corresponding firmware. 
```shell   
>> dronekit-sitl ./fw/ac3.5.2_port5760
```
3. Open another terminal and spawn MAVProxy to connect the dronekit-sitl instantce (ports may vary depending on the compiled firmware).
```shell
>> mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
```
If there is no `default_eeprom.bin` when running the dronekit-sitl, there should be some PreArm warnings like "RC not calibrated" "ACC not calibrated". This is the correct behavior, since there's no EEPROM and the parameters are all "clean".
        
4. Load the parameters from a file and overwrite the `eeprom.bin` use `param load <FILE_PATH>/<NAME>.parm` and the warnings should be cleared.
5. After the parameters are loaded, search in the filesystem `eeprom.bin` in `<File System>/tmp/`. Find the latest accessed `eeprom.bin` and copy it to the folder that holds the compiled firmwares, and rename it to `default_eeprom.bin` (this name is hard-coded in dronekit-sitl).
6. Kill all MAVProxy and dronekit-sitl, reboot the system and repeat steps 2 and 3. There should not be any PreArm warnings before takeoff. Try showing some of the parameters and check if they are consistent with the parameter list by `>> param show <PARAM_NAME>`.
    
__Note:__ For multi copter situations with all the compiled firmwares in the same folder, they share the same `default_eeprom.bin`. If one need to prepare different parameters for different drones, isolate them in different folders.

