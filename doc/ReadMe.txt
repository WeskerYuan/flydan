# Readme file.

# This text file  contains the system requirement of the multi-drone control
# project and some setup tutorials.

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


I. Run the onboard script on an multi-copter drone. ===========================
i. Hardware Requirement 
    Components:
    1. A multi-copter using Pixhawk as its flight controller.
    2. A mini linux companion computer. (e.g. Raspberry Pi)
    3. An XBee module with a USB adapter. (e.g. XBee S1, XBee S2C, etc.)
        Note: Zigbee's are not recommended as they are relatively slow and have
            small data throughput volume. Zigbee modules tend to get stuck often.
            The XBee Pro S1 with DIJI Mesh firmware is tested to be working very
            well. A new hardware upgrade by DIJI unifies XBee and Zigbee to `S2C`
            version, which are now compatible across all the DIJI product lines.
    4. (Optional) A USB-TTL adapter (e.g. FT232, CP2102/CP2104, do not use PL2303)
    5. (Optional) A 3-pin twisted UART cable.
    
    Connection (RaspberryPi 3B as the companion computer)£º
    1. XBee (base dock) -- (USB cable) ------------> RaspberryPi-USB (/dev/ttyUSB0)
    2. Pixhawk (Telem2) -- (3-pin twisted cable) --> RaspberryPi-UART (/dev/ttyACM0)
    3. Power-supply for the copter and RaspberryPi.
        Note: The Pixhawk's Serial 2 should be reconfigured to run at 921600bps
            baudrate. The RaspberryPi should enable it's hardware-UART. 
            This configuration may vary, denpending on the LinuxOS used on the Rpi.
        Ref: http://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html
            Appendix I. Enable UART feature on the RaspberryPi.
        
    
    Configuration:
    1. XBee should work under one-to-multipoint broadcast mode. Baudrate 57600bps.
        The XBees are recommended to flash the DigiMesh firmware. Every one of the-Loop
        XBees should have the same PAN ID. One can run some benchmark test to choose
        the channel with the best performance (i.e. better reception, less noise, etc.)

        Note: By the time this file is written, XBee Pro S1 modules seem to have
            a hardware bug that they tend to get stuck when running at 115200bps.
            A newer hardware version (i.e. XBee S2C Pro) may have resolved this.
            
    2. RaspberryPi should enable the hardware-UART. 
    
ii. Software Environment
    1. Ubuntu MATE 16.04 for ARM devices. (compatible with RaspberryPi 3B)
    2. Python 2.7 environment.
    3. Python packages: 
        For actual onboard: dronekit, xbee, numpy
        For SITL simulation: dronekit, dronekit-sitl, xbee, numpy, pyzmq
        
        Installation: use `pip` package manager to install these pacakges.
            >> sudo apt update
            >> sudo apt upgrade
            >> sudo apt install python-pip python-dev
            >> sudo pip install pip dronekit dronekit-sitl xbee numpy gps pyzmq --upgrade
    
    4. Linux packages: python-serial
       Installation: use `apt` package manager to install these pacakges.
            >> sudo apt install python-serial

iii. Field test and script execution
    Assume you have already configured your onboard companion computer to auto-login
    on boot, and automatically runs the onboard.py script with certain arguments.
    
    A basic running script command may look like this:
        >> python onboard.py -id 01 -alt 25 -a MPC -xbee /dev/ttyUSB0 -pix /dev/ttyAMA0
    Note some of the arguments have their default values and can be omitted.
    
    See Appendix IV. and Appendix V. for configuring auto-login and auto-run.
    Refer to the source code docstrings for detailed script arguments.
    
    For a safe and sound fiedtest, one shall follow these steps:
    0. Find a clear test field, clear away unnecessary personnel.
    1. Turn on the RC transmitter.
    2. Connect battery to the copter, wait for the Pixhawk to perform pre-flight check.
    3. Wait until satellites number fit the requirement, preferably over 10
        satellites or HDOP <= 1.4
    4. (Optional) You may try to arm the copter in Loiter mode. If it is possible 
        to arm, than the GNSS signal is good enough.
    5. Connect power cable to the RPi and the script will auto-run on boot.


    7. Turn on the laptop, plug in the GNSS modules and XBee and run the GCS 
        program. Some specific steps are necessary for a correct recognition of 
        the GNSS module (due to the gpsd), see the GCS section of this file for 
        more details.
    8. Follow the takeoff sequence in the script. Set HOME_ORIGIN, send it, and 
        then takeoff in GUIDED mode after all the copters echo `standby`.
    9. (CRITICAL!) Make sure to move your throttle stick to 50%£¡£¡£¡
        When there's mode change or in emergency takeover, a 0% throttle has 
        a risk letting the copter free-fall! (Painful lesson!)
    9. (Optional) Broadcast the rendezvous coordinates if needed.
    10. Send `Land` command or `RTL` command through the GCS to call the birds 
        back.
        Alternatively, you can us the RC transmitter switch to manually override 
        the flight mode to RTL, the script will detect the external signal and 
        respond to it by killing the high-level control thread. As mentioned in 
        step 9, make sure the throttle stick is around 50%!
    
    NOTE: For extra safety, assign different flight level of each drone to prevent
        mid-air collision!

II. Run the ground station script on a computer. ==============================
i. Hardware Requirement 
    Components:
    1. A linux laptop. (virtual machine is okay)
    2. A GNSS module. If the GNSS module is identified as an ACM device 
        (e.g./dev/ttyACM0), you will need to do a walkaround method. If it is 
        identified as a standard USB device, like /dev/ttyUSB0, then nothing
        should be done. See Appendix II. for more details.
    3. An XBee module with a USB adapter. (e.g. XBee S1, XBee S2C, etc.)

    Connection£º
    XBee <-- (USB) --> Linux computer <-- (USB) --> GNSS module
    
    Configuration:
    1. XBee should work under one-to-multipoint broadcast mode. Baudrate 57600bps.
        The XBees are recommended to flash the DigiMesh firmware. Every one of the-Loop
        XBees should have the same PAN ID.           
    2. RaspberryPi should enable the hardware-UART. 
    3. IMPORTANT: configure your gpsd default behavior --> Appendix II.
    
ii. Software Environment
    1. Ubuntu MATE 16.04 for ARM devices. (compatible with RaspberryPi 3B)
    2. Python 2.7 environment.
    3. Python packages: 
        For actual groundstation: dronekit, xbee, numpy, gps
        For SITL simulation: dronekit, dronekit-sitl, xbee, numpy, pyzmq
    4. Linux packages: python-gps, python-serial, gpsd, gpsd-clients 
    
iii. Script Execution
    1. Turn on the computer.
    2. Connect your linux computer with the Xbee and GNSS module. Connect the 
        GNSS module first because gpsd will grab it automatically. Connect the
        XBee module afterwards. Double check the USB ports if needed.
            >> lsusb
            >> ls /dev/ttyUSB0   # if identified as USB devices
            >> ls /dev/ttyUSB1
            >> ls /dev/ttyACM0   # if identified as ACM devices
        
    3. (Optional) Reset gpsd service:
        First use `>> cgps` to see if the GNSS data are correctly fed. If not:
            >> sudo sh gpsd_reset.sh
        After reset, use cgps to check if the data flow is correct.
            >> cgps
       
    4. Run the gcs.py script: 
            >> sudo python gcs.py -a MPC
        Refer to the source code docstrings for detailed script arguments.

    5. The console will start prompting some information. 
        Type accordingly to execute certain operations.

III. Run the SITL simulation script on a computer. ============================
i. Requirement 
    1. A linux laptop. (virtual machine is okay, but must be x86 compatible)
    2. Dronekit-SITL (other native SITLs are also possible)
    3. Dronekit simulation codes
    4. ArduPilot source code and eeprom binaries for the Dronekit-SITL
    
ii. Build ArduCopter firmware 
     Ref: https://github.com/ArduPilot/ardupilot
          https://github.com/ArduPilot/ardupilot/pull/4883
    
    1. Install git and clone the repository
        >> sudo apt install git
        >> git clone https://github.com/ardupilot/ardupilot
        >> git submodule update --recursive
    
    2. Checkout to the desired branch (e.g. Copter-3.5.2)
        >> git checkout Copter-3.5.2
        (You can list the tags by "git tag")

    3. (CRITICAL) Modify the TCP ports accordingly.
        Once built/compiled, the TCP ports for SITLare hard-coded in the firmware.
        The following step enables the use of multiple copters in swarming or 
        flocking algorithms by assigning different TCP ports.
        Each simulated copter has a firmware binary, coded with its assigned TCP ports.
       
        Open the file or use nano:
            <Parent Folder>/ardupilot/libraries/AP_HAL_SITL/SITL_cmdline.cpp
        
        Locate:
            const int BASE_PORT = 5760;
            const int RCIN_PORT = 5501;
            const int RCOUT_PORT = 5502;
            const int FG_VIEW_PORT = 5503;

            const int SIM_IN_PORT = 9003;
            const int SIM_OUT_PORT = 9002;
            const int IRLOCK_PORT = 9005;
        
        and modify these ports if necessary without overlapping.

    4. (CRITICAL) Modify Mavlink system ID
        Open the file or use nano:
            <Parent Folder>/ardupilot/ArduCopter/config.h
         
        Locate:
            #ifndef MAV_SYSTEM_ID
               # define MAV_SYSTEM_ID          1
            #endif
         and modify the macro for different firmware compilation.
         
        This macro MAV_SYSTEM_ID is copied into parameter SYSID_THISMAV in the 
        firmware as the default value. Make sure each built firmware a unique 
        system ID so that the GCS can recognize them as different vehicles.
         
    4. Configure for WAF building (ArduPilot is now using WAF instead of make)
        >> cd <Parent Folder>/ardupilot/
        >> ./waf clean
        >> ./waf distclean
        >> ./waf configure
        >> ./waf configure --board=sitl
        >> ./waf --targets=bin/arducopter (>> ./waf list to see available targets)
            OR
        >> ./waf copter (this will build all targets under copter)

    5. After the build completed, locate the firmware binaries in
        <Parent Folder>/ardupilot/build/sitl/bin/
        Copy the compiled firmware to the folder containing firmwares
        and rename the file with suffix "portABCD" where "ABCD" is the four 
        digit TCP port (e.g. "ac3.5.2_port6760")
        Note this naming rule is coded in the script with regular expressions.
        Adapt your own code if you use another naming convention. The ports 
        should be consistent with the firmware before building.
       
        Also, the default eeprom binaries should also be copied under this
        folder, see the following sections.

iii. SITL Parameters
    During SITL, one may want to load the parameters from an actual copter, 
    or modify certain 'default' ones.
    
    Because the SITL uses simulated vehicles, not actual boards, if you save the 
    parameters from an actual quadcopter in Mission Planner, the accelerometers 
    and magnetometers will be 'inconsistent' since the offsets and scaling 
    factors compensate the real-world errors while there are no errors in the 
    SITL, everything is perfectly ideal. 

    To deal with this problem, you should manually edit the parameter file and 
    make sure the 'compass' and 'accelerometers' are consistent, i.e.:
        INS_ACCOFFS_X   0.078671 <== equal value ==> INS_ACC2OFFS_X   0.078671
        INS_ACCOFFS_Y  -0.300440                     INS_ACC2OFFS_Y  -0.300440
        INS_ACCOFFS_Z  -0.877461                     INS_ACC2OFFS_Z  -0.877461
        INS_ACCSCAL_X   1.004638                     INS_ACC2SCAL_X   1.004638
        INS_ACCSCAL_Y   1.007017                     INS_ACC2SCAL_Y   1.007017
        INS_ACCSCAL_Z   0.982079                     INS_ACC2SCAL_Z   0.982079
        
        The same thing goes for the compasses, i.e.:
        COMPASS_OFS_X  -52.000000 <== equal value ==> COMPASS_OFS2_X  -52.000000
        COMPASS_OFS_Y  -41.000000                     COMPASS_OFS2_Y  -41.000000
        COMPASS_OFS_Z   68.000000                     COMPASS_OFS2_Z   68.000000
        
        If there are a third accelerometer or compass, do the same.
    
    In MAVProxy, load the parameters with the following command:
        >> param load <FILE_PATH>/<NAME>.parm

    Note the *.param saved from Mission Planner and the *.parm by MAVProxy are both 
    recognized.
    
    (CRITICAL) For making the default_eeprom.bin for SITL (as will be described 
    next) with multiple copters, delete SYSID_THISMAV to prevent from it affecting 
    other drones (because the SYSID_THISMAV is built inside the firmwares).
    Also, disable the battery and current sensor to prevent battery warning in 
    the SITL.
    
iv. Make the default_eeprom.bin of the SITL firmware
    The Dronekit-SITL will load default_eeprom.bin each time it's launched. So the 
    changes made between simulations are discarded.
    
    Natually, one may want to preserve the parameters in the eeprom. In the 
    ardupilot SITL, this is done through MAVProxy, and the parameters are 
    saved in the simulated eeprom.bin, located in the temp directory
    <FILESYSTEM>/tmp/. Dronekit-SITL is also compatible with this method.

        a. Reboot your Linux system and clear the tmp files. In Ubuntu, the tmp files 
           are automatically cleared on boot.
        b. Run the dronekit-sitl in terminal and specify the corresponding firmware. 
            e.g.    
                >> dronekit-sitl ./<FIRMWARE_PATH>/ac3.5.2_port5760
        c.  Open another terminal and spawn MAVProxy to connect the dronekit-sitl instantce
                >> mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 
                    --out 127.0.0.1:14550 --out 127.0.0.1:14551
                NOTE: the ports may vary depending on the compiled firmware
            
            If there is no default_eeprom.bin when running the dronekit-sitl, you should
            see some PreArm warnings like "RC not calibrated" "ACC not calibrated".
            This is the correct behavior, since there's no eeprom and the parameters
            are all 'clean'.
            
        d.  Load the parameters from a file and overwrite the eeprom.bin use:
                >> param load <FILE_PATH>/<NAME>.parm  (or <NAME>.param)
            
            You may save the parameters to your real copter from Mission Planner.
            Need to modify the file according to section III.iii.
            
            Make sure the warnings are out and you can try taking the copter off 
            ground in MAVProxy¡£
            
        e.  After the parameters are loaded, use the search function in the 
            filesystem to search for "eeprom.bin" in <File System>/tmp.
            Find the latest accessed "eeprom.bin" and copy it to the folder
            that holds the compiled firmwares, and rename it to 'default_eeprom.bin'.
            (this name is hard-coded in dronekit-sitl)
            
        f.  Kill all MAVProxy and dronekit-sitl, reboot your system and repeat step ii. and iii.
            You should not see any PreArm warnings before takeoff. 
            You can try showing some of the parameters and check if they are consistent 
            with the parameter file:
                >> param show  INS_ACCSCAL_Z
        
        Note: For multi copter situations with all the compiled firmwares in the 
            same folder, they share the same `default_eeprom.bin`. 
            If you need to prepare different parameters for different drones, 
            isolate them in different folders.
        
iv. Execute SITL script and QGC
    1. run your script
    Code the SITL using Dronekit-SITL API, as is given in the script.
    Execute by passing in the compiled firmware path. 
        >> sudo python sitl.py -id 01 -pix fw/ac3.5.2_port6760
   
    See --help or source code for details.
    Multiple drone simulation can be initialzied using a simple shell script to
    prevent excessive human labor.
    An example is provided in `doc/shell/sitl_multirun.sh`.
    
    2. open QGC (for installation and running, see: http://qgroundcontrol.org/)
       and set comm links as the TCP ports with offset 2 or 3
       e.g. the SITL base port is 120.7.0.0.1:6760;
            the dronekit-python API instantiates a Vehicle instance and is 
            connected to the SITL via port 6760;
            the SITL still has port 6762 and 6763 available as serial 2 and serial 3
            which you can connect the QGC or other supported groundstations like MAVproxy
 
    If you want to use actual Xbee modules, provide correct port path to args.xbee;
    by default, the SITL script uses ZeroMQ simulated subscriber/publisher.
   
=============================== Appendix =======================================
Appendix I. Enable UART feature on the RaspberryPi.
    On RaspberryPi 3B, the hardware UART is by default configured for the Bluetooth 
    and a CPU core-clock-based miniUART is configured for GPIOs.
    Unfortunately, the CPU core-clock is varying when the Rpi is running, which 
    leads to a broken GPIO UART.

    The way out of this situation is to stabilize the core-clock frequency by 
    forcing a constant GPU clock (core_freq), because the GPU clock also handles
    the L1 cache, responsible for the SDRAM.
    Ref: https://frillip.com/raspberry-pi-3-uart-baud-rate-workaround/
    
    Ubuntu MATE 16.04 LTS is used as an example because it provides the full
    config.txt. For other OS, your steps may vary.
    
    1. Edit 'FILESYSTEM/boot/config.txt'. Find the commented block reffering to
        GPU clock, and uncomment the sentence "#core_freq = 250" to force the core-clock
        to be at 250MHz. It is okay because the default frequency is 250MHz.
        Or, you can simply add a new line at the bottom of the <config.txt> file.
    
            core_freq = 250
     
        For Ubuntu MATE 16.04LTS, the bluetooth works fine, and occupies the UART 
        by default, so you have to first disable it. 
        Ref:https://openenergymonitor.org/forum-archive/node/12311.html
        
        To do so:
            >> sudo nano /boot/config.txt
        add this line at the bottom
            dtoverlay=pi3-disable-bt
        save and exit, and stop the bluetooth modem by:
            >> sudo systemctl disable hciuart
        reboot and the UART will be available.
        
        To use the UART as communication interface, 
            delete"console=ttyAMA0,115200 kgdboc=ttyAMA0,115200" in `boot/cmdline.txt`
            
        To boost the UART baudrate and clock, modify /boot/config.txt
        change init_uart_clock to 16MHz, and init_uart_baudrate to 921600
        
        To use the UART as a terminal console, see the next step.
        
    2. Edit 'FILESYSTEM/boot/cmdline.txt' and add the two command to register 
       console to the serial port:
        
        The file should look like this by default (or similar):
            dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait
         
         What you need to do is add two script into the boot command£º
            console=ttyAMA0,115200 kgdboc=ttyAMA0,115200

         When you're done, the file "cmdline.txt" should look like this:
            dwc_otg.lpm_enable=0 console=ttyAMA0,115200 kgdboc=ttyAMA0,115200 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline rootwait

         Save and exit "cmdline.txt".
        
        Use a USB-TTL adapter and a terminal program (e.g. TeraTerm) to use the 
        UART as a terminal.
 
Appendix II. Modify gpsd default behaviors.
    How to utilize GNSS modules that are IDed as ACM devices for the gpsd:

    If you are using a virtual USB COM device, the GNSS module is identified as 
    an ACM device, like /dev/ttyACM0. 
    If you are using USB-TTL adapter, the adapter should be identified as 
    a standard USB port, like /dev/ttyUSB0.
  
    Because the gpsd package is poorly coded, with a lot of bugs, it automatically
    grabs the "/dev/ttyUSB0" as the default device. When using other USB ports
    or ACM devices, it often fails. Currently the way to walkaround is to modify 
    '/etc/default/gpsd' and change GPSD_OPTIONS="/dev/ttyXXX" 
    (by default ttyACM0, if using USB virtual COM GNSS modules; 
    if using USB-TTL adapters, it should be ttyUSBx)

    By default, if your GNSS module is using USB-TTL adapter, and is IDed as
    a standard USB device, than the gpsd works just fine. If your GNSS modules 
    are IDed as an ACM device, then the gpsd won't work properly without 
    appropriate settings.
    
    To do so:
    1. >> sudo nano /etc/default/gpsd   
        # nano may be firstly installed by  >> sudo apt install nano

    2. change the GPSD_OPTIONS='' to GPSD_OPTIONS='/dev/ttyXXX'
       where XXX should be your device port, either USB0 or ACM0, depending on
       your situation.
       
    3. Ctrl + O to save and Ctrl + X to exit.

    By default, the gpsd automatically runs after boot-up, and it grabs the USB
    port by itself, even before you know it. To correctly bind your GNSS device to 
    the gpsd, you'll need to manually reset it by the following steps:
        >> sudo killall gpsd             # kill gpsd
        >> sudo rm /var/run/gpsd.sock    # clean up socket files
        >> sudo /etc/init.d/gpsd restart # restart, (assuming you had already modified /etc/default/gpsd)

    A standalone shell script is provided at the project folder as <gpsd_reset.sh>
    You can run it with: >> sudo sh doc/shell/gpsd_reset.sh 

Appendix III. XBee configuration referece.
    Xbee Module configuration specification (modified only):
    Operating Channel:      CH = 11
    Network ID:             ID = 3333
    Unicast Retries:        RR = 0
    Routing Mode:           CE = End Device[2]
    Broadcast Hops:         BH = 1 (for follower agents)    BH = 0 (maximum hop, for groundstation)
    Node Identifier:        NI = <string>
    Baudrate:               BD = 57600  (115200 will stuck. it's a hardware glitch for Xbee S1 series)
    API mode:               AP = API mode with escapes [2]
    DIO6/RTS:               D6 = RTS flow control [1]

Appendix IV. Overclocking the RaspberryPi 3B and enable autologin & SSH.
    i. overclocking
        Edit /boot/config.txt and find the overclocking section, adjust these values

        force_turbo = 1
        arm_freq = 1200
        core_freq = 600
        sdram_freq = 500
        over_voltage = 6  # which is 1.35V
        
        Save and exit, reboot.
        WARNING: Make sure the cooling system is good!! 

    ii. autologin (autologin through CLI, when there's no GUI)
        Edit /lib/systemd/system/getty@.service and modify the following line:
        
        [Service]
        ExecStart=-/sbin/agetty -a <USERNAME> --noclear %I $TERM
        
        add -a <USERNAME> where <USERNAME> is your account name.
        Save and exit, reboot.

        WARNING: The file getty@.service may be overwritten by package upgrade! 
            Check after each update.

    iii. SSH
        openssh-server is installed by default on Ubuntu MATE for Rpi3.
        What you need to do is bind a static IP in your wireless router
        and simply use terminal software such as teraterm to login.

        WARNING: The file getty@.service may be overwritten by package upgrade! 
            Check after each update.

Appendix V. Configure auto-run on boot. 
    Configure auto-run feature of the Ubuntu 16.04 LTS
    i.  make sure you enabled auto-login
    ii. simply put the shell script you wanna run under the directory /etc/profile.d
    Note in some cases, echo password when using sudo command is necessary.

