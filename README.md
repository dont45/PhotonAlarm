# PhotonAlarm
A Cloud based General Alarm system based upon the Particle Photon.
The design intent is to create a small, inexpensive, flexable system to manage a wide range of sensors / detectors and provide alarm system conditioin management via an iPhone or similar device.

The current hardware in proto-type form and this software is currently running in a beta test as a storage shed alarm.  It currently has three sensors, magnetic door switch, motion detector and smoke detector.

The hardware consists of a Particle Photon, a MCP9808 precision thermometer, a DS2482 i2c to 1-wire interface, a pololu 5 volt regulator, and two status LEDs.  The three sensors are connected via the 1-wire bus, and utilize ds2406 switches connected to the actual sensor devices.

The system communicates with the 'system manager' via Pushover notification messages pushed from the Particle Cloud.  The manager communicates to the alarm system via the Particle Cloud function calls.  Currently these calls are initiated with curl but will be integrated into an iPhone management app.  Critical commands such as Disble, Enable and Alarm-State Acknowledge are securely handled via secrets.  A request for a new random 'secred' is made via the management app.  The secret is sent as an alert to the manager's phone.  This secret is then sent back to the PhotonAlarm along with the desired command.  The command is only performed if the secret matches. The secret is used only once, and times out in a few seconds.  This provides a very secure management platform even from web based messages.

The software utilizes the power of the Photon Particle build system which provides a robust c++ development environment.  The system is designed using several c++ classes and utilizes the c++ standard template library for several critical data structures, e.g. std::list for the sensor list and tripped-sensor list, and std::queue for the outgoing messages.

Currently the configuration is generated into the Photon's eeprom vi a hard-coded sensor list (writeTestConfiguration).  A future version will build a new configuration by 1) detection of a master key (1-wire iButton) at reset,  2) scanning the 1-wire bus for devices and creating the device list, and 3) assignment of name and function for each device detected via the iPhone app.  Master keys are created by having two iButton devices present at reset.  This provides a simple three-step initial (or updated) setup process.  1) Create the master keys, 2) Scan for sensors, and 3) Edit the sensor's name and function.

Currently the code contains heavy use of various debug messages via Serial.print.  These are controlled by a couple of #defines which will ultimately be removed.

Don Thompson
March 6, 2016

