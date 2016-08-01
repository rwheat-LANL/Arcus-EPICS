# Arcus-EPICS
Arcus motor drivers for EPICS
Driver for Arcus Stepper Motors, PMX-4ET-SA, DMX-ETH, DMX-K-SA-nn
==================================

Robert M. Wheat, Los Alamos National Laboratory (rwheat@lanl.gov)
05 April, 2014

Acknowledgement
***************

This Arcus driver code is based on the work of Till Straumann (SLAC) from 2011,
which was based on code from Mark Rivers. There's no need to be reinventing the
motor, I mean, wheel.

Disclaimer
**********

I am not an expert with the motor record module for EPICS, though I've
implemented it with more than one type of motor. In places I will probably not
use the appropriate terminology, and other places my statement may be
innaccurate or even totally wrong, but I hope my notes will be usable
nonetheless. Please feel free to correct my mistakes, and actually, I'd
appreciate it, since learning is a part of why I do this stuff anyway.

File Locations
**************

For these notes, the root directory is considered to be motorR6-8. On my system,
that location is...
/home/rwheat/epics/modules/motorR6-8

motorApp/ArcusSrc
This directory contains the motor module source code for the supported Arcus
motors. This code, arcusMotorDriver.cpp/h, devArcusMotor.dbd, and Makefile,
should be compiled and the results will be placed in the appropriate directories
for use as/with an IOC. This code should not typically be modified unless there
are bugs to be fixed or additional functionality to be inserted in the driver
itself.

motorExApp/arcusMotorDriver
This directory contains the example application, a simple 'c' program that calls
an IOC shell and then sleeps. When this compiled program is called with a
command line argument of a script file, the IOC is executed using the
appropriate database file, libraries, etc. Compiling the code in this directory
creates the executable and build the appropriate database file for the Arcus
motors. The executable is placed in the bin/linux-x86_64 directory and the dbd
file in the dbd directory.

iocBoot/iocArcusMotor
This directory contains the startup script for starting the IOC, and the
motor.substitutions file, which provides for runtime configuration of certain
motor parameters. To start the IOC from this directory type the following:

../../bin/linux-x86_64/ArcusMotor st.cmd

Further notes as far as what should be placed in the startup script file,
st.cmd, for each supported Arcus motor can be found in that file. Please see the
motor record documentation for more information on the motor.substitutions file.




Introduction
************

This driver supports Arcus motors PMX-4ET-SA, DMX-ETH, DMX-K-SA-nn (at least) by
implementing subclasses of the 'asynMotorController' and 'asynMotorAxis'
objects.

Communication with the hardware is established via 'asyn port' driver layers,
either AsynIP ports or AsynSerial ports depending on the motor and its interface
which is determined by configuration lines in the startup script.

Note: the reader is assumed to be familiar with both the asyn record as well as
the motor record. Please consult relevant documentation.

Restrictions
************

The driver does not support all features of the supported Arcus motors (some of
which are outside of the scope of the motor record). Neither are all features of
the motor record supported.

Limit switches are not supported.



Building an Application
- - - - - - - - - - - -
Building the driver into an application requires

a) the 'asyn', 'motor' and 'arcusMotorDriver' (this driver) packages/modules to
   be built and installed. The application's RELEASE file must point to these
   packages so that the build process locates headers and libraries etc.

b) the application's '.dbd' file must contain
    - motorSupport.dbd
    - devArcusMotor.dbd
   as well as an 'asyn' port driver for serial or Ethernet (IP) communication.
   This would be either
    - drvAsynSerialPort.dbd
   or
    - drvAsynIPPort.dbd

   These '.dbd' files are best listed in the
   application Makefile:
    <app>_DBD += motorSupport.dbd
    <app>_DBD += devSmarActMCSMotor.dbd
    <app>_DBD += drvAsynIPPort.dbd
    (or a variation of the above. See the example Makefile and App in the
    motorExApp/ArcusMotor directory.)

c) the application must be linked against
   the 'arcusMotor', 'motor' and 'asyn'
   libraries, e.g.,

    <app>_LIBS += arcusMotor motor asyn


Driver Run-Time Configuration (see the example st.cmd file in
       iocBoot/iocArcusMotor)
*****************************

For each Arcus controller (some Arcus controllers support multiple motors) a
driver instance needs to be configured (most commonly) from a startup script.
In addition an asyn port driver for serial or ethernet communication must be
created (again, from the startup script). For example:

  drvAsynIPPortConfigure("Ether","172.18.4.103:5001",0,0,0)
  or
  drvAsynSerialPortConfigure("Serial", "/dev/ttyUSB0", 0, 0, 0)

Next, an Arcus controller driver instance is created. The respective call takes
the following form:

arcusCreateController(
     const char *motorPortName,
     const char *ioPortName,
     int         numAxes,
     double      movingPollPeriod,
     double      idlePollPeriod);

  motorPortName: unique string to identify this instance to be used in the
                 motor record's 'OUT' field.
  ioPortName:    asyn port name identifying the serial or Ethernet link for
                 communication with the Arcus controller. Note that such a link
                 must be created prior to creating an Arcus controller driver
                 instance.
  numAxes:       number of axes or motors this Arcus controller supports.
  movingPollPeriod: period (in seconds - since this is  a 'double' parameter
                 fractional seconds are possible) at which the Arcus controller
                 is polled for status changes while the positioner is moving.
  idlePollPeriod: period (in seconds) at which the Arcus controller is polled
                 for status changes while the positioner is stopped.

E.g., to configure a driver with one axis using the ethernet connection 'Ether'
configured in the above example we use

arcusCreateController("myMotor1","Ether",1,0.05,1.0)

This results in a polling interval of 50ms while moving and 1s while idle. The
driver instance is named 'myMotor1' and can be addressed from a motorRecord's
OUT field:

  field(DTYP, "asynMotor")
  field(OUT,  "asyn(myMotor1,0)")

After creating a controller, one or more Arcus axes are created for that
controller.  The respective call takes the following form:

arcusCreateAxis(
        const char *motorPortName,
        int        axisNumber,
        int        channel)
{

 motorPortName: unique string to identify this instance to be used in the motor
                record's 'OUT' field.
 axisNumber:    axis number being created.
 channel:       the channel should be the same as the axis. This is a leftover
                variable from previous implementations of the code but is
                retained for future use.?.?

Call the arcusCreateAxis() function for each axis or motor that needs to be
configured for the given controller.
