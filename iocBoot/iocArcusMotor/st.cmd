# Tell EPICS all about the record types, device-support modules, drivers, etc.
epicsEnvSet("TOP","/home/rwheat/epics/modules/motorR6-8")
epicsEnvSet("MOTOR","/home/rwheat/epics/modules/motorR6-8")
dbLoadDatabase("../../dbd/ArcusMotor.dbd")
ArcusMotor_registerRecordDeviceDriver(pdbbase)

### Motors
# Motors substitutions, customize this for your motor
dbLoadTemplate "motor.substitutions"

# Configure the port
# drvAsynIPPortConfigure("Ether","127.0.0.1:5001",0,0,0)

# For the PMX-4ET-SA...
# drvAsynIPPortConfigure("Ether","172.18.4.103:5001",0,0,0)

# For the DMX-ETH that I have, there's no EOS, so, set the noProcessEosIn flag
# to '1' to keep from timing out with every read.
# drvAsynIPPortConfigure("Ether","172.18.4.250:5001",0,0,1)

# For the DMX-K-SA-17/23 on RS-232, using my USB to Serial adapter.
drvAsynSerialPortConfigure("Ether", "/dev/ttyUSB0", 0, 0, 0)
asynSetOption("Ether", -1, "baud", "9600")
asynSetOption("Ether", -1, "bits", "8")
asynSetOption("Ether", -1, "parity", "none")
asynSetOption("Ether", -1, "stop", "1")
#asynSetOption("Ether", -1, "clocal", "Y")
#asynSetOption("Ether", -1, "crtscts", "N")

asynOctetSetInputEos("Ether",0,"\r")
asynOctetSetOutputEos("Ether",0,"\r")
#asynInterposeEosConfig("Ether",0,0,0)

# For debugging output uncomment the next 2 lines.
#asynSetTraceIOMask("Ether", 0, 2)
#asynSetTraceMask("Ether", 0, 9)
### asynSetTraceMask("Ether", 0, 25)

# var drvArcusMotordebug 4

# Controller port, asyn port, number of axis, moving poll period, idle poll period, Arcus Controller Flag (0=Normal, 1=RS-485 Style)
# arcusCreateController(const char *motorPortName, const char *ioPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
arcusCreateController("P0", "Ether", 1, 0.050, 2.0, 1)

# Controller port, axis letter, controller channel
# arcusCreateAxis(const char *motorPortName, int axisNumber, int channel)
# arcusCreateAxis("P0", 0, 1);
arcusCreateAxis("P0", 0, 0);

iocInit()

