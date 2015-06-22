
TOP = ../..
include $(TOP)/configure/CONFIG

PROD_IOC_DEFAULT = ArcusMotor

COMMONDBDS  = base.dbd
COMMONDBDS += motorSupport.dbd

COMMONDBDS += devSoftMotor.dbd
COMMONDBDS += motorSimSupport.dbd

COMMONDBDS += drvAsynSerialPort.dbd
COMMONDBDS += drvAsynIPPort.dbd
COMMONDBDS += busySupport.dbd
COMMONDBDS += devArcusMotor.dbd

DBD += ArcusMotor.dbd
ArcusMotor_DBD += $(COMMONDBDS)
ifdef SNCSEQ
ArcusMotor_DBD += devSequencer.dbd
endif

ArcusMotor_SRCS   += ArcusMotor_registerRecordDeviceDriver.cpp
ArcusMotor_SRCS_DEFAULT += ArcusMotorMain.cpp

COMMONLIBS += softMotor
COMMONLIBS += motorSimSupport
COMMONLIBS += motor
COMMONLIBS += arcusMotor

ArcusMotor_LIBS += $(COMMONLIBS)
ArcusMotor_LIBS += asyn
ifdef SNCSEQ
ArcusMotor_LIBS += seqDev seq pv
endif
ArcusMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

ArcusMotor_LIBS += busy

include $(TOP)/configure/RULES

