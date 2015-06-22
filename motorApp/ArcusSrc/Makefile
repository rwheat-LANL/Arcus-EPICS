# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = arcusMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devArcusMotor.dbd

INC += arcusMotorDriver.h

# The following are compiled and added to the Support library
arcusMotor_SRCS += arcusMotorDriver.cpp

arcusMotor_LIBS += motor
arcusMotor_LIBS += asyn
arcusMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

