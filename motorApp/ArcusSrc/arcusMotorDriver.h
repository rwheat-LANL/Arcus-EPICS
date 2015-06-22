/*************************************************************************\
* Copyright (c) 2015, Los Alamos National Laboratory
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#ifndef ARCUS_MOTOR_DRIVER_H
#define ARCUS_MOTOR_DRIVER_H

/* Motor driver support for Arcus Motor Controllers                           */
/* For now, supporting PMX-4ET-SA, DMX-ETH, DMX-K-SA-nn                       */
/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28                  */
/* and                                                                        */
/* Till Straumann <strauman@slac.stanford.edu>, 9/11                          */
/*                                                                            */
/* Robert M. Wheat, Los Alamos National Laboratory (rwheat@lanl.gov)          */
/* 05 April, 2014                                                             */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>

enum arcusExceptionType {
	MCSUnknownError,
	MCSConnectionError,
	MCSCommunicationError,
};

class arcusException : public std::exception {
public:
	arcusException(arcusExceptionType t, const char *fmt, ...);
	arcusException(arcusExceptionType t)
		: t_(t)
		{ str_[0] = 0; }
	arcusException()
		: t_(MCSUnknownError)
		{ str_[0] = 0; }
	arcusException(arcusExceptionType t, const char *fmt, va_list ap);
	arcusExceptionType getType()
		const { return t_; }
	virtual const char *what()
		const throw() {return str_ ;}
protected:
	char str_[100];	
	arcusExceptionType t_;
};


class arcusAxis : public asynMotorAxis
{
public:
	arcusAxis(class arcusController *cnt_p, int axis, int channel);
	asynStatus  poll(bool *moving_p);
	asynStatus  move(double position, int relative, double min_vel, double max_vel, double accel);
	asynStatus  home(double min_vel, double max_vel, double accel, int forwards);
	asynStatus  stop(double acceleration);
	asynStatus  setPosition(double position);
	asynStatus  moveVelocity(double min_vel, double max_vel, double accel);

	/* virtual asynStatus getVal(const char *parm, int *val_p); */
	virtual asynStatus moveCmd(int count);
	//virtual int getClosedLoop();
	int getVel() const { return vel_; }
   asynStatus getAxisStatus(int axis, int *val);
   asynStatus getEncoderVal(int axis, int *val);
   asynStatus getPositionVal(int axis, int *val);

protected:
	asynStatus setSpeed(double velocity);
   asynStatus setSpeed(double velocity, double lowSpeed, double accel);

private:
	arcusController *c_p_;  // pointer to asynMotorController for this axis
	asynStatus  comStatus_;
	int         vel_;
	unsigned    holdTime_;
   int         axis_;
	char        channel_;
   char        Arcus_Com_Prefix[4];

friend class arcusController;
};

/* NOTE: For now, the Arcus Controller Flag tells whether or not to use the   */
/* RS-485 style addressing or not. The RS-485 style addressing requires the   */
/* command to be prefixed with a '@' followed by a number greater than zero   */
/* such as '@01'. This is only for the purposes of determining what kind of   */
/* controller we're dealing with. The axis will figure this out as well.      */
class arcusController : public asynMotorController
{
public:
	arcusController(const char *portName, const char *IOPortName, int numAxes,
       double movingPollPeriod, double idlePollPeriod, int ArcusControllerFlag);
	virtual asynStatus sendCmd(size_t *got_p, char *rep, int len, double timeout,
           const char *cmd, int cmdLen);
	
	static int parseReply(const char *reply, int *ax_p, int *val_p);

   enum ControllerType_t {UNKNOWN, DMX_ETH, PMX_4ET_SA, DMX_K_SA};
   static const char *ControllerTypeStrings[];
   ControllerType_t ArcusModel;

protected:
	arcusAxis **pAxes_;

private:
	asynUser *asynUserMot_p_;
friend class arcusAxis;
};

const char *arcusController::ControllerTypeStrings[] = {"UNKNOWN",
   "DMX-SERIES-ETH", "Performax-4ET-SA", "DriveMax-K-SA"};
   
#endif // _cplusplus
#endif // ARCUS_MOTOR_DRIVER_H
