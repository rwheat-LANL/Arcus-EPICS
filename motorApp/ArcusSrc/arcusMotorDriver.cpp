/* ex: set shiftwidth=3 tabstop=3 expandtab: */

/*************************************************************************\
* Copyright (c) 2015, Los Alamos National Laboratory
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

/* Motor driver support for Arcus Motor Controllers.                          */
/* For now, supporting PMX-4ET-SA, DMX-ETH, DMX-K-SA-nn                       */
/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28                  */
/* and                                                                        */
/* Till Straumann <strauman@slac.stanford.edu>, 9/11                          */
/*                                                                            */
/* Robert M. Wheat, Los Alamos National Laboratory (rwheat@lanl.gov)          */
/* 05 April, 2014                                                             */

#include <iocsh.h>

#include <asynCommonSyncIO.h>
#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <arcusMotorDriver.h>
#include <errlog.h>

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

/* Static configuration parameters (compile-time constants) */
//#undef  DEBUG
//#define DEBUG 1

#define CMD_LEN 50
#define REP_LEN 50
#define DEFLT_TIMEOUT 1.00

#define HOLD_FOREVER 60000
#define HOLD_NEVER       0
#define FAR_AWAY     1000000000 /*nm*/

int DEBUG=1;

// Windows does not have rint()
#ifdef _WIN32
double rint(double x)
{
  //middle value point test
  if (ceil(x+0.5) == floor(x+0.5))
  {
    int a = (int)ceil(x);
    if (a%2 == 0) return ceil(x);
    else return floor(x);
  }
  else return floor(x+0.5);
}
#endif
  

/* The asyn motor driver apparently can't cope with exceptions */
#undef  ASYN_CANDO_EXCEPTIONS
/* Define this if exceptions should be thrown and it is OK to abort the       */
/* application */
#undef  DO_THROW_EXCEPTIONS

#if defined(ASYN_CANDO_EXCEPTIONS) || defined(DO_THROW_EXCEPTIONS)
#define THROW_(e) throw e
#else
#define THROW_(e) epicsPrintf("%s\n",e.what());
#endif

enum arcusPMXStatus {
	PMX_Stopped      = 0,
	PMX_Accelerating = 1,
	PMX_Decelerating = 2,
	PMX_Constant_Spd = 4,
	PMX_Alarm_Status = 8,
	PMX_Plus_Limit   = 16,
	PMX_Minus_Limit  = 32,
	PMX_Home_Switch  = 64,
	PMX_Plus_Lim_Err = 128,
   PMX_Minus_Lim_Err = 256,
   PMX_Alarm_Err    = 512
};

enum arcusDMXStatus {
	DMX_Stopped      = 0,
   DMX_Constant_Spd = 1,
	DMX_Accelerating = 2,
	DMX_Decelerating = 4,
	DMX_Home_Switch  = 8,
   DMX_Minus_Limit  = 16,
   DMX_Plus_Limit   = 32,
   DMX_Minus_Lim_Err= 64,
   DMX_Plus_Lim_Err = 128,
	DMX_Latch_In_Stat= 256,
   DMX_Z_Index_Stat = 512,
   DMX_TOC_TO_Stat  = 1024
};

arcusException::arcusException(arcusExceptionType t, const char *fmt, ...)
	: t_(t)
{
va_list ap;
	if ( fmt ) {
		va_start(ap, fmt);
		epicsVsnprintf(str_, sizeof(str_), fmt, ap);
		va_end(ap);
	} else {
		str_[0] = 0;
	}
};

arcusException::arcusException(arcusExceptionType t, const char *fmt, va_list ap)
		: t_(t)
{
	epicsVsnprintf(str_, sizeof(str_), fmt, ap);
}

arcusController::arcusController(const char *portName, const char *IOPortName,
   int numAxes, double movingPollPeriod, double idlePollPeriod,
   int ArcusControllerFlag /* 0=Normal?, 1=RS-485 style */)
	: asynMotorController(portName, numAxes,
   0, // parameters
	0, // interface mask
	0, // interrupt mask
	ASYN_CANBLOCK | ASYN_MULTIDEVICE,
	1, // autoconnect
	0,0) // default priority
	, asynUserMot_p_(0)
{
   asynStatus status;
   char       junk[100];
   size_t     got_junk;
   int        eomReason;
   pAxes_ = (arcusAxis **)(asynMotorController::pAxes_);
   /* Additional var needed to determine the Arus Controller type.            */
   char rbuf[80];
   char wbuf[80];
   size_t outCount, inCount, bufLen;
   
	if (pasynCommonSyncIO->connect(IOPortName, 0, &asynUserCommonMot_p_, NULL)
    || pasynOctetSyncIO->connect(IOPortName, 0, &asynUserMot_p_, NULL)) {
		asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
         "arcusController:arcusController: cannot connect to MCS controller\n");
		THROW_(arcusException(MCSConnectionError,
         "arcusController: unable to connect I/O channel."));
	}

   asynPrint(asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\narcusController: ArcusControllerFlag = %d.\n", ArcusControllerFlag);

	/* Clear the read buffer, just in case.  */                                  
	pasynOctetSyncIO->read(asynUserMot_p_, junk, sizeof(junk), 2.0, &got_junk,
      &eomReason);
	if(got_junk)
   {
		epicsPrintf("arcusController(%s): WARNING: unexpected characters, (%s).\n",
         portName, IOPortName);
	}
   
   /* Determine which flavor Arcus controller we're talking to.               */
   //if(ArcusControllerFlag == 0)
   //{
   //   sprintf(wbuf, "ID\n");
   //   outCount = 2;
   //}
   //else if(ArcusControllerFlag == 1)
   //{
   //   sprintf(wbuf, "@01ID\n");
   //   outCount = 5;
   //}

   sprintf(wbuf, "ID\n");
   outCount = 2;
   bufLen = 80;
   status = pasynOctetSyncIO->writeRead(asynUserMot_p_, wbuf, outCount,
            rbuf, bufLen, DEFLT_TIMEOUT, &outCount, &inCount, &eomReason);

   if(inCount == 0)
   {
      sprintf(wbuf, "@01ID\n");
      outCount = 5;
      bufLen = 80;
      status = pasynOctetSyncIO->writeRead(asynUserMot_p_, wbuf, outCount,
            rbuf, bufLen, DEFLT_TIMEOUT, &outCount, &inCount, &eomReason);
   }

   if(strstr(rbuf, ControllerTypeStrings[1]) != NULL)
   {
      ArcusModel = DMX_ETH;
      //if(DEBUG)
         asynPrint(asynUserMot_p_, ASYN_TRACEIO_DRIVER,
            "\nController Type is %s.\n", ControllerTypeStrings[ArcusModel]);
   }
   else if(strstr(rbuf, ControllerTypeStrings[2]) != NULL)
   {
      ArcusModel = PMX_4ET_SA;
      //if(DEBUG)
         asynPrint(asynUserMot_p_, ASYN_TRACEIO_DRIVER,
            "\nController Type is %s.\n", ControllerTypeStrings[ArcusModel]);
   }
   else if(strstr(rbuf, ControllerTypeStrings[3]) != NULL)
   {
      ArcusModel = DMX_K_SA;
      //if(DEBUG)
         asynPrint(asynUserMot_p_, ASYN_TRACEIO_DRIVER,
            "\nController Type is %s.\n", ControllerTypeStrings[ArcusModel]);
   }
   else
   {
      ArcusModel = UNKNOWN;
      //if(DEBUG)
         asynPrint(asynUserMot_p_, ASYN_TRACEIO_DRIVER,
            "\nController Type is %s.\n", ControllerTypeStrings[ArcusModel]);
   }
   
   /* Save the following config commands for the startup file, st.cmd.        */
	//pasynOctetSyncIO->setInputEos ( asynUserMot_p_, "\r", 1 );
	//pasynOctetSyncIO->setOutputEos( asynUserMot_p_, "\r", 1 );

	startPoller(movingPollPeriod, idlePollPeriod, 0);
}

/* got_p   - Number of bytes read.                                            */
/* rep     - The buffer holding the response.                                 */
/* len     - The length of the response buffer.                               */
/* timeout - Obvious                                                          */
/* cmd     - The command to send, already formatted.                          */
/* cmsLen  - The llength of the command being sent.                           */
asynStatus arcusController::sendCmd(size_t *got_p, char *rep, int len,
    double timeout, const char *cmd, int cmdLen)
{
   //char       buf[CMD_LEN];
   size_t     nwrite;
   int        eomReason;
   asynStatus status;
   int pass = 0;

	//epicsVsnprintf(buf, sizeof(buf), fmt, ap);

   for (;;) {
      status = pasynOctetSyncIO->writeRead(asynUserMot_p_, cmd, cmdLen, rep,
                                    len, timeout, &nwrite, got_p, &eomReason);
      if (status == asynSuccess) break;
      asynPrint(asynUserMot_p_, ASYN_TRACEIO_DRIVER,
               "sendCmd(\"%s\"), status:%d, inCount:%d, pass:%d\n",
                                               cmd, status, (int)*got_p, pass);
      if (++pass == 5) break;
      if (pass > 1) {
         status = pasynCommonSyncIO->disconnectDevice(asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(asynUserMot_p_, ASYN_TRACE_ERROR,
                              "Warning -- unable to disconnect from device\n");
         }
         status = pasynCommonSyncIO->connectDevice(asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(asynUserMot_p_, ASYN_TRACE_ERROR,
                                 "Warning -- unable to reconnect to device\n");
         }
      }
   }

	//asynPrint(c_p_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "sendCmd()=%s", buf);

	return status;
}

/* Parse reply from ARCUS and return the value converted to a number.
 * So far, I don't think this function is used for the ARCUS.
 *
 * If the string cannot be parsed, i.e., is not in the format
 *  ':' , <string_of_upper_case_letters> , <number1> , ',' , <number2>
 * 
 * then the routine returns '-1'.
 *
 * Otherwise, if <string_of_upper_case_letters> starts with 'E'
 * (which means an 'Error' code) then the (always non-negative)
 * error code is returned (may be zero in case of an 'acknowledgement'
 * message in synchronous mode).
 *
 * If the string is parsed successfully then <number2> is passed up
 * in *val_p.
 *
 * Hence - return code nonzero -> error (error code if > 0, parse error
 *                                otherwise)
 *       - return code zero    -> successful ACK or command reply; value
 *                                in *val_p.
 */ 
int arcusController::parseReply(const char *reply, int *ax_p, int *val_p)
{
   char cmd[10];
   
	if ( 3 != sscanf(reply, ":%10[A-Z]%i,%i", cmd, ax_p, val_p) )
		return -1;
	return 'E' == cmd[0] ? *val_p : 0;
}

/* For the Arcus motor controllers, axis 0 corresponds to X, 1-Y, 2-Z, 3-U    */
/* For now, channel means the same thing.                                     */
arcusAxis::arcusAxis(class arcusController *cnt_p, int axis, int channel)
	: asynMotorAxis(cnt_p, axis), c_p_(cnt_p)
{
	int val;
   if(channel == 0)
   	channel_ = 'X';
   else if(channel == 1)
      channel_ = 'Y';
   else if(channel == 2)
      channel_ = 'Z';
   else if(channel == 3)
      channel_ = 'U';
   else
      channel_ = '?';

   if(c_p_->ArcusModel == arcusController::DMX_K_SA)
      sprintf(Arcus_Com_Prefix, "@%02d", channel + 1);
   else
      Arcus_Com_Prefix[0] = 0;
   
   axis_ = axis; /* Need to remember our axis number.                         */
   
	asynPrint(/*c_p_->pasynUserSelf*/c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
             "\narcusAxis::arcusAxis -- creating axis %u\n", axis);

	comStatus_ = getAxisStatus(axis, &val);
   
   asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\narcusAxis: Status of %u returned %i(%d)\n", axis, comStatus_, val);

	if(comStatus_ == 0)
   {
		setIntegerParam(c_p_->motorStatusHasEncoder_, 1);
		setIntegerParam(c_p_->motorStatusGainSupport_, 1);
	}

	callParamCallbacks();

	if(comStatus_)
   {
		THROW_(arcusException(MCSCommunicationError,
         "arcusAxis::arcusAxis -- channel %u ASYN error %i", axis, comStatus_));
	}
}

/* Request the Motor Status from the ARCUS controller. This is really a       */
/* controller function, but each axis should be able to get its own value as  */
/* well. The status values are different between the PMX and DMX controllers  */
/* according to the manuals, but that's not what I see in the lab. More later */
asynStatus arcusAxis::getAxisStatus(int axis, int *val)
{
   asynStatus status;
   char rbuf[80];
   char wbuf[80];
   size_t outCount, inCount, bufLen;
   int eomReason;
   int one, two, three, four;
   int pass = 0;

   /* This command is common to all (so far) Arcus controllers??              */
   sprintf(wbuf, "%sMST", Arcus_Com_Prefix);
   outCount = strlen(wbuf);
   bufLen = 80;
   for (;;) {
      status = pasynOctetSyncIO->writeRead(c_p_->asynUserMot_p_, wbuf, outCount,
                  rbuf, bufLen, DEFLT_TIMEOUT, &outCount, &inCount, &eomReason);
      if (status == asynSuccess) break;
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
               "getAxisStatus: status:%d inCount:%d eomReason:%d\n",
                                             status, (int)inCount, eomReason);
      if (++pass == 5) break;
      if (pass > 1) {
         status = pasynCommonSyncIO->disconnectDevice(c_p_->asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                              "Warning -- unable to disconnect from device\n");
         }
         status = pasynCommonSyncIO->connectDevice(c_p_->asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                                 "Warning -- unable to reconnect to device\n");
         }
      }
   }
   
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\ngetAxisStatus: Status = %d, eomReason = %d.\n", status, eomReason);
   /* For some reason, when the DMX type controllers return a single 0, the   */
   /* writeRead function returns a timeout.?. For now, we'll check for that   */
   /* and just continue as if nothing happened.                               */
   //if((status == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
   //   status = (asynStatus)0;
   
   if(status == 0)
   {
      /* The PMX-4ET-SA is the uniques controller. Deal with it.              */
      if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
      {
         sscanf(rbuf, "%d:%d:%d:%d", &one, &two, &three, &four);
         //printf("arcusAxis: %s, %u.\n", rbuf, (unsigned int)inCount);
         switch(axis)
         {
            case 0:
               *val = one;
               break;
            case 1:
               *val = two;
               break;
            case 2:
               *val = three;
               break;
            case 3:
               *val = four;
               break;
         }
      }
      /* The rest of the supported models respond the same.                   */
      else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      {
         sscanf(rbuf, "%d", &one);
         *val = one;
      }
   }

   return(status);
}

/* Request the encoder value from the controller for given axis.              */
asynStatus arcusAxis::getEncoderVal(int axis, int *val)
{
   asynStatus status;
   char rbuf[80];
   char wbuf[80];
   size_t outCount, inCount, bufLen;
   int eomReason;
   int one, two, three, four;
   int pass = 0;

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
      sprintf(wbuf, "PE"); /* EX for the DMX-ETH. */
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      sprintf(wbuf, "%sEX", Arcus_Com_Prefix);
   outCount = strlen(wbuf);
   bufLen = 80;
   for (;;) {
      status = pasynOctetSyncIO->writeRead(c_p_->asynUserMot_p_, wbuf, outCount,
               rbuf, bufLen, DEFLT_TIMEOUT, &outCount, &inCount, &eomReason);
      if (status == asynSuccess) break;
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                "getEncoderVal: status:%d inCount:%d eomReason:%d pass:%d\n",
                                         status, (int)inCount, eomReason, pass);
      if (++pass == 5) break;
      if (pass > 1) {
         status = pasynCommonSyncIO->disconnectDevice(c_p_->asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                              "Warning -- unable to disconnect from device\n");
         }
         status = pasynCommonSyncIO->connectDevice(c_p_->asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                                 "Warning -- unable to reconnect to device\n");
         }
      }
   }
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\ngetEncoderValue: Status = %d, inCount = %lu.\n", status, inCount);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   //if((status == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA) &&
   //   (inCount > 0))
   //   status = (asynStatus)0;
   
   if(status == 0)
   {
      /* The PMX-4ET-SA is the uniques controller. Deal with it.              */
      if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
      {
         sscanf(rbuf, "%d:%d:%d:%d", &one, &two, &three, &four);
         //printf("arcusAxis: %s, %u.\n", rbuf, (unsigned int)inCount);
         switch(axis)
         {
            case 0:
               *val = one;
               break;
            case 1:
               *val = two;
               break;
            case 2:
               *val = three;
               break;
            case 3:
               *val = four;
               break;
         }
      }
      /* The rest of the supported models respond the same.                   */
      else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      {
         sscanf(rbuf, "%d", &one);
         *val = one;
      }
   }

   return(status);
}

asynStatus arcusAxis::getPositionVal(int axis, int *val)
{
   asynStatus status;
   char rbuf[80];
   char wbuf[80];
   size_t outCount, inCount, bufLen;
   int eomReason;
   int one, two, three, four;
   int pass = 0;

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
      sprintf(wbuf, "PP");
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      sprintf(wbuf, "%sPX", Arcus_Com_Prefix);
   outCount = strlen(wbuf);
   bufLen = 80;
   for (;;) {
      status = pasynOctetSyncIO->writeRead(c_p_->asynUserMot_p_, wbuf, outCount,
               rbuf, bufLen, DEFLT_TIMEOUT, &outCount, &inCount, &eomReason);
      if (status == asynSuccess) break;
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
               "getPositionVal: status:%d inCount:%d eomReason:%d\n",
                                            status, (int)inCount, eomReason);
      if (++pass == 5) break;
      if (pass > 1) {
         status = pasynCommonSyncIO->disconnectDevice(c_p_->asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                              "Warning -- unable to disconnect from device\n");
         }
         status = pasynCommonSyncIO->connectDevice(c_p_->asynUserCommonMot_p_);
         if (status != asynSuccess) {
            asynPrint(c_p_->asynUserMot_p_, ASYN_TRACE_ERROR,
                                 "Warning -- unable to reconnect to device\n");
         }
      }
   }
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   //if((status == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA) &&
   //   (inCount > 0))
   //   status = (asynStatus)0;
   
   if(status == 0)
   {
      /* The PMX-4ET-SA is the uniques controller. Deal with it.              */
      if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
      {
         sscanf(rbuf, "%d:%d:%d:%d", &one, &two, &three, &four);
         //printf("arcusAxis: %s, %u.\n", rbuf, (unsigned int)inCount);
         switch(axis)
         {
            case 0:
               *val = one;
               break;
            case 1:
               *val = two;
               break;
            case 2:
               *val = three;
               break;
            case 3:
               *val = four;
               break;
         }
      }
      /* The rest of the supported models respond the same.                   */
      else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      {
         sscanf(rbuf, "%d", &one);
         *val = one;
      }
   }

   return(status);
}

/* Read a parameter from the ARCUS (nothing to do with asyn's parameter
 * library).
 *
 * parm_cmd: ARCUS command to read parameter
 * val_p:    where to store the value returned by the MCS
 * 
 * RETURNS:  asynError if an error occurred, asynSuccess otherwise.
 */
 /*
asynStatus arcusAxis::getVal(const char *parm_cmd, int *val_p)
{
   char       rep[REP_LEN];
   char       cmd[CMD_LEN];
   size_t     cmd_len, rep_len;
   asynStatus st;
   //int        ax;

	sprintf(cmd, "%s", parm_cmd);
   cmd_len = strlen(cmd);
	st = c_p_->sendCmd(&rep_len, rep, sizeof(rep), 1.0, cmd, cmd_len);
	
	return(st);
	//return c_p_->parseReply(rep, &ax, val_p) ? asynError: asynSuccess;
}
*/

/* Polling for current position, status. For now, check all encoder values.   */
asynStatus arcusAxis::poll(bool *moving_p)
{
   int val;
   enum arcusPMXStatus PMXStatus;
   enum arcusDMXStatus DMXStatus;

	if((comStatus_ = getEncoderVal(axis_, &val)))
   {
		setIntegerParam(c_p_->motorStatusProblem_,    comStatus_ ? 1 : 0 );
	   setIntegerParam(c_p_->motorStatusCommsError_, comStatus_ ? 1 : 0 );
      callParamCallbacks();
   	return(comStatus_);
   }
	setDoubleParam(c_p_->motorEncoderPosition_, (double)val);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\narcusAxis: Encoder value for %c is %d\n", channel_, val);

   if((comStatus_ = getPositionVal(axis_, &val)))
   {
		setIntegerParam(c_p_->motorStatusProblem_,    comStatus_ ? 1 : 0 );
	   setIntegerParam(c_p_->motorStatusCommsError_, comStatus_ ? 1 : 0 );
      callParamCallbacks();
   	return(comStatus_);
   }
	setDoubleParam(c_p_->motorPosition_, (double)val);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\narcusAxis: Position value for %c is %d\n", channel_, val);

   comStatus_ = getAxisStatus(axis_, &val);

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      PMXStatus = (arcusPMXStatus)val;
      switch(PMXStatus)
      {
         default:
            *moving_p = false;
            break;
         case PMX_Accelerating:
         case PMX_Constant_Spd:
         case PMX_Decelerating:
            *moving_p = true;
         break;
      }
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      DMXStatus = (arcusDMXStatus)val;
      switch(DMXStatus)
      {
         default:
            *moving_p = false;
            break;
         case DMX_Accelerating:
         case DMX_Constant_Spd:
         case DMX_Decelerating:
            *moving_p = true;
         break;
      }
   }
   
	setIntegerParam(c_p_->motorStatusDone_, ! *moving_p );

   if(DEBUG)
	   asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\narcusAxis: Status for %u is %d\n", axis_, val);

	callParamCallbacks();

	return comStatus_;
}

asynStatus arcusAxis::moveCmd(int count)
{
   char    rep[REP_LEN];
   char    cmd[CMD_LEN];
   size_t  got, cmdLen;
   double  tout = DEFLT_TIMEOUT;

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   	sprintf(cmd, "%c%d", channel_, count);
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      sprintf(cmd, "%sX%d", Arcus_Com_Prefix, count);
   cmdLen = strlen(cmd);
	comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nmoveCmd: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;

	return(comStatus_);
}

asynStatus arcusAxis::setSpeed(double velocity)
{
   //long       vel;
   char       rep[REP_LEN];
   char       cmd[CMD_LEN];
   size_t     got, cmdLen;
   asynStatus status = asynSuccess;
   double     tout = DEFLT_TIMEOUT;

   /* change speed */
   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      //sprintf(cmd, "HS%c=%ld\0LS%c=%ld\0ACC%c=%ld\0", channel_, (long)velocity,
      //     channel_, (long)(velocity/10), channel_, (long)(velocity/30));
      sprintf(cmd, "HS%c=%ld", channel_, (long)velocity);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "LS%c=%ld", channel_, (long)(velocity/10));
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "ACC%c=%ld", channel_, (long)(velocity/30));
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      //sprintf(cmd, "HSPD=%ld\0LSPD=%ld\0ACC=%ld\0", (long)velocity,
      //     (long)(velocity/10), (long)(velocity/30));
      sprintf(cmd, "%sHSPD=%ld", Arcus_Com_Prefix, (long)velocity);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sLSPD=%ld", Arcus_Com_Prefix, (long)(velocity/10));
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sACC=%ld", Arcus_Com_Prefix, (long)(velocity/30));
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   //cmdLen = strlen(cmd);
   //status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nsetSpeed1: Status = %d.\n", status);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   //if((status == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
   //   status = (asynStatus)0;

	return(status);
}

asynStatus arcusAxis::setSpeed(double velocity, double lowSpeed, double accel)
{
   //long       vel;
   char       rep[REP_LEN];
   char       cmd[CMD_LEN];
   size_t     got, cmdLen;
   asynStatus status = asynSuccess;
   double     tout = DEFLT_TIMEOUT;

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      //sprintf(cmd, "HS%c=%ld\nLS%c=%ld\nACC%c=%ld\n", channel_, (long)velocity,
      //        channel_, (long)lowSpeed, channel_, (long)accel);
      sprintf(cmd, "HS%c=%ld", channel_, (long)velocity);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "LS%c=%ld", channel_, (long)lowSpeed);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "ACC%c=%ld", channel_, (long)accel);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      //sprintf(cmd, "HSPD=%ld\nLSPD=%ld\nACC=%ld\n", (long)velocity,
      //        (long)lowSpeed, (long)accel);
      sprintf(cmd, "%sHSPD=%ld", Arcus_Com_Prefix, (long)velocity);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sLSPD=%ld", Arcus_Com_Prefix, (long)lowSpeed);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sACC=%ld", Arcus_Com_Prefix, (long)accel);
      cmdLen = strlen(cmd);
      status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   //cmdLen = strlen(cmd);
   //status = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nsetSpeed2: Status = %d.\n", status);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   //if((status == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
   //   status = (asynStatus)0;
   
	return(status);
}

asynStatus arcusAxis::move(double position, int relative, double min_vel,
           double max_vel, double accel)
{
   char   rep[REP_LEN];
   char   cmd[CMD_LEN];
   size_t got, cmdLen;
   double tout = DEFLT_TIMEOUT;
   double newMin;

   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\narcusAxis:move position = %f, min_vel = %f, max_vel = %f, accel = %f\n",
         position, min_vel, max_vel, accel);
   if(min_vel < 100.0)
      newMin = max_vel / 10.0;
   else
      newMin = min_vel;
	comStatus_ = setSpeed(max_vel, newMin, accel);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nmove: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;
   if(comStatus_ != 0)
   {
      if(DEBUG)
         asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
            "\narcusAxis:move1 Error setting speed (%d).\n", comStatus_);
      setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
      return(comStatus_);
   }
   if(relative)
      sprintf(cmd, "%sINC", Arcus_Com_Prefix);
   else
      sprintf(cmd, "%sABS", Arcus_Com_Prefix);
   cmdLen = strlen(cmd);
   comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      sprintf(cmd, "EO%d=1", axis_+1);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%c%d", channel_, (int)position);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      sprintf(cmd, "%sEO=1", Arcus_Com_Prefix);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sX%d", Arcus_Com_Prefix, (int)position);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }

   //sprintf(cmd, "%c%f", channel_, position);
   //cmdLen = strlen(cmd);
   //comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nmove2: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;
	
	return(comStatus_);
}

asynStatus arcusAxis::home(double min_vel, double max_vel,
           double accel, int forwards)
{
   char   rep[REP_LEN];
   char   cmd[CMD_LEN];
   size_t got, cmdLen;
   double tout = DEFLT_TIMEOUT;
   char   direction;

   if(max_vel < 0)
      direction = '-';
   else
      direction = '+';

	comStatus_ = setSpeed(max_vel, min_vel, accel);
   if(comStatus_ != 0)
   {
      setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
      return(comStatus_);
   }

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      //sprintf(cmd, "EO%d=1\0H%c%c\0", axis_+1, channel_, direction);
      sprintf(cmd, "EO%d=1", axis_+1);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "H%c%c", channel_, direction);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      //sprintf(cmd, "EO=1\0H%c\0", direction);
      sprintf(cmd, "%sEO=1", Arcus_Com_Prefix);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sH%c", Arcus_Com_Prefix, direction);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   //cmdLen = strlen(cmd);
   //comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nhome: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;
      
	return(comStatus_);
}

asynStatus arcusAxis::stop(double acceleration)
{
   char       rep[REP_LEN];
   char       cmd[CMD_LEN];
   size_t     got, cmdLen;
   double     tout = DEFLT_TIMEOUT;

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
      sprintf(cmd, "STOP%c", channel_);
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
      sprintf(cmd, "%sSTOP", Arcus_Com_Prefix);
   cmdLen = strlen(cmd);
   comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nstop: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;

	if(comStatus_)
   {
		setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
   
	return comStatus_;
}

asynStatus arcusAxis::setPosition(double position)
{
   char       rep[REP_LEN];
   char       cmd[CMD_LEN];
   size_t     got, cmdLen;
   double     tout = DEFLT_TIMEOUT;

   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      //sprintf(cmd, "EO%d=1\0ABS\0%c%d\0", axis_+1, channel_, (int)position);
      sprintf(cmd, "EO%d=1", axis_+1);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "ABS");
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      printf(cmd, "%c%d", channel_, (int)position);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      //sprintf(cmd, "EO=1\0ABS\0X%d\0", (int)position);
      sprintf(cmd, "%sEO=1", Arcus_Com_Prefix);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sABS", Arcus_Com_Prefix);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      printf(cmd, "%sX%d", Arcus_Com_Prefix, (int)position);
      cmdLen = strlen(cmd);
      comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   //cmdLen = strlen(cmd);
   //comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nsetPosition: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;

	if(comStatus_)
   {
		setIntegerParam(c_p_->motorStatusProblem_,    1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
	}
	return comStatus_;
}

asynStatus arcusAxis::moveVelocity(double min_vel, double max_vel, double accel)
{
   long   speed = (long)rint(fabs(max_vel));
   char   rep[REP_LEN];
   char   cmd[CMD_LEN];
   size_t got, cmdLen;
   char   direction;
   double tout = DEFLT_TIMEOUT;

	if(max_vel < 0)
		direction = '-'; 
	else
      direction = '+';

	comStatus_ = setSpeed((double)speed, min_vel, accel);
   if(comStatus_ != 0)
   {
      setIntegerParam(c_p_->motorStatusProblem_, 1);
		setIntegerParam(c_p_->motorStatusCommsError_, 1);
		callParamCallbacks();
      return(comStatus_);
   }
   if(c_p_->ArcusModel == arcusController::PMX_4ET_SA)
   {
      //sprintf(cmd, "EO%d=1\0J%c%c\0", channel_, channel_, direction);
      sprintf(cmd, "EO%d=1", axis_+1);
      cmdLen = strlen(cmd);
	   comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "J%c%c", channel_, direction);
      cmdLen = strlen(cmd);
	   comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   else if(c_p_->ArcusModel != arcusController::UNKNOWN)
   {
      //sprintf(cmd, "EO=1\0J%c\0", direction);
      sprintf(cmd, "%sEO=1", Arcus_Com_Prefix);
      cmdLen = strlen(cmd);
	   comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
      sprintf(cmd, "%sJ%c", Arcus_Com_Prefix, direction);
      cmdLen = strlen(cmd);
	   comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   }
   //cmdLen = strlen(cmd);
	//comStatus_ = c_p_->sendCmd(&got, rep, sizeof(rep), tout, cmd, cmdLen);
   if(DEBUG)
      asynPrint(c_p_->asynUserMot_p_, ASYN_TRACEIO_DRIVER,
         "\nmoveVelocity: Status = %d.\n", comStatus_);
   /* Again, with the DMX motors, I'm getting a timeout status even though    */
   /* the data seems to be good. I'll need to figure that out, but in the mean*/
   /* time, force the status to be good.                                      */
   if((comStatus_ == 1) && (c_p_->ArcusModel != arcusController::PMX_4ET_SA))
      comStatus_ = (asynStatus)0;

	return comStatus_;
}

/* Obtain value of the 'motorClosedLoop_' parameter (which
 * maps to the record's CNEN field)
 */
//int arcusAxis::getClosedLoop()
//{
//int val;
//	c_p_->getIntegerParam(axisNo_, c_p_->motorClosedLoop_, &val);
//	return val;
//}

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]",              iocshArgString};
static const iocshArg cc_a1 = {"I/O port name [string]",          iocshArgString};
static const iocshArg cc_a2 = {"Number of axes [int]",            iocshArgInt};
static const iocshArg cc_a3 = {"Moving poll period (s) [double]", iocshArgDouble};
static const iocshArg cc_a4 = {"Idle poll period (s) [double]",   iocshArgDouble};
static const iocshArg cc_a5 = {"Arcus Controller Flag [int]",     iocshArgInt};

static const iocshArg * const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4,
             &cc_a5};

static const iocshFuncDef cc_def = {"arcusCreateController",
             sizeof(cc_as)/sizeof(cc_as[0]), cc_as};

extern "C" void *arcusCreateController(
	const char *motorPortName,
	const char *ioPortName,
	int         numAxes,
	double      movingPollPeriod,
	double      idlePollPeriod,
   int         ArcusControllerFlag)
{
   void *rval = 0;
   
	/* the asyn stuff doesn't seem to allow for exceptions. I get segfaults    */
	/* if constructing a controller (or axis) incurs an exception even if its  */
	/* caught (IMHO asyn should behave as if controller/axis never existed.)   */
#ifdef ASYN_CANDO_EXCEPTIONS
	try
   {
#endif
		rval = new arcusController(motorPortName, ioPortName, numAxes,
             movingPollPeriod, idlePollPeriod, ArcusControllerFlag);
#ifdef ASYN_CANDO_EXCEPTIONS
	}
   catch(arcusException &e)
   {
		epicsPrintf("arcusCreateController failed (exception caught):\n%s\n", e.what());
		rval = 0;
	}
#endif

	return rval;
}

static void cc_fn(const iocshArgBuf *args)
{
	arcusCreateController(
		args[0].sval,
		args[1].sval,
		args[2].ival,
		args[3].dval,
		args[4].dval,
      args[5].ival);
}


static const iocshArg ca_a0 = {"Controller Port name [string]",    iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]",                iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]",                    iocshArgInt};

static const iocshArg * const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* arcusCreateAxis called to create each axis of the arcus controller*/
static const iocshFuncDef ca_def = {"arcusCreateAxis", 3, ca_as};

extern "C" void *arcusCreateAxis(
	const char *controllerPortName,
	int        axisNumber,
	int        channel)
{
   void *rval = 0;
   arcusController *pC;
   arcusAxis *pAxis;
   asynMotorAxis *pAsynAxis;

	/* the asyn stuff doesn't seem to allow for exceptions. I get segfaults    */
	/* if constructing a controller (or axis) incurs an exception even if its  */
	/* caught (IMHO asyn should behave as if controller/axis never existed.)   */
#ifdef ASYN_CANDO_EXCEPTIONS
	try
   {
#endif
//		rval = new arcusAxis(, axisNumber, channel);
		pC = (arcusController*)findAsynPortDriver(controllerPortName);
		if(!pC)
      {
			printf("arcusCreateAxis: Error port %s not found\n", controllerPortName);
			rval = 0;
			return(rval);
		}
		/* check if axis number already exists                                  */
		pAsynAxis = pC->getAxis(axisNumber);
		if(pAsynAxis != NULL) // axis already exists
      {
			epicsPrintf("arcusCreateAxis failed:axis %u already exists\n",
                     axisNumber);
#ifdef ASYN_CANDO_EXCEPTIONS
			THROW_(arcusException(MCSCommunicationError, "axis %u already exists",
                axisNumber));
#endif
			rval = 0;
			return(rval);
		}
		pC->lock();
		pAxis = new arcusAxis(pC, axisNumber, channel);
      rval = (void *)pAxis; /* Wheat */
		pAxis = NULL;
		pC->unlock();

#ifdef ASYN_CANDO_EXCEPTIONS
	}
   catch(arcusException &e)
   {
		epicsPrintf("arcusAxis failed (exception caught):\n%s\n", e.what());
		rval = 0;
	}
#endif

   /* Do we not need to return anything but zero here? I've assigned a value  */
   /* to rval above, but before that, rval never had a value. Hmmmm...Wheat.  */
	return(rval);
}

static void ca_fn(const iocshArgBuf *args)
{
	arcusCreateAxis(args[0].sval, args[1].ival, args[2].ival);
}

static void arcusMotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn);  // arcusCreateController
  iocshRegister(&ca_def, ca_fn);  // arcusCreateAxis
}

extern "C"
{
   epicsExportRegistrar(arcusMotorRegister);
}
