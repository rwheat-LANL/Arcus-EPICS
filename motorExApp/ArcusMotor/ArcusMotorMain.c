/*************************************************************************\
* Copyright (c) 2015, Los Alamos National Laboratory
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

/* An example application, but enough to get he EPICS Motor stuff going.      */
#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"

int main(int argc,char *argv[])
{
    if (argc >= 2)
    {
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    iocsh(NULL);
    epicsExit(0);
    return(0);
}
