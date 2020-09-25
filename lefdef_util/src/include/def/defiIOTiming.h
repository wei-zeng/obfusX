/*
 * This  file  is  part  of  the  Cadence  LEF/DEF  Open   Source
 * Distribution,  Product Version 5.7, and is subject to the Cadence LEF/DEF
 * Open Source License Agreement.   Your  continued  use  of this file
 * constitutes your acceptance of the terms of the LEF/DEF Open Source
 * License and an agreement to abide by its  terms.   If you  don't  agree
 * with  this, you must remove this and any other files which are part of the
 * distribution and  destroy any  copies made.
 * 
 * For updates, support, or to become part of the LEF/DEF Community, check
 * www.openeda.org for details.
 */

#ifndef defiIOTiming_h
#define defiIOTiming_h
#include <stdio.h>
#include "defiKRDefs.h"

/*
 * debug print
 */

typedef struct defiIOTiming_s {
  char           *inst_;
  int             instLength_;
  char           *pin_;
  int             pinLength_;
  char           *from_;
  int             fromLength_;
  char           *to_;
  int             toLength_;
  char           *driveCell_;
  char            driveCellLength_;
  char            hasVariableRise_;
  char            hasVariableFall_;
  char            hasSlewRise_;
  char            hasSlewFall_;
  char            hasCapacitance_;
  char            hasDriveCell_;
  char            hasFrom_;
  char            hasTo_;
  char            hasParallel_;
  double          variableFallMin_;
  double          variableRiseMin_;
  double          variableFallMax_;
  double          variableRiseMax_;
  double          slewFallMin_;
  double          slewRiseMin_;
  double          slewFallMax_;
  double          slewRiseMax_;
  double          capacitance_;
  double          parallel_;
} defiIOTiming;

EXTERN defiIOTiming *
defiIOTiming_Create
  PROTO_PARAMS((  ));

EXTERN void
defiIOTiming_Init
  PROTO_PARAMS(( defiIOTiming * this ));

EXTERN void
defiIOTiming_Destroy
  PROTO_PARAMS(( defiIOTiming * this ));

EXTERN void
defiIOTiming_Delete
  PROTO_PARAMS(( defiIOTiming * this ));

EXTERN void
defiIOTiming_clear
  PROTO_PARAMS(( defiIOTiming * this ));

EXTERN void
defiIOTiming_setName
  PROTO_PARAMS(( defiIOTiming * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiIOTiming_setVariable
  PROTO_PARAMS(( defiIOTiming * this,
                 const char *riseFall,
                 double min,
                 double max ));

EXTERN void
defiIOTiming_setSlewRate
  PROTO_PARAMS(( defiIOTiming * this,
                 const char *riseFall,
                 double min,
                 double max ));

EXTERN void
defiIOTiming_setCapacitance
  PROTO_PARAMS(( defiIOTiming * this,
                 double num ));

EXTERN void
defiIOTiming_setDriveCell
  PROTO_PARAMS(( defiIOTiming * this,
                 const char *name ));

EXTERN void
defiIOTiming_setFrom
  PROTO_PARAMS(( defiIOTiming * this,
                 const char *name ));

EXTERN void
defiIOTiming_setTo
  PROTO_PARAMS(( defiIOTiming * this,
                 const char *name ));

EXTERN void
defiIOTiming_setParallel
  PROTO_PARAMS(( defiIOTiming * this,
                 double num ));

EXTERN int
defiIOTiming_hasVariableRise
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasVariableFall
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasSlewRise
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasSlewFall
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasCapacitance
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasDriveCell
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasFrom
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasTo
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN int
defiIOTiming_hasParallel
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN const char *
defiIOTiming_inst
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN const char *
defiIOTiming_pin
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_variableFallMin
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_variableRiseMin
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_variableFallMax
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_variableRiseMax
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_slewFallMin
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_slewRiseMin
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_slewFallMax
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_slewRiseMax
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_capacitance
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN const char *
defiIOTiming_driveCell
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN const char *
defiIOTiming_from
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN const char *
defiIOTiming_to
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN double
defiIOTiming_parallel
  PROTO_PARAMS(( const defiIOTiming * this ));

EXTERN void
defiIOTiming_print
  PROTO_PARAMS(( const defiIOTiming * this,
                 FILE * f ));

#endif
