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

#ifndef defiPartition_h
#define defiPartition_h
#include <stdio.h>
#include "defiKRDefs.h"

/*
 * "CLOCK" or "IO" or "COMP"
 */

/*
 * debug print
 */

/*
 * 'F' or 'T'
 */

/*
 * 'L'-clock   'I'-IO  'C'-comp
 */

typedef struct defiPartition_s {
  char           *name_;
  int             nameLength_;
  char            setup_;
  char            hold_;
  char            hasMin_;
  char            hasMax_;
  char            direction_;
  char            type_;
  char           *inst_;
  int             instLength_;
  char           *pin_;
  int             pinLength_;
  double          min_, max_;
  int             numPins_;
  int             pinsAllocated_;
  char          **pins_;
  char            hasRiseMin_;
  char            hasFallMin_;
  char            hasRiseMax_;
  char            hasFallMax_;
  char            hasRiseMinRange_;
  char            hasFallMinRange_;
  char            hasRiseMaxRange_;
  char            hasFallMaxRange_;
  double          riseMin_;
  double          fallMin_;
  double          riseMax_;
  double          fallMax_;
  double          riseMinLeft_;
  double          fallMinLeft_;
  double          riseMaxLeft_;
  double          fallMaxLeft_;
  double          riseMinRight_;
  double          fallMinRight_;
  double          riseMaxRight_;
  double          fallMaxRight_;
} defiPartition;

EXTERN defiPartition *
defiPartition_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPartition_Init
  PROTO_PARAMS(( defiPartition * this ));

EXTERN void
defiPartition_Destroy
  PROTO_PARAMS(( defiPartition * this ));

EXTERN void
defiPartition_Delete
  PROTO_PARAMS(( defiPartition * this ));

EXTERN void
defiPartition_clear
  PROTO_PARAMS(( defiPartition * this ));

EXTERN void
defiPartition_setName
  PROTO_PARAMS(( defiPartition * this,
                 const char *name ));

EXTERN void
defiPartition_addTurnOff
  PROTO_PARAMS(( defiPartition * this,
                 const char *setup,
                 const char *hold ));

EXTERN void
defiPartition_setFromClockPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiPartition_setFromCompPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiPartition_setFromIOPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *inst ));

EXTERN void
defiPartition_setToClockPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiPartition_setToCompPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiPartition_set
  PROTO_PARAMS(( defiPartition * this,
                 char dir,
                 char typ,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiPartition_setToIOPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *inst ));

EXTERN void
defiPartition_setMin
  PROTO_PARAMS(( defiPartition * this,
                 double min,
                 double max ));

EXTERN void
defiPartition_setMax
  PROTO_PARAMS(( defiPartition * this,
                 double min,
                 double max ));

EXTERN void
defiPartition_addPin
  PROTO_PARAMS(( defiPartition * this,
                 const char *name ));

EXTERN void
defiPartition_addRiseMin
  PROTO_PARAMS(( defiPartition * this,
                 double d ));

EXTERN void
defiPartition_addRiseMax
  PROTO_PARAMS(( defiPartition * this,
                 double d ));

EXTERN void
defiPartition_addFallMin
  PROTO_PARAMS(( defiPartition * this,
                 double d ));

EXTERN void
defiPartition_addFallMax
  PROTO_PARAMS(( defiPartition * this,
                 double d ));

EXTERN void
defiPartition_addRiseMinRange
  PROTO_PARAMS(( defiPartition * this,
                 double l,
                 double h ));

EXTERN void
defiPartition_addRiseMaxRange
  PROTO_PARAMS(( defiPartition * this,
                 double l,
                 double h ));

EXTERN void
defiPartition_addFallMinRange
  PROTO_PARAMS(( defiPartition * this,
                 double l,
                 double h ));

EXTERN void
defiPartition_addFallMaxRange
  PROTO_PARAMS(( defiPartition * this,
                 double l,
                 double h ));

EXTERN const char *
defiPartition_name
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN char
defiPartition_direction
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN const char *
defiPartition_itemType
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN const char *
defiPartition_pinName
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN const char *
defiPartition_instName
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_numPins
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN const char *
defiPartition_pin
  PROTO_PARAMS(( const defiPartition * this,
                 int index ));

EXTERN int
defiPartition_isSetupRise
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_isSetupFall
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_isHoldRise
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_isHoldFall
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasMin
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasMax
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasRiseMin
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasFallMin
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasRiseMax
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasFallMax
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasRiseMinRange
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasFallMinRange
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasRiseMaxRange
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN int
defiPartition_hasFallMaxRange
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_partitionMin
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_partitionMax
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_riseMin
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_fallMin
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_riseMax
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_fallMax
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_riseMinLeft
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_fallMinLeft
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_riseMaxLeft
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_fallMaxLeft
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_riseMinRight
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_fallMinRight
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_riseMaxRight
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN double
defiPartition_fallMaxRight
  PROTO_PARAMS(( const defiPartition * this ));

EXTERN void
defiPartition_print
  PROTO_PARAMS(( const defiPartition * this,
                 FILE * f ));

#endif
