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

#ifndef defiAssertion_h
#define defiAssertion_h
#include "defiKRDefs.h"
#include <stdio.h>

/*
 * Struct holds the data for one assertion/constraint.
 */

/*
 * An assertion or constraint is either a net/path rule or a
 */

/*
 * wired logic rule.
 */

/*
 * A net/path rule is an item or list of items plus specifications.
 */

/*
 * The specifications are: rise/fall min/max.
 */

/*
 * The items are a list of (one or more) net names or paths or a
 */

/*
 * combination of both.
 */

/*
 * A wired logic rule is a netname and a distance.
 */

/*
 * We will NOT allow the mixing of wired logic rules and net/path delays
 */

/*
 * in the same assertion/constraint.
 */

/*
 * We will allow the rule to be a sum of sums (which will be interpreted
 */

/*
 * as just one list).
 */

/*
 * Either isAssertion or isConstraint is true
 */

/*
 * Either isWiredlogic or isDelay is true
 */

/*
 * Wired logic net name
 */

/*
 * Wired logic distance
 */

/*
 * number of paths or nets
 */

/*
 * is item #index a path?
 */

/*
 * is item #index a net?
 */

/*
 * Get path data for item #index
 */

/*
 * Get net data for item #index
 */

/*
 * wired logic net name
 */

/*
 * also used to store the wired logic dist
 */

/*
 * not really integers.
 */

typedef struct defiAssertion_s {
  char            isAssertion_;
  char            isSum_;
  char            isDiff_;
  char            hasRiseMin_;
  char            hasRiseMax_;
  char            hasFallMin_;
  char            hasFallMax_;
  char            isWiredlogic_;
  char            isDelay_;
  char           *netName_;
  int             netNameLength_;
  double          riseMin_;
  double          riseMax_;
  double          fallMin_;
  double          fallMax_;
  int             numItems_;
  int             numItemsAllocated_;
  char           *itemTypes_;
  int           **items_;
} defiAssertion;

EXTERN defiAssertion *
defiAssertion_Create
  PROTO_PARAMS((  ));

EXTERN void
defiAssertion_Init
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_Destroy
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_Delete
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setConstraintMode
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setAssertionMode
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setSum
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setDiff
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setNetName
  PROTO_PARAMS(( defiAssertion * this,
                 const char *name ));

EXTERN void
defiAssertion_setRiseMin
  PROTO_PARAMS(( defiAssertion * this,
                 double num ));

EXTERN void
defiAssertion_setRiseMax
  PROTO_PARAMS(( defiAssertion * this,
                 double num ));

EXTERN void
defiAssertion_setFallMin
  PROTO_PARAMS(( defiAssertion * this,
                 double num ));

EXTERN void
defiAssertion_setFallMax
  PROTO_PARAMS(( defiAssertion * this,
                 double num ));

EXTERN void
defiAssertion_setDelay
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setWiredlogicMode
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_setWiredlogic
  PROTO_PARAMS(( defiAssertion * this,
                 const char *net,
                 double dist ));

EXTERN void
defiAssertion_addNet
  PROTO_PARAMS(( defiAssertion * this,
                 const char *name ));

EXTERN void
defiAssertion_addPath
  PROTO_PARAMS(( defiAssertion * this,
                 const char *fromInst,
                 const char *fromPin,
                 const char *toInst,
                 const char *toPin ));

EXTERN void
defiAssertion_bumpItems
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_unsetSum
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN int
defiAssertion_isAssertion
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_isConstraint
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_isWiredlogic
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_isDelay
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_isSum
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_isDiff
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_hasRiseMin
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_hasRiseMax
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_hasFallMin
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_hasFallMax
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN double
defiAssertion_riseMin
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN double
defiAssertion_riseMax
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN double
defiAssertion_fallMin
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN double
defiAssertion_fallMax
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN const char *
defiAssertion_netName
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN double
defiAssertion_distance
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_numItems
  PROTO_PARAMS(( const defiAssertion * this ));

EXTERN int
defiAssertion_isPath
  PROTO_PARAMS(( const defiAssertion * this,
                 int index ));

EXTERN int
defiAssertion_isNet
  PROTO_PARAMS(( const defiAssertion * this,
                 int index ));

EXTERN void
defiAssertion_path
  PROTO_PARAMS(( const defiAssertion * this,
                 int index,
                 char **fromInst,
                 char **fromPin,
                 char **toInst,
                 char **toPin ));

EXTERN void
defiAssertion_net
  PROTO_PARAMS(( const defiAssertion * this,
                 int index,
                 char **netName ));

EXTERN void
defiAssertion_clear
  PROTO_PARAMS(( defiAssertion * this ));

EXTERN void
defiAssertion_print
  PROTO_PARAMS(( const defiAssertion * this,
                 FILE * f ));

#endif
