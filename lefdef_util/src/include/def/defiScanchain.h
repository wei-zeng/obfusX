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

#ifndef defiScanchain_h
#define defiScanchain_h
#include "defiKRDefs.h"
#include <stdio.h>

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

typedef struct defiOrdered_s {
  int             num_;
  int             allocated_;
  char          **inst_;
  char          **in_;
  char          **out_;
  int            *bits_;
} defiOrdered;

EXTERN void
defiOrdered_addOrdered
  PROTO_PARAMS(( defiOrdered * this,
                 const char *inst ));

EXTERN void
defiOrdered_addIn
  PROTO_PARAMS(( defiOrdered * this,
                 const char *pin ));

EXTERN void
defiOrdered_addOut
  PROTO_PARAMS(( defiOrdered * this,
                 const char *pin ));

EXTERN void
defiOrdered_setOrderedBits
  PROTO_PARAMS(( defiOrdered * this,
                 int bits ));

EXTERN void
defiOrdered_bump
  PROTO_PARAMS(( defiOrdered * this ));

EXTERN void
defiOrdered_Init
  PROTO_PARAMS(( defiOrdered * this ));

EXTERN void
defiOrdered_Destroy
  PROTO_PARAMS(( defiOrdered * this ));

EXTERN void
defiOrdered_clear
  PROTO_PARAMS(( defiOrdered * this ));

EXTERN int
defiOrdered_num
  PROTO_PARAMS(( const defiOrdered * this ));

EXTERN char **
defiOrdered_inst
  PROTO_PARAMS(( const defiOrdered * this ));

EXTERN char **
defiOrdered_in
  PROTO_PARAMS(( const defiOrdered * this ));

EXTERN char **
defiOrdered_out
  PROTO_PARAMS(( const defiOrdered * this ));

EXTERN int *
defiOrdered_bits
  PROTO_PARAMS(( const defiOrdered * this ));

/*
 * Struct holds the data for one Scan chain.
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * If the pin part of these routines were not supplied in the DEF
 */

/*
 * then a NULL pointer will be returned.
 */

/*
 * There could be many ORDERED constructs in the DEF.  The data in
 */

/*
 * each ORDERED construct is stored in its own array.  The numOrderedLists()
 */

/*
 * routine tells how many lists there are.
 */

/*
 * This routine will return an array of instances and
 */

/*
 * an array of in and out pins.
 */

/*
 * The number if things in the arrays is returned in size.
 */

/*
 * The inPin and outPin entry is optional for each instance.
 */

/*
 * If an entry is not given, then that char* is NULL.
 */

/*
 * For example if the second instance has
 */

/*
 * instnam= "FOO" and IN="A", but no OUT given, then inst[1] points
 */

/*
 * to "FOO"  inPin[1] points to "A" and outPin[1] is a NULL pointer.
 */

/*
 * All of the floating constructs in the scan chain are
 */

/*
 * stored in this one array.
 */

/*
 * If the IN or OUT of an entry is not supplied then the array will have
 */

/*
 * a NULL pointer in that place.
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * Array of floating names
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

/*
 * 5.4.1
 */

typedef struct defiScanchain_s {
  char           *name_;
  char            hasStart_;
  char            hasStop_;
  int             nameLength_;
  int             numOrderedAllocated_;
  int             numOrdered_;
  defiOrdered   **ordered_;
  int             numFloatingAllocated_;
  int             numFloating_;
  char          **floatInst_;
  char          **floatIn_;
  char          **floatOut_;
  int            *floatBits_;
  char           *stopInst_;
  char           *stopPin_;
  char           *startInst_;
  char           *startPin_;
  char           *commonInPin_;
  char           *commonOutPin_;
  char            hasPartition_;
  char           *partName_;
  int             maxBits_;
} defiScanchain;

EXTERN defiScanchain *
defiScanchain_Create
  PROTO_PARAMS((  ));

EXTERN void
defiScanchain_Init
  PROTO_PARAMS(( defiScanchain * this ));

EXTERN void
defiScanchain_Destroy
  PROTO_PARAMS(( defiScanchain * this ));

EXTERN void
defiScanchain_Delete
  PROTO_PARAMS(( defiScanchain * this ));

EXTERN void
defiScanchain_setName
  PROTO_PARAMS(( defiScanchain * this,
                 const char *name ));

EXTERN void
defiScanchain_clear
  PROTO_PARAMS(( defiScanchain * this ));

EXTERN void
defiScanchain_addOrderedList
  PROTO_PARAMS(( defiScanchain * this ));

EXTERN void
defiScanchain_addOrderedInst
  PROTO_PARAMS(( defiScanchain * this,
                 const char *inst ));

EXTERN void
defiScanchain_addOrderedIn
  PROTO_PARAMS(( defiScanchain * this,
                 const char *inPin ));

EXTERN void
defiScanchain_addOrderedOut
  PROTO_PARAMS(( defiScanchain * this,
                 const char *outPin ));

EXTERN void
defiScanchain_setOrderedBits
  PROTO_PARAMS(( defiScanchain * this,
                 int bits ));

EXTERN void
defiScanchain_addFloatingInst
  PROTO_PARAMS(( defiScanchain * this,
                 const char *inst ));

EXTERN void
defiScanchain_addFloatingIn
  PROTO_PARAMS(( defiScanchain * this,
                 const char *inPin ));

EXTERN void
defiScanchain_addFloatingOut
  PROTO_PARAMS(( defiScanchain * this,
                 const char *outPin ));

EXTERN void
defiScanchain_setFloatingBits
  PROTO_PARAMS(( defiScanchain * this,
                 int bits ));

EXTERN void
defiScanchain_setStart
  PROTO_PARAMS(( defiScanchain * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiScanchain_setStop
  PROTO_PARAMS(( defiScanchain * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiScanchain_setCommonIn
  PROTO_PARAMS(( defiScanchain * this,
                 const char *pin ));

EXTERN void
defiScanchain_setCommonOut
  PROTO_PARAMS(( defiScanchain * this,
                 const char *pin ));

EXTERN void
defiScanchain_setPartition
  PROTO_PARAMS(( defiScanchain * this,
                 const char *partName,
                 int maxBits ));

EXTERN const char *
defiScanchain_name
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasStart
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasStop
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasFloating
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasOrdered
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasCommonInPin
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasCommonOutPin
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasPartition
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_hasPartitionMaxBits
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN void
defiScanchain_start
  PROTO_PARAMS(( const defiScanchain * this,
                 char **inst,
                 char **pin ));

EXTERN void
defiScanchain_stop
  PROTO_PARAMS(( const defiScanchain * this,
                 char **inst,
                 char **pin ));

EXTERN int
defiScanchain_numOrderedLists
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN void
defiScanchain_ordered
  PROTO_PARAMS(( const defiScanchain * this,
                 int index,
                 int *size,
                 char ***inst,
                 char ***inPin,
                 char ***outPin,
                 int **bits ));

EXTERN void
defiScanchain_floating
  PROTO_PARAMS(( const defiScanchain * this,
                 int *size,
                 char ***inst,
                 char ***inPin,
                 char ***outPin,
                 int **bits ));

EXTERN const char *
defiScanchain_commonInPin
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN const char *
defiScanchain_commonOutPin
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN const char *
defiScanchain_partitionName
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN int
defiScanchain_partitionMaxBits
  PROTO_PARAMS(( const defiScanchain * this ));

EXTERN void
defiScanchain_print
  PROTO_PARAMS(( const defiScanchain * this,
                 FILE * f ));

#endif
