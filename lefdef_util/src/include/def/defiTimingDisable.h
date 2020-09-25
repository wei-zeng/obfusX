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

#ifndef defiTimingDisable_h
#define defiTimingDisable_h
#include <stdio.h>
#include "defiKRDefs.h"

/*
 * A Timing disable can be a from-to  or a thru or a macro.
 */

/*
 * A macro is either a fromto macro or a thru macro.
 */

/*
 * Also macro thru
 */

/*
 * debug print
 */

/*
 * also macro name and thru inst
 */

/*
 * also macro thru and thru pin
 */

typedef struct defiTimingDisable_s {
  char           *fromInst_;
  int             fromInstLength_;
  char           *toInst_;
  int             toInstLength_;
  char           *fromPin_;
  int             fromPinLength_;
  char           *toPin_;
  int             toPinLength_;
  int             hasFromTo_;
  int             hasThru_;
  int             hasMacro_;
  int             hasReentrantPathsFlag_;
} defiTimingDisable;

EXTERN defiTimingDisable *
defiTimingDisable_Create
  PROTO_PARAMS((  ));

EXTERN void
defiTimingDisable_Init
  PROTO_PARAMS(( defiTimingDisable * this ));

EXTERN void
defiTimingDisable_Destroy
  PROTO_PARAMS(( defiTimingDisable * this ));

EXTERN void
defiTimingDisable_Delete
  PROTO_PARAMS(( defiTimingDisable * this ));

EXTERN void
defiTimingDisable_clear
  PROTO_PARAMS(( defiTimingDisable * this ));

EXTERN void
defiTimingDisable_setFromTo
  PROTO_PARAMS(( defiTimingDisable * this,
                 const char *fromInst,
                 const char *fromPin,
                 const char *toInst,
                 const char *toPin ));

EXTERN void
defiTimingDisable_setThru
  PROTO_PARAMS(( defiTimingDisable * this,
                 const char *fromInst,
                 const char *fromPin ));

EXTERN void
defiTimingDisable_setMacro
  PROTO_PARAMS(( defiTimingDisable * this,
                 const char *name ));

EXTERN void
defiTimingDisable_setMacroThru
  PROTO_PARAMS(( defiTimingDisable * this,
                 const char *thru ));

EXTERN void
defiTimingDisable_setMacroFromTo
  PROTO_PARAMS(( defiTimingDisable * this,
                 const char *fromPin,
                 const char *toPin ));

EXTERN void
defiTimingDisable_setReentrantPathsFlag
  PROTO_PARAMS(( defiTimingDisable * this ));

EXTERN int
defiTimingDisable_hasMacroThru
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN int
defiTimingDisable_hasMacroFromTo
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN int
defiTimingDisable_hasThru
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN int
defiTimingDisable_hasFromTo
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN int
defiTimingDisable_hasReentrantPathsFlag
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_fromPin
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_toPin
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_fromInst
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_toInst
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_macroName
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_thruPin
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN const char *
defiTimingDisable_thruInst
  PROTO_PARAMS(( const defiTimingDisable * this ));

EXTERN void
defiTimingDisable_print
  PROTO_PARAMS(( const defiTimingDisable * this,
                 FILE * f ));

#endif
