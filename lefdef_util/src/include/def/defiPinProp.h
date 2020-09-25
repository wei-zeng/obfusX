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

#ifndef defiPinProp_h
#define defiPinProp_h
#include "defiKRDefs.h"
#include <stdio.h>

typedef struct defiPinProp_s {
  char            isPin_;
  int             instNameSize_;
  char           *instName_;
  int             pinNameSize_;
  char           *pinName_;
  int             numProps_;
  int             propsAllocated_;
  char          **propNames_;
  char          **propValues_;
  double         *propDValues_;
  char           *propTypes_;
} defiPinProp;

EXTERN defiPinProp *
defiPinProp_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPinProp_Init
  PROTO_PARAMS(( defiPinProp * this ));

EXTERN void
defiPinProp_Delete
  PROTO_PARAMS(( defiPinProp * this ));

EXTERN void
defiPinProp_Destroy
  PROTO_PARAMS(( defiPinProp * this ));

EXTERN void
defiPinProp_clear
  PROTO_PARAMS(( defiPinProp * this ));

EXTERN void
defiPinProp_setName
  PROTO_PARAMS(( defiPinProp * this,
                 const char *inst,
                 const char *pin ));

EXTERN void
defiPinProp_addProperty
  PROTO_PARAMS(( defiPinProp * this,
                 const char *name,
                 const char *value,
                 const char type ));

EXTERN void
defiPinProp_addNumProperty
  PROTO_PARAMS(( defiPinProp * this,
                 const char *name,
                 const double d,
                 const char *value,
                 const char type ));

EXTERN int
defiPinProp_isPin
  PROTO_PARAMS(( const defiPinProp * this ));

EXTERN const char *
defiPinProp_instName
  PROTO_PARAMS(( const defiPinProp * this ));

EXTERN const char *
defiPinProp_pinName
  PROTO_PARAMS(( const defiPinProp * this ));

EXTERN int
defiPinProp_numProps
  PROTO_PARAMS(( const defiPinProp * this ));

EXTERN const char *
defiPinProp_propName
  PROTO_PARAMS(( const defiPinProp * this,
                 int index ));

EXTERN const char *
defiPinProp_propValue
  PROTO_PARAMS(( const defiPinProp * this,
                 int index ));

EXTERN double
defiPinProp_propNumber
  PROTO_PARAMS(( const defiPinProp * this,
                 int index ));

EXTERN const char
defiPinProp_propType
  PROTO_PARAMS(( const defiPinProp * this,
                 int index ));

EXTERN int
defiPinProp_propIsNumber
  PROTO_PARAMS(( const defiPinProp * this,
                 int index ));

EXTERN int
defiPinProp_propIsString
  PROTO_PARAMS(( const defiPinProp * this,
                 int index ));

EXTERN void
defiPinProp_print
  PROTO_PARAMS(( const defiPinProp * this,
                 FILE * f ));

#endif
