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

#ifndef defiRegion_h
#define defiRegion_h
#include "defiKRDefs.h"
#include <stdio.h>

/*
 * Struct holds the data for one property.
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

typedef struct defiRegion_s {
  char           *name_;
  int             nameLength_;
  int             numRectangles_;
  int             rectanglesAllocated_;
  int            *xl_;
  int            *yl_;
  int            *xh_;
  int            *yh_;
  int             numProps_;
  int             propsAllocated_;
  char          **propNames_;
  char          **propValues_;
  double         *propDValues_;
  char           *propTypes_;
  char           *type_;
} defiRegion;

EXTERN defiRegion *
defiRegion_Create
  PROTO_PARAMS((  ));

EXTERN void
defiRegion_Init
  PROTO_PARAMS(( defiRegion * this ));

EXTERN void
defiRegion_Destroy
  PROTO_PARAMS(( defiRegion * this ));

EXTERN void
defiRegion_Delete
  PROTO_PARAMS(( defiRegion * this ));

EXTERN void
defiRegion_clear
  PROTO_PARAMS(( defiRegion * this ));

EXTERN void
defiRegion_setup
  PROTO_PARAMS(( defiRegion * this,
                 const char *name ));

EXTERN void
defiRegion_addRect
  PROTO_PARAMS(( defiRegion * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiRegion_addProperty
  PROTO_PARAMS(( defiRegion * this,
                 const char *name,
                 const char *value,
                 const char type ));

EXTERN void
defiRegion_addNumProperty
  PROTO_PARAMS(( defiRegion * this,
                 const char *name,
                 const double d,
                 const char *value,
                 const char type ));

EXTERN void
defiRegion_setType
  PROTO_PARAMS(( defiRegion * this,
                 const char *type ));

EXTERN const char *
defiRegion_name
  PROTO_PARAMS(( const defiRegion * this ));

EXTERN int
defiRegion_numProps
  PROTO_PARAMS(( const defiRegion * this ));

EXTERN const char *
defiRegion_propName
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN const char *
defiRegion_propValue
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN double
defiRegion_propNumber
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN const char
defiRegion_propType
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN int
defiRegion_propIsNumber
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN int
defiRegion_propIsString
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN int
defiRegion_hasType
  PROTO_PARAMS(( const defiRegion * this ));

EXTERN const char *
defiRegion_type
  PROTO_PARAMS(( const defiRegion * this ));

EXTERN int
defiRegion_numRectangles
  PROTO_PARAMS(( const defiRegion * this ));

EXTERN int
defiRegion_xl
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN int
defiRegion_yl
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN int
defiRegion_xh
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN int
defiRegion_yh
  PROTO_PARAMS(( const defiRegion * this,
                 int index ));

EXTERN void
defiRegion_print
  PROTO_PARAMS(( const defiRegion * this,
                 FILE * f ));

#endif
