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

#ifndef defiGroup_h
#define defiGroup_h
#include <stdio.h>
#include "defiKRDefs.h"

/*
 * Struct holds the data for one property.
 */

/*
 * debug print
 */

typedef struct defiGroup_s {
  char           *name_;
  int             nameLength_;
  char           *region_;
  int             regionLength_;
  int             rectsAllocated_;
  int             numRects_;
  int            *xl_;
  int            *yl_;
  int            *xh_;
  int            *yh_;
  int             maxX_;
  int             maxY_;
  int             perim_;
  char            hasRegionBox_;
  char            hasRegionName_;
  char            hasPerim_;
  char            hasMaxX_;
  char            hasMaxY_;
  int             numProps_;
  int             propsAllocated_;
  char          **propNames_;
  char          **propValues_;
  double         *propDValues_;
  char           *propTypes_;
} defiGroup;

EXTERN defiGroup *
defiGroup_Create
  PROTO_PARAMS((  ));

EXTERN void
defiGroup_Init
  PROTO_PARAMS(( defiGroup * this ));

EXTERN void
defiGroup_Destroy
  PROTO_PARAMS(( defiGroup * this ));

EXTERN void
defiGroup_Delete
  PROTO_PARAMS(( defiGroup * this ));

EXTERN void
defiGroup_clear
  PROTO_PARAMS(( defiGroup * this ));

EXTERN void
defiGroup_setup
  PROTO_PARAMS(( defiGroup * this,
                 const char *name ));

EXTERN void
defiGroup_addProperty
  PROTO_PARAMS(( defiGroup * this,
                 const char *name,
                 const char *value,
                 const char type ));

EXTERN void
defiGroup_addNumProperty
  PROTO_PARAMS(( defiGroup * this,
                 const char *name,
                 const double d,
                 const char *value,
                 const char type ));

EXTERN void
defiGroup_addRegionRect
  PROTO_PARAMS(( defiGroup * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiGroup_setRegionName
  PROTO_PARAMS(( defiGroup * this,
                 const char *name ));

EXTERN void
defiGroup_setMaxX
  PROTO_PARAMS(( defiGroup * this,
                 int x ));

EXTERN void
defiGroup_setMaxY
  PROTO_PARAMS(( defiGroup * this,
                 int y ));

EXTERN void
defiGroup_setPerim
  PROTO_PARAMS(( defiGroup * this,
                 int p ));

EXTERN const char *
defiGroup_name
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN const char *
defiGroup_regionName
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_hasRegionBox
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_hasRegionName
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_hasMaxX
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_hasMaxY
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_hasPerim
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN void
defiGroup_regionRects
  PROTO_PARAMS(( defiGroup * this,
                 int *size,
                 int **xl,
                 int **yl,
                 int **xh,
                 int **yh ));

EXTERN int
defiGroup_maxX
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_maxY
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_perim
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN int
defiGroup_numProps
  PROTO_PARAMS(( const defiGroup * this ));

EXTERN const char *
defiGroup_propName
  PROTO_PARAMS(( const defiGroup * this,
                 int index ));

EXTERN const char *
defiGroup_propValue
  PROTO_PARAMS(( const defiGroup * this,
                 int index ));

EXTERN double
defiGroup_propNumber
  PROTO_PARAMS(( const defiGroup * this,
                 int index ));

EXTERN const char
defiGroup_propType
  PROTO_PARAMS(( const defiGroup * this,
                 int index ));

EXTERN int
defiGroup_propIsNumber
  PROTO_PARAMS(( const defiGroup * this,
                 int index ));

EXTERN int
defiGroup_propIsString
  PROTO_PARAMS(( const defiGroup * this,
                 int index ));

EXTERN void
defiGroup_print
  PROTO_PARAMS(( const defiGroup * this,
                 FILE * f ));

#endif
