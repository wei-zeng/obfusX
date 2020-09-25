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

#ifndef defiRowTrack_h
#define defiRowTrack_h
#include "defiKRDefs.h"
#include <stdio.h>

/*
 * 5.6, DO is optional
 */

/*
 * 5.6, STEP is optional in DO
 */

typedef struct defiRow_s {
  int             nameLength_;
  char           *name_;
  int             macroLength_;
  char           *macro_;
  double          x_;
  double          y_;
  double          xNum_;
  double          yNum_;
  int             orient_;
  double          xStep_;
  double          yStep_;
  int             hasDo_;
  int             hasDoStep_;
  int             numProps_;
  int             propsAllocated_;
  char          **propNames_;
  char          **propValues_;
  double         *propDValues_;
  char           *propTypes_;
} defiRow;

EXTERN defiRow *
defiRow_Create
  PROTO_PARAMS((  ));

EXTERN void
defiRow_Init
  PROTO_PARAMS(( defiRow * this ));

EXTERN void
defiRow_Delete
  PROTO_PARAMS(( defiRow * this ));

EXTERN void
defiRow_Destroy
  PROTO_PARAMS(( defiRow * this ));

EXTERN void
defiRow_clear
  PROTO_PARAMS(( defiRow * this ));

EXTERN void
defiRow_setup
  PROTO_PARAMS(( defiRow * this,
                 const char *name,
                 const char *macro,
                 double x,
                 double y,
                 int orient ));

EXTERN void
defiRow_setDo
  PROTO_PARAMS(( defiRow * this,
                 double x_num,
                 double y_num,
                 double x_step,
                 double y_step ));

EXTERN void
defiRow_setHasDoStep
  PROTO_PARAMS(( defiRow * this ));

EXTERN void
defiRow_addProperty
  PROTO_PARAMS(( defiRow * this,
                 const char *name,
                 const char *value,
                 const char type ));

EXTERN void
defiRow_addNumProperty
  PROTO_PARAMS(( defiRow * this,
                 const char *name,
                 const double d,
                 const char *value,
                 const char type ));

EXTERN const char *
defiRow_name
  PROTO_PARAMS(( const defiRow * this ));

EXTERN const char *
defiRow_macro
  PROTO_PARAMS(( const defiRow * this ));

EXTERN double
defiRow_x
  PROTO_PARAMS(( const defiRow * this ));

EXTERN double
defiRow_y
  PROTO_PARAMS(( const defiRow * this ));

EXTERN int
defiRow_orient
  PROTO_PARAMS(( const defiRow * this ));

EXTERN const char *
defiRow_orientStr
  PROTO_PARAMS(( const defiRow * this ));

EXTERN int
defiRow_hasDo
  PROTO_PARAMS(( const defiRow * this ));

EXTERN double
defiRow_xNum
  PROTO_PARAMS(( const defiRow * this ));

EXTERN double
defiRow_yNum
  PROTO_PARAMS(( const defiRow * this ));

EXTERN int
defiRow_hasDoStep
  PROTO_PARAMS(( const defiRow * this ));

EXTERN double
defiRow_xStep
  PROTO_PARAMS(( const defiRow * this ));

EXTERN double
defiRow_yStep
  PROTO_PARAMS(( const defiRow * this ));

EXTERN int
defiRow_numProps
  PROTO_PARAMS(( const defiRow * this ));

EXTERN const char *
defiRow_propName
  PROTO_PARAMS(( const defiRow * this,
                 int index ));

EXTERN const char *
defiRow_propValue
  PROTO_PARAMS(( const defiRow * this,
                 int index ));

EXTERN double
defiRow_propNumber
  PROTO_PARAMS(( const defiRow * this,
                 int index ));

EXTERN const char
defiRow_propType
  PROTO_PARAMS(( const defiRow * this,
                 int index ));

EXTERN int
defiRow_propIsNumber
  PROTO_PARAMS(( const defiRow * this,
                 int index ));

EXTERN int
defiRow_propIsString
  PROTO_PARAMS(( const defiRow * this,
                 int index ));

EXTERN void
defiRow_print
  PROTO_PARAMS(( const defiRow * this,
                 FILE * f ));

/*
 * allocated size of macro_;
 */

/*
 * allocated size of layers_
 */

/*
 * number of places used in layers_
 */

typedef struct defiTrack_s {
  int             macroLength_;
  char           *macro_;
  double          x_;
  double          xNum_;
  double          xStep_;
  int             layersLength_;
  int             numLayers_;
  char          **layers_;
} defiTrack;

EXTERN defiTrack *
defiTrack_Create
  PROTO_PARAMS((  ));

EXTERN void
defiTrack_Init
  PROTO_PARAMS(( defiTrack * this ));

EXTERN void
defiTrack_Delete
  PROTO_PARAMS(( defiTrack * this ));

EXTERN void
defiTrack_Destroy
  PROTO_PARAMS(( defiTrack * this ));

EXTERN void
defiTrack_setup
  PROTO_PARAMS(( defiTrack * this,
                 const char *macro ));

EXTERN void
defiTrack_setDo
  PROTO_PARAMS(( defiTrack * this,
                 double x,
                 double x_num,
                 double x_step ));

EXTERN void
defiTrack_addLayer
  PROTO_PARAMS(( defiTrack * this,
                 const char *layer ));

EXTERN const char *
defiTrack_macro
  PROTO_PARAMS(( const defiTrack * this ));

EXTERN double
defiTrack_x
  PROTO_PARAMS(( const defiTrack * this ));

EXTERN double
defiTrack_xNum
  PROTO_PARAMS(( const defiTrack * this ));

EXTERN double
defiTrack_xStep
  PROTO_PARAMS(( const defiTrack * this ));

EXTERN int
defiTrack_numLayers
  PROTO_PARAMS(( const defiTrack * this ));

EXTERN const char *
defiTrack_layer
  PROTO_PARAMS(( const defiTrack * this,
                 int index ));

EXTERN void
defiTrack_print
  PROTO_PARAMS(( const defiTrack * this,
                 FILE * f ));


typedef struct defiGcellGrid_s {
  int             macroLength_;
  char           *macro_;
  int             x_;
  int             xNum_;
  double          xStep_;
} defiGcellGrid;

EXTERN defiGcellGrid *
defiGcellGrid_Create
  PROTO_PARAMS((  ));

EXTERN void
defiGcellGrid_Init
  PROTO_PARAMS(( defiGcellGrid * this ));

EXTERN void
defiGcellGrid_Delete
  PROTO_PARAMS(( defiGcellGrid * this ));

EXTERN void
defiGcellGrid_Destroy
  PROTO_PARAMS(( defiGcellGrid * this ));

EXTERN void
defiGcellGrid_setup
  PROTO_PARAMS(( defiGcellGrid * this,
                 const char *macro,
                 int x,
                 int xNum,
                 double xStep ));

EXTERN const char *
defiGcellGrid_macro
  PROTO_PARAMS(( const defiGcellGrid * this ));

EXTERN int
defiGcellGrid_x
  PROTO_PARAMS(( const defiGcellGrid * this ));

EXTERN int
defiGcellGrid_xNum
  PROTO_PARAMS(( const defiGcellGrid * this ));

EXTERN double
defiGcellGrid_xStep
  PROTO_PARAMS(( const defiGcellGrid * this ));

EXTERN void
defiGcellGrid_print
  PROTO_PARAMS(( const defiGcellGrid * this,
                 FILE * f ));

#endif
