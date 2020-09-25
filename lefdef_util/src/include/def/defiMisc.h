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

#ifndef defiMisc_h
#define defiMisc_h
#include <stdio.h>
#include "defiKRDefs.h"
struct defiPoints {
  int             numPoints;
  int            *x;
  int            *y;
};

typedef struct defiGeometries_s {
  int             numPoints_;
  int             pointsAllocated_;
  int            *x_;
  int            *y_;
} defiGeometries;

EXTERN defiGeometries *
defiGeometries_Create
  PROTO_PARAMS((  ));

EXTERN void
defiGeometries_Init
  PROTO_PARAMS(( defiGeometries * this ));

EXTERN void
defiGeometries_Reset
  PROTO_PARAMS(( defiGeometries * this ));

EXTERN void
defiGeometries_Destroy
  PROTO_PARAMS(( defiGeometries * this ));

EXTERN void
defiGeometries_Delete
  PROTO_PARAMS(( defiGeometries * this ));

EXTERN void
defiGeometries_startList
  PROTO_PARAMS(( defiGeometries * this,
                 int x,
                 int y ));

EXTERN void
defiGeometries_addToList
  PROTO_PARAMS(( defiGeometries * this,
                 int x,
                 int y ));

EXTERN int
defiGeometries_numPoints
  PROTO_PARAMS(( defiGeometries * this ));

EXTERN void
defiGeometries_points
  PROTO_PARAMS(( defiGeometries * this,
                 int index,
                 int *x,
                 int *y ));


typedef struct defiStyles_s {
  int             styleNum_;
  struct defiPoints *polygon_;
  int             numPointAlloc_;
} defiStyles;

EXTERN defiStyles *
defiStyles_Create
  PROTO_PARAMS((  ));

EXTERN void
defiStyles_Init
  PROTO_PARAMS(( defiStyles * this ));

EXTERN void
defiStyles_Destroy
  PROTO_PARAMS(( defiStyles * this ));

EXTERN void
defiStyles_Delete
  PROTO_PARAMS(( defiStyles * this ));

EXTERN void
defiStyles_clear
  PROTO_PARAMS(( defiStyles * this ));

EXTERN void
defiStyles_setStyle
  PROTO_PARAMS(( defiStyles * this,
                 int styleNum ));

EXTERN void
defiStyles_setPolygon
  PROTO_PARAMS(( defiStyles * this,
                 defiGeometries * geom ));

EXTERN int
defiStyles_style
  PROTO_PARAMS(( const defiStyles * this ));

EXTERN struct defiPoints
defiStyles_getPolygon
  PROTO_PARAMS(( const defiStyles * this ));

#endif
