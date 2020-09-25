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

#ifndef defiSLOT_h
#define defiSLOT_h
#include <stdio.h>
#include "defiKRDefs.h"
#include "defiMisc.h"

/*
 * 5.6
 */

/*
 * 5.6
 */

/*
 * 5.6
 */

/*
 * 5.6
 */

/*
 * 5.6
 */

typedef struct defiSlot_s {
  int             hasLayer_;
  char           *layerName_;
  int             layerNameLength_;
  int             numRectangles_;
  int             rectsAllocated_;
  int            *xl_;
  int            *yl_;
  int            *xh_;
  int            *yh_;
  int             numPolys_;
  int             polysAllocated_;
  struct defiPoints **polygons_;
} defiSlot;

EXTERN defiSlot *
defiSlot_Create
  PROTO_PARAMS((  ));

EXTERN void
defiSlot_Init
  PROTO_PARAMS(( defiSlot * this ));

EXTERN void
defiSlot_Destroy
  PROTO_PARAMS(( defiSlot * this ));

EXTERN void
defiSlot_Delete
  PROTO_PARAMS(( defiSlot * this ));

EXTERN void
defiSlot_clear
  PROTO_PARAMS(( defiSlot * this ));

EXTERN void
defiSlot_clearPoly
  PROTO_PARAMS(( defiSlot * this ));

EXTERN void
defiSlot_setLayer
  PROTO_PARAMS(( defiSlot * this,
                 const char *name ));

EXTERN void
defiSlot_addRect
  PROTO_PARAMS(( defiSlot * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiSlot_addPolygon
  PROTO_PARAMS(( defiSlot * this,
                 defiGeometries * geom ));

EXTERN int
defiSlot_hasLayer
  PROTO_PARAMS(( const defiSlot * this ));

EXTERN const char *
defiSlot_layerName
  PROTO_PARAMS(( const defiSlot * this ));

EXTERN int
defiSlot_numRectangles
  PROTO_PARAMS(( const defiSlot * this ));

EXTERN int
defiSlot_xl
  PROTO_PARAMS(( const defiSlot * this,
                 int index ));

EXTERN int
defiSlot_yl
  PROTO_PARAMS(( const defiSlot * this,
                 int index ));

EXTERN int
defiSlot_xh
  PROTO_PARAMS(( const defiSlot * this,
                 int index ));

EXTERN int
defiSlot_yh
  PROTO_PARAMS(( const defiSlot * this,
                 int index ));

EXTERN int
defiSlot_numPolygons
  PROTO_PARAMS(( const defiSlot * this ));

EXTERN struct defiPoints
defiSlot_getPolygon
  PROTO_PARAMS(( const defiSlot * this,
                 int index ));

EXTERN void
defiSlot_print
  PROTO_PARAMS(( const defiSlot * this,
                 FILE * f ));

#endif
