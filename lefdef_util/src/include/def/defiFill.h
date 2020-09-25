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

#ifndef defiFILL_h
#define defiFILL_h
#include <stdio.h>
#include "defiKRDefs.h"
#include "defiMisc.h"

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.6
 */

/*
 * 5.6
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
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
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

/*
 * 5.7
 */

typedef struct defiFill_s {
  int             hasLayer_;
  char           *layerName_;
  int             layerNameLength_;
  int             layerOpc_;
  int             numRectangles_;
  int             rectsAllocated_;
  int            *xl_;
  int            *yl_;
  int            *xh_;
  int            *yh_;
  int             numPolys_;
  int             polysAllocated_;
  struct defiPoints **polygons_;
  int             hasVia_;
  char           *viaName_;
  int             viaNameLength_;
  int             viaOpc_;
  int             numPts_;
  int             ptsAllocated_;
  struct defiPoints **viaPts_;
} defiFill;

EXTERN defiFill *
defiFill_Create
  PROTO_PARAMS((  ));

EXTERN void
defiFill_Init
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_Destroy
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_Delete
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_clear
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_clearPoly
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_clearPts
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_setLayer
  PROTO_PARAMS(( defiFill * this,
                 const char *name ));

EXTERN void
defiFill_setLayerOpc
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_addRect
  PROTO_PARAMS(( defiFill * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiFill_addPolygon
  PROTO_PARAMS(( defiFill * this,
                 defiGeometries * geom ));

EXTERN void
defiFill_setVia
  PROTO_PARAMS(( defiFill * this,
                 const char *name ));

EXTERN void
defiFill_setViaOpc
  PROTO_PARAMS(( defiFill * this ));

EXTERN void
defiFill_addPts
  PROTO_PARAMS(( defiFill * this,
                 defiGeometries * geom ));

EXTERN int
defiFill_hasLayer
  PROTO_PARAMS(( const defiFill * this ));

EXTERN const char *
defiFill_layerName
  PROTO_PARAMS(( const defiFill * this ));

EXTERN int
defiFill_hasLayerOpc
  PROTO_PARAMS(( const defiFill * this ));

EXTERN int
defiFill_numRectangles
  PROTO_PARAMS(( const defiFill * this ));

EXTERN int
defiFill_xl
  PROTO_PARAMS(( const defiFill * this,
                 int index ));

EXTERN int
defiFill_yl
  PROTO_PARAMS(( const defiFill * this,
                 int index ));

EXTERN int
defiFill_xh
  PROTO_PARAMS(( const defiFill * this,
                 int index ));

EXTERN int
defiFill_yh
  PROTO_PARAMS(( const defiFill * this,
                 int index ));

EXTERN int
defiFill_numPolygons
  PROTO_PARAMS(( const defiFill * this ));

EXTERN struct defiPoints
defiFill_getPolygon
  PROTO_PARAMS(( const defiFill * this,
                 int index ));

EXTERN int
defiFill_hasVia
  PROTO_PARAMS(( const defiFill * this ));

EXTERN const char *
defiFill_viaName
  PROTO_PARAMS(( const defiFill * this ));

EXTERN int
defiFill_hasViaOpc
  PROTO_PARAMS(( const defiFill * this ));

EXTERN int
defiFill_numViaPts
  PROTO_PARAMS(( const defiFill * this ));

EXTERN struct defiPoints
defiFill_getViaPts
  PROTO_PARAMS(( const defiFill * this,
                 int index ));

EXTERN void
defiFill_print
  PROTO_PARAMS(( const defiFill * this,
                 FILE * f ));

#endif
