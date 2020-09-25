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

#ifndef defiPath_h
#define defiPath_h
#include <stdio.h>
#include "defiKRDefs.h"
struct defiPnt {
  int             x;
  int             y;
  int             ext;
};

/*
 * 5.4.1 1-D & 2-D Arrays of Vias in SPECIALNET Section
 */
struct defiViaData {
  int             numX;
  int             numY;
  int             stepX;
  int             stepY;
};

/*
 * value returned by the next() routine.
 */
enum defiPath_e {
  DEFIPATH_DONE = 0,
  DEFIPATH_LAYER,
  DEFIPATH_VIA,
  DEFIPATH_VIAROTATION,
  DEFIPATH_WIDTH,
  DEFIPATH_POINT,
  DEFIPATH_FLUSHPOINT,
  DEFIPATH_TAPER,
  DEFIPATH_SHAPE,
  DEFIPATH_STYLE,
  DEFIPATH_TAPERRULE,
  DEFIPATH_VIADATA

};

/*
 * To traverse the path and get the parts.
 */

/*
 * Initialize the traverse.
 */

/*
 * Initialize the traverse in reverse.
 */

/*
 * Get the next element.
 */

/*
 * Get the next element in reverse.
 */

/*
 * Get the layer.
 */

/*
 * Get the rule.
 */

/*
 * Get the via.
 */

/*
 * Get the shape.
 */

/*
 * Get the style.
 */

/*
 * Get the via rotation.
 */

/*
 * Return via rotation in string format
 */

/*
 * 5.4.1
 */

/*
 * Get the width.
 */

/*
 * Get the point.
 */

/*
 * Get the point.
 */

/*
 * These routines are called by the parser to fill the path.
 */

/*
 * 5.4.1
 */

/*
 * debug printing
 */

/*
 * type of item in path
 */

/*
 * extra data
 */

/*
 * number of items used in array
 */

/*
 * allocated size of keys and data
 */

/*
 * traversal pointer
 */

typedef struct defiPath_s {
  int            *keys_;
  void          **data_;
  int             numUsed_;
  int             numAllocated_;
  int             pointer_;
  int             numX_;
  int             numY_;
  int             stepX_;
  int             stepY_;
} defiPath;

EXTERN defiPath *
defiPath_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPath_Init
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_Destroy
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_Delete
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_clear
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_reverseOrder
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_initTraverse
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_initTraverseBackwards
  PROTO_PARAMS(( defiPath * this ));

EXTERN int
defiPath_next
  PROTO_PARAMS(( defiPath * this ));

EXTERN int
defiPath_prev
  PROTO_PARAMS(( defiPath * this ));

EXTERN const char *
defiPath_getLayer
  PROTO_PARAMS(( defiPath * this ));

EXTERN const char *
defiPath_getTaperRule
  PROTO_PARAMS(( defiPath * this ));

EXTERN const char *
defiPath_getVia
  PROTO_PARAMS(( defiPath * this ));

EXTERN const char *
defiPath_getShape
  PROTO_PARAMS(( defiPath * this ));

EXTERN int
defiPath_getStyle
  PROTO_PARAMS(( defiPath * this ));

EXTERN int
defiPath_getViaRotation
  PROTO_PARAMS(( defiPath * this ));

EXTERN const char *
defiPath_getViaRotationStr
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_getViaData
  PROTO_PARAMS(( defiPath * this,
                 int *numX,
                 int *numY,
                 int *stepX,
                 int *stepY ));

EXTERN int
defiPath_getWidth
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_getPoint
  PROTO_PARAMS(( defiPath * this,
                 int *x,
                 int *y ));

EXTERN void
defiPath_getFlushPoint
  PROTO_PARAMS(( defiPath * this,
                 int *x,
                 int *y,
                 int *ext ));

EXTERN void
defiPath_addWidth
  PROTO_PARAMS(( defiPath * this,
                 int w ));

EXTERN void
defiPath_addPoint
  PROTO_PARAMS(( defiPath * this,
                 int x,
                 int y ));

EXTERN void
defiPath_addFlushPoint
  PROTO_PARAMS(( defiPath * this,
                 int x,
                 int y,
                 int ext ));

EXTERN void
defiPath_addLayer
  PROTO_PARAMS(( defiPath * this,
                 const char *layer ));

EXTERN void
defiPath_addVia
  PROTO_PARAMS(( defiPath * this,
                 const char *name ));

EXTERN void
defiPath_addViaRotation
  PROTO_PARAMS(( defiPath * this,
                 int orient ));

EXTERN void
defiPath_addViaData
  PROTO_PARAMS(( defiPath * this,
                 int numX,
                 int numY,
                 int stepX,
                 int stepY ));

EXTERN void
defiPath_setTaper
  PROTO_PARAMS(( defiPath * this ));

EXTERN void
defiPath_addTaperRule
  PROTO_PARAMS(( defiPath * this,
                 const char *rule ));

EXTERN void
defiPath_addShape
  PROTO_PARAMS(( defiPath * this,
                 const char *shape ));

EXTERN void
defiPath_addStyle
  PROTO_PARAMS(( defiPath * this,
                 int style ));

EXTERN void
defiPath_print
  PROTO_PARAMS(( defiPath * this,
                 FILE * fout ));

EXTERN void
defiPath_bumpSize
  PROTO_PARAMS(( defiPath * this,
                 int size ));

#endif
