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

#ifndef defiVia_h
#define defiVia_h
#include "defiKRDefs.h"
#include <stdio.h>
#include "defiMisc.h"

/*
 * Struct holds the data for one property.
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

/*
 * 5.6
 */

/*
 * 5.6 layerName for POLYGON
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

/*
 * 5.6
 */

typedef struct defiVia_s {
  char           *name_;
  int             nameLength_;
  char           *pattern_;
  int             patternLength_;
  char          **layers_;
  int             hasPattern_;
  int            *xl_;
  int            *yl_;
  int            *xh_;
  int            *yh_;
  int             layersLength_;
  int             numLayers_;
  int             numPolys_;
  char          **polygonNames_;
  int             polysAllocated_;
  struct defiPoints **polygons_;
  char           *viaRule_;
  int             viaRuleLength_;
  int             hasViaRule_;
  int             xSize_;
  int             ySize_;
  char           *botLayer_;
  char           *cutLayer_;
  char           *topLayer_;
  int             botLayerLength_;
  int             cutLayerLength_;
  int             topLayerLength_;
  int             xCutSpacing_;
  int             yCutSpacing_;
  int             xBotEnc_;
  int             yBotEnc_;
  int             xTopEnc_;
  int             yTopEnc_;
  int             rows_;
  int             cols_;
  int             xOffset_;
  int             yOffset_;
  int             xBotOffset_;
  int             yBotOffset_;
  int             xTopOffset_;
  int             yTopOffset_;
  char           *cutPattern_;
  int             cutPatternLength_;
  int             hasCutPattern_;
} defiVia;

EXTERN defiVia *
defiVia_Create
  PROTO_PARAMS((  ));

EXTERN void
defiVia_Init
  PROTO_PARAMS(( defiVia * this ));

EXTERN void
defiVia_clear
  PROTO_PARAMS(( defiVia * this ));

EXTERN void
defiVia_Destroy
  PROTO_PARAMS(( defiVia * this ));

EXTERN void
defiVia_Delete
  PROTO_PARAMS(( defiVia * this ));

EXTERN void
defiVia_setup
  PROTO_PARAMS(( defiVia * this,
                 const char *name ));

EXTERN void
defiVia_addPattern
  PROTO_PARAMS(( defiVia * this,
                 const char *patt ));

EXTERN void
defiVia_addLayer
  PROTO_PARAMS(( defiVia * this,
                 const char *layer,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiVia_addPolygon
  PROTO_PARAMS(( defiVia * this,
                 const char *layer,
                 defiGeometries * geom ));

EXTERN void
defiVia_addViaRule
  PROTO_PARAMS(( defiVia * this,
                 char *viaRuleName,
                 int xSize,
                 int ySize,
                 char *botLayer,
                 char *cutLayer,
                 char *topLayer,
                 int xSpacing,
                 int ySpacing,
                 int xBotEnc,
                 int yBotEnc,
                 int xTopEnc,
                 int yTopEnc ));

EXTERN void
defiVia_addRowCol
  PROTO_PARAMS(( defiVia * this,
                 int numCutRows,
                 int numCutCols ));

EXTERN void
defiVia_addOrigin
  PROTO_PARAMS(( defiVia * this,
                 int xOffset,
                 int yOffset ));

EXTERN void
defiVia_addOffset
  PROTO_PARAMS(( defiVia * this,
                 int xBotOs,
                 int yBotOs,
                 int xTopOs,
                 int yTopOs ));

EXTERN void
defiVia_addCutPattern
  PROTO_PARAMS(( defiVia * this,
                 char *cutPattern ));

EXTERN const char *
defiVia_name
  PROTO_PARAMS(( const defiVia * this ));

EXTERN const char *
defiVia_pattern
  PROTO_PARAMS(( const defiVia * this ));

EXTERN int
defiVia_hasPattern
  PROTO_PARAMS(( const defiVia * this ));

EXTERN int
defiVia_numLayers
  PROTO_PARAMS(( const defiVia * this ));

EXTERN void
defiVia_layer
  PROTO_PARAMS(( const defiVia * this,
                 int index,
                 char **layer,
                 int *xl,
                 int *yl,
                 int *xh,
                 int *yh ));

EXTERN int
defiVia_numPolygons
  PROTO_PARAMS(( const defiVia * this ));

EXTERN const char *
defiVia_polygonName
  PROTO_PARAMS(( const defiVia * this,
                 int index ));

EXTERN struct defiPoints
defiVia_getPolygon
  PROTO_PARAMS(( const defiVia * this,
                 int index ));

EXTERN int
defiVia_hasViaRule
  PROTO_PARAMS(( const defiVia * this ));

EXTERN void
defiVia_viaRule
  PROTO_PARAMS(( const defiVia * this,
                 char **viaRuleName,
                 int *xSize,
                 int *ySize,
                 char **botLayer,
                 char **cutLayer,
                 char **topLayer,
                 int *xCutSpacing,
                 int *yCutSpacing,
                 int *xBotEnc,
                 int *yBotEnc,
                 int *xTopEnc,
                 int *yTopEnc ));

EXTERN int
defiVia_hasRowCol
  PROTO_PARAMS(( const defiVia * this ));

EXTERN void
defiVia_rowCol
  PROTO_PARAMS(( const defiVia * this,
                 int *numCutRows,
                 int *numCutCols ));

EXTERN int
defiVia_hasOrigin
  PROTO_PARAMS(( const defiVia * this ));

EXTERN void
defiVia_origin
  PROTO_PARAMS(( const defiVia * this,
                 int *xOffset,
                 int *yOffset ));

EXTERN int
defiVia_hasOffset
  PROTO_PARAMS(( const defiVia * this ));

EXTERN void
defiVia_offset
  PROTO_PARAMS(( const defiVia * this,
                 int *xBotOffset,
                 int *yBotOffset,
                 int *xTopOffset,
                 int *yTopOffset ));

EXTERN int
defiVia_hasCutPattern
  PROTO_PARAMS(( const defiVia * this ));

EXTERN const char *
defiVia_cutPattern
  PROTO_PARAMS(( const defiVia * this ));

EXTERN void
defiVia_print
  PROTO_PARAMS(( const defiVia * this,
                 FILE * f ));

#endif
