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

#ifndef defiNonDefault_h
#define defiNonDefault_h
#include <stdio.h>
#include "defiKRDefs.h"
#include "defiMisc.h"

/*
 * A non default rule can have one or more layers.
 */

/*
 * The layer information is kept in an array.
 */

/*
 * Will be obsoleted in 5.7
 */

/*
 * Will be obsoleted in 5.7
 */

/*
 * Will be obsoleted in 5.7
 */

/*
 * Will be obsoleted in 5.7
 */

/*
 * Debug print
 */

/*
 * Layer information
 */

typedef struct defiNonDefault_s {
  char           *name_;
  char            hardSpacing_;
  int             numLayers_;
  int             layersAllocated_;
  char          **layerName_;
  double         *width_;
  char           *hasDiagWidth_;
  double         *diagWidth_;
  char           *hasSpacing_;
  double         *spacing_;
  char           *hasWireExt_;
  double         *wireExt_;
  int             numVias_;
  int             viasAllocated_;
  char          **viaNames_;
  int             numViaRules_;
  int             viaRulesAllocated_;
  char          **viaRuleNames_;
  int             numMinCuts_;
  int             minCutsAllocated_;
  char          **cutLayerName_;
  int            *numCuts_;
  int             numProps_;
  int             propsAllocated_;
  char          **names_;
  char          **values_;
  double         *dvalues_;
  char           *types_;
} defiNonDefault;

EXTERN defiNonDefault *
defiNonDefault_Create
  PROTO_PARAMS((  ));

EXTERN void
defiNonDefault_Init
  PROTO_PARAMS(( defiNonDefault * this ));

EXTERN void
defiNonDefault_Destroy
  PROTO_PARAMS(( defiNonDefault * this ));

EXTERN void
defiNonDefault_Delete
  PROTO_PARAMS(( defiNonDefault * this ));

EXTERN void
defiNonDefault_clear
  PROTO_PARAMS(( defiNonDefault * this ));

EXTERN void
defiNonDefault_setName
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name ));

EXTERN void
defiNonDefault_setHardspacing
  PROTO_PARAMS(( defiNonDefault * this ));

EXTERN void
defiNonDefault_addLayer
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name ));

EXTERN void
defiNonDefault_addWidth
  PROTO_PARAMS(( defiNonDefault * this,
                 double num ));

EXTERN void
defiNonDefault_addDiagWidth
  PROTO_PARAMS(( defiNonDefault * this,
                 double num ));

EXTERN void
defiNonDefault_addSpacing
  PROTO_PARAMS(( defiNonDefault * this,
                 double num ));

EXTERN void
defiNonDefault_addWireExt
  PROTO_PARAMS(( defiNonDefault * this,
                 double num ));

EXTERN void
defiNonDefault_addVia
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name ));

EXTERN void
defiNonDefault_addViaRule
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name ));

EXTERN void
defiNonDefault_addMinCuts
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name,
                 int numCuts ));

EXTERN void
defiNonDefault_addProperty
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name,
                 const char *value,
                 const char type ));

EXTERN void
defiNonDefault_addNumProperty
  PROTO_PARAMS(( defiNonDefault * this,
                 const char *name,
                 const double d,
                 const char *value,
                 const char type ));

EXTERN void
defiNonDefault_end
  PROTO_PARAMS(( defiNonDefault * this ));

EXTERN const char *
defiNonDefault_name
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN int
defiNonDefault_hasHardspacing
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN int
defiNonDefault_numProps
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN const char *
defiNonDefault_propName
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN const char *
defiNonDefault_propValue
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN double
defiNonDefault_propNumber
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN const char
defiNonDefault_propType
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_propIsNumber
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_propIsString
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_numLayers
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN const char *
defiNonDefault_layerName
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN double
defiNonDefault_layerWidth
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_layerWidthVal
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_hasLayerDiagWidth
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN double
defiNonDefault_layerDiagWidth
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_layerDiagWidthVal
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_hasLayerSpacing
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN double
defiNonDefault_layerSpacing
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_layerSpacingVal
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_hasLayerWireExt
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN double
defiNonDefault_layerWireExt
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_layerWireExtVal
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_numVias
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN const char *
defiNonDefault_viaName
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_numViaRules
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN const char *
defiNonDefault_viaRuleName
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_numMinCuts
  PROTO_PARAMS(( const defiNonDefault * this ));

EXTERN const char *
defiNonDefault_cutLayerName
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN int
defiNonDefault_numCuts
  PROTO_PARAMS(( const defiNonDefault * this,
                 int index ));

EXTERN void
defiNonDefault_print
  PROTO_PARAMS(( defiNonDefault * this,
                 FILE * f ));

#endif
