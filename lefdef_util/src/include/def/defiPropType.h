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

#ifndef defiPropType_h
#define defiPropType_h
#include "defiKRDefs.h"
#include <stdio.h>

/*
 * Struct holds the data type for one property, if the property is
 */

/*
 * either REAL or INTEGER.
 */

/*
 * name.
 */

/*
 * 'R' == "REAL", 'I' == "INTEGER"
 */

typedef struct defiPropType_s {
  int             numProperties_;
  int             propertiesAllocated_;
  char          **propNames_;
  char           *propTypes_;
} defiPropType;

EXTERN defiPropType *
defiPropType_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPropType_Init
  PROTO_PARAMS(( defiPropType * this ));

EXTERN void
defiPropType_Destroy
  PROTO_PARAMS(( defiPropType * this ));

EXTERN void
defiPropType_Delete
  PROTO_PARAMS(( defiPropType * this ));

EXTERN void
defiPropType_setPropType
  PROTO_PARAMS(( defiPropType * this,
                 const char *name,
                 const char type ));

EXTERN void
defiPropType_Clear
  PROTO_PARAMS(( defiPropType * this ));

EXTERN const char
defiPropType_propType
  PROTO_PARAMS(( const defiPropType * this,
                 char *name ));

EXTERN void
defiPropType_bumpProps
  PROTO_PARAMS(( defiPropType * this ));

#endif
