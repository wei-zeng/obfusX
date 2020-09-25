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

#ifndef defiProp_h
#define defiProp_h
#include "defiKRDefs.h"
#include <stdio.h>

/*
 * Struct holds the data for one property.
 */

/*
 * either I:integer R:real S:string Q:quotedstring N:nameMapString
 */

/*
 * "design" ...
 */

/*
 * name.
 */

/*
 * allocated size of name.
 */

/*
 * either 0:NO or 1:YES.
 */

/*
 * either 0:NO or 1:YES.
 */

/*
 * either I:integer R:real S:string Q:quotedstring.
 */

/*
 * N:nameMapString
 */

/*
 * if it is a string the data is here.
 */

/*
 * allocated size of stringData.
 */

/*
 * if it has a range the numbers are here.
 */

/*
 * if it is a real or int the number is here.
 */

typedef struct defiProp_s {
  char           *propType_;
  char           *propName_;
  int             nameSize_;
  char            hasRange_;
  char            hasNumber_;
  char            hasNameMapString_;
  char            dataType_;
  char           *stringData_;
  int             stringLength_;
  double          left_, right_;
  double          d_;
} defiProp;

EXTERN defiProp *
defiProp_Create
  PROTO_PARAMS((  ));

EXTERN void
defiProp_Init
  PROTO_PARAMS(( defiProp * this ));

EXTERN void
defiProp_Destroy
  PROTO_PARAMS(( defiProp * this ));

EXTERN void
defiProp_Delete
  PROTO_PARAMS(( defiProp * this ));

EXTERN void
defiProp_setPropType
  PROTO_PARAMS(( defiProp * this,
                 const char *typ,
                 const char *string ));

EXTERN void
defiProp_setRange
  PROTO_PARAMS(( defiProp * this,
                 double left,
                 double right ));

EXTERN void
defiProp_setNumber
  PROTO_PARAMS(( defiProp * this,
                 double num ));

EXTERN void
defiProp_setPropInteger
  PROTO_PARAMS(( defiProp * this ));

EXTERN void
defiProp_setPropReal
  PROTO_PARAMS(( defiProp * this ));

EXTERN void
defiProp_setPropString
  PROTO_PARAMS(( defiProp * this ));

EXTERN void
defiProp_setPropQString
  PROTO_PARAMS(( defiProp * this,
                 const char *string ));

EXTERN void
defiProp_setPropNameMapString
  PROTO_PARAMS(( defiProp * this,
                 const char *string ));

EXTERN void
defiProp_clear
  PROTO_PARAMS(( defiProp * this ));

EXTERN const char *
defiProp_string
  PROTO_PARAMS(( const defiProp * this ));

EXTERN const char *
defiProp_propType
  PROTO_PARAMS(( const defiProp * this ));

EXTERN const char *
defiProp_propName
  PROTO_PARAMS(( const defiProp * this ));

EXTERN char
defiProp_dataType
  PROTO_PARAMS(( const defiProp * this ));

EXTERN int
defiProp_hasNumber
  PROTO_PARAMS(( const defiProp * this ));

EXTERN int
defiProp_hasRange
  PROTO_PARAMS(( const defiProp * this ));

EXTERN int
defiProp_hasString
  PROTO_PARAMS(( const defiProp * this ));

EXTERN int
defiProp_hasNameMapString
  PROTO_PARAMS(( const defiProp * this ));

EXTERN double
defiProp_number
  PROTO_PARAMS(( const defiProp * this ));

EXTERN double
defiProp_left
  PROTO_PARAMS(( const defiProp * this ));

EXTERN double
defiProp_right
  PROTO_PARAMS(( const defiProp * this ));

EXTERN void
defiProp_bumpSize
  PROTO_PARAMS(( defiProp * this,
                 int size ));

EXTERN void
defiProp_bumpName
  PROTO_PARAMS(( defiProp * this,
                 int size ));

EXTERN void
defiProp_print
  PROTO_PARAMS(( const defiProp * this,
                 FILE * f ));

#endif
