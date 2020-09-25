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

#ifndef defiFPC_h
#define defiFPC_h
#include <stdio.h>
#include "defiKRDefs.h"

/*
 * Return the constraint number "index" where index is
 */

/*
 * from 0 to numParts()
 */

/*
 * The returned corner is 'B' for bottom left  'T' for topright
 */

/*
 * The returned typ is 'R' for rows   'C' for comps
 */

/*
 * The returned char* points to name of the item.
 */

/*
 * debug print
 */

/*
 * H or V
 */

/*
 * Bottomleft or Topright
 */

/*
 * allocated size of names_ and rowOrComp_
 */

/*
 * number of entries used in the arrays
 */

typedef struct defiFPC_s {
  char           *name_;
  int             nameLength_;
  char            direction_;
  char            hasAlign_;
  char            hasMin_;
  char            hasMax_;
  char            hasEqual_;
  char            corner_;
  double          minMaxEqual_;
  int             namesAllocated_;
  int             namesUsed_;
  char           *rowOrComp_;
  char          **names_;
} defiFPC;

EXTERN defiFPC *
defiFPC_Create
  PROTO_PARAMS((  ));

EXTERN void
defiFPC_Init
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_Destroy
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_Delete
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_clear
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_setName
  PROTO_PARAMS(( defiFPC * this,
                 const char *name,
                 const char *direction ));

EXTERN void
defiFPC_setAlign
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_setMax
  PROTO_PARAMS(( defiFPC * this,
                 double num ));

EXTERN void
defiFPC_setMin
  PROTO_PARAMS(( defiFPC * this,
                 double num ));

EXTERN void
defiFPC_setEqual
  PROTO_PARAMS(( defiFPC * this,
                 double num ));

EXTERN void
defiFPC_setDoingBottomLeft
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_setDoingTopRight
  PROTO_PARAMS(( defiFPC * this ));

EXTERN void
defiFPC_addRow
  PROTO_PARAMS(( defiFPC * this,
                 const char *name ));

EXTERN void
defiFPC_addComps
  PROTO_PARAMS(( defiFPC * this,
                 const char *name ));

EXTERN void
defiFPC_addItem
  PROTO_PARAMS(( defiFPC * this,
                 char typ,
                 const char *name ));

EXTERN const char *
defiFPC_name
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_isVertical
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_isHorizontal
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_hasAlign
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_hasMax
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_hasMin
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_hasEqual
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN double
defiFPC_alignMax
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN double
defiFPC_alignMin
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN double
defiFPC_equal
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN int
defiFPC_numParts
  PROTO_PARAMS(( const defiFPC * this ));

EXTERN void
defiFPC_getPart
  PROTO_PARAMS(( const defiFPC * this,
                 int index,
                 int *corner,
                 int *typ,
                 char **name ));

EXTERN void
defiFPC_print
  PROTO_PARAMS(( const defiFPC * this,
                 FILE * f ));

#endif
