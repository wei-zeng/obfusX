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

/*
 * User header file for the DEF Interface.  This includes all of the header
 * files which are relevant to both the reader and the writer.
 * 
 * defrReader.h and defwWriter.h include this file, so that an application only
 * needs to include either defwReader.h or defwWriter.h.
 */

#ifndef DEFI_USER_H
#define DEFI_USER_H
/* General utilities. */
/* #include "defiMalloc.h" */
/* #include "defiUtils.h" */

/*
 * API objects
 */
#include "defiDebug.h"
#include "defiProp.h"
#include "defiSite.h"
#include "defiComponent.h"
#include "defiNet.h"
#include "defiPath.h"
#include "defiPinCap.h"
#include "defiRowTrack.h"
#include "defiVia.h"
#include "defiRegion.h"
#include "defiGroup.h"
#include "defiAssertion.h"
#include "defiScanchain.h"
#include "defiIOTiming.h"
#include "defiFPC.h"
#include "defiTimingDisable.h"
#include "defiPartition.h"
#include "defiPinProp.h"
#include "defiBlockage.h"
#include "defiSlot.h"
#include "defiFill.h"
#include "defiNonDefault.h"
#include "defiPropType.h"

/*
 * NEW CALLBACK - If you are creating a new .cpp and .hpp file to describe a
 * new class of object in the parser, then add a reference to the .hpp here.
 * 
 * You must also add an entry for the .h and the .hpp in the package_list file
 * of the ../../../release directory.
 */
#endif
