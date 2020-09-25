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

#ifndef defiPinCap_h
#define defiPinCap_h
#include "defiKRDefs.h"
#include "defiMisc.h"
#include <stdio.h>

/*
 * pin num
 */

/*
 * capacitance
 */

typedef struct defiPinCap_s {
  int             pin_;
  double          cap_;
} defiPinCap;

EXTERN void
defiPinCap_setPin
  PROTO_PARAMS(( defiPinCap * this,
                 int p ));

EXTERN void
defiPinCap_setCap
  PROTO_PARAMS(( defiPinCap * this,
                 double d ));

EXTERN int
defiPinCap_pin
  PROTO_PARAMS(( const defiPinCap * this ));

EXTERN double
defiPinCap_cap
  PROTO_PARAMS(( const defiPinCap * this ));

EXTERN void
defiPinCap_print
  PROTO_PARAMS(( const defiPinCap * this,
                 FILE * f ));

/*
 * 5.5
 */

/*
 * ANTENNAPINGATEAREA
 */

/*
 * ANTENNAPINMAXAREACAR
 */

/*
 * ANTENNAPINMAXSIDEAREACAR
 */

/*
 * ANTENNAPINMAXCUTCAR
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinGateArea
 */

/*
 * 5.4 Layer
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinMaxAreaCar
 */

/*
 * 5.4 Layer
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinMaxSideAreaCar
 */

/*
 * 5.4 Layer
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinMaxCutCar
 */

/*
 * 5.4 Layer
 */

typedef struct defiPinAntennaModel_s {
  char           *oxide_;
  int             numAPinGateArea_;
  int             APinGateAreaAllocated_;
  int            *APinGateArea_;
  char          **APinGateAreaLayer_;
  int             numAPinMaxAreaCar_;
  int             APinMaxAreaCarAllocated_;
  int            *APinMaxAreaCar_;
  char          **APinMaxAreaCarLayer_;
  int             numAPinMaxSideAreaCar_;
  int             APinMaxSideAreaCarAllocated_;
  int            *APinMaxSideAreaCar_;
  char          **APinMaxSideAreaCarLayer_;
  int             numAPinMaxCutCar_;
  int             APinMaxCutCarAllocated_;
  int            *APinMaxCutCar_;
  char          **APinMaxCutCarLayer_;
} defiPinAntennaModel;

EXTERN defiPinAntennaModel *
defiPinAntennaModel_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPinAntennaModel_Init
  PROTO_PARAMS(( defiPinAntennaModel * this ));

EXTERN void
defiPinAntennaModel_Delete
  PROTO_PARAMS(( defiPinAntennaModel * this ));

EXTERN void
defiPinAntennaModel_clear
  PROTO_PARAMS(( defiPinAntennaModel * this ));

EXTERN void
defiPinAntennaModel_Destroy
  PROTO_PARAMS(( defiPinAntennaModel * this ));

EXTERN void
defiPinAntennaModel_setAntennaModel
  PROTO_PARAMS(( defiPinAntennaModel * this,
                 int oxide ));

EXTERN void
defiPinAntennaModel_addAPinGateArea
  PROTO_PARAMS(( defiPinAntennaModel * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPinAntennaModel_addAPinMaxAreaCar
  PROTO_PARAMS(( defiPinAntennaModel * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPinAntennaModel_addAPinMaxSideAreaCar
  PROTO_PARAMS(( defiPinAntennaModel * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPinAntennaModel_addAPinMaxCutCar
  PROTO_PARAMS(( defiPinAntennaModel * this,
                 int value,
                 const char *layer ));

EXTERN char *
defiPinAntennaModel_antennaOxide
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_hasAPinGateArea
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_numAPinGateArea
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_APinGateArea
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinGateAreaLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN const char *
defiPinAntennaModel_APinGateAreaLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinMaxAreaCar
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_numAPinMaxAreaCar
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_APinMaxAreaCar
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinMaxAreaCarLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN const char *
defiPinAntennaModel_APinMaxAreaCarLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinMaxSideAreaCar
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_numAPinMaxSideAreaCar
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_APinMaxSideAreaCar
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinMaxSideAreaCarLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN const char *
defiPinAntennaModel_APinMaxSideAreaCarLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinMaxCutCar
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_numAPinMaxCutCar
  PROTO_PARAMS(( const defiPinAntennaModel * this ));

EXTERN int
defiPinAntennaModel_APinMaxCutCar
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN int
defiPinAntennaModel_hasAPinMaxCutCarLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

EXTERN const char *
defiPinAntennaModel_APinMaxCutCarLayer
  PROTO_PARAMS(( const defiPinAntennaModel * this,
                 int index ));

/*
 * 5.7
 */

typedef struct defiPinPort_s {
  int             layersAllocated_;
  int             numLayers_;
  char          **layers_;
  int            *layerMinSpacing_;
  int            *layerEffectiveWidth_;
  int            *xl_, *yl_, *xh_, *yh_;
  int             polysAllocated_;
  int             numPolys_;
  char          **polygonNames_;
  int            *polyMinSpacing_;
  int            *polyEffectiveWidth_;
  struct defiPoints **polygons_;
  int             viasAllocated_;
  int             numVias_;
  char          **viaNames_;
  int            *viaX_;
  int            *viaY_;
  char            placeType_;
  int             x_, y_;
  char            orient_;
} defiPinPort;

EXTERN defiPinPort *
defiPinPort_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPinPort_Init
  PROTO_PARAMS(( defiPinPort * this ));

EXTERN void
defiPinPort_Delete
  PROTO_PARAMS(( defiPinPort * this ));

EXTERN void
defiPinPort_clear
  PROTO_PARAMS(( defiPinPort * this ));

EXTERN void
defiPinPort_addLayer
  PROTO_PARAMS(( defiPinPort * this,
                 const char *layer ));

EXTERN void
defiPinPort_addLayerSpacing
  PROTO_PARAMS(( defiPinPort * this,
                 int minSpacing ));

EXTERN void
defiPinPort_addLayerDesignRuleWidth
  PROTO_PARAMS(( defiPinPort * this,
                 int effectiveWidth ));

EXTERN void
defiPinPort_addLayerPts
  PROTO_PARAMS(( defiPinPort * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiPinPort_addPolygon
  PROTO_PARAMS(( defiPinPort * this,
                 const char *layerName ));

EXTERN void
defiPinPort_addPolySpacing
  PROTO_PARAMS(( defiPinPort * this,
                 int minSpacing ));

EXTERN void
defiPinPort_addPolyDesignRuleWidth
  PROTO_PARAMS(( defiPinPort * this,
                 int effectiveWidth ));

EXTERN void
defiPinPort_addPolygonPts
  PROTO_PARAMS(( defiPinPort * this,
                 defiGeometries * geom ));

EXTERN void
defiPinPort_addVia
  PROTO_PARAMS(( defiPinPort * this,
                 const char *via,
                 int viaX,
                 int viaY ));

EXTERN void
defiPinPort_setPlacement
  PROTO_PARAMS(( defiPinPort * this,
                 int typ,
                 int x,
                 int y,
                 int orient ));

EXTERN int
defiPinPort_numLayer
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN const char *
defiPinPort_layer
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN void
defiPinPort_bounds
  PROTO_PARAMS(( const defiPinPort * this,
                 int index,
                 int *xl,
                 int *yl,
                 int *xh,
                 int *yh ));

EXTERN int
defiPinPort_hasLayerSpacing
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_hasLayerDesignRuleWidth
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_layerSpacing
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_layerDesignRuleWidth
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_numPolygons
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN const char *
defiPinPort_polygonName
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN struct defiPoints
defiPinPort_getPolygon
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_hasPolygonSpacing
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_hasPolygonDesignRuleWidth
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_polygonSpacing
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_polygonDesignRuleWidth
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_numVias
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN const char *
defiPinPort_viaName
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_viaPtX
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_viaPtY
  PROTO_PARAMS(( const defiPinPort * this,
                 int index ));

EXTERN int
defiPinPort_hasPlacement
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN int
defiPinPort_isPlaced
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN int
defiPinPort_isCover
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN int
defiPinPort_isFixed
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN int
defiPinPort_placementX
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN int
defiPinPort_placementY
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN int
defiPinPort_orient
  PROTO_PARAMS(( const defiPinPort * this ));

EXTERN const char *
defiPinPort_orientStr
  PROTO_PARAMS(( const defiPinPort * this ));

/*
 * 5.6 setLayer is changed to addLayer due to multiple LAYER are allowed
 */

/*
 * in 5.6
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
 * 5.5
 */

/*
 * 5.7
 */

/*
 * 5.7 port statements, which may have LAYER, POLYGON, &| VIA
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
 * For OA to modify the pinName
 */

/*
 * optional parts
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
 * 5.4
 */

/*
 * ANTENNAPINPARTIALMETALAREA
 */

/*
 * ANTENNAPINPARTIALMETALSIDEAREA
 */

/*
 * ANTENNAPINDIFFAREA
 */

/*
 * ANTENNAPINPARTIALCUTAREA
 */

/*
 * 5.5
 */

/*
 * 5.7
 */

/*
 * allocated size of pin name
 */

/*
 * allocated size of net name
 */

/*
 * orient 0-7
 */

/*
 * allocated size of length
 */

/*
 * allocated size of direction
 */

/*
 * 5.6, changed to array
 */

/*
 * 5.6, changed to arrays
 */

/*
 * 5.6, SPACING in LAYER
 */

/*
 * 5.6, DESIGNRULEWIDTH in LAYER
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
 * 5.6, SPACING in POLYGON
 */

/*
 * 5.6, DESIGNRULEWIDTH in POLYGON
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
 * placement
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

/*
 * 5.7
 */

/*
 * 5.5 AntennaModel
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinPartialMetalArea
 */

/*
 * 5.4 Layer
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinPartialMetalSideArea
 */

/*
 * 5.4 Layer
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinDiffArea
 */

/*
 * 5.4 Layer
 */

/*
 * 5.4
 */

/*
 * 5.4 AntennaPinPartialCutArea
 */

/*
 * 5.4 Layer
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

typedef struct defiPin_s {
  int             pinNameLength_;
  char           *pinName_;
  int             netNameLength_;
  char           *netName_;
  char            hasDirection_;
  char            hasUse_;
  char            placeType_;
  char            orient_;
  int             useLength_;
  char           *use_;
  int             directionLength_;
  char           *direction_;
  char          **layers_;
  int            *xl_, *yl_, *xh_, *yh_;
  int            *layerMinSpacing_;
  int            *layerEffectiveWidth_;
  int             layersAllocated_;
  int             numLayers_;
  char          **polygonNames_;
  int            *polyMinSpacing_;
  int            *polyEffectiveWidth_;
  int             numPolys_;
  int             polysAllocated_;
  struct defiPoints **polygons_;
  int             x_, y_;
  int             hasSpecial_;
  int             numVias_;
  int             viasAllocated_;
  char          **viaNames_;
  int            *viaX_;
  int            *viaY_;
  int             numPorts_;
  int             portsAllocated_;
  defiPinPort   **pinPort_;
  int             numAntennaModel_;
  int             antennaModelAllocated_;
  defiPinAntennaModel **antennaModel_;
  int             numAPinPartialMetalArea_;
  int             APinPartialMetalAreaAllocated_;
  int            *APinPartialMetalArea_;
  char          **APinPartialMetalAreaLayer_;
  int             numAPinPartialMetalSideArea_;
  int             APinPartialMetalSideAreaAllocated_;
  int            *APinPartialMetalSideArea_;
  char          **APinPartialMetalSideAreaLayer_;
  int             numAPinDiffArea_;
  int             APinDiffAreaAllocated_;
  int            *APinDiffArea_;
  char          **APinDiffAreaLayer_;
  int             numAPinPartialCutArea_;
  int             APinPartialCutAreaAllocated_;
  int            *APinPartialCutArea_;
  char          **APinPartialCutAreaLayer_;
  int             netExprLength_;
  char            hasNetExpr_;
  char           *netExpr_;
  int             supplySensLength_;
  char            hasSupplySens_;
  char           *supplySens_;
  int             groundSensLength_;
  char            hasGroundSens_;
  char           *groundSens_;
} defiPin;

EXTERN defiPin *
defiPin_Create
  PROTO_PARAMS((  ));

EXTERN void
defiPin_Init
  PROTO_PARAMS(( defiPin * this ));

EXTERN void
defiPin_Delete
  PROTO_PARAMS(( defiPin * this ));

EXTERN void
defiPin_Destroy
  PROTO_PARAMS(( defiPin * this ));

EXTERN void
defiPin_Setup
  PROTO_PARAMS(( defiPin * this,
                 const char *pinName,
                 const char *netName ));

EXTERN void
defiPin_setDirection
  PROTO_PARAMS(( defiPin * this,
                 const char *dir ));

EXTERN void
defiPin_setUse
  PROTO_PARAMS(( defiPin * this,
                 const char *use ));

EXTERN void
defiPin_addLayer
  PROTO_PARAMS(( defiPin * this,
                 const char *layer ));

EXTERN void
defiPin_addLayerSpacing
  PROTO_PARAMS(( defiPin * this,
                 int minSpacing ));

EXTERN void
defiPin_addLayerDesignRuleWidth
  PROTO_PARAMS(( defiPin * this,
                 int effectiveWidth ));

EXTERN void
defiPin_addLayerPts
  PROTO_PARAMS(( defiPin * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiPin_addPolygon
  PROTO_PARAMS(( defiPin * this,
                 const char *layerName ));

EXTERN void
defiPin_addPolySpacing
  PROTO_PARAMS(( defiPin * this,
                 int minSpacing ));

EXTERN void
defiPin_addPolyDesignRuleWidth
  PROTO_PARAMS(( defiPin * this,
                 int effectiveWidth ));

EXTERN void
defiPin_addPolygonPts
  PROTO_PARAMS(( defiPin * this,
                 defiGeometries * geom ));

EXTERN void
defiPin_setNetExpr
  PROTO_PARAMS(( defiPin * this,
                 const char *netExpr ));

EXTERN void
defiPin_setSupplySens
  PROTO_PARAMS(( defiPin * this,
                 const char *pinName ));

EXTERN void
defiPin_setGroundSens
  PROTO_PARAMS(( defiPin * this,
                 const char *pinName ));

EXTERN void
defiPin_setPlacement
  PROTO_PARAMS(( defiPin * this,
                 int typ,
                 int x,
                 int y,
                 int orient ));

EXTERN void
defiPin_setSpecial
  PROTO_PARAMS(( defiPin * this ));

EXTERN void
defiPin_addAntennaModel
  PROTO_PARAMS(( defiPin * this,
                 int oxide ));

EXTERN void
defiPin_addAPinPartialMetalArea
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinPartialMetalSideArea
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinGateArea
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinDiffArea
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinMaxAreaCar
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinMaxSideAreaCar
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinPartialCutArea
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addAPinMaxCutCar
  PROTO_PARAMS(( defiPin * this,
                 int value,
                 const char *layer ));

EXTERN void
defiPin_addVia
  PROTO_PARAMS(( defiPin * this,
                 const char *via,
                 int viaX,
                 int viaY ));

EXTERN void
defiPin_addPort
  PROTO_PARAMS(( defiPin * this ));

EXTERN void
defiPin_addPortLayer
  PROTO_PARAMS(( defiPin * this,
                 const char *layer ));

EXTERN void
defiPin_addPortLayerSpacing
  PROTO_PARAMS(( defiPin * this,
                 int minSpacing ));

EXTERN void
defiPin_addPortLayerDesignRuleWidth
  PROTO_PARAMS(( defiPin * this,
                 int effectiveWidth ));

EXTERN void
defiPin_addPortLayerPts
  PROTO_PARAMS(( defiPin * this,
                 int xl,
                 int yl,
                 int xh,
                 int yh ));

EXTERN void
defiPin_addPortPolygon
  PROTO_PARAMS(( defiPin * this,
                 const char *layerName ));

EXTERN void
defiPin_addPortPolySpacing
  PROTO_PARAMS(( defiPin * this,
                 int minSpacing ));

EXTERN void
defiPin_addPortPolyDesignRuleWidth
  PROTO_PARAMS(( defiPin * this,
                 int effectiveWidth ));

EXTERN void
defiPin_addPortPolygonPts
  PROTO_PARAMS(( defiPin * this,
                 defiGeometries * geom ));

EXTERN void
defiPin_addPortVia
  PROTO_PARAMS(( defiPin * this,
                 const char *via,
                 int viaX,
                 int viaY ));

EXTERN void
defiPin_setPortPlacement
  PROTO_PARAMS(( defiPin * this,
                 int typ,
                 int x,
                 int y,
                 int orient ));

EXTERN void
defiPin_clear
  PROTO_PARAMS(( defiPin * this ));

EXTERN void
defiPin_changePinName
  PROTO_PARAMS(( defiPin * this,
                 const char *pinName ));

EXTERN const char *
defiPin_pinName
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_netName
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasDirection
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasUse
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasLayer
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasPlacement
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_isUnplaced
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_isPlaced
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_isCover
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_isFixed
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_placementX
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_placementY
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_direction
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_use
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numLayer
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_layer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN void
defiPin_bounds
  PROTO_PARAMS(( const defiPin * this,
                 int index,
                 int *xl,
                 int *yl,
                 int *xh,
                 int *yh ));

EXTERN int
defiPin_hasLayerSpacing
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasLayerDesignRuleWidth
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_layerSpacing
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_layerDesignRuleWidth
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_numPolygons
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_polygonName
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN struct defiPoints
defiPin_getPolygon
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasPolygonSpacing
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasPolygonDesignRuleWidth
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_polygonSpacing
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_polygonDesignRuleWidth
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasNetExpr
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasSupplySensitivity
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasGroundSensitivity
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_netExpr
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_supplySensitivity
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_groundSensitivity
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_orient
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_orientStr
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_hasSpecial
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numVias
  PROTO_PARAMS(( const defiPin * this ));

EXTERN const char *
defiPin_viaName
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_viaPtX
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_viaPtY
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinPartialMetalArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numAPinPartialMetalArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_APinPartialMetalArea
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinPartialMetalAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN const char *
defiPin_APinPartialMetalAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinPartialMetalSideArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numAPinPartialMetalSideArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_APinPartialMetalSideArea
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinPartialMetalSideAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN const char *
defiPin_APinPartialMetalSideAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinDiffArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numAPinDiffArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_APinDiffArea
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinDiffAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN const char *
defiPin_APinDiffAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinPartialCutArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numAPinPartialCutArea
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_APinPartialCutArea
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasAPinPartialCutAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN const char *
defiPin_APinPartialCutAreaLayer
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_numAntennaModel
  PROTO_PARAMS(( const defiPin * this ));

EXTERN defiPinAntennaModel *
defiPin_antennaModel
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN int
defiPin_hasPort
  PROTO_PARAMS(( const defiPin * this ));

EXTERN int
defiPin_numPorts
  PROTO_PARAMS(( const defiPin * this ));

EXTERN defiPinPort *
defiPin_pinPort
  PROTO_PARAMS(( const defiPin * this,
                 int index ));

EXTERN void
defiPin_print
  PROTO_PARAMS(( const defiPin * this,
                 FILE * f ));

#endif
