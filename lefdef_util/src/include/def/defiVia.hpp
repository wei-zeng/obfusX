/*
 *     This  file  is  part  of  the  Cadence  LEF/DEF  Open   Source
 *  Distribution,  Product Version 5.7, and is subject to the Cadence
 *  LEF/DEF Open Source License Agreement.   Your  continued  use  of
 *  this file constitutes your acceptance of the terms of the LEF/DEF
 *  Open Source License and an agreement to abide by its  terms.   If
 *  you  don't  agree  with  this, you must remove this and any other
 *  files which are part of the distribution and  destroy any  copies
 *  made.
 *
 *     For updates, support, or to become part of the LEF/DEF Community,
 *  check www.openeda.org for details.
 */

#ifndef defiVia_h
#define defiVia_h

#include "defiKRDefs.hpp"
#include <stdio.h>
#include "defiMisc.hpp"

// Struct holds the data for one property.
class defiVia {
public:
  defiVia();
  void Init();

  void clear();
  void Destroy();
  ~defiVia();

  void setup(const char* name);
  void addPattern(const char* patt);
  void addLayer(const char* layer, int xl, int yl, int xh, int yh);
  // 5.6
  void addPolygon(const char* layer, defiGeometries* geom);
  void addViaRule(char* viaRuleName, int xSize, int ySize, char* botLayer,
                  char* cutLayer, char* topLayer, int xSpacing, int ySpacing,
                  int xBotEnc, int yBotEnc, int xTopEnc, int yTopEnc);
  void addRowCol(int numCutRows, int numCutCols);
  void addOrigin(int xOffset, int yOffset);
  void addOffset(int xBotOs, int yBotOs, int xTopOs, int yTopOs);
  void addCutPattern(char* cutPattern);

  const char* name() const;
  const char* pattern() const;
  int hasPattern() const;
  int numLayers() const;
  void layer(int index, char** layer, int* xl, int* yl, int* xh, int* yh) const;
  int numPolygons() const;                        // 5.6
  const char* polygonName(int index) const;       // 5.6
  struct defiPoints getPolygon(int index) const;  // 5.6
  int hasViaRule() const;
  void viaRule(char** viaRuleName, int* xSize, int* ySize, char** botLayer,
               char** cutLayer, char** topLayer, int* xCutSpacing,
               int* yCutSpacing, int* xBotEnc, int* yBotEnc, int* xTopEnc,
               int* yTopEnc) const;
  int hasRowCol() const;
  void rowCol(int* numCutRows, int* numCutCols) const;
  int hasOrigin() const;
  void origin(int* xOffset, int* yOffset) const;
  int hasOffset() const;
  void offset(int* xBotOffset, int* yBotOffset, int* xTopOffset,
              int* yTopOffset) const;
  int hasCutPattern() const;
  const char* cutPattern() const;

  void print(FILE* f) const;

protected:
  char* name_;
  int nameLength_;
  char* pattern_;
  int patternLength_;
  char** layers_;
  int hasPattern_;
  int* xl_;
  int* yl_;
  int* xh_;
  int* yh_;
  int layersLength_;
  int numLayers_;
  int numPolys_;            // 5.6
  char** polygonNames_;     // 5.6 layerName for POLYGON
  int polysAllocated_;      // 5.6
  struct defiPoints** polygons_; // 5.6
  char* viaRule_;           // 5.6
  int   viaRuleLength_;     // 5.6
  int   hasViaRule_;        // 5.6
  int xSize_;               // 5.6
  int ySize_;               // 5.6
  char* botLayer_;          // 5.6
  char* cutLayer_;          // 5.6
  char* topLayer_;          // 5.6
  int   botLayerLength_;    // 5.6
  int   cutLayerLength_;    // 5.6
  int   topLayerLength_;    // 5.6
  int   xCutSpacing_;       // 5.6
  int   yCutSpacing_;       // 5.6
  int   xBotEnc_;           // 5.6
  int   yBotEnc_;           // 5.6
  int   xTopEnc_;           // 5.6
  int   yTopEnc_;           // 5.6
  int   rows_;              // 5.6
  int   cols_;              // 5.6
  int   xOffset_;           // 5.6
  int   yOffset_;           // 5.6
  int   xBotOffset_;        // 5.6
  int   yBotOffset_;        // 5.6
  int   xTopOffset_;        // 5.6
  int   yTopOffset_;        // 5.6
  char* cutPattern_;        // 5.6
  int   cutPatternLength_;  // 5.6
  int   hasCutPattern_;     // 5.6
};


#endif
