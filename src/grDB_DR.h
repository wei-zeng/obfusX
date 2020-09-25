#ifndef _GRDB_DR_H_
#define _GRDB_DR_H_

#include <set>
#include <vector>
#include <unordered_map>
#include <cassert>
#include <cstring>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <algorithm>

#include "baseDB.h"
#include "grDB.h"
#include "layoutDB_DR.h"
#include "a_star.h"
#include "shap.h"

class RoutingDB_DR {

	//
	//  Layout Information
	//
	friend class MyDefWriter;
public:
	RoutingDB_DR(const string &designName, const LayoutDR &layout) :
			_designName(designName), _numLayers(layout._numLayers), _numGlobalNets(
					0), _dirty(false) {
	}
	bool operator==(const RoutingDB_DR &other) {
		return _gnets == other._gnets && _edges == other._edges
				&& _vias == other._vias && _dirty == other._dirty;
	}

private:
	string _designName;
	unordered_map<Gcell, size_t, HashGcell3d> _uMapLoc2EdgeId;
	unordered_map<Gcell, size_t, HashGcell3d> _uMapLoc2ViaId;
	int _numLayers;
	vector<unordered_set<Gcell, HashGcell3d>> _grids;
	vector<unordered_map<Gcell, int, HashGcell3d>> _occupiedGnetId; // to avoid shorts; -1: not occupied
	vector<RoutingDir> _dirLayers;
	vector<int> _trackDemands;
	vector<int> _cntEdgeLayers;
	vector<int> _cntViaLayers;
	vector<int> _capOnLayer;
	vector<int> _upViaSize;
	vector<int> _dnViaSize;
	bool _dirty;
	//
	//  Edge Related Objects
	//
public:
	tuple<vector<unsigned>, vector<unsigned>> cntEdgesXY(
			vector<double> trackOffsetX, vector<double> trackOffsetY,
			vector<double> trackWidth, vector<double> trackHeight,
			vector<unsigned> numTracksX, vector<unsigned> numTracksY);
	tuple<vector<unsigned>, vector<unsigned>> cntViasXY(
			vector<double> trackOffsetX, vector<double> trackOffsetY,
			vector<double> trackWidth, vector<double> trackHeight,
			vector<unsigned> numTracksX, vector<unsigned> numTracksY);
	void initEdges(const LayoutDR &layout);
	void initGlobalEdgeProfile(const LayoutDR &layout);
	void printEdgeInfoRange(const unsigned int start, const unsigned int end);
	void printViaInfoRange(const unsigned int start, const unsigned int end);
private:
	void refreshNodeBlkInfo(const LayoutDR &layout, const Node &node);
	void refreshRectBlkInfo(const LayoutDR &layout, const Shape &shape,
			const int layerId);
	void updateEdgeFreeList(const LayoutDR &layout, const int edgeId,
			const Shape &shape);
	void updateSingleEdgeFreeList(Edge &edge, const double blkStart,
			const double blkEnd);

	// int findEdge( const int x, const int y, const int z ) const;
	RoutingDir getRoutingDir(const int layer) const;
	// Gcell getEdgeLoc(const int edgeId) const;
	int getEdgeLayer(const int edgeId) const;
	// int getEdgeDemand( const int edgeId ) const;
	int getEdgeOf(const int edgeId) const;
	void addEdgeDemand(const int edgeId);
	void reduceEdgeDemand(const int edgeId);
	void ripUpEdge(const int edgeId);
	void restoreEdge(const int edgeId);

public:
	//tuple<int, int, int> ptToGrid(const LayoutDR &layout, int x, int y,
	//		int z) const;
	//tuple<int, int, int> ptToVia(const LayoutDR &layout, int x, int y,
	//		int z) const;
	int findEdge(const int x, const int y, const int z) const;
	Gcell getEdgeLoc(const int edgeId) const;
	vector<Edge> getEdges() const {
		return _edges;
	}
	int getEdgeDemand(const int edgeId) const;
	int getEdgeBlk(const int edgeId) const;
	int getEdgeCap(const int edgeId) const;
	void clearEdgeDemands();
private:

	void checkEdgeFreeList(const int edgeId);
	//  gWire related methods
	bool isUpViaGlobalWire(const Gnet &gnet, const GlobalWire &gWire) const;
	bool isDnViaGlobalWire(const Gnet &gnet, const GlobalWire &gWire) const;
	void bruteForceFindAllChildWires(const Gnet &gnet, const int wireId,
			vector<int> &cache);

	inline void rmEdgeDemand(const int edgeId);
	vector<Edge> _edges;
	vector<Gcell> _ep1, _ep2;
	void printEdgeUsage(const size_t edgeId);
	void printCompleteEdgeInfo(const int edgeId);

	//
	//  Via Related Objects
	//
public:
	void initVias(const LayoutDR &layout);
	vector<Via> getVias() const {
		return _vias;
	}
private:
	long findVia(const int x, const int y, const int z) const;
	void addViaDemand(const int viaId);
	void reduceViaDemand(const int viaId);
	void ripUpVia(const int viaId);
	void restoreVia(const int viaId);

	Gcell getViaLoc(const int viaId) const;
	int getViaLayer(const int viaId) const;
	int getViaOf(const int viaId) const;

	vector<Via> _vias;
	void printViaUsage(const size_t viaId);

	//
	//  Read Global Routes
	//
public:
	void initGlobalNets(LayoutDR &layout);
	void readGlobalWires(const LayoutDR &layout);
	void writeGlobalWires(const LayoutDR &layout, const char *wiresFile) const;
	void writeGlobalWires(const LayoutDR &layout,
			const string &wiresFile) const;
	void initRoutingTreeForAllGnets();
	bool checkAllRoutingTrees();
	int getViaDemand(const int viaId) const;
	int getViaCap(const int viaId) const;
	vector<Gnet>& getGnets() {
		return _gnets;
	}
private:
	void initRoutingTree(Gnet &gnet);

	//
	//  Routing Tree objects
	//
	vector<vector<PtInfo> > _treeTable;
	vector<unordered_set<int> > _vecSharePin;
	// Structure of PtInfo is in baseDB.h: (int x, int y, int z, int indexId, int distPrev, int distNext, bool hasDnVia, bool hasUpVia)
	vector<PtInfo> tmp;

	vector<unordered_map<Gcell, int, HashGcell2d> > _uMapCell2LocInTable;
	vector<Gcell> frontier;
	vector<Gnet> _gnets;
	int _numGlobalNets;
	/* used for removing antenna wires */
	vector<int> _offset;
	vector<int> _numChildren;
	vector<bool> _validWires;
	vector<GlobalWire> _cleanWires;
	bool checkChildren(Gnet &gnet, vector<int> &_numChildren,
			vector<bool> &_validWires, double &wlReduced, int splitLayer,
			double &wlReducedAboveSL, int curr, const LayoutDR &layout,
			bool &endsAtvpLower, bool &endsAtvpUpper, Gcell &vpGridLower,
			Gcell &vpGridUpper, const Vpin &vp);

public:
	void updateEdgeDemands(const LayoutDR &layout);
	void updateGridOccupation(const int gnetId, bool noReset);
	//std::tuple<vector<Vpin>, vector<int>, vector<int>, double, double> ripUpNet(
	//		const LayoutDR &layout, const int gnetID, const int splitLayer);
	std::tuple<std::unordered_set<Gcell, HashGcell3d>,
			std::unordered_set<Gcell, HashGcell3d>,
			std::unordered_set<Gcell, HashGcell3d>,
			std::unordered_set<Gcell, HashGcell3d>, vector<int>, vector<int>,
			double, double> ripUpNet(const LayoutDR &layout, Vpin &vp);
	std::tuple<double, vector<int>, vector<int>, int,
			std::unordered_set<Gcell, HashGcell3d>> rerouteNet(
			const LayoutDR &layout, Vpin &vp,
			const std::unordered_set<Gcell, HashGcell3d> &bannedPts,
			const int zlLim, const int zuLim, const int maxMargin,
			const bool singleMargin, const bool writeToDB,
			const bool updateRoot, const size_t initNumGoals, const Vpin &matchingVpin);
	std::tuple<double, vector<int>, vector<int>, int> rerouteNet(
			const LayoutDR &layout, const Vpin &vp1,
			const unordered_set<Gcell, HashGcell3d> &otherVpinGrids,
			const unordered_set<Gcell, HashGcell3d> &otherVpinVertices,
			const int zlLim, const int zuLim, const int maxMargin,
			const bool singleMargin, const bool writeToDB,
			const size_t initNumGoals, const Vpin &matchingVpin) {
		return rerouteNet(layout, vp1, otherVpinGrids, otherVpinVertices, zlLim,
				zuLim, maxMargin, singleMargin, writeToDB, initNumGoals, false,
				1, zlLim - 1, matchingVpin);
	}
	std::tuple<double, vector<int>, vector<int>, int> rerouteNet(
			const LayoutDR &layout, const Vpin &vp1,
			const unordered_set<Gcell, HashGcell3d> &otherVpinGrids,
			const unordered_set<Gcell, HashGcell3d> &otherVpinVertices,
			const int zlLim, const int zuLim, const int maxMargin,
			const bool singleMargin, const bool writeToDB,
			const size_t initNumGoals, bool lift, int liftPenalty,
			int splitLayer, const Vpin &matchingVpin);
	void restoreNet(const int gnetID, const Gnet gnet,
			const std::map<size_t, Edge> &cleanEdges,
			const std::map<size_t, Via> &cleanVias, bool restoreCongestion);
	int getTotalWL() const;
	int getWlByNet(int gnetID) const;
	int getTotalWLByDemand() const;
	bool isDirty() const {
		return _dirty;
	}
	vector<Vpin> getVpins(const LayoutDR &layout, const int splitLayer,
			const bool twoCutNetsOnly, bool includeAllPIOs) const;
	vector<Vpin> getVpins(const LayoutDR &layout, const int gnetID,
			const int splitLayer, const bool twoCutNetsOnly,
			bool includeAllPIOs) const;
	void outputCSV(const LayoutDR &layout, const int splitLayer,
			const string fileName) const;
private:
	void createBlockage(const int xStart, const int yStart, const int xEnd,
			const int yEnd, const int z, LayoutDR &layout);

// A-star:
public:
	pair<Graph, std::unordered_map<Gcell, Vertex, HashGcell3d>> build_graph(
			const size_t gnetId,
			const std::unordered_set<Gcell, HashGcell3d> &bannedPt, int xlLim,
			int xuLim, int ylLim, int yuLim, int zlLim, int zuLim,
			const LayoutDR &layout, bool lift, int liftPenalty, int splitLayer,
			const Vpin& vpin, const Vpin &matchingVpin) const;
	unordered_set<Vertex> build_goals(
			const unordered_set<Gcell, HashGcell3d> &vpinGrids, const int xlLim,
			int xuLim, int ylLim, int yuLim, int zlLim, int zuLim,
			const unordered_map<Gcell, size_t, HashGcell3d> &uMapGridInGraph,
			const LayoutDR &layout, Graph &g) const;

	std::pair<double, vector<Vertex>> a_star(Graph &graph3D, const Vertex &s,
			const unordered_set<Vertex> &goals) const;
// void layerAssignment(const LayoutDR & layout, Graph & graph3D, const Point3D & root_pt);
	std::vector<EdgeType> kruskalMST(Graph &graphMST) const;

	std::pair<ObfusOption, bool> traverseObfusOpts(const LayoutDR &layout,
			int layer, bool singleVpin, size_t sortedVpinID, int axis,
			const vector<double> &featVec, const SHAP&, const vector<Vpin>&,
			int maxLayer);
	std::pair<ObfusOption, bool> traverseLiftingOpts(const LayoutDR &layout,
			int layer, bool singleVpin, size_t sortedVpinID, int axis,
			const vector<double> &featVec, const SHAP&, const vector<Vpin>&,
			int maxLayer, int splitLayer);
};

bool isOptionValid(const ObfusOption &opt, const RoutingDB_DR &routingDB);
bool congestionEqual(const RoutingDB_DR &db1, const RoutingDB_DR &db2);

#endif
