#include "grDB_DR.h"

bool congestionEqual(const RoutingDB_DR &db1, const RoutingDB_DR &db2) {
	auto es1 = db1.getEdges();
	auto es2 = db2.getEdges();
	if (es1.size() != es2.size()) {
		cerr << "Via size different!" << endl;
		return false;
	}
	for (size_t i = 0; i < es1.size(); i++) {
		if (!(es1[i] == es2[i])) {
			cerr << i << "-th edge different" << endl;
			Gcell gc = db1.getEdgeLoc(i);
			cerr << gc._x << "," << gc._y << "," << gc._z << endl;
			cerr << "Demand: old " << es1[i].demand() << " vs new "
					<< es2[i].demand() << endl;
			cerr << "Cap: old " << es1[i].cap() << " vs new " << es2[i].cap()
					<< endl;
			cerr << "Block: old " << es1[i].blk() << " vs new " << es2[i].blk()
					<< endl;
			return false;
		}
	}
	auto vs1 = db1.getVias();
	auto vs2 = db2.getVias();
	if (vs1.size() != vs2.size()) {
		cerr << "Via size different!" << endl;
		return false;
	}
	for (size_t i = 0; i < vs1.size(); i++) {
		if (!(vs1[i] == vs2[i])) {
			cerr << i << "-th via different" << endl;
			return false;
		}
	}
	return true; //db1.getEdges() == db2.getEdges() && db1.getVias() == db2.getVias();
}

tuple<vector<unsigned>, vector<unsigned>> RoutingDB_DR::cntEdgesXY(
		vector<double> trackOffsetX, vector<double> trackOffsetY,
		vector<double> trackWidth, vector<double> trackHeight,
		vector<unsigned> numTracksX, vector<unsigned> numTracksY) {
	vector<unsigned> numEdgesX, numEdgesY;
	size_t numLayers = trackOffsetX.size();
	numEdgesX.resize(numLayers);
	numEdgesY.resize(numLayers);
	size_t cntEdges = 0;
	double x, y;
	size_t maxNumEdges = 0;
	for (size_t z = 0; z < numLayers - 1; z++) {
		maxNumEdges += 2 * (numTracksX[z] + numTracksX[z + 1])
				* (numTracksY[z] + numTracksY[z + 1]);
	}
	for (size_t z = 0; z < numLayers; z++) {
		maxNumEdges += 2 * numTracksX[z] * numTracksY[z];
	}
	_edges.resize(maxNumEdges);
	_grids.resize(numLayers);
	_occupiedGnetId.resize(numLayers);
	_cntEdgeLayers.resize(numLayers + 1);
	for (size_t z = 0; z < numLayers; ++z) {
		// Add grids
		_cntEdgeLayers[z] = cntEdges;
		// same layer grids (caused by wires)
		for (unsigned j = 0; j < numTracksY[z]; ++j) {
			for (unsigned k = 0; k < numTracksX[z]; ++k) {
				x = trackOffsetX[z] + k * trackWidth[z];
				y = trackOffsetY[z] + j * trackHeight[z];
				auto res = _grids[z].insert(
						Gcell(round(x), round(y), z));
				_occupiedGnetId[z][Gcell(round(x), round(y), z)] = -1;
			}
		}
		// cross layer grids (caused by vias)
		if (z % 2 == 0) {
			numEdgesY[z] = numTracksY[z] - 1;
			numEdgesX[z] = 0;
			for (unsigned j = 0; j < numTracksY[z]; ++j) {
				if (z > 0) {
					for (unsigned k = 0; k < numTracksX[z - 1]; ++k) {
						x = trackOffsetX[z - 1] + k * trackWidth[z - 1];
						y = trackOffsetY[z] + j * trackHeight[z];
						auto res = _grids[z].insert(
								Gcell(round(x), round(y), z));
						_occupiedGnetId[z][Gcell(round(x), round(y), z)] = -1;
					}
				}
				if (z < numLayers - 1) {
					for (unsigned k = 0; k < numTracksX[z + 1]; ++k) {
						x = trackOffsetX[z + 1] + k * trackWidth[z + 1];
						y = trackOffsetY[z] + j * trackHeight[z];
						auto res = _grids[z].insert(
								Gcell(round(x), round(y), z));
						_occupiedGnetId[z][Gcell(round(x), round(y), z)] = -1;
					}
				}
			}
		} else {
			numEdgesX[z] = numTracksX[z] - 1;
			numEdgesY[z] = 0;
			for (unsigned j = 0; j < numTracksX[z]; ++j) {
				if (z < numLayers - 1) {
					for (unsigned k = 0; k < numTracksY[z + 1]; ++k) {
						x = trackOffsetX[z] + j * trackWidth[z];
						y = trackOffsetY[z + 1] + k * trackHeight[z + 1];
						auto res = _grids[z].insert(Gcell(round(x), round(y), z));
						_occupiedGnetId[z][Gcell(round(x), round(y), z)] = -1;
					}
				}
				if (z > 0) {
					for (unsigned k = 0; k < numTracksY[z - 1]; ++k) {
						x = trackOffsetX[z] + j * trackWidth[z];
						y = trackOffsetY[z - 1] + k * trackHeight[z - 1];
						auto res = _grids[z].insert(Gcell(round(x), round(y), z));
						_occupiedGnetId[z][Gcell(round(x), round(y), z)] = -1;
					}
				}
			}
		}
		// Add edges
		vector<PtInfo> vecGridsPtInfo_z;
		for (auto gc : _grids[z]) {
			vecGridsPtInfo_z.emplace_back(gc._x, gc._y, gc._z, -1, -1, -1,
					false, false);
		}
		// Add vertical edges
		sort(vecGridsPtInfo_z.begin(), vecGridsPtInfo_z.end(),
				comparePtInfoOnHorLayer());
		for (size_t i = 1; i < vecGridsPtInfo_z.size(); i++) {
			if (vecGridsPtInfo_z[i]._y == vecGridsPtInfo_z[i - 1]._y) {
				_edges[cntEdges].setLoc(
						(vecGridsPtInfo_z[i - 1]._x + vecGridsPtInfo_z[i]._x)
								/ 2, vecGridsPtInfo_z[i]._y, z);
				_ep1.emplace_back(vecGridsPtInfo_z[i - 1]._x,
						vecGridsPtInfo_z[i - 1]._y, vecGridsPtInfo_z[i - 1]._z);
				_ep2.emplace_back(vecGridsPtInfo_z[i]._x,
						vecGridsPtInfo_z[i]._y, vecGridsPtInfo_z[i]._z);
				_uMapLoc2EdgeId.insert(
						make_pair(
								Gcell(
										(vecGridsPtInfo_z[i - 1]._x
												+ vecGridsPtInfo_z[i]._x) / 2,
										vecGridsPtInfo_z[i]._y, z), cntEdges));
				cntEdges++;
			}
		}
		// Add horizontal edges
		sort(vecGridsPtInfo_z.begin(), vecGridsPtInfo_z.end(),
				comparePtInfoOnVerLayer());
		for (size_t i = 1; i < vecGridsPtInfo_z.size(); i++) {
			if (vecGridsPtInfo_z[i]._x == vecGridsPtInfo_z[i - 1]._x) {
				_edges[cntEdges].setLoc(vecGridsPtInfo_z[i]._x,
						(vecGridsPtInfo_z[i - 1]._y + vecGridsPtInfo_z[i]._y)
								/ 2, z);
				_ep1.emplace_back(vecGridsPtInfo_z[i - 1]._x,
						vecGridsPtInfo_z[i - 1]._y, vecGridsPtInfo_z[i - 1]._z);
				_ep2.emplace_back(vecGridsPtInfo_z[i]._x,
						vecGridsPtInfo_z[i]._y, vecGridsPtInfo_z[i]._z);
				_uMapLoc2EdgeId.insert(
						make_pair(
								Gcell(vecGridsPtInfo_z[i]._x,
										(vecGridsPtInfo_z[i - 1]._y
												+ vecGridsPtInfo_z[i]._y) / 2,
										z), cntEdges));
				cntEdges++;
			}
		}
	}
	_edges.resize(cntEdges);
	_cntEdgeLayers[numLayers] = cntEdges;
	return make_tuple(numEdgesX, numEdgesY);
}

tuple<vector<unsigned>, vector<unsigned>> RoutingDB_DR::cntViasXY(
		vector<double> trackOffsetX, vector<double> trackOffsetY,
		vector<double> trackWidth, vector<double> trackHeight,
		vector<unsigned> numTracksX, vector<unsigned> numTracksY) {
	vector<unsigned> numViasX, numViasY;
	size_t numLayers = trackOffsetX.size();
	size_t cntVias = 0;
	numViasX.resize(numLayers);
	numViasY.resize(numLayers);
	double x, y;
	size_t maxNumVias = 0;
	for (size_t z = 0; z < numLayers - 1; z++) {
		maxNumVias += (numTracksX[z] + numTracksX[z + 1])
				* (numTracksY[z] + numTracksY[z + 1]);
	}
	_vias.resize(maxNumVias);
	_cntViaLayers.resize(numLayers);
	for (size_t z = 0; z < numLayers - 1; ++z) {
		_cntViaLayers[z] = cntVias;
		if (_dirLayers[z] == H) { // M1, M3, M5, ...
			numViasY[z] = numTracksY[z];
			numViasX[z] = 0;
			for (unsigned j = 0; j < numTracksY[z]; ++j) {
				for (unsigned k = 0; k < numTracksX[z + 1]; ++k) {
					x = trackOffsetX[z + 1] + k * trackWidth[z + 1];
					y = trackOffsetY[z] + j * trackHeight[z];
					auto res = _uMapLoc2ViaId.insert(
							make_pair(Gcell(round(x), round(y), z), cntVias));
					if (res.second) {
						numViasX[z]++;
						_vias[cntVias].setLoc(round(x), round(y), z);
						cntVias++;
					}
				}
			}
		} else { // M2, M4, M6, ...
			numViasX[z] = numTracksX[z];
			numViasY[z] = 0;
			for (unsigned j = 0; j < numTracksX[z]; ++j) {
				for (unsigned k = 0; k < numTracksY[z + 1]; ++k) {
					x = trackOffsetX[z] + j * trackWidth[z];
					y = trackOffsetY[z + 1] + k * trackHeight[z + 1];
					auto res = _uMapLoc2ViaId.insert(
							make_pair(Gcell(round(x), round(y), z), cntVias));
					if (res.second) {
						numViasY[z]++;
						_vias[cntVias].setLoc(round(x), round(y), z);
						cntVias++;
					}
				}
			}
		}
	}
	_vias.resize(cntVias);
	_cntViaLayers[numLayers - 1] = cntVias;
	return make_tuple(numViasX, numViasY);
}
//  
//  Edge Related Functions
//
void RoutingDB_DR::initEdges(const LayoutDR &layout) {
	auto res = cntEdgesXY(layout._trackOffsetX, layout._trackOffsetY,
			layout._trackWidth, layout._trackHeight, layout._numTracksX,
			layout._numTracksY);
	_trackDemands.assign(_numLayers, 0);
	_dirLayers.assign(_numLayers, NA);
	_capOnLayer.assign(_numLayers, 0);
	_upViaSize.assign(_numLayers + 1, 0);

	//  non-via
	for (int layer = 0; layer < _numLayers; layer++) {
		// routing dir on that layer
		_dirLayers[layer] = layout._hCaps[layer] > 0 ? H :
							layout._vCaps[layer] > 0 ? V : NA;
		//  track demand on that layer
		_trackDemands[layer] = 1;

		//  via size
		if (layer > 0) {
			_upViaSize[layer - 1] = _trackDemands[layer];
		}
	}
	_upViaSize[_numLayers] = _upViaSize[_numLayers - 1];
	printf("<I> %-20s : ", "Demand Per Track");
	for (int i = 0; i < _numLayers; i++)
		printf("%6d", _trackDemands[i]);
	printf("\n");
	_edges.resize(_cntEdgeLayers[_numLayers]);
}

RoutingDir RoutingDB_DR::getRoutingDir(const int layer) const {
	return _dirLayers[layer];
}

int RoutingDB_DR::getEdgeLayer(const int edgeId) const {
	int layer = 0;
	while (_cntEdgeLayers[layer + 1] <= edgeId)
		layer++;
	return layer;
}

int RoutingDB_DR::getEdgeDemand(const int edgeId) const {
	return _edges[edgeId].demand();
}

int RoutingDB_DR::getEdgeBlk(const int edgeId) const {
	return _edges[edgeId].blk();
}

int RoutingDB_DR::getEdgeCap(const int edgeId) const {
	return _edges[edgeId].cap();
}

int RoutingDB_DR::getEdgeOf(const int edgeId) const {
	return _edges[edgeId].of();
}

Gcell RoutingDB_DR::getEdgeLoc(const int edgeId) const {
	return _edges[edgeId].getLoc();
}

void RoutingDB_DR::addEdgeDemand(const int edgeId) {
	_edges[edgeId].addDemand(_trackDemands[getEdgeLayer(edgeId)]);
}

void RoutingDB_DR::reduceEdgeDemand(const int edgeId) {
	_edges[edgeId].addDemand(-_trackDemands[getEdgeLayer(edgeId)]);
}

void RoutingDB_DR::ripUpEdge(const int edgeId) {
	reduceEdgeDemand(edgeId);
}

void RoutingDB_DR::restoreEdge(const int edgeId) {
	addEdgeDemand(edgeId);
}

// Set _blk for all edges
void RoutingDB_DR::initGlobalEdgeProfile(const LayoutDR &layout) {
	//for (size_t i = 0; i < layout._blkId.size(); i++) {
	//	const Node &blkNode = layout._nodes[layout._blkId[i]];
	//	refreshNodeBlkInfo(layout, blkNode);
	//}
	int edgeId = 0;
	for (int layer = 0; layer < _numLayers; layer++) {
		_capOnLayer[layer] =
				_dirLayers[layer] == H ? layout._hCaps[layer] :
				_dirLayers[layer] == V ? layout._vCaps[layer] : 0;
		if (_dirLayers[layer] == NA)
			continue;
		else {
			//const double fullLength =
			//		_dirLayers[layer] == H ?
			//				layout._cellHeight : layout._cellWidth;
			const int edgeNumOnLayer = _cntEdgeLayers[layer + 1]
					- _cntEdgeLayers[layer];
			//		_dirLayers[layer] == H ?
			//				_numTilesY * (_numTilesX - 1) :
			//				_numTilesX * (_numTilesY - 1);
			for (int i = 0; i < edgeNumOnLayer; i++) {
				Edge &edge = _edges[edgeId];
				edge.setCap(_capOnLayer[layer]);
				//int numAvailableTracks = floor(
				//edge.cap() * (edge.getFreeLength() / fullLength)
				//		/ _trackDemands[layer]);
				edge.setBlk(0);
				//		edge.cap() - _trackDemands[layer] * numAvailableTracks);
				edgeId++;
			}
		}
	}
	printf("<I> %-20s : %d\n", "# Global Edges", edgeId);
}

int RoutingDB_DR::findEdge(const int x, const int y, const int z) const {
	if (_uMapLoc2EdgeId.count(Gcell(x, y, z)) == 0) {
		return -1;
	}
	return _uMapLoc2EdgeId.at(Gcell(x, y, z));
}

int RoutingDB_DR::getViaLayer(const int viaId) const {
	return _vias[viaId].getLoc()._z;
}

int RoutingDB_DR::getViaDemand(const int viaId) const {
	return _vias[viaId].demand();
}

int RoutingDB_DR::getViaCap(const int viaId) const {
	return _vias[viaId].cap();
}

int RoutingDB_DR::getViaOf(const int viaId) const {
	return _vias[viaId].of();
}

Gcell RoutingDB_DR::getViaLoc(const int viaId) const {
	return _vias[viaId].getLoc();
}

// Prints all edge info including free list of edges
void RoutingDB_DR::printCompleteEdgeInfo(const int edgeId) {
	const Edge &edge = _edges[edgeId];
	printf("<D> Loc (%4d,%4d,%4d) %c %8d%8d%8d%8.1f%20s\n",
			getEdgeLoc(edgeId)._x, getEdgeLoc(edgeId)._y, getEdgeLayer(edgeId),
			_dirLayers[getEdgeLayer(edgeId)] == H ? 'H' :
			_dirLayers[getEdgeLayer(edgeId)] == V ? 'V' : 'N', edge.cap(),
			edge.blk(), edge.demand(), edge.getFreeLength(),
			edge.blk() + edge.demand() > edge.cap() ? "OVERFLOW" : "");
	printf("<D> Print Edge Free List : \n");
	for (auto itr = edge._freeList.begin(); itr != edge._freeList.end();
			itr++) {
		printf("<D> \tFrom %f To %f\n", itr->first, itr->second);
	}
}

// Prints info on edges
void RoutingDB_DR::printEdgeUsage(const size_t edgeId) {
	const Edge &edge = _edges[edgeId];
	printf("Edge( %d, %d, %d ) %c cap = %d, blk = %d, demand = %d, OF = %d",
			getEdgeLoc(edgeId)._x, getEdgeLoc(edgeId)._y, getEdgeLayer(edgeId),
			_dirLayers[getEdgeLayer(edgeId)] == H ? 'H' :
			_dirLayers[getEdgeLayer(edgeId)] == V ? 'V' : 'N', edge.cap(),
			edge.blk(), edge.demand(), edge.of());
}

// Prints info on edges
void RoutingDB_DR::printViaUsage(const size_t viaId) {
	const Via &via = _vias[viaId];
	printf("Via( %d, %d, %d ) cap = %d, demand = %d, OF = %d",
			getViaLoc(viaId)._x, getViaLoc(viaId)._y, getViaLayer(viaId),
			via.cap(), via.demand(), via.of());
}

//  gWire related methods
bool RoutingDB_DR::isUpViaGlobalWire(const Gnet &gnet,
		const GlobalWire &gWire) const {
	return (gnet._gWires[gWire._pWireId]._z > gWire._z);
}

bool RoutingDB_DR::isDnViaGlobalWire(const Gnet &gnet,
		const GlobalWire &gWire) const {
	return (gnet._gWires[gWire._pWireId]._z < gWire._z);
}

void RoutingDB_DR::bruteForceFindAllChildWires(const Gnet &gnet,
		const int wireId, vector<int> &cache) {
	cache.clear();
	for (int i = 0; i < static_cast<int>(gnet._gWires.size()); i++)
		if (gnet._gWires[i]._pWireId == wireId)
			cache.push_back(i);
}

void RoutingDB_DR::clearEdgeDemands() {
	for (size_t i = 0; i < _edges.size(); i++)
		_edges[i].clearDemand();
}

//
//  Via Related Objects
//
void RoutingDB_DR::initVias(const LayoutDR &layout) {
	auto res = cntViasXY(layout._trackOffsetX, layout._trackOffsetY,
			layout._trackWidth, layout._trackHeight, layout._numTracksX,
			layout._numTracksY);
	int cnt = 0;
	for (int layer = 0; layer < _numLayers - 1; layer++) {
		const int cap =
				_dirLayers[layer + 1] == H ?
						layout._hCaps[layer + 1] : layout._vCaps[layer + 1];
		const int area = cap * cap;
		for (int j = 0; j < _cntViaLayers[layer + 1] - _cntViaLayers[layer];
				j++)
			_vias[cnt++].setCap(area);
	}
}

long RoutingDB_DR::findVia(const int x, const int y, const int z) const {
	if (_uMapLoc2ViaId.count(Gcell(x, y, z)) == 0) {
		return -1;
	}
	return _uMapLoc2ViaId.at(Gcell(x, y, z));
}

void RoutingDB_DR::addViaDemand(const int viaId) {
	int layer = getViaLayer(viaId);
	_vias[viaId].addDemand(_trackDemands[layer + 1] * _trackDemands[layer + 1]);
}

void RoutingDB_DR::reduceViaDemand(const int viaId) {
	int layer = getViaLayer(viaId);
	_vias[viaId].addDemand(
			-_trackDemands[layer + 1] * _trackDemands[layer + 1]);
}

void RoutingDB_DR::ripUpVia(const int viaId) {
	reduceViaDemand(viaId);
}

void RoutingDB_DR::restoreVia(const int viaId) {
	addViaDemand(viaId);
}

void RoutingDB_DR::printEdgeInfoRange(const unsigned int start,
		const unsigned int end) {
	for (unsigned i = start; i < end; i++) {
		if (i >= 0 && i < _edges.size()) {
			printEdgeUsage(i);
			printf("\n");
		}
	}
}

void RoutingDB_DR::printViaInfoRange(const unsigned int start,
		const unsigned int end) {
	for (unsigned i = start; i < end; i++) {
		if (i >= 0 && i < _vias.size()) {
			printViaUsage(i);
			printf("\n");
		}
	}
}

