#include "grDB_DR.h"
#include <random>
#include <algorithm>
#include <iterator>
#include <boost/graph/connected_components.hpp>

#define MAX_MARGIN_OFFGRID 5000

//  Read Global Routes and initialize _gnets data structure
void RoutingDB_DR::initGlobalNets(LayoutDR &layout) {
	const int size = layout._nets.size();
	_gnets.resize(size);
	set<Gcell> gPins;
	set<Gcell> gPins2D;
	for (int i = 0; i < size; i++) {

		gPins.clear();
		gPins2D.clear();

		Net &net = layout._nets[i];
		for (auto pin : net._netPins) {
			Gcell gpin = layout.getGcell(pin);
			gPins2D.insert(Gcell(gpin._x, gpin._y, 0));
			gPins.insert(gpin);
		}

		_gnets[i]._name = net.getName();
		_gnets[i]._net = net;

		if (gPins2D.size() < 2)
			continue;

		vector<Gcell> pins(gPins.begin(), gPins.end());
		_gnets[i]._isGlobal = true;
		_gnets[i]._gPins.swap(pins);

		vector<Gcell> pins2D(gPins2D.begin(), gPins2D.end());
		_gnets[i]._gPins2D.swap(pins2D);

		++_numGlobalNets;
	}

	printf("<I> %-20s : %u\n", "# Global Nets", _numGlobalNets);
}

// Read routing file and initialize list of gnet points
void RoutingDB_DR::readGlobalWires(const LayoutDR &layout) {
	initVias(layout);
	int netId = 0;
	float x1, y1, x2, y2;
	int z1, z2;
	auto &ldp = layout._ldp;
	for (auto &net : ldp.def_.get_net_umap()) {
		for (auto &wire : net.second->wires_) {
			for (auto &seg : wire->wire_segments_) {
				for (size_t i = 1; i < seg->rpoints_.size(); ++i) {
					x1 = seg->rpoints_[i - 1]->x_;
					y1 = seg->rpoints_[i - 1]->y_;
					x2 = seg->rpoints_[i]->x_;
					y2 = seg->rpoints_[i]->y_;
					z1 = z2 = stoi(seg->layer_name_.substr(5));
					if (x1 != x2)
						assert(y1 == y2);
					if (y1 != y2)
						assert(x1 == x2);
					_gnets[netId].addPtPair(
							layout.getGcell(Point3D(x1, y1, z1 - 1)),
							layout.getGcell(Point3D(x2, y2, z2 - 1)));
					_gnets[netId]._isRealNet = true;
				}
			}
		}
		for (auto &via : net.second->vias_) {
			x1 = x2 = via->x_;
			y1 = y2 = via->y_;
			z1 = stoi(via->name_.substr(3, 1));
			z2 = z1 + 1;
			_gnets[netId].addPtPair(layout.getGcell(Point3D(x1, y1, z1 - 1)),
					layout.getGcell(Point3D(x2, y2, z2 - 1)));
			_gnets[netId]._isRealNet = true;
		}
		netId++;
	}
}

void RoutingDB_DR::writeGlobalWires(const LayoutDR &layout,
		const char *wiresFile) const {
	writeGlobalWires(layout, string(wiresFile));
}

void RoutingDB_DR::writeGlobalWires(const LayoutDR &layout,
		const string &wiresFile) const {
	ofstream outp(wiresFile);
	if (!outp.is_open()) {
		printf("ERROR %s %s %d\n", __FILE__, __func__, __LINE__);
		exit(1);
	}
	char buf[1024];
	float x1, y1, x2, y2;
	int z1, z2;
	cout << "Writing rtFile" << endl;
	for (size_t netId = 0; netId < _gnets.size(); netId++) {
		if (netId % 10000 == 0) {
			cout << netId << "/" << _gnets.size() << endl;
		}
		auto &gnet = _gnets[netId];
		if (gnet.isRealNet()) {
			sprintf(buf, "n%lu %lu", netId, netId);
			outp << buf << endl;
			for (auto gw : gnet._gWires) {
				if (gw._pWireId != -1) {
					x1 = gw._x;
					y1 = gw._y;
					z1 = gw._z;
					x2 = gnet._gWires[gw._pWireId]._x;
					y2 = gnet._gWires[gw._pWireId]._y;
					z2 = gnet._gWires[gw._pWireId]._z;
					if (x1 > x2 || y1 > y2 || z1 > z2) {
						swap(x1, x2);
						swap(y1, y2);
						swap(z1, z2);
					}
					sprintf(buf, "(%.0f,%.0f,%d)-(%.0f,%.0f,%d)", x1, y1,
							z1 + 1, x2, y2, z2 + 1);
					outp << buf << endl;
				}
			}
			//sprintf(buf, "(%.0f,%.0f,%d)-(%.0f,%.0f,%d)", xs * xGrid,
			//		ys * yGrid, zs + 1, xf * xGrid, yf * yGrid, zf + 1);
			//outp << buf << endl;
			outp << "!" << endl;
		}
	}
	outp.close();
}

// Initialize routing tree for every Gnet
void RoutingDB_DR::initRoutingTreeForAllGnets() {
	_treeTable.resize(_numLayers);
	_uMapCell2LocInTable.resize(_numLayers);

	for (size_t i = 0; i < _gnets.size(); i++) {
		Gnet &gnet = _gnets[i];
		if (!gnet.isGlobal())
			continue;
		initRoutingTree(gnet);
	}
	_treeTable.clear();
	_uMapCell2LocInTable.clear();
}

// Initialize routing tree, populate treeTable for routing purposes
// Structure of treeTable : (int x, int y, int z, int indexId, int distPrev, int distNext, bool hasDnVia, bool hasUpVia)
void RoutingDB_DR::initRoutingTree(Gnet &gnet) {
	for (int z = 0; z < _numLayers; z++) {
		_treeTable[z].clear();
		_uMapCell2LocInTable[z].clear();
	}

	int indexId = 0;

// Add points to treeTable
	for (auto ptPair : gnet._gpPairArray) {
		Gcell &pt1 = ptPair.first;
		Gcell &pt2 = ptPair.second;
		if (pt1 == pt2)
			continue;
		const int dist = pt1 - pt2;
		if (pt1._z == pt2._z) {
			_treeTable[pt1._z].push_back(
					PtInfo(pt1._x, pt1._y, pt1._z, -1,
							(_dirLayers[pt1._z] == V ?
									(pt1._y > pt2._y) : (pt1._x > pt2._x)) ?
									dist : -1,
							(_dirLayers[pt1._z] == V ?
									(pt1._y < pt2._y) : (pt1._x < pt2._x)) ?
									dist : -1,
							(_dirLayers[pt1._z] == H ?
									(pt1._y > pt2._y) : (pt1._x > pt2._x)) ?
									dist : -1,
							(_dirLayers[pt1._z] == H ?
									(pt1._y < pt2._y) : (pt1._x < pt2._x)) ?
									dist : -1, false, false));
			_treeTable[pt2._z].push_back(
					PtInfo(pt2._x, pt2._y, pt2._z, -1,
							(_dirLayers[pt2._z] == V ?
									(pt1._y < pt2._y) : (pt1._x < pt2._x)) ?
									dist : -1,
							(_dirLayers[pt2._z] == V ?
									(pt1._y > pt2._y) : (pt1._x > pt2._x)) ?
									dist : -1,
							(_dirLayers[pt2._z] == H ?
									(pt1._y < pt2._y) : (pt1._x < pt2._x)) ?
									dist : -1,
							(_dirLayers[pt2._z] == H ?
									(pt1._y > pt2._y) : (pt1._x > pt2._x)) ?
									dist : -1, false, false));
		} else {
			const int zMin = min(pt1._z, pt2._z);
			const int zMax = max(pt1._z, pt2._z);
			for (int z = zMin; z <= zMax; z++)
				_treeTable[z].push_back(
						PtInfo(pt1._x, pt1._y, z, -1, -1, -1, z != zMin,
								z != zMax));
		}
	}

	for (int z = 0; z < _numLayers; z++) {
		if (_treeTable[z].empty())
			continue;
		tmp.clear();
		if (_dirLayers[z] == V)
			sort(_treeTable[z].begin(), _treeTable[z].end(),
					comparePtInfoOnVerLayer());
		else
			sort(_treeTable[z].begin(), _treeTable[z].end(),
					comparePtInfoOnHorLayer());

		int ptNum = 0;
		for (vector<PtInfo>::iterator itr = _treeTable[z].begin();
				itr != _treeTable[z].end(); itr++) {
			if (tmp.empty() || tmp.back() != (*itr)) {
				_uMapCell2LocInTable[z].insert(
						make_pair(itr->getLoc(), ptNum++));
				tmp.push_back(
						PtInfo(itr->_x, itr->_y, itr->_z, -1, -1, -1, false,
								false));
			}
			PtInfo &pt = tmp.back();
			if (itr->_indexId != -1) {
				try {
					if (pt._indexId != -1)
						throw runtime_error("gPins cannot overlap in gNet");
				} catch (const runtime_error &e) {
					cout << "Exception Caught : " << e.what() << endl;
				}
				pt._indexId = itr->_indexId;
			}
			if (itr->_distPrev > pt._distPrev)
				pt._distPrev = itr->_distPrev;
			if (itr->_distNext > pt._distNext)
				pt._distNext = itr->_distNext;
			if (itr->_distPrevNP > pt._distPrevNP)
				pt._distPrevNP = itr->_distPrevNP;
			if (itr->_distNextNP > pt._distNextNP)
				pt._distNextNP = itr->_distNextNP;
			if (itr->_hasDnVia)
				pt._hasDnVia = true;
			if (itr->_hasUpVia)
				pt._hasUpVia = true;
		}

		_treeTable[z].swap(tmp);

		if (_treeTable[z].size() == 1)
			continue;

		for (int cur = 0; cur < static_cast<int>(_treeTable[z].size()); cur++) {
			PtInfo &pt = _treeTable[z][cur];
			if (pt._distNext > -1) {
				int next = _uMapCell2LocInTable[z][
						_dirLayers[z] == H ?
								Gcell(pt._x + pt._distNext, pt._y, pt._z) :
								Gcell(pt._x, pt._y + pt._distNext, pt._z)];
				for (int i = cur; i < next; i++) {
					int dist = _treeTable[z][i] - _treeTable[z][i + 1];
					_treeTable[z][i]._distNext = dist;
					_treeTable[z][i + 1]._distPrev = dist;
				}
			}
			if (pt._distNextNP > -1) {
				int next = _uMapCell2LocInTable[z][
						_dirLayers[z] == V ?
								Gcell(pt._x + pt._distNextNP, pt._y, pt._z) :
								Gcell(pt._x, pt._y + pt._distNextNP, pt._z)];
				for (int i = cur, j = i + 1; j < next; j++) {
					if (_dirLayers[z] == V ?
							_treeTable[z][i]._y == _treeTable[z][j]._y :
							_treeTable[z][i]._x == _treeTable[z][j]._x) {
						int dist = _treeTable[z][i] - _treeTable[z][j];
						_treeTable[z][i]._distNextNP = dist;
						_treeTable[z][j]._distPrevNP = dist;
						i = j;
					}
				}
			}
		}
	}

	_vecSharePin.clear();
	_vecSharePin.resize(gnet._gPins.size());
	for (auto gPin : gnet._gPins) { // look for points within the gpin's shape
		for (size_t i = 0; i < _treeTable[gPin._z].size(); ++i) {
			auto &pt = _treeTable[gPin._z][i];
			if (gPin._shape.coversPoint(pt._x, pt._y, extValue[pt._z])) {
				if (indexId == 0 && gnet._gWires.size() == 0) {
					gnet._gWires.emplace_back(pt._x, pt._y, pt._z, -1, 0); // add the root wire
					gnet._root = 0;
				}
				pt._indexId = indexId;
				_vecSharePin[indexId].insert(i);
			}
		}
		indexId++;
	}
	frontier.clear();
	if (gnet._gWires.size() == 0) {
		cout << gnet.getName() << endl;
		exit(-3);
	}
	frontier.emplace_back(gnet._gWires[0]._x, gnet._gWires[0]._y,
			gnet._gWires[0]._z);
	set<pair<int, int>> connected_part; // as indices of _treeTable
	size_t head = 0;
	size_t sharePinCnt = 0;
	size_t existingPts = 0;
	while (head < frontier.size()) {
		Gcell &pt = frontier[head];
		int loc = _uMapCell2LocInTable[pt._z][pt];
		PtInfo &ptInfo = _treeTable[pt._z][loc];
		if (ptInfo._indexId > -1) {
			if (_vecSharePin[ptInfo._indexId].size() > 1) {
				for (auto otherLoc : _vecSharePin[ptInfo._indexId]) {
					if (otherLoc == loc)
						continue;
					PtInfo &neighorPtInfo = _treeTable[pt._z][otherLoc];
					if (connected_part.insert(make_pair(pt._z, otherLoc)).second) {
						gnet._gWires.emplace_back(neighorPtInfo._x,
								neighorPtInfo._y, neighorPtInfo._z, head,
								neighorPtInfo._indexId);
						frontier.emplace_back(neighorPtInfo._x,
								neighorPtInfo._y, neighorPtInfo._z);
						sharePinCnt++;
					} else {
						existingPts++;
					}
				}
			}
			_vecSharePin[ptInfo._indexId].clear();
		}
		if (ptInfo._distNext > -1) {
			PtInfo &neighorPtInfo = _treeTable[pt._z][loc + 1];
			neighorPtInfo._distPrev = -1;
			if (connected_part.insert(make_pair(pt._z, loc + 1)).second) {
				gnet._gWires.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z, head, neighorPtInfo._indexId);
				frontier.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z);
			} else {
				existingPts++;
			}
		}
		if (ptInfo._distPrev > -1) {
			PtInfo &neighorPtInfo = _treeTable[pt._z][loc - 1];
			neighorPtInfo._distNext = -1;
			if (connected_part.insert(make_pair(pt._z, loc - 1)).second) {
				gnet._gWires.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z, head, neighorPtInfo._indexId);
				frontier.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z);
			} else {
				existingPts++;
			}
		}
		if (ptInfo._distNextNP > -1) {
			int otherLoc = _uMapCell2LocInTable[pt._z][
					_dirLayers[pt._z] == V ?
							Gcell(pt._x + ptInfo._distNextNP, pt._y, pt._z) :
							Gcell(pt._x, pt._y + ptInfo._distNextNP, pt._z)];
			PtInfo &neighorPtInfo = _treeTable[pt._z][otherLoc];
			neighorPtInfo._distPrevNP = -1;
			if (connected_part.insert(make_pair(pt._z, otherLoc)).second) {
				gnet._gWires.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z, head, neighorPtInfo._indexId);
				frontier.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z);
			} else {
				existingPts++;
			}
		}
		if (ptInfo._distPrevNP > -1) {
			int otherLoc = _uMapCell2LocInTable[pt._z][
					_dirLayers[pt._z] == V ?
							Gcell(pt._x - ptInfo._distPrevNP, pt._y, pt._z) :
							Gcell(pt._x, pt._y - ptInfo._distPrevNP, pt._z)];
			PtInfo &neighorPtInfo = _treeTable[pt._z][otherLoc];
			neighorPtInfo._distNextNP = -1;
			if (connected_part.insert(make_pair(pt._z, otherLoc)).second) {
				gnet._gWires.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z, head, neighorPtInfo._indexId);
				frontier.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z);
			} else {
				existingPts++;
			}
		}
		if (ptInfo._hasUpVia) {
			int otherLoc = _uMapCell2LocInTable[pt._z + 1][Gcell(pt._x, pt._y,
					pt._z + 1)];
			PtInfo &neighorPtInfo = _treeTable[pt._z + 1][otherLoc];
			neighorPtInfo._hasDnVia = false;
			if (connected_part.insert(make_pair(pt._z + 1, otherLoc)).second) {
				gnet._gWires.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z, head, neighorPtInfo._indexId);
				frontier.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z);
			} else {
				existingPts++;
			}
		}
		if (ptInfo._hasDnVia) {
			int otherLoc = _uMapCell2LocInTable[pt._z - 1][Gcell(pt._x, pt._y,
					pt._z - 1)];
			PtInfo &neighorPtInfo = _treeTable[pt._z - 1][otherLoc];
			neighorPtInfo._hasUpVia = false;
			if (connected_part.insert(make_pair(pt._z - 1, otherLoc)).second) {
				gnet._gWires.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z, head, neighorPtInfo._indexId);
				frontier.emplace_back(neighorPtInfo._x, neighorPtInfo._y,
						neighorPtInfo._z);
			} else {
				existingPts++;
			}
		}
		head++;
	}
	if (existingPts)
		cout << "<I> Existing points: " << existingPts << " on net "
				<< gnet._name << endl;
	if (!(gnet._gWires.size()
			== gnet._gpPairArray.size() + 1 + sharePinCnt - existingPts)) {
		cerr << gnet._gWires.size() << " != " << gnet._gpPairArray.size()
				<< " + 1 + " << sharePinCnt << " - " << existingPts << endl;
		gnet.printGnet();
		for (auto gp : gnet._gpPairArray) {
			cerr << "(" << gp.first._x << "," << gp.first._y << ","
					<< gp.first._z << ")--";
			cerr << "(" << gp.second._x << "," << gp.second._y << ","
					<< gp.second._z << ")" << endl;
		}
		// assert(0);
	}
	for (auto &gw : gnet._gWires) {
		if (gw._pWireId > -1) {
			auto &pw = gnet._gWires[gw._pWireId];
			assert(
					(gw._x == pw._x ? 1 : 0) + (gw._y == pw._y ? 1 : 0)
							+ (gw._z == pw._z ? 1 : 0) == 2
							|| (gw._realPinId > -1
									&& gw._realPinId == pw._realPinId
									&& gw._z == pw._z
									&& (gw._x != pw._x || gw._y != pw._y)));
		}
	}
#if 0
    /* read through the wire vector and remove antenna wires in linear time */
    _numChildren.assign( gnet._gWires.size(), 0 );
    _offset.assign( gnet._gWires.size(), 0 );
    _validWires.assign( gnet._gWires.size(), true );
    for( size_t i = 1; i < gnet._gWires.size(); i++ )
        _numChildren[gnet._gWires[i]._pWireId] ++;
    for( size_t i = 0; i < gnet._gWires.size(); i++ ) {
        if( _numChildren[i] == 0 && gnet._gWires[i]._realPinId == -1 ) {
            /* antenna wire found */
            int curr = i;
            while( curr != -1 && _numChildren[curr] == 0 && gnet._gWires[curr]._realPinId == -1 && _validWires[curr] ) {
                _validWires[curr] = false;
                curr = gnet._gWires[curr]._pWireId;
                if( curr != -1 )
                    _numChildren[curr] --;
            }
        }
    }
    int currOffset = 0;
    int curr = 0;
    _cleanWires.resize( gnet._gWires.size() );
    for( size_t i = 0; i < gnet._gWires.size(); i++ ) {
    	if( _validWires[i] ) {
            _cleanWires[curr] = gnet._gWires[i];
            if( _cleanWires[curr]._pWireId != -1 ) {
                _cleanWires[curr]._pWireId -= _offset[ _cleanWires[curr]._pWireId ];
            }
            curr ++;
        }
        else {
            currOffset ++;
        }
        _offset[i] = currOffset;
    }
    assert( curr + currOffset == static_cast<int>(gnet._gWires.size()) );
    _cleanWires.resize( curr );
    gnet._gWires.swap( _cleanWires );
#endif
}

// Update edge and via demands from info on _gWires
void RoutingDB_DR::updateEdgeDemands(const LayoutDR &layout) {
	int cnt_less = 0;
	for (size_t k = 0; k < _gnets.size(); ++k) {
		auto &gnet = _gnets[k];
		if (!gnet.isGlobal())
			continue;
		// All pins
		for (size_t i = 0; i < gnet._gPins.size(); ++i) {
			int &z = gnet._gPins[i]._z;
			for (int y = gnet._gPins[i]._ly; y < gnet._gPins[i]._uy; y += 10) {
				for (int x = gnet._gPins[i]._lx; x <= gnet._gPins[i]._ux; x +=
						10) {
					if (_occupiedGnetId[z].count(Gcell(x, y, z))
							&& gnet._gPins[i]._shape.coversPoint(x, y)) {
						if (!(_occupiedGnetId[z].at(Gcell(x, y, z)) == -1
								|| _occupiedGnetId[z].at(Gcell(x, y, z))
										== static_cast<int>(k))) {
							cerr << _gnets[k].getName() << " vs "
									<< _occupiedGnetId[z].at(Gcell(x, y, z))
									<< " @ (" << x << "," << y << "," << z
									<< ")" << endl;
						}
						_occupiedGnetId[z].at(Gcell(x, y, z)) = k;
					}
				}
			}
			for (int y = gnet._gPins[i]._ly; y < gnet._gPins[i]._uy; y += 5) {
				for (int x = gnet._gPins[i]._lx; x <= gnet._gPins[i]._ux; x +=
						5) {
					int edgeId = findEdge(x, y, z);
					if (edgeId != -1
							&& gnet._gPins[i]._shape.coversPoint(x, y)) {
						addEdgeDemand(edgeId);
						gnet._occupiedEdges.insert(edgeId);
					}
				}
			}
		}
		// gWires not connected to pins
		for (size_t i = 0; i < gnet._gWires.size(); i++) {
			const GlobalWire &gWire = gnet._gWires[i];
			if (gWire._pWireId == -1)
				continue;
			const GlobalWire &pWire = gnet._gWires[gWire._pWireId];
			if (gWire._realPinId == pWire._realPinId && gWire._realPinId != -1)
				continue;
			int y_to_use;
			if (pWire._z == gWire._z) {
				int z = gWire._z;
				//if (gWire._realPinId == -1) {
				if (gWire._x != pWire._x) {
					if (gWire._realPinId > -1) {
						assert(
								gnet._gPins[gWire._realPinId]._shape.coversPoint(
										gWire._x, pWire._y, extValue[z]));
						y_to_use = pWire._y;
					} else if (pWire._realPinId > -1) {
						assert(
								gnet._gPins[pWire._realPinId]._shape.coversPoint(
										pWire._x, gWire._y, extValue[z]));
						y_to_use = gWire._y;
					} else {
						assert(gWire._y == pWire._y);
						y_to_use = pWire._y;
					}
					int cnt = 0, cnt_grid = 0;
					for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin +=
							10) {
						for (int y = y_to_use - margin; y <= y_to_use + margin;
								y += (margin == 0 ? 1 : 2 * margin)) {
							for (int x = min(gWire._x, pWire._x);
									x <= max(gWire._x, pWire._x); x += 10) {
								if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
									assert(
											_occupiedGnetId[z].at(
													Gcell(x, y, z)) == -1
													|| _occupiedGnetId[z].at(
															Gcell(x, y, z))
															== static_cast<int>(k));
									_occupiedGnetId[z].at(Gcell(x, y, z)) = k;
									cnt_grid++;
								}
							}
							if (cnt_grid)
								break;
						}
						if (cnt_grid)
							break;
					}
					for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin +=
							5) {
						for (int y = y_to_use - margin; y <= y_to_use + margin;
								y += (margin == 0 ? 1 : 2 * margin)) {
							for (int x = min(gWire._x, pWire._x);
									x <= max(gWire._x, pWire._x); x += 5) {
								int edgeId = findEdge(x, y, z);
								if (edgeId != -1) {
									addEdgeDemand(edgeId);
									gnet._occupiedEdges.insert(edgeId);
									cnt++;
								}
							}
							if (cnt)
								break;
						}
						if (cnt)
							break;
					}
					if (cnt == 0
							&& abs(gWire._x - pWire._x)
									>= layout._trackWidth[gWire._z]) {
						cerr << gWire._x << " " << pWire._x << " " << pWire._y
								<< " " << pWire._z << endl;
						exit(-1);
					}
				} else if (gWire._y != pWire._y) {
					// assert(gWire._realPinId > -1 || gWire._x == pWire._x);
					int cnt = 0, cnt_grid = 0;
					for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin +=
							10) {
						for (int x = pWire._x - margin; x <= pWire._x + margin;
								x += (margin == 0 ? 1 : 2 * margin)) {
							for (int y = min(gWire._y, pWire._y);
									y <= max(gWire._y, pWire._y); y += 10) {
								if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
									if (!(_occupiedGnetId[z].at(Gcell(x, y, z))
											== -1
											|| _occupiedGnetId[z].at(
													Gcell(x, y, z))
													== static_cast<int>(k))) {
										cerr
												<< _occupiedGnetId[z].at(
														Gcell(x, y, z))
												<< " vs " << k << "@" << x
												<< "," << y << "," << z << endl;
										assert(0);
									}
									_occupiedGnetId[z].at(Gcell(x, y, z)) = k;
									cnt_grid++;
								}
							}
							if (cnt_grid)
								break;
						}
						if (cnt_grid)
							break;
					}
					for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin +=
							5) {
						for (int x = pWire._x - margin; x <= pWire._x + margin;
								x += (margin == 0 ? 1 : 2 * margin)) {
							for (int y = min(gWire._y, pWire._y);
									y <= max(gWire._y, pWire._y); y += 5) {
								int edgeId = findEdge(x, y, z);
								if (edgeId != -1) {
									addEdgeDemand(edgeId);
									gnet._occupiedEdges.insert(edgeId);
									cnt++;
								}
							}
							if (cnt)
								break;
						}
						if (cnt)
							break;
					}
					if (cnt == 0
							&& abs(gWire._y - pWire._y)
									>= layout._trackHeight[gWire._z]) {
						cerr << pWire._x << " " << pWire._z << endl;
						exit(-1);
					}
				} else {
					assert(0);
				}
				//}
				// edges not connected to pins
			} else { // gWire._z != pWire._z
				for (int z = min(gWire._z, pWire._z);
						z < max(gWire._z, pWire._z); z++) {
					assert(
							gWire._realPinId > -1
									|| abs(gWire._x - pWire._x)
											<= extValue[z] + extValue[z + 1]);
					assert(
							gWire._realPinId > -1
									|| abs(gWire._y - pWire._y)
											<= extValue[z] + extValue[z + 1]);
					// fill it and its 8 neighbors
					double x_start, x_mid, x_end, x_step, y_start, y_mid, y_end,
							y_step;
					if (_dirLayers[z] == V) {
						x_mid = round(
								(pWire._x - layout._trackOffsetX[z])
										/ layout._trackWidth[z])
								* layout._trackWidth[z]
								+ layout._trackOffsetX[z];
						x_step = layout._trackWidth[z];
						x_start = max(layout._trackOffsetX[z], x_mid - x_step);
						x_end = min(
								layout._trackOffsetX[z]
										+ layout._numTracksX[z]
												* layout._trackWidth[z],
								x_mid + x_step);
						y_mid = round(
								(pWire._y - layout._trackOffsetY[z + 1])
										/ layout._trackHeight[z + 1])
								* layout._trackHeight[z + 1]
								+ layout._trackOffsetY[z + 1];
						y_step = layout._trackHeight[z + 1];
						y_start = max(layout._trackOffsetY[z + 1],
								y_mid - y_step);
						y_end = min(
								layout._trackOffsetY[z + 1]
										+ layout._numTracksY[z + 1]
												* layout._trackHeight[z + 1],
								y_mid + y_step);
					} else { // dirLayer == H
						x_mid = round(
								(pWire._x - layout._trackOffsetX[z + 1])
										/ layout._trackWidth[z + 1])
								* layout._trackWidth[z + 1]
								+ layout._trackOffsetX[z + 1];
						x_step = layout._trackWidth[z + 1];
						x_start = max(layout._trackOffsetX[z + 1],
								x_mid - x_step);
						x_end = min(
								layout._trackOffsetX[z + 1]
										+ layout._numTracksX[z + 1]
												* layout._trackWidth[z + 1],
								x_mid + x_step);
						y_mid = round(
								(pWire._y - layout._trackOffsetY[z])
										/ layout._trackHeight[z])
								* layout._trackHeight[z]
								+ layout._trackOffsetY[z];
						y_step = layout._trackHeight[z];
						y_start = max(layout._trackOffsetY[z], y_mid - y_step);
						y_end = min(
								layout._trackOffsetY[z]
										+ layout._numTracksY[z]
												* layout._trackHeight[z],
								y_mid + y_step);
					}
					int x_cnt = (x_start == x_mid || x_mid == x_end) ? 2 : 3;
					int y_cnt = (y_start == y_mid || y_mid == y_end) ? 2 : 3;
					cnt_less += 9 - x_cnt * y_cnt;
					//for (int x = x_start; x <= x_end; x += x_step) {
					//	for (int y = y_start; y <= y_end; y += y_step) {
					int x = x_mid, y = y_mid;
					int viaId = findVia(x, y, z);
					if (viaId != -1) {
						addViaDemand(viaId);
						gnet._occupiedVias.insert(viaId);
					}
					if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
						if (_occupiedGnetId[z].at(Gcell(x, y, z))
								!= static_cast<int>(k)) {
							if (_occupiedGnetId[z].at(Gcell(x, y, z)) != -1) {
								cerr << "line 742" << " (" << x << "," << y
										<< "," << z << "): "
										<< _gnets[_occupiedGnetId[z].at(
												Gcell(x, y, z))].getName()
										<< " vs " << _gnets[k].getName()
										<< endl;
								assert(0);
							}
							_occupiedGnetId[z].at(Gcell(x, y, z)) = k;
						}
					}
					//}
					//}
#if 0
					// fill 8 edges located like below on upper and lower metal layers
					//            _| |_
					//            _via_    (To ensure metal spacing caused by the via)
					//             | |
					for (int zz = z; zz <= z + 1; zz++) {
						int x_cell = round(
								(pWire._x - layout._trackOffsetX[zz])
										/ layout._trackWidth[zz]);
						int y_cell = round(
								(pWire._y - layout._trackOffsetY[zz])
										/ layout._trackHeight[zz]);
						if (x_cell > 0 && y_cell > 0) {
							// lower-left corner, vertical
							int x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									- layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									- layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
							// lower-left corner, horizontal
							x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									- layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									- layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
						}
						if (x_cell
								< static_cast<int>(layout._numTracksX[zz]) - 1
								&& y_cell > 0) {
							// lower-right corner, vertical
							int x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									+ layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									- layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
							// lower-right corner, horizontal
							x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									+ layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									- layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
						}
						if (x_cell
								< static_cast<int>(layout._numTracksX[zz]) - 1
								&& y_cell
										< static_cast<int>(layout._numTracksY[zz])
												- 1) {
							// upper-right corner, vertical
							int x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									+ layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									+ layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
							// lower-right corner, horizontal
							x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									+ layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									+ layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
						}
						if (x_cell > 0
								&& y_cell
										< static_cast<int>(layout._numTracksY[zz])
												- 1) {
							// upper-left corner, vertical
							int x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									- layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									+ layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
							// upper-left corner, horizontal
							x = x_cell * layout._trackWidth[zz]
									+ layout._trackOffsetX[zz]
									- layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
									+ layout._trackOffsetY[zz]
									+ layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							addEdgeDemand(edgeId);
							gnet._occupiedEdges.insert(edgeId);
						}
					} // for zz
#endif
				} // for z
			} // if (gWire._z == pWire._z) else ...
		} // for gWire
	} // for gnet
	cout << "<I> Via demand less on boundaries: " << cnt_less << endl;
}

bool RoutingDB_DR::checkChildren(Gnet &gnet, vector<int> &_numChildren,
		vector<bool> &_validWires, double &wlReduced, int splitLayer,
		double &wlReducedAboveSL, int curr, const LayoutDR &layout,
		bool &endsAtvpLower, bool &endsAtvpUpper, Gcell &vpGridLower,
		Gcell &vpGridUpper, const Vpin &vp) {
	bool hasRemovedWires = false;
	if (gnet._gWires[curr]._realPinId != -1) { // removed a pin..
		gnet._gWires[curr]._pWireId = -1;
		_validWires[curr] = true;
		return hasRemovedWires;
	}
	if (_numChildren[curr] == 0)
		return hasRemovedWires;
	if (_numChildren[curr] == 1) {
		size_t j;
		for (j = 0; j < gnet._gWires.size(); j++) {
			if (gnet._gWires[j]._pWireId == curr && _validWires[j]) {
				if (min(gnet._gWires[curr]._z, gnet._gWires[j]._z) <= splitLayer
						&& max(gnet._gWires[curr]._z, gnet._gWires[j]._z)
								> splitLayer) {
					if (!(gnet._gWires[curr]._x == vp.xCoord
							&& gnet._gWires[curr]._y == vp.yCoord)) {
						if (gnet._gWires[curr]._z > splitLayer) {
							endsAtvpUpper = true;
							vpGridUpper = Gcell(gnet._gWires[curr]._x,
									gnet._gWires[curr]._y,
									gnet._gWires[curr]._z);
						} else {
							endsAtvpLower = true;
							vpGridLower = Gcell(gnet._gWires[curr]._x,
									gnet._gWires[curr]._y,
									gnet._gWires[curr]._z);
						}
					}
					return hasRemovedWires;
				} else {
					curr = j;
					break;
				}
			}
		}
//if (gnet._gWires[curr]._realPinId != -1) { // A is a pin
//	if (gnet._gWires[curr]._pWireId != -1) {
//		_numChildren[gnet._gWires[curr]._pWireId]--;
//		gnet._gWires[curr]._pWireId = -1;
//	}
//	return hasRemovedWires;
//}
		_validWires[curr] = false;

		hasRemovedWires = true;
		GlobalWire &gWire = gnet._gWires[curr];
		GlobalWire &pWire = gnet._gWires[gWire._pWireId];
		wlReduced += Gcell(gWire._x, gWire._y, gWire._z).viaWeightedDist(
				Gcell(pWire._x, pWire._y, pWire._z));
		//cout << "Remove " << gWire._x << "," << gWire._y << "," << gWire._z
		//		<< "-" << pWire._x << "," << pWire._y << "," << pWire._z
		//		<< endl;

		if (gWire._z > splitLayer && pWire._z > splitLayer) {
			wlReducedAboveSL +=
					Gcell(gWire._x, gWire._y, gWire._z).viaWeightedDist(
							Gcell(pWire._x, pWire._y, pWire._z));
		}
//printf("Removing gWire %d(%d,%d,%d)-%d(%d,%d,%d)\n", curr, gWire._x,
//		gWire._y, gWire._z, gWire._pWireId, pWire._x, pWire._y,
//		pWire._z);
		if (pWire._z == gWire._z) {
			if (gWire._x != pWire._x) {
				assert(
						abs(gWire._y - pWire._y) <= 2 * extValue[pWire._z]
								|| gWire._realPinId != -1
								|| pWire._realPinId != -1);
				int &z = gWire._z;
				int cnt = 0;
				for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin += 5) {
					for (int y = pWire._y - margin; y <= pWire._y + margin; y +=
							(margin == 0 ? 1 : 2 * margin)) {
						for (int x = min(gWire._x, pWire._x);
								x <= max(gWire._x, pWire._x); x += 5) {
							//			printf("dem/cap=%d/%d, ",
							//					_edges[findEdge( x, gWire._y, gWire._z )].demand(),
							//					_edges[findEdge( x, gWire._y, gWire._z )].cap());
							int edgeId = findEdge(x, y, z);
							if (edgeId != -1) {
								if (!(gnet.findCleanEdge(edgeId))) {
									gnet.setCleanEdge(edgeId, _edges[edgeId]);
								}
								cnt++;
								ripUpEdge(edgeId);
								gnet.removeEdge(edgeId);
								gnet._occupiedEdges.erase(edgeId);
								_dirty = true;
								//			printf("rm (%d,%d,%d)-(%d,%d,%d), dem/cap=%d/%d\n", x, gWire._y, gWire._z, x+1, gWire._y, gWire._z,
								//					_edges[findEdge( x, gWire._y, gWire._z )].demand(),
								//					_edges[findEdge( x, gWire._y, gWire._z )].cap());
							}
						}
						if (cnt)
							break;
					}
					if (cnt)
						break;
				}
				assert(
						cnt
								|| abs(gWire._x - pWire._x)
										< layout._trackWidth[gWire._z]);
			} else if (gWire._y != pWire._y) {
				assert(
						abs(gWire._x - pWire._x) <= 2 * extValue[pWire._z]
								|| gWire._realPinId != -1
								|| pWire._realPinId != -1);
				int &z = pWire._z;
				int cnt = 0;
				for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin += 5) {
					for (int x = pWire._x - margin; x <= pWire._x + margin; x +=
							(margin == 0 ? 1 : 2 * margin)) {
						for (int y = min(gWire._y, pWire._y);
								y <= max(gWire._y, pWire._y); y += 5) {
							//				printf("dem/cap=%d/%d, ",
							//						_edges[findEdge( gWire._x, y, gWire._z )].demand(),
							//						_edges[findEdge( gWire._x, y, gWire._z )].cap());
							int edgeId = findEdge(x, y, z);
							if (edgeId != -1) {
								if (!(gnet.findCleanEdge(edgeId))) {
									gnet.setCleanEdge(edgeId, _edges[edgeId]);
								}
								cnt++;
								ripUpEdge(edgeId);
								gnet.removeEdge(edgeId);
								gnet._occupiedEdges.erase(edgeId);
								_dirty = true;
								//					printf("rm (%d,%d,%d)-(%d,%d,%d), dem/cap=%d/%d\n", gWire._x, y, gWire._z, gWire._x, y+1, gWire._z,
								//							_edges[findEdge( gWire._x, y, gWire._z )].demand(),
								//							_edges[findEdge( gWire._x, y, gWire._z )].cap());
							}
						}
						if (cnt)
							break;
					}
					if (cnt)
						break;
				}
				assert(
						cnt
								|| abs(gWire._y - pWire._y)
										< layout._trackHeight[gWire._z]);
			} else
				assert(0);
		} else {
			for (int z = min(gWire._z, pWire._z); z < max(gWire._z, pWire._z);
					z++) {
				assert(
						abs(gWire._y - pWire._y)
								<= extValue[z] + extValue[z + 1]
								|| gWire._realPinId != -1
								|| pWire._realPinId != -1);
				assert(
						abs(gWire._x - pWire._x)
								<= extValue[z] + extValue[z + 1]
								|| gWire._realPinId != -1
								|| pWire._realPinId != -1);

				// fill it and its 8 neighbors
				double x_start, x_mid, x_end, x_step, y_start, y_mid, y_end,
						y_step;
				if (_dirLayers[z] == V) {
					x_mid = round(
							(pWire._x - layout._trackOffsetX[z])
									/ layout._trackWidth[z])
							* layout._trackWidth[z] + layout._trackOffsetX[z];
					x_step = layout._trackWidth[z];
					x_start = max(layout._trackOffsetX[z], x_mid - x_step);
					x_end = min(
							layout._trackOffsetX[z]
									+ layout._numTracksX[z]
											* layout._trackWidth[z],
							x_mid + x_step);
					y_mid = round(
							(pWire._y - layout._trackOffsetY[z + 1])
									/ layout._trackHeight[z + 1])
							* layout._trackHeight[z + 1]
							+ layout._trackOffsetY[z + 1];
					y_step = layout._trackHeight[z + 1];
					y_start = max(layout._trackOffsetY[z + 1], y_mid - y_step);
					y_end = min(
							layout._trackOffsetY[z + 1]
									+ layout._numTracksY[z + 1]
											* layout._trackHeight[z + 1],
							y_mid + y_step);
				} else { // dirLayer == H
					x_mid = round(
							(pWire._x - layout._trackOffsetX[z + 1])
									/ layout._trackWidth[z + 1])
							* layout._trackWidth[z + 1]
							+ layout._trackOffsetX[z + 1];
					x_step = layout._trackWidth[z + 1];
					x_start = max(layout._trackOffsetX[z + 1], x_mid - x_step);
					x_end = min(
							layout._trackOffsetX[z + 1]
									+ layout._numTracksX[z + 1]
											* layout._trackWidth[z + 1],
							x_mid + x_step);
					y_mid = round(
							(pWire._y - layout._trackOffsetY[z])
									/ layout._trackHeight[z])
							* layout._trackHeight[z] + layout._trackOffsetY[z];
					y_step = layout._trackHeight[z];
					y_start = max(layout._trackOffsetY[z], y_mid - y_step);
					y_end = min(
							layout._trackOffsetY[z]
									+ layout._numTracksY[z]
											* layout._trackHeight[z],
							y_mid + y_step);
				}
				//for (int x = x_start; x <= x_end; x += x_step) {
				//	for (int y = y_start; y <= y_end; y += y_step) {
				int x = x_mid, y = y_mid;
				int viaId = findVia(x, y, z);
				if (!(gnet.findCleanVia(viaId))) {
					gnet.setCleanVia(viaId, _vias[viaId]);
				}
				if (viaId != -1) {
					ripUpVia(viaId);
					gnet.removeVia(viaId);
					gnet._occupiedVias.erase(viaId);
					_dirty = true;
				}
				//					printf("rm (%d,%d,%d)-(%d,%d,%d)\n", gWire._x, gWire._y, z, gWire._x, gWire._y, z+1);
				//}
				//}
#if 0
				// remove 8 edges located like below on upper and lower metal layers
				//            _| |_
				//            _via_    (To ensure metal spacing caused by the via)
				//             | |
				for (int zz = z; zz <= z + 1; zz++) {
					int x_cell = round(
							(pWire._x - layout._trackOffsetX[zz])
							/ layout._trackWidth[zz]);
					int y_cell = round(
							(pWire._y - layout._trackOffsetY[zz])
							/ layout._trackHeight[zz]);
					if (x_cell > 0 && y_cell > 0) {
						// lower-left corner, vertical
						int x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						- layout._trackWidth[zz] / 2;
						int y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						- layout._trackHeight[zz];
						size_t edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
						// lower-left corner, horizontal
						x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						- layout._trackWidth[zz];
						y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						- layout._trackHeight[zz] / 2;
						edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
					}
					if (x_cell < static_cast<int>(layout._numTracksX[zz]) - 1
							&& y_cell > 0) {
						// lower-right corner, vertical
						int x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						+ layout._trackWidth[zz] / 2;
						int y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						- layout._trackHeight[zz];
						size_t edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
						// lower-right corner, horizontal
						x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						+ layout._trackWidth[zz];
						y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						- layout._trackHeight[zz] / 2;
						edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
					}
					if (x_cell < static_cast<int>(layout._numTracksX[zz]) - 1
							&& y_cell
							< static_cast<int>(layout._numTracksY[zz])
							- 1) {
						// upper-right corner, vertical
						int x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						+ layout._trackWidth[zz] / 2;
						int y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						+ layout._trackHeight[zz];
						size_t edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
						// lower-right corner, horizontal
						x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						+ layout._trackWidth[zz];
						y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						+ layout._trackHeight[zz] / 2;
						edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
					}
					if (x_cell > 0
							&& y_cell
							< static_cast<int>(layout._numTracksY[zz])
							- 1) {
						// upper-left corner, vertical
						int x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						- layout._trackWidth[zz] / 2;
						int y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						+ layout._trackHeight[zz];
						size_t edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
						// upper-left corner, horizontal
						x = x_cell * layout._trackWidth[zz]
						+ layout._trackOffsetX[zz]
						- layout._trackWidth[zz];
						y = y_cell * layout._trackHeight[zz]
						+ layout._trackOffsetY[zz]
						+ layout._trackHeight[zz] / 2;
						edgeId = findEdge(x, y, zz);
						reduceEdgeDemand(edgeId);
						gnet._occupiedEdges.erase(edgeId);
					}
				} // for zz
#endif
			}
		}
		int tmp = gnet._gWires[curr]._pWireId;
		if (tmp != -1)
			_numChildren[tmp]--;
		bool stop = false; // should we stop the recursion that checks children?
		if (gnet._gWires[curr]._realPinId != -1) { // A is a pin
			_validWires[curr] = true;
			if (gnet._gWires[curr]._pWireId != -1) {
				gnet._gWires[curr]._pWireId = -1;
			}
			stop = true;
		} else { // check if A is a v-pin
			for (j = 0; j < gnet._gWires.size(); j++) {
				if (gnet._gWires[j]._pWireId == curr && _validWires[j]) {
					if (min(gnet._gWires[curr]._z, gnet._gWires[j]._z)
							<= splitLayer
							&& max(gnet._gWires[curr]._z, gnet._gWires[j]._z)
									> splitLayer) { // ends at a v-pin
						if (!(gnet._gWires[curr]._x == vp.xCoord
								&& gnet._gWires[curr]._y == vp.yCoord)) { // which is not itself
							if (gnet._gWires[curr]._z > splitLayer) {
								endsAtvpUpper = true;
								vpGridUpper = Gcell(gnet._gWires[curr]._x,
										gnet._gWires[curr]._y,
										gnet._gWires[curr]._z);
							} else {
								endsAtvpLower = true;
								vpGridLower = Gcell(gnet._gWires[curr]._x,
										gnet._gWires[curr]._y,
										gnet._gWires[curr]._z);
							}
							_validWires[curr] = true;
							if (gnet._gWires[curr]._pWireId != -1) {
								gnet._gWires[curr]._pWireId = -1;
							}
						}
						stop = true;
					}
					break;
				}
			}
		}
		if (!stop)
			checkChildren(gnet, _numChildren, _validWires, wlReduced,
					splitLayer, wlReducedAboveSL, curr, layout, endsAtvpLower,
					endsAtvpUpper, vpGridLower, vpGridUpper, vp);
	}
	return hasRemovedWires;
}

#define MAX_MARGIN_OFFGRID_XY 5000
void RoutingDB_DR::updateGridOccupation(const int gnetId, bool noReset) {
	if (!noReset) {
		// reset
		for (size_t z = 0; z < _occupiedGnetId.size(); ++z) {
			for (auto &item : _occupiedGnetId[z]) {
				if (item.second == gnetId) {
					item.second = -1;
				}
			}
		}
	}
// occupy
	auto &gnet = _gnets[gnetId];
	// All pins
	for (size_t i = 0; i < gnet._gPins.size(); ++i) {
		int &z = gnet._gPins[i]._z;
		for (int y = gnet._gPins[i]._ly; y < gnet._gPins[i]._uy; y += 10) {
			for (int x = gnet._gPins[i]._lx; x <= gnet._gPins[i]._ux; x += 10) {
				if (_occupiedGnetId[z].count(Gcell(x, y, z))
						&& gnet._gPins[i]._shape.coversPoint(x, y)) {
					if (!(_occupiedGnetId[z].at(Gcell(x, y, z)) == -1
							|| _occupiedGnetId[z].at(Gcell(x, y, z))
									== static_cast<int>(gnetId))) {
						cerr << gnetId << " vs "
								<< _occupiedGnetId[z].at(Gcell(x, y, z))
								<< " @ (" << x << "," << y << "," << z << ")"
								<< endl;
					}
					_occupiedGnetId[z].at(Gcell(x, y, z)) = gnetId;
				}
			}
		}
	}
	// wire/via connections
	for (size_t i = 0; i < gnet._gWires.size(); ++i) {
		const GlobalWire &gWire = gnet._gWires[i];
		if (gWire._pWireId == -1)
			continue;
		const GlobalWire &pWire = gnet._gWires[gWire._pWireId];
		if (gWire._realPinId == pWire._realPinId && gWire._realPinId != -1)
			continue;
		int y_to_use;

		if (pWire._z == gWire._z) {
			int z = gWire._z;
			//if (gWire._realPinId == -1) {
			if (gWire._x != pWire._x) {
				if (gWire._realPinId > -1) {
					y_to_use = pWire._y;
				} else if (pWire._realPinId > -1) {
					y_to_use = gWire._y;
				} else {
					assert(gWire._y == pWire._y);
					y_to_use = pWire._y;
				}
				int cnt_grid = 0;
				for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin +=
						10) {
					for (int y = y_to_use - margin; y <= y_to_use + margin; y +=
							(margin == 0 ? 1 : 2 * margin)) {
						for (int x = min(gWire._x, pWire._x);
								x <= max(gWire._x, pWire._x); x += 10) {
							if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
								assert(
										_occupiedGnetId[z].at(Gcell(x, y, z))
												== -1
												|| _occupiedGnetId[z].at(
														Gcell(x, y, z))
														== static_cast<int>(gnetId));
								_occupiedGnetId[z].at(Gcell(x, y, z)) = gnetId;
								cnt_grid++;
							}
						}
						if (cnt_grid)
							break;
					}
					if (cnt_grid)
						break;
				}
				assert(cnt_grid);
			} else if (gWire._y != pWire._y) {
				// assert(gWire._realPinId > -1 || gWire._x == pWire._x);
				int cnt_grid = 0;
				for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin +=
						10) {
					for (int x = pWire._x - margin; x <= pWire._x + margin; x +=
							(margin == 0 ? 1 : 2 * margin)) {
						for (int y = min(gWire._y, pWire._y);
								y <= max(gWire._y, pWire._y); y += 10) {
							if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
								if (!(_occupiedGnetId[z].at(Gcell(x, y, z))
										== -1
										|| _occupiedGnetId[z].at(Gcell(x, y, z))
												== static_cast<int>(gnetId))) {
									cerr
											<< _occupiedGnetId[z].at(
													Gcell(x, y, z)) << " vs "
											<< gnetId << "@" << x << "," << y
											<< "," << z << endl;
									exit(-1);
								}
								_occupiedGnetId[z].at(Gcell(x, y, z)) = gnetId;
								cnt_grid++;
							}
						}
						if (cnt_grid)
							break;
					}
					if (cnt_grid)
						break;
				}
				assert(cnt_grid);
			} else {
				assert(0);
			}
		} else { // gWire._z != pWire._z
			int cnt_grid = 0;
			for (int z = min(gWire._z, pWire._z); z < max(gWire._z, pWire._z);
					z++) {
				assert(
						gWire._realPinId > -1 || pWire._realPinId > -1
								|| abs(gWire._x - pWire._x)
										<= extValue[z] + extValue[z + 1]);
				assert(
						gWire._realPinId > -1 || pWire._realPinId > -1
								|| abs(gWire._y - pWire._y)
										<= extValue[z] + extValue[z + 1]);

				if (gWire._realPinId == -1 && pWire._realPinId == -1) {
					for (int x = min(gWire._x, pWire._x);
							x <= max(gWire._x, pWire._x); x += 10) {
						for (int y = min(gWire._y, pWire._y);
								y <= max(gWire._y, pWire._y); y += 10) {
							if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
								if (_occupiedGnetId[z].at(Gcell(x, y, z))
										!= static_cast<int>(gnetId)) {
									if (_occupiedGnetId[z].at(Gcell(x, y, z))
											!= -1) {
										cerr << "line 1379" << " (" << x << ","
												<< y << "," << z << "): "
												<< _gnets[_occupiedGnetId[z].at(
														Gcell(x, y, z))].getName()
												<< " vs "
												<< _gnets[gnetId].getName()
												<< endl;
										exit(-1);
									}
									_occupiedGnetId[z].at(Gcell(x, y, z)) =
											gnetId;
								}
								cnt_grid++;
							}
							if (cnt_grid)
								break;
						}
						if (cnt_grid)
							break;
					}
					//assert(cnt_grid);
				} else { // at least one of gWire and pWire is a pin
					int z = gWire._z;
					for (int x = min(gWire._x, pWire._x);
							x <= max(gWire._x, pWire._x); x += 10) {
						for (int y = min(gWire._y, pWire._y);
								y <= max(gWire._y, pWire._y); y += 10) {
							if (!((gWire._realPinId == -1
									|| gnet._gPins[gWire._realPinId]._shape.coversPoint(
											x, y, extValue[z]))
									&& (pWire._realPinId == -1
											|| gnet._gPins[pWire._realPinId]._shape.coversPoint(
													x, y, extValue[z]))))
								continue;
							if (_occupiedGnetId[z].count(Gcell(x, y, z))) {
								if (_occupiedGnetId[z].at(Gcell(x, y, z))
										!= static_cast<int>(gnetId)) {
									if (_occupiedGnetId[z].at(Gcell(x, y, z))
											!= -1) {
										cerr << "line 1414" << " (" << x << ","
												<< y << "," << z << "): "
												<< _gnets[_occupiedGnetId[z].at(
														Gcell(x, y, z))].getName()
												<< " vs "
												<< _gnets[gnetId].getName()
												<< endl;
										exit(-1);
									}
									_occupiedGnetId[z].at(Gcell(x, y, z)) =
											gnetId;
								}
								cnt_grid++;
							}
							if (cnt_grid)
								break;
						}
						if (cnt_grid)
							break;
					}
					// assert(cnt_grid);
				}
			}
		}
	}
}

/* Rip-up connections to a v-pin, yielding two connected components:
 * one below SL and the other above SL.
 * Other vpins are intact.
 *
 * NOTE: If a rip-up process ends at another v-pin P,
 * then when rerouting, the broken point has to be connected to P's lower/upper
 * endpoint if the rip-up is performed below/above SL,
 * otherwise we can connect it to any grid below/above SL in its connected component.
 ************************************************************************************/
std::tuple<std::unordered_set<Gcell, HashGcell3d>, // other grids above SL
		std::unordered_set<Gcell, HashGcell3d>, // other grids on or below SL
		std::unordered_set<Gcell, HashGcell3d>, // other vertices above SL
		std::unordered_set<Gcell, HashGcell3d>, // other vertices on or below SL
		vector<int>, vector<int>, double, double> RoutingDB_DR::ripUpNet(
		const LayoutDR &layout, Vpin &vp) {
// returns: (vpins, WL reduced)
	using namespace boost;
	vector<int> removedEdges;
	vector<int> removedVias;
	std::unordered_set<Gcell, HashGcell3d> otherVpinGridsAboveSL,

	otherVpinGridsOnBelowSL, otherVpinVerticesAboveSL,
			otherVpinVerticesOnBelowSL;
	double wlReduced = 0;
	double wlReducedAboveSL = 0;
	bool endsAtvpUpper = false, endsAtvpLower = false;
	Gcell vpGridUpper(0, 0, -1), vpGridLower(0, 0, -1);
	Gnet &gnet = _gnets[vp.gnetID];
	std::unordered_set<Gcell, HashGcell3d> remainingGnetVertices;
	std::unordered_set<Gcell, HashGcell3d> remainingGnetGrids;
//		for (int i = 0; i < static_cast<int>(gnet._gWires.size()); ++i) {
//			GlobalWire & gWire = gnet._gWires[i];
//			printf("<I> gWire[%d] (%d,%d,%d)/par=%d,pin=%d\n", i, gWire._x, gWire._y, gWire._z, gWire._pWireId, gWire._realPinId);
//		}
//gnet.printGnet();
	Graph routingTree4cc; // for calculating connected components
	for (auto &gWire : gnet._gWires) {
		add_vertex(routingTree4cc);
	}
	for (size_t i = 0; i < gnet._gWires.size(); i++) {
		auto &gWire = gnet._gWires[i];
		if (gWire._pWireId != -1)
			add_edge(i, gWire._pWireId, routingTree4cc);
	}
	std::vector<int> c(num_vertices(routingTree4cc));
	int nParts = connected_components(routingTree4cc, &c[0]);
	assert(nParts == 1);
	int splitLayer = vp.zCoord;
// Flag the wires to be removed
// (1/2) Remove the vpin wire
	_numChildren.assign(gnet._gWires.size(), 0);
	_offset.assign(gnet._gWires.size(), 0);
	_validWires.assign(gnet._gWires.size(), true);
	for (size_t i = 0; i < gnet._gWires.size(); i++)
		if (gnet._gWires[i]._pWireId != -1)
			_numChildren[gnet._gWires[i]._pWireId]++;
	size_t i;
	for (i = 0; i < gnet._gWires.size(); i++) {
		auto &gWire = gnet._gWires[i];
		if (gWire._pWireId == -1)
			continue;
		auto &pWire = gnet._gWires[gWire._pWireId];
		if (min(gWire._z, pWire._z) == vp.zCoord
				&& max(gWire._z, pWire._z) == vp.zCoord + 1
				&& (gWire._z < pWire._z ? gWire._x : pWire._x) == vp.xCoord
				&& (gWire._z < pWire._z ? gWire._y : pWire._y) == vp.yCoord)
			break;
	}
	if (i >= gnet._gWires.size()) {
		_gnets[vp.gnetID].printGnet();
		assert(0);
	}
	_validWires[i] = false;
	auto &gWire1 = gnet._gWires[i];
	auto &pWire1 = gnet._gWires[gWire1._pWireId];
	wlReduced += Gcell(gWire1._x, gWire1._y, gWire1._z).viaWeightedDist(
			Gcell(pWire1._x, pWire1._y, pWire1._z));
	//cout << "Remove " << gWire1._x << "," << gWire1._y << "," << gWire1._z
	//		<< "-" << pWire1._x << "," << pWire1._y << "," << pWire1._z << endl;
	if (gWire1._z > splitLayer && pWire1._z > splitLayer) {
		wlReducedAboveSL +=
				Gcell(gWire1._x, gWire1._y, gWire1._z).viaWeightedDist(
						Gcell(pWire1._x, pWire1._y, pWire1._z));
	}
	int vpAboveWireID =
			(gnet._gWires[i]._z == vp.zCoord + 1) ?
					i : gnet._gWires[i]._pWireId;
	GlobalWire &gWire = gnet._gWires[i];
	GlobalWire &pWire = gnet._gWires[gWire._pWireId];
	remove_edge(i, gWire._pWireId, routingTree4cc);
	nParts = connected_components(routingTree4cc, &c[0]);
//for (size_t i = 0; i < num_vertices(routingTree4cc); i++)
//	cout << c[i] << " ";
//cout << endl;
	assert(nParts == 2);
	vp.hasOriginalRoot = (c[gnet._root] != c[vpAboveWireID]);
//		printf("Removing gWire (%d,%d,%d)-(%d,%d,%d)\n", gWire._x, gWire._y, gWire._z,
//				pWire._x, pWire._y, pWire._z);

	if (pWire._z == gWire._z) {
		if (gWire._x != pWire._x) {
			assert(gWire._y == pWire._y);
			int &z = gWire._z;
			int cnt = 0;
			for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin += 5) {
				for (int y = pWire._y - margin; y <= pWire._y + margin;
						y += (margin == 0 ? 1 : 2 * margin)) {
					for (int x = min(gWire._x, pWire._x);
							x <= max(gWire._x, pWire._x); x += 5) {
						//		printf("dem/cap=%d/%d, ",
						//											_edges[findEdge( x, gWire._y, gWire._z )].demand(),
						//											_edges[findEdge( x, gWire._y, gWire._z )].cap());
						int edgeId = findEdge(x, y, z);
						if (edgeId != -1) {
							if (!(gnet.findCleanEdge(edgeId))) {
								gnet.setCleanEdge(edgeId, _edges[edgeId]);
							}
							cnt++;
							ripUpEdge(edgeId);
							gnet.removeEdge(edgeId);
							gnet._occupiedEdges.erase(edgeId);
							_dirty = true;
							//		printf("rm (%d,%d,%d)-(%d,%d,%d), dem/cap=%d/%d\n", x, gWire._y, gWire._z, x+1, gWire._y, gWire._z,
							//				_edges[findEdge( x, gWire._y, gWire._z )].demand(),
							//				_edges[findEdge( x, gWire._y, gWire._z )].cap());
						}
					}
					if (cnt)
						break;
				}
				if (cnt)
					break;
			}
			assert(
					cnt
							|| abs(gWire._x - pWire._x)
									< layout._trackWidth[gWire._z]);
		} else if (gWire._y != pWire._y) {
			assert(abs(gWire._x - pWire._x) <= 2 * extValue[pWire._z]);
			int &z = pWire._z;
			int cnt = 0;
			for (int margin = 0; margin < MAX_MARGIN_OFFGRID; margin += 5) {
				for (int x = pWire._x - margin; x <= pWire._x + margin;
						x += (margin == 0 ? 1 : 2 * margin)) {
					for (int y = min(gWire._y, pWire._y);
							y <= max(gWire._y, pWire._y); y += 5) {
						//		printf("dem/cap=%d/%d, ",
						//								_edges[findEdge( gWire._x, y, gWire._z )].demand(),
						//								_edges[findEdge( gWire._x, y, gWire._z )].cap());
						int edgeId = findEdge(x, y, z);
						if (edgeId != -1) {
							if (!(gnet.findCleanEdge(edgeId))) {
								gnet.setCleanEdge(edgeId, _edges[edgeId]);
							}
							cnt++;
							ripUpEdge(edgeId);
							gnet.removeEdge(edgeId);
							gnet._occupiedEdges.erase(edgeId);
							_dirty = true;
							//		printf("rm (%d,%d,%d)-(%d,%d,%d)\n", gWire._x, y, gWire._z, gWire._x, y+1, gWire._z);
						}
					}
					if (cnt)
						break;
				}
				if (cnt)
					break;
			}
			assert(
					cnt
							|| abs(gWire._y - pWire._y)
									< layout._trackHeight[gWire._z]);
		} else
			assert(0);
	} else {
		for (int z = min(gWire._z, pWire._z); z < max(gWire._z, pWire._z);
				z++) {
			assert(
					abs(gWire._y - pWire._y) <= 2 * extValue[z]
							|| gWire._realPinId != -1
							|| pWire._realPinId != -1);
			assert(
					abs(gWire._x - pWire._x) <= 2 * extValue[z]
							|| gWire._realPinId != -1
							|| pWire._realPinId != -1);
			// fill it and its 8 neighbors
			double x_start, x_mid, x_end, x_step, y_start, y_mid, y_end, y_step;
			if (_dirLayers[z] == V) {
				x_mid = round(
						(pWire._x - layout._trackOffsetX[z])
								/ layout._trackWidth[z]) * layout._trackWidth[z]
						+ layout._trackOffsetX[z];
				x_step = layout._trackWidth[z];
				x_start = max(layout._trackOffsetX[z], x_mid - x_step);
				x_end = min(
						layout._trackOffsetX[z]
								+ layout._numTracksX[z] * layout._trackWidth[z],
						x_mid + x_step);
				y_mid = round(
						(pWire._y - layout._trackOffsetY[z + 1])
								/ layout._trackHeight[z + 1])
						* layout._trackHeight[z + 1]
						+ layout._trackOffsetY[z + 1];
				y_step = layout._trackHeight[z + 1];
				y_start = max(layout._trackOffsetY[z + 1], y_mid - y_step);
				y_end = min(
						layout._trackOffsetY[z + 1]
								+ layout._numTracksY[z + 1]
										* layout._trackHeight[z + 1],
						y_mid + y_step);
			} else { // dirLayer == H
				x_mid = round(
						(pWire._x - layout._trackOffsetX[z + 1])
								/ layout._trackWidth[z + 1])
						* layout._trackWidth[z + 1]
						+ layout._trackOffsetX[z + 1];
				x_step = layout._trackWidth[z + 1];
				x_start = max(layout._trackOffsetX[z + 1], x_mid - x_step);
				x_end = min(
						layout._trackOffsetX[z + 1]
								+ layout._numTracksX[z + 1]
										* layout._trackWidth[z + 1],
						x_mid + x_step);
				y_mid = round(
						(pWire._y - layout._trackOffsetY[z])
								/ layout._trackHeight[z])
						* layout._trackHeight[z] + layout._trackOffsetY[z];
				y_step = layout._trackHeight[z];
				y_start = max(layout._trackOffsetY[z], y_mid - y_step);
				y_end = min(
						layout._trackOffsetY[z]
								+ layout._numTracksY[z]
										* layout._trackHeight[z],
						y_mid + y_step);
			}
			// for (int x = x_start; x <= x_end; x += x_step) {
			// for (int y = y_start; y <= y_end; y += y_step) {
			int x = x_mid, y = y_mid;
			size_t viaId = findVia(x, y, z);
			if (!(gnet.findCleanVia(viaId))) {
				gnet.setCleanVia(viaId, _vias[viaId]);
			}
			ripUpVia(viaId);
			gnet.removeVia(viaId);
			gnet._occupiedVias.erase(viaId);
			_dirty = true;
			//	printf("rm (%d,%d,%d)-(%d,%d,%d)\n", gWire._x, gWire._y, z, gWire._x, gWire._y, z+1);
			//}
			//}
#if 0
		// remove 8 edges located like below on upper and lower metal layers
		//            _| |_
		//            _via_    (To ensure metal spacing caused by the via)
		//             | |
		for (int zz = z; zz <= z + 1; zz++) {
			int x_cell = round(
					(pWire._x - layout._trackOffsetX[zz])
					/ layout._trackWidth[zz]);
			int y_cell = round(
					(pWire._y - layout._trackOffsetY[zz])
					/ layout._trackHeight[zz]);
			if (x_cell > 0 && y_cell > 0) {
				// lower-left corner, vertical
				int x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz]
				- layout._trackWidth[zz] / 2;
				int y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				- layout._trackHeight[zz];
				size_t edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
				// lower-left corner, horizontal
				x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz] - layout._trackWidth[zz];
				y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				- layout._trackHeight[zz] / 2;
				edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
			}
			if (x_cell < static_cast<int>(layout._numTracksX[zz]) - 1
					&& y_cell > 0) {
				// lower-right corner, vertical
				int x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz]
				+ layout._trackWidth[zz] / 2;
				int y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				- layout._trackHeight[zz];
				size_t edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
				// lower-right corner, horizontal
				x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz] + layout._trackWidth[zz];
				y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				- layout._trackHeight[zz] / 2;
				edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
			}
			if (x_cell < static_cast<int>(layout._numTracksX[zz]) - 1
					&& y_cell
					< static_cast<int>(layout._numTracksY[zz])
					- 1) {
				// upper-right corner, vertical
				int x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz]
				+ layout._trackWidth[zz] / 2;
				int y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				+ layout._trackHeight[zz];
				size_t edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
				// lower-right corner, horizontal
				x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz] + layout._trackWidth[zz];
				y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				+ layout._trackHeight[zz] / 2;
				edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
			}
			if (x_cell > 0
					&& y_cell
					< static_cast<int>(layout._numTracksY[zz])
					- 1) {
				// upper-left corner, vertical
				int x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz]
				- layout._trackWidth[zz] / 2;
				int y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				+ layout._trackHeight[zz];
				size_t edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
				// upper-left corner, horizontal
				x = x_cell * layout._trackWidth[zz]
				+ layout._trackOffsetX[zz] - layout._trackWidth[zz];
				y = y_cell * layout._trackHeight[zz]
				+ layout._trackOffsetY[zz]
				+ layout._trackHeight[zz] / 2;
				edgeId = findEdge(x, y, zz);
				reduceEdgeDemand(edgeId);
				gnet._occupiedEdges.erase(edgeId);
			}
		} // for zz
#endif
		} // for z
	}
// reduce its parent's children count
	int curr = gnet._gWires[i]._pWireId;
	if (curr != -1)
		_numChildren[curr]--;

// if it has only one child node A, and A is not a pin or v-pin,
// then remove node A and continue to check A's child(ren)
	checkChildren(gnet, _numChildren, _validWires, wlReduced, splitLayer,
			wlReducedAboveSL, i, layout, endsAtvpLower, endsAtvpUpper,
			vpGridLower, vpGridUpper, vp);

// for (size_t i = 1; i < gnet._gWires.size(); i++)
//	if (gnet._gWires[i]._pWireId != -1
//			&& !_validWires[gnet._gWires[i]._pWireId])
//		gnet._gWires[i]._pWireId = -1;
// (2/2) Remove antenna wires beyond the (two) connected components of pins.
	while (true) {
		bool hasRemovedWires = false;
		for (size_t i = 0; i < gnet._gWires.size(); i++) {
			int curr = i;
			// no child nodes
			while (curr != -1 && gnet._gWires[curr]._pWireId != -1
					&& _numChildren[curr] == 0
					&& gnet._gWires[curr]._realPinId == -1
					&& !(min(gnet._gWires[curr]._z,
							gnet._gWires[gnet._gWires[curr]._pWireId]._z)
							<= splitLayer
							&& max(gnet._gWires[curr]._z,
									gnet._gWires[gnet._gWires[curr]._pWireId]._z)
									> splitLayer) && _validWires[curr]) {
				_validWires[curr] = false;
				hasRemovedWires = true;
				GlobalWire &gWire = gnet._gWires[curr];
				GlobalWire &pWire = gnet._gWires[gWire._pWireId];
				wlReduced +=
						Gcell(gWire._x, gWire._y, gWire._z).viaWeightedDist(
								Gcell(pWire._x, pWire._y, pWire._z));
				//cout << "Remove " << gWire._x << "," << gWire._y << ","
				//		<< gWire._z << "-" << pWire._x << "," << pWire._y << ","
				//		<< pWire._z << endl;
				if (gWire._z > splitLayer && pWire._z > splitLayer) {
					wlReducedAboveSL +=
							Gcell(gWire._x, gWire._y, gWire._z).viaWeightedDist(
									Gcell(pWire._x, pWire._y, pWire._z));
				}
				//printf("Removing gWire %d(%d,%d,%d)-%d(%d,%d,%d)\n", curr,
				//		gWire._x, gWire._y, gWire._z, gWire._pWireId,
				//		pWire._x, pWire._y, pWire._z);
				//cout << flush;
				if (pWire._z == gWire._z) {
					if (pWire._x != gWire._x) {
						assert(gWire._y == pWire._y);
						int &z = gWire._z;
						int cnt = 0;
						for (int margin = 0; margin < MAX_MARGIN_OFFGRID;
								margin += 5) {
							for (int y = pWire._y - margin;
									y <= pWire._y + margin;
									y += (margin == 0 ? 1 : 2 * margin)) {
								for (int x = min(gWire._x, pWire._x);
										x <= max(gWire._x, pWire._x); x += 5) {
									int edgeId = findEdge(x, y, z);
									if (edgeId != -1) {
										if (!(gnet.findCleanEdge(edgeId))) {
											gnet.setCleanEdge(edgeId,
													_edges[edgeId]);
										}
										cnt++;
										ripUpEdge(edgeId);
										gnet.removeEdge(edgeId);
										gnet._occupiedEdges.erase(edgeId);
										_dirty = true;
										//     			printf("rma (%d,%d,%d)-(%d,%d,%d)\n", x, gWire._y, gWire._z, x+1, gWire._y, gWire._z);
									}
								}
								if (cnt)
									break;
							}
							if (cnt)
								break;
						}
						assert(
								cnt
										|| abs(gWire._x - pWire._x)
												< layout._trackWidth[gWire._z]);
					} else if (pWire._y != gWire._y) {
						assert(
								abs(gWire._x - pWire._x)
										<= 2 * extValue[pWire._z]);
						int &z = pWire._z;
						int cnt = 0;
						for (int margin = 0; margin < MAX_MARGIN_OFFGRID;
								margin += 5) {
							for (int x = pWire._x - margin;
									x <= pWire._x + margin;
									x += (margin == 0 ? 1 : 2 * margin)) {
								for (int y = min(gWire._y, pWire._y);
										y <= max(gWire._y, pWire._y); y += 5) {
									int edgeId = findEdge(x, y, z);
									if (edgeId != -1) {
										if (!(gnet.findCleanEdge(edgeId))) {
											gnet.setCleanEdge(edgeId,
													_edges[edgeId]);
										}
										cnt++;
										ripUpEdge(edgeId);
										gnet.removeEdge(edgeId);
										gnet._occupiedEdges.erase(edgeId);
										_dirty = true;
										//       			printf("rma (%d,%d,%d)-(%d,%d,%d)\n", gWire._x, y, gWire._z, gWire._x, y+1, gWire._z);
									}
								}
								if (cnt)
									break;
							}
							if (cnt)
								break;
						}
						assert(
								cnt
										|| abs(gWire._y - pWire._y)
												< layout._trackHeight[gWire._z]);
					} else
						assert(0);
				} else {
					//if (gWire._y != pWire._y)
					//	gnet.printGnet();
					for (int z = min(gWire._z, pWire._z);
							z < max(gWire._z, pWire._z); z++) {
						assert(
								abs(gWire._y - pWire._y)
										<= extValue[z] + extValue[z + 1]);
						assert(
								abs(gWire._x - pWire._x)
										<= extValue[z] + extValue[z + 1]);
						// fill it and its 8 neighbors
						double x_start, x_mid, x_end, x_step, y_start, y_mid,
								y_end, y_step;
						if (_dirLayers[z] == V) {
							x_mid = round(
									(pWire._x - layout._trackOffsetX[z])
											/ layout._trackWidth[z])
									* layout._trackWidth[z]
									+ layout._trackOffsetX[z];
							x_step = layout._trackWidth[z];
							x_start = max(layout._trackOffsetX[z],
									x_mid - x_step);
							x_end = min(
									layout._trackOffsetX[z]
											+ layout._numTracksX[z]
													* layout._trackWidth[z],
									x_mid + x_step);
							y_mid = round(
									(pWire._y - layout._trackOffsetY[z + 1])
											/ layout._trackHeight[z + 1])
									* layout._trackHeight[z + 1]
									+ layout._trackOffsetY[z + 1];
							y_step = layout._trackHeight[z + 1];
							y_start = max(layout._trackOffsetY[z + 1],
									y_mid - y_step);
							y_end =
									min(
											layout._trackOffsetY[z + 1]
													+ layout._numTracksY[z + 1]
															* layout._trackHeight[z
																	+ 1],
											y_mid + y_step);
						} else { // dirLayer == H
							x_mid = round(
									(pWire._x - layout._trackOffsetX[z + 1])
											/ layout._trackWidth[z + 1])
									* layout._trackWidth[z + 1]
									+ layout._trackOffsetX[z + 1];
							x_step = layout._trackWidth[z + 1];
							x_start = max(layout._trackOffsetX[z + 1],
									x_mid - x_step);
							x_end = min(
									layout._trackOffsetX[z + 1]
											+ layout._numTracksX[z + 1]
													* layout._trackWidth[z + 1],
									x_mid + x_step);
							y_mid = round(
									(pWire._y - layout._trackOffsetY[z])
											/ layout._trackHeight[z])
									* layout._trackHeight[z]
									+ layout._trackOffsetY[z];
							y_step = layout._trackHeight[z];
							y_start = max(layout._trackOffsetY[z],
									y_mid - y_step);
							y_end = min(
									layout._trackOffsetY[z]
											+ layout._numTracksY[z]
													* layout._trackHeight[z],
									y_mid + y_step);
						}
						//for (int x = x_start; x <= x_end; x += x_step) {
						//for (int y = y_start; y <= y_end; y += y_step) {
						int x = x_mid, y = y_mid;
						size_t viaId = findVia(x, y, z);
						if (!(gnet.findCleanVia(viaId))) {
							gnet.setCleanVia(viaId, _vias[viaId]);
						}
						ripUpVia(viaId);
						gnet.removeVia(viaId);
						gnet._occupiedVias.erase(viaId);
						_dirty = true;
						//         		printf("rma (%d,%d,%d)-(%d,%d,%d)\n", gWire._x, gWire._y, z, gWire._x, gWire._y, z+1);
						//}
						//}
#if 0
					// remove 8 edges located like below on upper and lower metal layers
					//            _| |_
					//            _via_    (To ensure metal spacing caused by the via)
					//             | |
					for (int zz = z; zz <= z + 1; zz++) {
						int x_cell = round(
								(pWire._x - layout._trackOffsetX[zz])
								/ layout._trackWidth[zz]);
						int y_cell = round(
								(pWire._y - layout._trackOffsetY[zz])
								/ layout._trackHeight[zz]);
						if (x_cell > 0 && y_cell > 0) {
							// lower-left corner, vertical
							int x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							- layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							- layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
							// lower-left corner, horizontal
							x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							- layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							- layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
						}
						if (x_cell
								< static_cast<int>(layout._numTracksX[zz])
								- 1 && y_cell > 0) {
							// lower-right corner, vertical
							int x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							+ layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							- layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
							// lower-right corner, horizontal
							x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							+ layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							- layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
						}
						if (x_cell
								< static_cast<int>(layout._numTracksX[zz])
								- 1
								&& y_cell
								< static_cast<int>(layout._numTracksY[zz])
								- 1) {
							// upper-right corner, vertical
							int x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							+ layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							+ layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
							// lower-right corner, horizontal
							x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							+ layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							+ layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
						}
						if (x_cell > 0
								&& y_cell
								< static_cast<int>(layout._numTracksY[zz])
								- 1) {
							// upper-left corner, vertical
							int x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							- layout._trackWidth[zz] / 2;
							int y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							+ layout._trackHeight[zz];
							size_t edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
							// upper-left corner, horizontal
							x = x_cell * layout._trackWidth[zz]
							+ layout._trackOffsetX[zz]
							- layout._trackWidth[zz];
							y = y_cell * layout._trackHeight[zz]
							+ layout._trackOffsetY[zz]
							+ layout._trackHeight[zz] / 2;
							edgeId = findEdge(x, y, zz);
							reduceEdgeDemand(edgeId);
							gnet._occupiedEdges.erase(edgeId);
						}
					} // for zz
#endif
					}
				}
				curr = gnet._gWires[curr]._pWireId;
				if (curr != -1)
					_numChildren[curr]--;
			}
			if (curr != -1 && gnet._gWires[curr]._pWireId != -1
					&& _numChildren[curr] == 0
					&& gnet._gWires[curr]._realPinId == -1
					&& min(gnet._gWires[curr]._z,
							gnet._gWires[gnet._gWires[curr]._pWireId]._z)
							<= splitLayer
					&& max(gnet._gWires[curr]._z,
							gnet._gWires[gnet._gWires[curr]._pWireId]._z)
							> splitLayer
					&& !(gnet._gWires[curr]._x == vp.xCoord
							&& gnet._gWires[curr]._y == vp.yCoord)) {
				// ends at another v-pin
				if (gnet._gWires[i]._z > splitLayer) {
					endsAtvpUpper = true;
					vpGridUpper =
							Gcell(gnet._gWires[curr]._x, gnet._gWires[curr]._y,
									max(gnet._gWires[curr]._z,
											gnet._gWires[gnet._gWires[curr]._pWireId]._z));
				} else {
					endsAtvpLower = true;
					vpGridLower =
							Gcell(gnet._gWires[curr]._x, gnet._gWires[curr]._y,
									min(gnet._gWires[curr]._z,
											gnet._gWires[gnet._gWires[curr]._pWireId]._z));
				}
			}
		}
		for (size_t i = 0; i < gnet._gWires.size(); i++)
			if (!_validWires[i]) {
				auto res = checkChildren(gnet, _numChildren, _validWires,
						wlReduced, splitLayer, wlReducedAboveSL, i, layout,
						endsAtvpLower, endsAtvpUpper, vpGridLower, vpGridUpper,
						vp);
				hasRemovedWires = hasRemovedWires || res;
			}
// cout << "hasRemovedWires: " << hasRemovedWires << endl;
		if (!hasRemovedWires)
			break;
	}
// For a removed gWire with two or more children (like this: ->.<-)
// add it to GnetVertices and its corresponding connections to GnetGrids.
	for (int i = 0; i < static_cast<int>(gnet._gWires.size()); i++) {
		if (!_validWires[i] && _numChildren[i] >= 2) {
			auto &gWire2 = gnet._gWires[i];
			remainingGnetVertices.insert(
					Gcell(gWire2._x, gWire2._y, gWire2._z));
			remainingGnetGrids.insert(Gcell(gWire2._x, gWire2._y, gWire2._z));
			if (c[i] == c[vpAboveWireID]) {
				if (gWire._z > splitLayer) {
					otherVpinVerticesAboveSL.insert(
							Gcell(gWire2._x, gWire2._y, gWire2._z));
					otherVpinGridsAboveSL.insert(
							Gcell(gWire2._x, gWire2._y, gWire2._z));
				} /*else {
				 otherVpinVerticesOnBelowSL.insert(
				 Gcell(gWire2._x, gWire2._y, gWire2._z));
				 otherVpinGridsOnBelowSL.insert(
				 Gcell(gWire2._x, gWire2._y, gWire2._z));
				 } */
			}
			if (vp.gnetGrids.count(Gcell(gWire2._x, gWire2._y, gWire2._z)) == 0
					&& gWire2._z <= splitLayer) {
				otherVpinGridsOnBelowSL.insert(
						Gcell(gWire2._x, gWire2._y, gWire2._z));
			}
			// fill grids with wires connecting gWire to each children
			for (int j = 0; j < static_cast<int>(gnet._gWires.size()); j++) {
				if (_validWires[j] && gnet._gWires[j]._pWireId == i) { // it is a valid child of gWire
					int x1 = gWire2._x;
					int y1 = gWire2._y;
					int z1 = gWire2._z;
					int x2 = gnet._gWires[j]._x;
					int y2 = gnet._gWires[j]._y;
					int z2 = gnet._gWires[j]._z;
					if (x1 != x2 || y1 != y2) {
						assert(z1 == z2);
						for (int x = min(x1, x2); x <= max(x1, x2); x += 10) {
							for (int y = min(y1, y2); y <= max(y1, y2); y +=
									10) {
								if (_grids[z1].count(Gcell(x, y, z1))) {
									if (x1 != x2 && y1 != y2) {
										assert(
												gWire2._realPinId != -1
														|| gnet._gWires[j]._realPinId
																!= -1);
										int realPinId =
												gWire2._realPinId != -1 ?
														gWire2._realPinId :
														gnet._gWires[j]._realPinId;
										if (!(gnet._gPins[realPinId]._z == z1
												&& gnet._gPins[realPinId]._shape.coversPoint(
														x, y, extValue[z1])))
											continue;
									}
									remainingGnetGrids.insert(Gcell(x, y, z1));
									if (c[j] == c[vpAboveWireID]) {
										if (z1 > splitLayer) {
											otherVpinGridsAboveSL.insert(
													Gcell(x, y, z1));
										} /*else {
										 otherVpinGridsOnBelowSL.insert(
										 Gcell(x, y, z1));
										 }*/
									}
									if (vp.gnetGrids.count(Gcell(x, y, z1)) == 0
											&& z1 <= splitLayer) {
										otherVpinGridsOnBelowSL.insert(
												Gcell(x, y, z1));
									}
								}
							}
						}
					} else if (z1 != z2) {
						assert(x1 == x2);
						assert(y1 == y2);
						for (int z = min(z1, z2); z <= max(z1, z2); ++z) {
							remainingGnetGrids.insert(Gcell(x1, y1, z));
							if (c[j] == c[vpAboveWireID]) {
								if (z > splitLayer) {
									otherVpinGridsAboveSL.insert(
											Gcell(x1, y1, z));
								} /*else {
								 otherVpinGridsOnBelowSL.insert(
								 Gcell(x1, y1, z));
								 } */
							}
							if (vp.gnetGrids.count(Gcell(x1, y1, z)) == 0
									&& z <= splitLayer) {
								otherVpinGridsOnBelowSL.insert(
										Gcell(x1, y1, z));
							}
						}
					}
				}
			}
		}
	}

	for (size_t i = 0; i < gnet._gWires.size(); i++) {
		if (_validWires[i]) {
			const GlobalWire &gWire = gnet._gWires[i];
			remainingGnetVertices.insert(Gcell(gWire._x, gWire._y, gWire._z));
			remainingGnetGrids.insert(Gcell(gWire._x, gWire._y, gWire._z));
			if (c[i] == c[vpAboveWireID]) {
				if (gWire._z > splitLayer) {
					otherVpinVerticesAboveSL.insert(
							Gcell(gWire._x, gWire._y, gWire._z));
					otherVpinGridsAboveSL.insert(
							Gcell(gWire._x, gWire._y, gWire._z));
				} /*else {
				 otherVpinVerticesOnBelowSL.insert(
				 Gcell(gWire._x, gWire._y, gWire._z));
				 otherVpinGridsOnBelowSL.insert(
				 Gcell(gWire._x, gWire._y, gWire._z));
				 }*/
			}
			if (vp.gnetGrids.count(Gcell(gWire._x, gWire._y, gWire._z)) == 0
					&& gWire._z <= splitLayer) {
				otherVpinGridsOnBelowSL.insert(
						Gcell(gWire._x, gWire._y, gWire._z));
			}

			if (gWire._pWireId >= 0) {
				const GlobalWire &pWire = gnet._gWires[gWire._pWireId];
				remainingGnetVertices.insert(
						Gcell(pWire._x, pWire._y, pWire._z));
				if (c[gWire._pWireId] == c[vpAboveWireID]) {
					if (pWire._z > splitLayer) {
						otherVpinGridsAboveSL.insert(
								Gcell(pWire._x, pWire._y, pWire._z));
					} /*else {
					 otherVpinGridsOnBelowSL.insert(
					 Gcell(pWire._x, pWire._y, pWire._z));
					 } */
				}
				if (vp.gnetGrids.count(Gcell(pWire._x, pWire._y, pWire._z)) == 0
						&& pWire._z <= splitLayer) {
					otherVpinGridsOnBelowSL.insert(
							Gcell(pWire._x, pWire._y, pWire._z));
				}

				int x1 = gWire._x;
				int y1 = gWire._y;
				int z1 = gWire._z;
				int x2 = pWire._x;
				int y2 = pWire._y;
				int z2 = pWire._z;
				if (abs(x1 - x2) > extValue[z1] + extValue[z2]
						|| abs(y1 - y2) > extValue[z1] + extValue[z2]) {
					assert(
							gWire._realPinId > -1 || pWire._realPinId > -1
									|| z1 == z2);
					for (int x = min(x1, x2); x <= max(x1, x2); x += 10) {
						for (int y = min(y1, y2); y <= max(y1, y2); y += 10) {
							if (abs(x1 - x2) > extValue[z1] + extValue[z2]
									&& abs(y1 - y2)
											> extValue[z1] + extValue[z2]) {
								assert(
										gWire._realPinId != -1
												|| pWire._realPinId != -1);
								int realPinId =
										gWire._realPinId != -1 ?
												gWire._realPinId :
												pWire._realPinId;
								if (!(gnet._gPins[realPinId]._z == z1
										&& gnet._gPins[realPinId]._shape.coversPoint(
												x, y, extValue[z1])))
									continue;
							}
							if (_grids[z1].count(Gcell(x, y, z1))) {
								remainingGnetGrids.insert(Gcell(x, y, z1));
								if (c[i] == c[vpAboveWireID]) {
									if (z1 > splitLayer) {
										otherVpinGridsAboveSL.insert(
												Gcell(x, y, z1));
									} /*else {
									 otherVpinGridsOnBelowSL.insert(
									 Gcell(x, y, z1));
									 }*/
								}
								if (vp.gnetGrids.count(Gcell(x, y, z1)) == 0
										&& z1 <= splitLayer) {
									otherVpinGridsOnBelowSL.insert(
											Gcell(x, y, z1));
								}
							}
						}
					}
				} else if (z1 != z2) {
					for (int z = min(z1, z2); z <= max(z1, z2); ++z) {
						assert(abs(x1 - x2) <= extValue[z] + extValue[z + 1]);
						assert(abs(y1 - y2) <= extValue[z] + extValue[z + 1]);
						int cnt = 0;
						for (int x = min(x1, x2); x <= max(x1, x2); x += 10) {
							for (int y = min(y1, y2); y <= max(y1, y2); y +=
									10) {
								if (_grids[z].count(Gcell(x, y, z))) {
									remainingGnetGrids.insert(Gcell(x, y, z));
									if (c[i] == c[vpAboveWireID]) {
										if (z > splitLayer) {
											otherVpinGridsAboveSL.insert(
													Gcell(x, y, z));
										} /*else {
										 otherVpinGridsOnBelowSL.insert(
										 Gcell(x1, y1, z));
										 }*/
									}
									if (vp.gnetGrids.count(Gcell(x, y, z)) == 0
											&& z <= splitLayer) {
										otherVpinGridsOnBelowSL.insert(
												Gcell(x, y, z));
									}
									cnt++;
								}
								if (cnt)
									break;
							}
							if (cnt)
								break;
						}
					}
				}
			}
		}
	}
	for (size_t i = 0; i < gnet._gWires.size(); i++)
		if (gnet._gWires[i]._pWireId != -1
				&& !_validWires[gnet._gWires[i]._pWireId])
			gnet._gWires[i]._pWireId = -1;
// Do the real removing
	int currOffset = 0;
	curr = 0;
	_cleanWires.resize(gnet._gWires.size());
	for (size_t i = 0; i < gnet._gWires.size(); i++) {
		if (_validWires[i]) {
			_cleanWires[curr] = gnet._gWires[i];
			curr++;
		} else {
			currOffset++;
		}
		_offset[i] = currOffset;
	}
	_cleanWires.resize(curr);
	for (size_t i = 0; i < _cleanWires.size(); i++) {
		if (_cleanWires[i]._pWireId != -1) {
			_cleanWires[i]._pWireId -= _offset[_cleanWires[i]._pWireId];
		}
	}
	assert(curr + currOffset == static_cast<int>(gnet._gWires.size()));
	_cleanWires.resize(curr);
	gnet._gWires.swap(_cleanWires);
// cout << "Removed " << currOffset << " gWires." << endl;

// For each vp, take intersection of (gwire grids) AND
// (grids in connected component of the vp)
	int reducedWL = 0;
	std::unordered_set<Gcell, HashGcell3d> resultGnetGrids;
	std::unordered_set<Gcell, HashGcell3d> resultGnetVertices;
	for (auto &gv : vp.gnetGrids)
		if (remainingGnetGrids.find(gv) != remainingGnetGrids.end())
			resultGnetGrids.insert(gv);
	vp.gnetGrids.swap(resultGnetGrids);
	for (auto &gv : vp.gnetVertices)
		if (remainingGnetVertices.find(gv) != remainingGnetVertices.end())
			resultGnetVertices.insert(gv);
	vp.gnetVertices.swap(resultGnetVertices);
	int originalwlToL1 = vp.wlToL1;
	vp.wlToL1 -= wlReduced - wlReducedAboveSL
			- Gcell(0, 0, vp.zCoord).viaWeights[vp.zCoord] /* vpin */;
	reducedWL += originalwlToL1 - vp.wlToL1;

#if 0
// For each vp, take intersection of (gwire grids) AND
// NOT (grids in connected component of the vp) AND (grids above splitLayer)
for (auto &gv : remainingGnetGrids)
if (vp.gnetGrids.find(gv) == vp.gnetGrids.end()) {
	if (gv._z > splitLayer)
	otherVpinGridsAboveSL.insert(gv);
	else
	otherVpinGridsOnBelowSL.insert(gv);
} else if (gv._z > splitLayer) {
	vp.gnetGrids.erase(gv);
	otherVpinGridsAboveSL.insert(gv);
}
for (auto &gv : remainingGnetVertices)
if (vp.gnetVertices.find(gv) == vp.gnetVertices.end()) {
	if (gv._z > splitLayer)
	otherVpinVerticesAboveSL.insert(gv);
	else
	otherVpinVerticesOnBelowSL.insert(gv);
} else if (gv._z > splitLayer) {
	vp.gnetVertices.erase(gv);
	otherVpinVerticesAboveSL.insert(gv);
}
#endif
// FIXME:
// assert(reducedWL == wlReduced - wlReducedAboveSL);
	/*for (auto i : _validWires)
	 cout << i << ",";
	 cout << endl;
	 for (auto i : _numChildren)
	 cout << i << ",";
	 cout << endl;
	 gnet.printGnet();*/

#if 0
for (auto vp : vpins) {
	cout << "Vpin(" << vp.xCoord << "," << vp.yCoord << "): "
	<< vp.gnetVertices.size() << " gnet vertices" << endl;
	for (auto gnetVertex : vp.gnetVertices) {
		cout << "-> (" << gnetVertex._x << ","
		<< gnetVertex._y << ","
		<< gnetVertex._z << ")" << endl;
	}
}
#endif
	/*if (endsAtvpLower) {
	 otherVpinVerticesOnBelowSL = { vpGridLower };
	 otherVpinGridsOnBelowSL = { vpGridLower };
	 }*/
	if (endsAtvpUpper) {
		otherVpinVerticesAboveSL = { vpGridUpper };
		otherVpinGridsAboveSL = { vpGridUpper };
	}
	return make_tuple(otherVpinGridsAboveSL, otherVpinGridsOnBelowSL,
			otherVpinVerticesAboveSL, otherVpinVerticesOnBelowSL, removedEdges,
			removedVias, wlReduced, wlReducedAboveSL);
}

void RoutingDB_DR::restoreNet(const int gnetID, const Gnet gn,
		const std::map<size_t, Edge> &cleanEdges,
		const std::map<size_t, Via> &cleanVias, bool restoreCongestion) {
	Gnet &gnet = _gnets[gnetID];
	if (restoreCongestion) {
		for (auto item : cleanEdges) {
			_edges[item.first] = item.second;
		}
		for (auto item : cleanVias) {
			_vias[item.first] = item.second;
		}
	}
	gnet = gn;
	_dirty = false;
}
#if 0
void RoutingDB::restoreNet(const string netName)
{
for( Gnet gnet : _gnets ) {
	if( !gnet.isGlobal() )
	continue;
	if( gnet._name != netName )
	continue;
	for( size_t i = 1; i < gnet._gWires.size(); i++ ) {
		const GlobalWire & gWire = gnet._gWires[i];
		const GlobalWire & pWire = gnet._gWires[ gWire._pWireId ];
		if( pWire._z == gWire._z ) {
			if( _dirLayers[ pWire._z ] == H ) {
				assert( gWire._y == pWire._y );
				for( int x = min( gWire._x, pWire._x ); x < max( gWire._x, pWire._x ); x++ ) {
					addEdgeDemand( findEdge( x, gWire._y, gWire._z ) );
				}
			} else if( _dirLayers[ pWire._z ] == V ) {
				assert( gWire._x == pWire._x );
				for( int y = min( gWire._y, pWire._y ); y < max( gWire._y, pWire._y ); y++ ) {
					addEdgeDemand( findEdge( gWire._x, y, gWire._z ) );
				}
			} else assert( 0 );
		} else {
			assert(gWire._y == pWire._y);
			assert(gWire._x == pWire._x);
			for (int z = min(gWire._z, pWire._z); z < max(gWire._z, pWire._z); z++) {
				addViaDemand( findVia(gWire._x, gWire._y, z) );
			}
		}
	}
}
}
#endif
// Create routing blockage and store info in layout data structure
void RoutingDB_DR::createBlockage(const int xStart, const int yStart,
		const int xEnd, const int yEnd, const int z, LayoutDR &layout) {

	int availableEdgeCap = 0;

	if (_dirLayers[z] == H) {
		for (int y = yStart; y < yEnd; y++) {
			for (int x = xStart; x < xEnd; x++) {
				const int eid = findEdge(x, y, z);
				availableEdgeCap += max(0,
						getEdgeCap(eid) - getEdgeBlk(eid) - getEdgeDemand(eid));
			}
		}
	} else if (_dirLayers[z] == V) {
		for (int x = xStart; x < xEnd; x++) {
			for (int y = yStart; y < yEnd; y++) {
				const int eid = findEdge(x, y, z);
				availableEdgeCap += max(0,
						getEdgeCap(eid) - getEdgeBlk(eid) - getEdgeDemand(eid));
			}
		}
	}

	int size = 0;
	while (size * size * _capOnLayer[z] < availableEdgeCap) {
		size++;
	}

	char objName[256];
	sprintf(objName, "o%d", layout._numRegularNodes);
	layout._numRegularNodes++;

	layout.addNodeBlock(string(objName), size, size, xStart, yStart, z);
}

int RoutingDB_DR::getTotalWL() const {
	return getWlByNet(-1);
}

int RoutingDB_DR::getWlByNet(int gnetID) const { // gnetID < 0 for all gnets
	int wl = 0;
	int via = 0;
	int weightedVia = 0;
	int penalty = 0;
	size_t start = (gnetID < 0 ? 0 : gnetID);
	size_t end = (gnetID < 0 ? _gnets.size() : gnetID + 1);
	for (size_t k = start; k < end; k++) {
		auto &gnet = _gnets[k];
		if (!gnet.isGlobal())
			continue;
		for (size_t i = 0; i < gnet._gWires.size(); i++) {
			const GlobalWire &gWire = gnet._gWires[i];
			if (gWire._pWireId == -1)
				continue;
			const GlobalWire &pWire = gnet._gWires[gWire._pWireId];
			if (pWire._z == gWire._z) {
				//	if (gWire._realPinId == -1) {
				if (gWire._x != pWire._x) {
					assert(
							gWire._realPinId > -1 || pWire._realPinId > -1
									|| abs(gWire._y - pWire._y)
											<= 2 * extValue[pWire._z]);
				} else if (gWire._y != pWire._y) {
					assert(
							gWire._realPinId > -1 || pWire._realPinId > -1
									|| abs(gWire._x - pWire._x)
											<= 2 * extValue[pWire._z]);
				} else
					assert(0);
				//	}
				wl += max(gWire._x, pWire._x) - min(gWire._x, pWire._x);
				wl += max(gWire._y, pWire._y) - min(gWire._y, pWire._y);
				if (_dirLayers[gWire._z] == H)
					penalty += max(gWire._y, pWire._y)
							- min(gWire._y, pWire._y);
				else
					penalty += max(gWire._x, pWire._x)
							- min(gWire._x, pWire._x);
			} else {
				if (!(gWire._realPinId > -1 || pWire._realPinId > -1
						|| abs(gWire._y - pWire._y)
								<= extValue[gWire._z] + extValue[pWire._z])) {
					gnet.printGnet();
					cerr << gWire._y << " " << pWire._y << endl;
					exit(-1);
				}
				if (!(gWire._realPinId > -1 || pWire._realPinId > -1
						|| abs(gWire._x - pWire._x)
								<= extValue[gWire._z] + extValue[pWire._z])) {
					gnet.printGnet();
					cerr << gWire._x << " " << pWire._x << endl;
					exit(-1);
				}
				wl += max(gWire._x, pWire._x) - min(gWire._x, pWire._x);
				wl += max(gWire._y, pWire._y) - min(gWire._y, pWire._y);
				via += max(gWire._z, pWire._z) - min(gWire._z, pWire._z);
				for (int z = min(gWire._z, pWire._z);
						z < max(gWire._z, pWire._z); z++) {
					weightedVia += Gcell(0, 0, z).viaWeights[z];
				}
			}
		}
	}
	printf("<I> Wirelength: %d, Via: %d, Total: %d, Penalty: %d, Total*: %d\n",
			wl, via, wl + weightedVia, penalty, wl + weightedVia + penalty);
	cout << flush;
	return wl + weightedVia;
}

int RoutingDB_DR::getTotalWLByDemand() const {
	int wl = 0;
	int via = 0;
	for (size_t e = 0; e < _edges.size(); e++) {
		wl += _edges[e].demand() / _trackDemands[getEdgeLayer(e)];
	}
	for (size_t v = 0; v < _vias.size(); v++) {
		via += _vias[v].demand() / _trackDemands[getViaLayer(v) + 1]
				/ _trackDemands[getViaLayer(v) + 1];
	}
	printf("<I> From demand: Wirelength: %d, Via: %d, Total: %d\n", wl, via,
			wl + via);
	cout << flush;
	return wl + via;
}
