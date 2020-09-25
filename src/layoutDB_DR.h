#ifndef _LAYOUT_DR_H_
#define _LAYOUT_DR_H_

#include "lefdef/LefDefParser.h"
#include "baseDB.h"
#include "nodes.h"
#include "nets.h"
#include "vias.h"
#include <stdio.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

class RoutingDB_DR;

class LayoutDR {
	friend class Parser;
	friend class RoutingDB_DR;
	friend class no_nearby_via;
public:
	LayoutDR(size_t numLayers) :
			_numTracksX(0), _numTracksY(0), _numLayers(numLayers), _numRegularNodes(
					0), _numPTerms(0), _blkPorosity(0), _ldp(
					MyLefDefParser::get_instance()) {
	}

	void readLEFDEF(const string filename_lef, const string filename_def);

	void initDesignName(const string &designName) {
		_designName = designName;
	}
	void initDesignName(const char *designName) {
		_designName = string(designName);
	}
	string name() const {
		return _designName;
	}
	//  .nodes
	unsigned findNodeId(const string &nodeName) const;

	//  .nets
	void addNetPin(const def::ConnectionPtr &pinptr, Net &net);

	//  .route
	void setGrid(size_t ub_z);
	void setVCaps();
	void setHCaps();
	void setMinWireWidth();
	void setMinWireSpacing();
	void setViaSpacing();
	void setBlkPorosity();

	//  modifier
	void addNodeBlock(const string &objName, const double width,
			const double height, const double llx, const double lly,
			const int z);

	def::Def& getDEF() {
		return _ldp.def_;
	}

protected:
	MyLefDefParser &_ldp;
	string _designName;
	vector<unsigned> _numTracksX;
	vector<unsigned> _numTracksY;
	unsigned _numLayers;
	//  .nodes file and .pl file
	vector<Node> _nodes;
	unordered_map<string, size_t> _nodeUmap;
	unsigned _numRegularNodes;
	unsigned _numPTerms;
	int _defDbu;

	//  .nets file
	vector<Net> _nets;

	//  .route file
	vector<double> _vCaps;
	vector<double> _hCaps;
	vector<double> _mww;   //  minimum wire width
	vector<double> _mws;   //  minimum wire spacing
	vector<double> _mvs;   //  minimum via spacing
	vector<double> _trackOffsetX, _trackOffsetY;
	vector<double> _trackWidth, _trackHeight;
	size_t _ub_z; // top layer with metal wires

	double _blkPorosity;

	//  blockage
	vector<unsigned> _blkId;

	//  global nets
	Gcell getGcell(const Point3D &pt) const {
		return Gcell(static_cast<int>(round(pt._lx)),
				static_cast<int>(round(pt._ly)),
				static_cast<int>(round(pt._ux)),
				static_cast<int>(round(pt._uy)), pt._z, pt._shape);
		//return Gcell(static_cast<int>((pt._x - _trackOffsetX[pt._z]) / _trackWidth[pt._z]),
		//		static_cast<int>((pt._y - _trackOffsetY[pt._z]) / _trackHeight[pt._z]), pt._z);
	}
	Gcell getGcell(const Point3D &pt, const vector<GnetPin> &gPins) const {
		for (auto gPin : gPins) {
			// absorb to the centroid of gPin bbox if inside its bbox
			if (pt._x >= gPin._lx && pt._x <= gPin._ux && pt._y >= gPin._ly
					&& pt._y <= gPin._uy && pt._z == gPin._z) {
				return gPin;
			}
		}
		return Gcell(static_cast<int>(round(pt._x)),
				static_cast<int>(round(pt._y)), pt._z);
		//return Gcell(static_cast<int>((pt._x - _trackOffsetX[pt._z]) / _trackWidth[pt._z]),
		//		static_cast<int>((pt._y - _trackOffsetY[pt._z]) / _trackHeight[pt._z]), pt._z);
	}
};

#endif
