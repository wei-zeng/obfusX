#include "layoutDB_DR.h"
#include <string>

void LayoutDR::readLEFDEF(const string filename_lef,
		const string filename_def) {
	_ldp.read_lef(filename_lef);
	_ldp.read_def(filename_def);
	_defDbu = _ldp.def_.get_dbu();
}

//  .nets

void LayoutDR::addNetPin(const def::ConnectionPtr &connPtr, Net &net) {
	string nodeName;
	PinDir pinType;
	double locLX, locUX, locLY, locUY;
	int locZ;
	Polygon shape;
	if (connPtr->lef_pin_ == nullptr) { // PIO
		nodeName = connPtr->name_;
		pinType = connPtr->pin_->dir_;
		locLX = locUX = connPtr->pin_->x_;
		locLY = locUY = connPtr->pin_->y_;
		switch (connPtr->pin_->orient_) {
		case 0: // N
			locLX += connPtr->pin_->lx_;
			locUX += connPtr->pin_->ux_;
			locLY += connPtr->pin_->ly_;
			locUY += connPtr->pin_->uy_;
			break;
		case 1: // W
			locLX -= connPtr->pin_->uy_;
			locUX -= connPtr->pin_->ly_;
			locLY += connPtr->pin_->lx_;
			locUY += connPtr->pin_->ux_;
			break;
		case 2: // S
			locLX -= connPtr->pin_->ux_;
			locUX -= connPtr->pin_->lx_;
			locLY -= connPtr->pin_->uy_;
			locUY -= connPtr->pin_->ly_;
			break;
		case 3: // E
			locLX += connPtr->pin_->ly_;
			locUX += connPtr->pin_->uy_;
			locLY -= connPtr->pin_->ux_;
			locUY -= connPtr->pin_->lx_;
			break;
		case 4: // FN
			locLX -= connPtr->pin_->ux_;
			locUX -= connPtr->pin_->lx_;
			locLY += connPtr->pin_->ly_;
			locUY += connPtr->pin_->uy_;
			break;
		case 5: // FW
			locLX += connPtr->pin_->ly_;
			locUX += connPtr->pin_->uy_;
			locLY += connPtr->pin_->lx_;
			locUY += connPtr->pin_->ux_;
			break;
		case 6: // FS
			locLX += connPtr->pin_->lx_;
			locUX += connPtr->pin_->ux_;
			locLY -= connPtr->pin_->uy_;
			locUY -= connPtr->pin_->ly_;
			break;
		case 7: // FE
			locLX -= connPtr->pin_->uy_;
			locUX -= connPtr->pin_->ly_;
			locLY -= connPtr->pin_->ux_;
			locUY -= connPtr->pin_->lx_;
			break;
		}
		locZ = stoi(connPtr->pin_->layer_.substr(5)) - 1;
		shape = Polygon(locLX, locLY, locUX, locUY);
		net._netPinTypes.push_back(
				pinType == PinDir::input ? PI :
				pinType == PinDir::output ? PO : N);
	} else { // cell pins
		nodeName = connPtr->component_->name_;
		pinType = connPtr->lef_pin_->dir_;
		locLX = locUX = connPtr->component_->x_;
		locLY = locUY = connPtr->component_->y_;
		shape = Polygon(connPtr->lef_pin_->ports_[0]->polygons_[0]);
		shape.translate(connPtr->component_->x_, connPtr->component_->y_);
		// shape.rotate_reflect(connPtr->component_->orient_);
		switch (connPtr->component_->orient_) {
		case 0: // N
			locLX += connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			locUX += connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			locLY += connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			locUY += connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			break;
		case 1: // W
			locLX -= connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			locUX -= connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			locLY += connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			locUY += connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			break;
		case 2: // S
			locLX -= connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			locUX -= connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			locLY -= connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			locUY -= connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			break;
		case 3: // E
			locLX += connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			locUX += connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			locLY -= connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			locUY -= connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			break;
		case 4: // FN
			locLX -= connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			locUX -= connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			locLY += connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			locUY += connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			break;
		case 5: // FW
			locLX += connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			locUX += connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			locLY += connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			locUY += connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			break;
		case 6: // FS
			locLX += connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			locUX += connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			locLY -= connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			locUY -= connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			break;
		case 7: // FE
			locLX -= connPtr->lef_pin_->bbox_.uy_ * _defDbu;
			locUX -= connPtr->lef_pin_->bbox_.ly_ * _defDbu;
			locLY -= connPtr->lef_pin_->bbox_.ux_ * _defDbu;
			locUY -= connPtr->lef_pin_->bbox_.lx_ * _defDbu;
			break;
		}
		locZ = stoi(connPtr->lef_pin_->ports_[0]->layer_name_.substr(5)) - 1;
		net._netPinTypes.push_back(
				pinType == PinDir::input ? CI :
				pinType == PinDir::output ? CO : N);
	}
	// const Node &node = _nodes[_nodeUmap.at(nodeName)];

	net._netPins.emplace_back(locLX, locLY, locUX, locUY, locZ, shape);
	net._netPinNodeIDs.push_back(_nodeUmap.at(nodeName));
}

//  .route 
void LayoutDR::setGrid(size_t ub_z) {
	_vCaps.resize(_numLayers);
	_hCaps.resize(_numLayers);
	_mww.resize(_numLayers);
	_mws.resize(_numLayers);
	_mvs.resize(_numLayers);
	_numTracksX.resize(_numLayers);
	_numTracksY.resize(_numLayers);
	_trackOffsetX.resize(_numLayers);
	_trackOffsetY.resize(_numLayers);
	_trackHeight.resize(_numLayers);
	_trackWidth.resize(_numLayers);
	_ub_z = ub_z;
	for (auto track : _ldp.def_.get_tracks()) {
		size_t z = stoi(track->layer_.substr(5)) - 1;
		if (z < _numLayers) {
			if (track->direction_ == TrackDir::x) {
				_numTracksX[z] = track->num_tracks_;
				_trackOffsetX[z] = track->location_;
				_trackWidth[z] = track->step_;
			} else if (track->direction_ == TrackDir::y) {
				_numTracksY[z] = track->num_tracks_;
				_trackOffsetY[z] = track->location_;
				_trackHeight[z] = track->step_;
			}
		}
	}
	for (size_t z = ub_z; z < _numLayers; z++) {
		_numTracksX[z] = _numTracksX[ub_z - 1];
		_trackOffsetX[z] = _trackOffsetX[ub_z - 1];
		_trackWidth[z] = _trackWidth[ub_z - 1];
		_numTracksY[z] = _numTracksY[ub_z - 1];
		_trackOffsetY[z] = _trackOffsetY[ub_z - 1];
		_trackHeight[z] = _trackHeight[ub_z - 1];
	}
}

void LayoutDR::setVCaps() {
	for (unsigned i = 0; i < _numLayers; i++)
		_vCaps[i] = i % 2;
}

void LayoutDR::setHCaps() {
	for (unsigned i = 0; i < _numLayers; i++)
		_hCaps[i] = 1 - i % 2;
}

void LayoutDR::setMinWireWidth() {
	for (unsigned i = 0; i < _numLayers; i++)
		_mww[i] = 0; // TODO
}

void LayoutDR::setMinWireSpacing() {
	for (unsigned i = 0; i < _numLayers; i++)
		_mws[i] = 0; // TODO
}

void LayoutDR::setViaSpacing() {
	for (unsigned i = 0; i < _numLayers; i++)
		_mvs[i] = 0; // TODO
}

void LayoutDR::setBlkPorosity() {
	_blkPorosity = 0;
}

void LayoutDR::addNodeBlock(const string &objName, const double width,
		const double height, const double llx, const double lly, const int z) {
	Node nodeBlock(objName, width, height, llx, lly, z, 0);
	_blkId.push_back(_nodes.size());
	_nodes.push_back(nodeBlock);
}
