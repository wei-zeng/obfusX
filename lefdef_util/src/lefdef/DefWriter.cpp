/**
 * @file    DefWriter.cpp
 * @author  Jinwook Jung (jinwookjung@kaist.ac.kr)
 * @date    2018-10-24 17:16:17
 *
 * Created on Wed Oct 24 17:16:17 2018.
 */

#include "DefWriter.h"
#include "def/defwWriter.hpp"

/**
 * Default ctor.
 */
MyDefWriter::MyDefWriter() {
	//
}

/**
 * Returns the singleton object of the detailed router.
 */
MyDefWriter& MyDefWriter::get_instance() {
	static MyDefWriter ldp;
	return ldp;
}

#define CHECK_STATUS(status) \
  if (status) {              \
     defwPrintError(status); \
     return;         \
  }

void MyDefWriter::write_rows(def::Def *def) {
	auto &rows = def->get_rows();

	for (auto &r : rows) {
		auto status = defwRow(r->name_.c_str(), r->macro_.c_str(), r->x_, r->y_,
				r->orient_, r->num_x_, r->num_y_, r->step_x_, r->step_y_);
		CHECK_STATUS(status);
	}
}

void MyDefWriter::write_tracks(def::Def *def) {
	auto &tracks = def->get_tracks();

	for (auto &t : tracks) {
		auto layers = (const char**) malloc(sizeof(char*) * 1);
		layers[0] = strdup(t->layer_.c_str());

		string dir_str = t->direction_ == TrackDir::x ? "X" : "Y";
		auto status = defwTracks(dir_str.c_str(), t->location_, t->num_tracks_,
				t->step_, 1, layers);
		CHECK_STATUS(status);
		free((char*) layers[0]);
		free((char*) layers);
	}

	auto status = defwNewLine();
	CHECK_STATUS(status);
}

void MyDefWriter::write_gcell_grids(def::Def *def) {
	auto &gcell_grids = def->get_gcell_grids();

	for (auto &g : gcell_grids) {
		string dir_str = g->direction_ == TrackDir::x ? "X" : "Y";
		auto status = defwGcellGrid(dir_str.c_str(), g->location_, g->num_,
				g->step_);
		CHECK_STATUS(status);
	}

	auto status = defwNewLine();
	CHECK_STATUS(status);
}

void MyDefWriter::write_components(def::Def *def) {
	auto &component_umap = def->get_component_umap();

	auto status = defwStartComponents(component_umap.size());
	CHECK_STATUS(status);

	for (auto it : component_umap) {
		auto c = it.second;

		string status_str = "UNPLACED";
		if (c->is_fixed_) {
			status_str = "FIXED";
		} else if (c->is_placed_) {
			status_str = "PLACED";
		}

		if (c->orient_str_ == "") {
			c->orient_str_ = "N";
		}

		status = defwComponentStr(c->name_.c_str(), c->ref_name_.c_str(), 0,
		NULL, NULL, NULL, NULL,
				NULL,    // Optionals
				0, NULL, NULL, NULL, NULL, status_str.c_str(), c->x_, c->y_,
				c->orient_str_.c_str(), 0, NULL, 0, 0, 0, 0);
		CHECK_STATUS(status);
	}

	status = defwEndComponents();
	CHECK_STATUS(status);
}

void MyDefWriter::write_pins(def::Def *def) {
	auto &pin_umap = def->get_pin_umap();

	auto status = defwStartPins(pin_umap.size());
	CHECK_STATUS(status);

	for (auto it : pin_umap) {
		auto p = it.second;

		string direction_str = "INOUT";
		if (p->dir_ == PinDir::input) {
			direction_str = "INPUT";
		} else if (p->dir_ == PinDir::output) {
			direction_str = "OUTPUT";
		}

		auto status = defwPinStr(p->name_.c_str(), p->net_name_.c_str(), 0,
				direction_str.c_str(), "SIGNAL", "FIXED", p->x_, p->y_,
				p->orient_str_.c_str(), p->layer_.c_str(), p->lx_, p->ly_,
				p->ux_, p->uy_);
		CHECK_STATUS(status);
	}

	status = defwEndPins();
	CHECK_STATUS(status);
}

void MyDefWriter::write_special_nets(def::Def *def) {
	auto &special_net_umap = def->get_special_net_umap();

	auto status = defwStartSpecialNets(special_net_umap.size());
	CHECK_STATUS(status);

	for (auto it : special_net_umap) {
		auto n = it.second;
		status = defwSpecialNet(n->name_.c_str());
		CHECK_STATUS(status);

		for (auto con : n->connections_) {
			if (con->component_ != nullptr) {
				status = defwSpecialNetConnection(
						con->component_->name_.c_str(),
						con->lef_pin_->name_.c_str(), 0);
			} else if (con->pin_ != nullptr) {
				status = defwSpecialNetConnection("PIN",
						con->pin_->name_.c_str(), 0);
			} else {
				status = defwSpecialNetConnection("PIN", con->name_.c_str(), 0);
			}
			CHECK_STATUS(status);
		}

		if (n->wires_.empty() == false) {
//            status = defwSpecialNetPathStart("ROUTED");
//            status = defwSpecialNetPathEnd();
		}

		status = defwSpecialNetEndOneNet();
		CHECK_STATUS(status);
	}

	status = defwEndSpecialNets();
	CHECK_STATUS(status);
}

void writePointPair(int x1, int y1, int x2, int y2, const char **coorX,
		const char **coorY, const char **coorValue) {
	stringstream ss;
	ss.str(std::string());
	ss.clear();
	ss << x1;
	string ss_str = ss.str();
	coorX[0] = strdup(ss_str.c_str());
	ss.str(std::string());
	ss.clear();
	ss << y1;
	ss_str = ss.str();
	coorY[0] = strdup(ss_str.c_str());
	coorValue[0] = NULL;

	if (x1 != x2) {
		ss.str(std::string());
		ss.clear();
		ss << x2;
		ss_str = ss.str();
		coorX[1] = strdup(ss_str.c_str());
	} else {
		coorX[1] = strdup("*");
	}
	if (y1 != y2) {
		ss.str(std::string());
		ss.clear();
		ss << y2;
		ss_str = ss.str();
		coorY[1] = strdup(ss_str.c_str());
	} else {
		coorY[1] = strdup("*");
	}
	coorValue[1] = NULL;

	int status = defwNetPathPoint(2, coorX, coorY, coorValue);
	CHECK_STATUS(status);

	free((char*) coorX[0]);
	free((char*) coorY[0]);
	free((char*) coorX[1]);
	free((char*) coorY[1]);
}

void MyDefWriter::write_nets(def::Def *def, const RoutingDB_DR &routingDB) {
	auto &net_umap = def->get_net_umap();

	auto status = defwStartNets(net_umap.size());
	CHECK_STATUS(status);
	assert(net_umap.size() == routingDB._gnets.size());
	for (auto gnet : routingDB._gnets) {
		auto n = net_umap.at(gnet.getName());

		status = defwNet(n->name_.c_str());
		CHECK_STATUS(status);

		for (auto con : n->connections_) {
			if (con->component_ != nullptr) {
				status = defwNetConnection(con->component_->name_.c_str(),
						con->lef_pin_->name_.c_str(), 0);
			} else {
				status = defwNetConnection("PIN", con->pin_->name_.c_str(), 0);
			}
			CHECK_STATUS(status);
		}

		const char **coorX = (const char**) malloc(sizeof(char*) * 3);
		const char **coorY = (const char**) malloc(sizeof(char*) * 3);
		const char **coorValue = (const char**) malloc(sizeof(char*) * 3);
		bool firstPath = true;
		for (size_t i = 0; i < gnet._gWires.size(); ++i) {
			auto &gWire = gnet._gWires[i];
			if (gWire._pWireId == -1)
				continue;
			auto &pWire = gnet._gWires[gWire._pWireId];

			if (gWire._z == pWire._z) {
				if ((gWire._x == pWire._x) != (gWire._y == pWire._y)) {
					status = defwNetPathStart(firstPath ? "ROUTED" : "NEW");
					CHECK_STATUS(status);
					stringstream ss;
					ss << "metal" << pWire._z + 1;
					string ss_str = ss.str();
					status = defwNetPathLayer(ss_str.c_str(), 0, NULL);
					CHECK_STATUS(status);
					writePointPair(pWire._x, pWire._y, gWire._x, gWire._y,
							coorX, coorY, coorValue);
				} else {
					assert(gWire._realPinId != -1 || pWire._realPinId != -1);
					if (gWire._realPinId == pWire._realPinId)
						continue;
					if (gWire._realPinId != -1) {
						int cnt = 0;
						int x = pWire._x;
						for (int y = min(gWire._y, pWire._y);
								y <= max(gWire._y, pWire._y); y += 10) {
							if (gnet._gPins[gWire._realPinId]._shape.coversPoint(
									x, y, extValue[gWire._z])) {
								//px, py -- px, gy
								if (pWire._y != gWire._y) {
									{
										status = defwNetPathStart(
												firstPath ? "ROUTED" : "NEW");
										CHECK_STATUS(status);
										stringstream ss;
										ss << "metal" << pWire._z + 1;
										string ss_str = ss.str();
										status = defwNetPathLayer(
												ss_str.c_str(), 0,
												NULL);
										CHECK_STATUS(status);
										writePointPair(x, pWire._y, x, gWire._y,
												coorX, coorY, coorValue);
									}
									//gx, gy -- px, gy
									{
										status = defwNetPathStart(
												firstPath ? "ROUTED" : "NEW");
										CHECK_STATUS(status);
										stringstream ss;
										ss << "metal" << pWire._z + 1;
										string ss_str = ss.str();
										status = defwNetPathLayer(
												ss_str.c_str(), 0,
												NULL);
										CHECK_STATUS(status);
										writePointPair(gWire._x, gWire._y, x,
												gWire._y, coorX, coorY,
												coorValue);
									}
								}
								cnt++;
								break;
							}
						}
						if (cnt == 0) {
							int y = pWire._y;
							for (int x = min(gWire._x, pWire._x);
									x <= max(gWire._x, pWire._x); x += 10) {
								if (gnet._gPins[gWire._realPinId]._shape.coversPoint(
										x, y, extValue[gWire._z])) {
									//px, py -- gx, py
									if (pWire._x != gWire._x) {
										{
											status = defwNetPathStart(
													firstPath ?
															"ROUTED" : "NEW");
											CHECK_STATUS(status);
											stringstream ss;
											ss << "metal" << pWire._z + 1;
											string ss_str = ss.str();
											status = defwNetPathLayer(
													ss_str.c_str(), 0, NULL);
											CHECK_STATUS(status);
											writePointPair(pWire._x, y,
													gWire._x, y, coorX, coorY,
													coorValue);
										}
										//gx, gy -- gx, py
										{
											status = defwNetPathStart(
													firstPath ?
															"ROUTED" : "NEW");
											CHECK_STATUS(status);
											stringstream ss;
											ss << "metal" << pWire._z + 1;
											string ss_str = ss.str();
											status = defwNetPathLayer(
													ss_str.c_str(), 0,
													NULL);
											CHECK_STATUS(status);
											writePointPair(gWire._x, gWire._y,
													gWire._x, y, coorX, coorY,
													coorValue);
										}
									}
									cnt++;
									break;
								}
							}
						}
						if (cnt == 0) {
							if (abs(gWire._x - def->get_die_lx())
									<= extValue[gWire._z]
									|| abs(gWire._x - def->get_die_ux())
											<= extValue[gWire._z]) {
								// (px,py)-(px,gy)-(gx,gy)
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(pWire._x, pWire._y, pWire._x,
											gWire._y, coorX, coorY, coorValue);
								}
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(pWire._x, gWire._y, gWire._x,
											gWire._y, coorX, coorY, coorValue);
								}
							} else if (abs(gWire._y - def->get_die_ly())
									<= extValue[gWire._z]
									|| abs(gWire._y - def->get_die_uy())
											<= extValue[gWire._z]) {
								// (px,py)-(gx,py)-(gx,gy)
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(pWire._x, pWire._y, gWire._x,
											pWire._y, coorX, coorY, coorValue);
								}
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(gWire._x, pWire._y, gWire._x,
											gWire._y, coorX, coorY, coorValue);
								}
							} else {
								// TODO
								gWire.printGlobalWire();
								pWire.printGlobalWire();
								cerr << "TODO1" << endl;
							}
						}
					} else { // pWire._realPinId != -1
						int cnt = 0;
						int x = gWire._x;
						for (int y = min(gWire._y, pWire._y);
								y <= max(gWire._y, pWire._y); y += 10) {
							if (gnet._gPins[pWire._realPinId]._shape.coversPoint(
									x, y, extValue[pWire._z])) {
								//gx, gy -- gx, py
								if (gWire._y != y) {
									{
										status = defwNetPathStart(
												firstPath ? "ROUTED" : "NEW");
										CHECK_STATUS(status);
										stringstream ss;
										ss << "metal" << pWire._z + 1;
										string ss_str = ss.str();
										status = defwNetPathLayer(
												ss_str.c_str(), 0,
												NULL);
										CHECK_STATUS(status);
										writePointPair(x, gWire._y, x, pWire._y,
												coorX, coorY, coorValue);
									}
									//px, py -- gx, py
									{
										status = defwNetPathStart(
												firstPath ? "ROUTED" : "NEW");
										CHECK_STATUS(status);
										stringstream ss;
										ss << "metal" << pWire._z + 1;
										string ss_str = ss.str();
										status = defwNetPathLayer(
												ss_str.c_str(), 0,
												NULL);
										CHECK_STATUS(status);
										writePointPair(pWire._x, pWire._y, x,
												pWire._y, coorX, coorY,
												coorValue);
									}
								}
								cnt++;
								break;
							}
						}
						if (cnt == 0) {
							int y = gWire._y;
							for (int x = min(gWire._x, pWire._x);
									x <= max(gWire._x, pWire._x); x += 10) {
								if (gnet._gPins[gWire._realPinId]._shape.coversPoint(
										x, y, extValue[pWire._z])) {
									//gx, gy -- px, gy
									if (gWire._x != x) {
										{
											status = defwNetPathStart(
													firstPath ?
															"ROUTED" : "NEW");
											CHECK_STATUS(status);
											stringstream ss;
											ss << "metal" << pWire._z + 1;
											string ss_str = ss.str();
											status = defwNetPathLayer(
													ss_str.c_str(), 0, NULL);
											CHECK_STATUS(status);
											writePointPair(gWire._x, y,
													pWire._x, y, coorX, coorY,
													coorValue);
										}
										//px, py -- px, gy
										{
											status = defwNetPathStart(
													firstPath ?
															"ROUTED" : "NEW");
											CHECK_STATUS(status);
											stringstream ss;
											ss << "metal" << pWire._z + 1;
											string ss_str = ss.str();
											status = defwNetPathLayer(
													ss_str.c_str(), 0,
													NULL);
											CHECK_STATUS(status);
											writePointPair(pWire._x, pWire._y,
													pWire._x, y, coorX, coorY,
													coorValue);
										}
									}
									cnt++;
									break;
								}
							}
						}
						if (cnt == 0) {
							if (abs(pWire._y - def->get_die_ly())
									<= extValue[pWire._z]
									|| abs(pWire._y - def->get_die_uy())
											<= extValue[pWire._z]) {
								// (px,py)-(gx,py)-(gx,gy)
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(pWire._x, pWire._y, gWire._x,
											pWire._y, coorX, coorY, coorValue);
								}
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(gWire._x, pWire._y, gWire._x,
											gWire._y, coorX, coorY, coorValue);
								}
							} else if (abs(pWire._y - def->get_die_ly())
									<= extValue[pWire._z]
									|| abs(pWire._y - def->get_die_uy())
											<= extValue[pWire._z]) {
								// (px,py)-(px,gy)-(gx,gy)
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(pWire._x, pWire._y, pWire._x,
											gWire._y, coorX, coorY, coorValue);
								}
								{
									status = defwNetPathStart(
											firstPath ? "ROUTED" : "NEW");
									CHECK_STATUS(status);
									stringstream ss;
									ss << "metal" << pWire._z + 1;
									string ss_str = ss.str();
									status = defwNetPathLayer(ss_str.c_str(), 0,
									NULL);
									CHECK_STATUS(status);
									writePointPair(pWire._x, gWire._y, gWire._x,
											gWire._y, coorX, coorY, coorValue);
								}
							} else {
								// TODO
								gWire.printGlobalWire();
								pWire.printGlobalWire();
								cerr << "TODO2" << endl;
							}
						}
					}
				}
			} else { // via
				int lower_z = min(gWire._z, pWire._z);
				int higher_z = max(gWire._z, pWire._z);
				assert(higher_z - lower_z == 1);
				status = defwNetPathStart(firstPath ? "ROUTED" : "NEW");
				CHECK_STATUS(status);
				stringstream ss;
				ss << "metal" << pWire._z + 1;
				string ss_str = ss.str();
				ss.str(std::string());
				status = defwNetPathLayer(ss_str.c_str(), 0, NULL);
				CHECK_STATUS(status);

				ss.clear();
				ss << gWire._x;
				ss_str = ss.str();
				coorX[0] = strdup(ss_str.c_str());
				ss.str(std::string());
				ss.clear();
				ss << gWire._y;
				ss_str = ss.str();
				coorY[0] = strdup(ss_str.c_str());
				coorValue[0] = NULL;

				status = defwNetPathPoint(1, coorX, coorY, coorValue);
				CHECK_STATUS(status);

				free((char*) coorX[0]);
				free((char*) coorY[0]);

				ss.str(std::string());
				ss.clear();
				ss << "via" << lower_z + 1 << "_0"; // NOTE: no via variations.
				ss_str = ss.str();
				status = defwNetPathVia(ss_str.c_str());
				CHECK_STATUS(status);
			}
			firstPath = false;
		}
		status = defwNetPathEnd();
		CHECK_STATUS(status);
		status = defwNetEndOneNet();
		CHECK_STATUS(status);
	}

	status = defwEndNets();
	CHECK_STATUS(status);
}

void MyDefWriter::write_def(def::Def &def, const RoutingDB_DR &routingDB,
		string filename) {
	def_ = &def;

	FILE *fout = fopen(filename.c_str(), "w");
	if (fout == nullptr) {
		fprintf(stderr, "ERROR: could not open output file\n");
		return;
	}

	int status;    // return code, if none 0 means error
	status = defwInitCbk(fout);
	CHECK_STATUS(status);
	status = defwVersion(5, 7);
	CHECK_STATUS(status);
	status = defwDividerChar("/");
	CHECK_STATUS(status);
	status = defwBusBitChars("[]");
	CHECK_STATUS(status);
	status = defwDesignName(def_->get_design_name().c_str());
	CHECK_STATUS(status);
	status = defwUnits(def_->get_dbu());
	CHECK_STATUS(status);

	status = defwNewLine();
	CHECK_STATUS(status);

// history
	status = defwHistory("Obfuscated with guidance of SHAP values.");
	CHECK_STATUS(status);
	status = defwNewLine();
	CHECK_STATUS(status);

// Die area
	status = defwDieArea(def_->get_die_lx(), def_->get_die_ly(),
			def_->get_die_ux(), def_->get_die_uy());
	CHECK_STATUS(status);

	status = defwNewLine();
	CHECK_STATUS(status);

// Rows
	write_rows(def_);

// Tracks
	write_tracks(def_);

// GCell grid
	write_gcell_grids(def_);

// Components
	write_components(def_);

// Pins
	write_pins(def_);

// Special Nets
	write_special_nets(def_);

// Nets
	write_nets(def_, routingDB);

	status = defwEnd();
	CHECK_STATUS(status);

	auto lineNumber = defwCurrentLineNumber();
	if (lineNumber == 0) {
		fprintf(stderr, "ERROR: nothing has been read.\n");
	}
	fclose(fout);
}

