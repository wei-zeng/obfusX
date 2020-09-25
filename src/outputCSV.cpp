/*
 * outputCSV.cpp
 *
 *  Created on: Aug 25, 2019
 *      Author: wzeng
 */

#include "grDB.h"
#include "grDB_DR.h"
#include "a_star.h" // for undirected Graph from boost
#include <iterator>
#include <fstream>
#include <cassert>
#include <set>
#include <boost/graph/connected_components.hpp>

vector<Vpin> RoutingDB::getVpins(const Layout &layout, int splitLayer,
		bool twoCutNetsOnly, bool includeAllPIOs) const { // splitLayer: 0-based
	vector<Vpin> vpins;
	using namespace boost;
	int totalVpin = 0;
	for (size_t gnetID = 0; gnetID < _gnets.size(); ++gnetID) {
		auto &gnet = _gnets[gnetID];
		Graph routingTree;
		for (auto &gWire : gnet._gWires) {
			Vertex v = add_vertex(routingTree);
			routingTree[v].x = gWire._x;
			routingTree[v].y = gWire._y;
			routingTree[v].z = gWire._z;
			routingTree[v].realPinId = gWire._realPinId;
			routingTree[v].isPin = (gWire._z == 0 && gWire._realPinId >= 0);
			routingTree[v].isRoot = (gWire._pWireId == -1
					&& gWire._realPinId >= 0);
		}
		for (size_t i = 0; i < gnet._gWires.size(); i++) {
			auto &gWire = gnet._gWires[i];
			if (gWire._pWireId >= 0
					&& !(gWire._z > splitLayer
							&& gnet._gWires[gWire._pWireId]._z > splitLayer)) {
				add_edge(vertex(gWire._pWireId, routingTree),
						vertex(i, routingTree), routingTree); // only connect if lower than or on splitLayer
			}
		}
		int vpCount = 0;
		EdgeIter e, eend;
		for (tie(e, eend) = edges(routingTree); e != eend; ++e) { // for each vpin
			if (routingTree[e->m_source].z > splitLayer
					|| routingTree[e->m_target].z > splitLayer) {
				routingTree[*e].isVpin = true;
				++vpCount;
				++totalVpin;
			}
		}
		if (vpCount < 2 || (twoCutNetsOnly && vpCount > 2))
			continue;
		int vpCountBase = totalVpin - vpCount;

		vpCount = 0;
		for (tie(e, eend) = edges(routingTree); e != eend; ++e) { // for each vpin
			if (routingTree[*e].isVpin) {
				Vpin vp;
				vp.vPinID = vpCountBase + vpCount;
				vp.gnetID = gnetID;
				vp.xCoord = routingTree[e->m_source].x;
				vp.yCoord = routingTree[e->m_source].y;
				vp.zCoord = splitLayer;
				std::vector<int> c(num_vertices(routingTree));
				connected_components(routingTree, &c[0]);
				VertexIter v, vend;
				EdgeIter e2, e2end;
				for (tie(e2, e2end) = edges(routingTree); e2 != e2end; ++e2) {
					if (c[e2->m_source] == c[e->m_source]) {
						int x1 = routingTree[e2->m_source].x;
						int y1 = routingTree[e2->m_source].y;
						int z1 = routingTree[e2->m_source].z;
						int x2 = routingTree[e2->m_target].x;
						int y2 = routingTree[e2->m_target].y;
						int z2 = routingTree[e2->m_target].z;
						if (x1 != x2) {
							assert(y1 == y2);
							assert(z1 == z2);
							for (int x = min(x1, x2); x <= max(x1, x2); ++x) {
								vp.gnetGrids.insert(Gcell(x, y1, z1));
							}
						} else if (y1 != y2) {
							assert(x1 == x2);
							assert(z1 == z2);
							for (int y = min(y1, y2); y <= max(y1, y2); ++y) {
								vp.gnetGrids.insert(Gcell(x1, y, z1));
							}
						} else if (z1 != z2) {
							assert(x1 == x2);
							assert(y1 == y2);
							for (int z = min(z1, z2); z <= max(z1, z2); ++z) {
								vp.gnetGrids.insert(Gcell(x1, y1, z));
							}
						}
					}
				}
				for (tie(v, vend) = vertices(routingTree); v != vend; ++v) {
					if (c[*v] == c[e->m_source]) {
						vp.gnetVertices.insert(
								Gcell(routingTree[*v].x, routingTree[*v].y,
										routingTree[*v].z));
						vp.gnetGrids.insert(
								Gcell(routingTree[*v].x, routingTree[*v].y,
										routingTree[*v].z));
						if (routingTree[*v].isRoot)
							vp.hasOriginalRoot = true;
					}

					if (c[*v] == c[e->m_source]
							&& routingTree[*v].isPin) {
						Gcell gcell = Gcell(routingTree[*v].x,
								routingTree[*v].y, routingTree[*v].z);
						// find pin type
						// find pin node
						Net net = gnet._net;
						assert(net._netPins.size() == net._netPinTypes.size());
						assert(
								net._netPins.size() == net._netPinNodeIDs.size());
						for (size_t i = 0; i < net._netPins.size(); ++i) {
							if (layout.getGcell(net._netPins[i]) == gcell) {
								if (vp.pinNodeIDs.insert(net._netPinNodeIDs[i]).second) {
									vp.pins.push_back(net._netPins[i]);
									vp.pinTypes.push_back(net._netPinTypes[i]);
									auto &the_node =
											layout._nodes[net._netPinNodeIDs[i]];
									if (the_node._isTerm
											&& the_node._numInputs == 1
											&& the_node._numOutputs == 0) { // PI
										assert(vp.pinTypes.back() == CI);
										vp.pinTypes.back() = PI;
									} /*else if (the_node._isTerm
											&& the_node._numInputs == 0
											&& the_node._numOutputs == 1) { // PO
										assert(vp.pinTypes.back() == CO);
										vp.pinTypes.back() = PO;
									}*/
									vp.cellAreas.push_back(
											layout._nodes[net._netPinNodeIDs[i]]._height
													* layout._nodes[net._netPinNodeIDs[i]]._width);
								}
							}
						}
					}
				}
				EdgeIter ewl, ewlend;
				int wl = 0;
				for (tie(ewl, ewlend) = edges(routingTree); ewl != ewlend;
						++ewl) {
					if (c[ewl->m_source] == c[e->m_source]
							&& c[ewl->m_target] == c[e->m_source]) {
						wl += Gcell(routingTree[ewl->m_source].x,
								routingTree[ewl->m_source].y,
								routingTree[ewl->m_source].z)
								- Gcell(routingTree[ewl->m_target].x,
										routingTree[ewl->m_target].y,
										routingTree[ewl->m_target].z);
					}
				}
				if (vp.pins.size() > 0) {
					vpCount++;
					vp.wlToL1 = wl;
					vpins.push_back(vp);
				}
			}
		}
		if (vpCount < 2 || (twoCutNetsOnly && vpCount > 2)) {
			while (vpCount--) {
				vpins.pop_back();
			}
		}
		// Find matching Vpin
		for (int i = 1; i <= vpCount; ++i) {
			for (int j = 1; j <= vpCount; ++j) {
				if (vpins[vpins.size() - i].vPinID
						!= vpins[vpins.size() - j].vPinID) {
					vpins[vpins.size() - i].matchingVpinIdx = vpins.size() - j;
					break;
				}
			}
		}
	}

	cout << "# Vpins: " << vpins.size() << endl;
	return vpins;
}

vector<Vpin> RoutingDB::getVpins(const Layout &layout, const int gnetID,
		const int splitLayer, bool twoCutNetsOnly, bool includeAllPIOs) const { // splitLayer: 0-based
	vector<Vpin> vpins;
	using namespace boost;
	const Gnet &gnet = _gnets[gnetID];
	bool noVpins = true;
	for (auto &gWire : gnet._gWires) {
		if (gWire._z > splitLayer) {
			noVpins = false;
			break;
		}
	}
	if (noVpins)
		return vpins;

	Graph routingTree;
	int totalVpin = 0;
	int vpCount = 0;
//vector<Vertex> vs;
	for (auto &gWire : gnet._gWires) {
		Vertex v = add_vertex(routingTree);
		routingTree[v].x = gWire._x;
		routingTree[v].y = gWire._y;
		routingTree[v].z = gWire._z;
		routingTree[v].realPinId = gWire._realPinId;
		routingTree[v].isPin = (gWire._realPinId != -1 && gWire._z == 0);
		routingTree[v].isRoot = (gWire._pWireId == -1 && gWire._realPinId >= 0);
	}
	for (size_t i = 0; i < gnet._gWires.size(); i++) {
		auto &gWire = gnet._gWires[i];
		if (gWire._pWireId >= 0
				&& !(gWire._z > splitLayer
						&& gnet._gWires[gWire._pWireId]._z > splitLayer)) {
			auto r = add_edge(vertex(gWire._pWireId, routingTree),
					vertex(i, routingTree), routingTree); // only connect if lower than or on splitLayer
			assert(r.second);
			EdgeType e = r.first;
			if (routingTree[e.m_source].z > splitLayer
					|| routingTree[e.m_target].z > splitLayer) {
				routingTree[e].isVpin = true;
				++vpCount;
				++totalVpin;
			}
		}
	}

	if (vpCount < 2 || (twoCutNetsOnly && vpCount > 2))
		return vpins;

	int vpCountBase = totalVpin - vpCount;
	vpCount = 0;
	EdgeIter ei, eiend;
	for (tie(ei, eiend) = edges(routingTree); ei != eiend; ++ei) { // for each vpin
		if (routingTree[*ei].isVpin) {
			Vpin vp;
			vp.vPinID = vpCountBase + vpCount;
			vp.gnetID = gnetID;
			vp.xCoord = routingTree[ei->m_source].x;
			vp.yCoord = routingTree[ei->m_source].y;
			vp.zCoord = splitLayer;
			vp.hasOriginalRoot = false;
			std::vector<int> c(num_vertices(routingTree));
			connected_components(routingTree, &c[0]);
			VertexIter vi, viend;
			EdgeIter e2, e2end;
			for (tie(e2, e2end) = edges(routingTree); e2 != e2end; ++e2) {
				if (c[e2->m_source] == c[ei->m_source]) {
					int x1 = routingTree[e2->m_source].x;
					int y1 = routingTree[e2->m_source].y;
					int z1 = routingTree[e2->m_source].z;
					int x2 = routingTree[e2->m_target].x;
					int y2 = routingTree[e2->m_target].y;
					int z2 = routingTree[e2->m_target].z;
					if (x1 != x2) {
						assert(y1 == y2);
						assert(z1 == z2);
						for (int x = min(x1, x2); x <= max(x1, x2); ++x) {
							vp.gnetGrids.insert(Gcell(x, y1, z1));
						}
					} else if (y1 != y2) {
						assert(x1 == x2);
						assert(z1 == z2);
						for (int y = min(y1, y2); y <= max(y1, y2); ++y) {
							vp.gnetGrids.insert(Gcell(x1, y, z1));
						}
					} else if (z1 != z2) {
						assert(x1 == x2);
						assert(y1 == y2);
						for (int z = min(z1, z2); z <= max(z1, z2); ++z) {
							vp.gnetGrids.insert(Gcell(x1, y1, z));
						}
					}
				}
			}
			for (tie(vi, viend) = vertices(routingTree); vi != viend; ++vi) {
				if (c[*vi] == c[ei->m_source]) {
					vp.gnetVertices.insert(
							Gcell(routingTree[*vi].x, routingTree[*vi].y,
									routingTree[*vi].z));
					vp.gnetGrids.insert(
							Gcell(routingTree[*vi].x, routingTree[*vi].y,
									routingTree[*vi].z));
					if (routingTree[*vi].isRoot)
						vp.hasOriginalRoot = true;
				}
				if (c[*vi] == c[ei->m_source]
						&& routingTree[*vi].isPin) {
					Gcell gcell = Gcell(routingTree[*vi].x, routingTree[*vi].y,
							routingTree[*vi].z);
					// find pin type
					// find pin node
					Net net = gnet._net;
					assert(net._netPins.size() == net._netPinTypes.size());
					assert(net._netPins.size() == net._netPinNodeIDs.size());
					for (size_t i = 0; i < net._netPins.size(); ++i) {
						if (layout.getGcell(net._netPins[i]) == gcell) {
							if (vp.pinNodeIDs.insert(net._netPinNodeIDs[i]).second) {
								vp.pins.push_back(net._netPins[i]);
								vp.pinTypes.push_back(net._netPinTypes[i]);
								auto &the_node =
										layout._nodes[net._netPinNodeIDs[i]];
								if (the_node._isTerm && the_node._numInputs == 1
										&& the_node._numOutputs == 0) { // PI
									assert(vp.pinTypes.back() == CI);
									vp.pinTypes.back() = PI;
								} /*else if (the_node._isTerm
										&& the_node._numInputs == 0
										&& the_node._numOutputs == 1) { // PO
									assert(vp.pinTypes.back() == CO);
									vp.pinTypes.back() = PO;
								}*/
								vp.cellAreas.push_back(
										layout._nodes[net._netPinNodeIDs[i]]._height
												* layout._nodes[net._netPinNodeIDs[i]]._width);
							}
						}
					}
				}
			}
			EdgeIter ewl, ewlend;
			int wl = 0;
			for (tie(ewl, ewlend) = edges(routingTree); ewl != ewlend; ++ewl) {
				if (c[ewl->m_source] == c[ei->m_source]
						&& c[ewl->m_target] == c[ei->m_source]) {
					wl += Gcell(routingTree[ewl->m_source].x,
							routingTree[ewl->m_source].y,
							routingTree[ewl->m_source].z)
							- Gcell(routingTree[ewl->m_target].x,
									routingTree[ewl->m_target].y,
									routingTree[ewl->m_target].z);
				}
			}
			if (vp.pins.size() > 0) {
				vpCount++;
				vp.wlToL1 = wl;
				vpins.push_back(vp);
			}
		}
	}
	if (vpCount < 2 || (twoCutNetsOnly && vpCount > 2)) {
		while (vpCount--) {
			vpins.pop_back();
		}
	}
// Find matching Vpin
	for (int i = 1; i <= vpCount; ++i) {
		for (int j = 1; j <= vpCount; ++j) {
			if (vpins[vpins.size() - i].vPinID
					!= vpins[vpins.size() - j].vPinID) {
				vpins[vpins.size() - i].matchingVpinIdx = vpins.size() - j;
				break;
			}
		}
	}

// cout << "# Vpins: " << vpins.size() << endl;
	assert(
			vpins.size() != 2 || vpins[0].hasOriginalRoot != vpins[1].hasOriginalRoot);
	return vpins;
}

vector<Vpin> RoutingDB_DR::getVpins(const LayoutDR &layout, int splitLayer,
		bool twoCutNetsOnly, bool includeAllPIOs) const { // splitLayer: 0-based
	vector<Vpin> vpins;
	using namespace boost;
	int totalVpin = 0;
	int vpGroupBase = 0;
	int beolNets = 0;
	for (size_t gnetID = 0; gnetID < _gnets.size(); ++gnetID) {
		auto &gnet = _gnets[gnetID];
		Graph routingTree;
		for (auto &gWire : gnet._gWires) {
			Vertex v = add_vertex(routingTree);
			routingTree[v].x = gWire._x;
			routingTree[v].y = gWire._y;
			routingTree[v].z = gWire._z;
			routingTree[v].realPinId = gWire._realPinId;
			routingTree[v].isRoot = (gWire._pWireId == -1
					&& gWire._realPinId >= 0);
		}
		for (size_t i = 0; i < gnet._gWires.size(); i++) {
			auto &gWire = gnet._gWires[i];
			if (gWire._pWireId >= 0
					&& !(gWire._z > splitLayer
							&& gnet._gWires[gWire._pWireId]._z > splitLayer)) {
				add_edge(vertex(gWire._pWireId, routingTree),
						vertex(i, routingTree), routingTree); // only connect if lower than or on splitLayer
			}
		}
		int vpCount = 0;
		EdgeIter e, eend;
		for (tie(e, eend) = edges(routingTree); e != eend; ++e) { // for each vpin
			if (routingTree[e->m_source].z > splitLayer
					|| routingTree[e->m_target].z > splitLayer) {
				routingTree[*e].isVpin = true;
				++vpCount;
				++totalVpin;
			}
		}
		if (vpCount == 0 || (twoCutNetsOnly && vpCount != 2))
			continue;
		int vpCountBase = totalVpin - vpCount;
		vpCount = 0;
		for (tie(e, eend) = edges(routingTree); e != eend; ++e) { // for each vpin
			if (routingTree[*e].isVpin) {
				Vpin vp;
				vp.vPinID = vpCountBase + vpCount;
				vp.gnetID = gnetID;
				vp.xCoord = routingTree[e->m_source].x;
				vp.yCoord = routingTree[e->m_source].y;
				vp.zCoord = splitLayer;
				vp.hasOriginalRoot = false;
				std::vector<int> c(num_vertices(routingTree));
				connected_components(routingTree, &c[0]);
				VertexIter v, vend;
				EdgeIter e2, e2end; // find grids in the same connected component as vp
				for (tie(e2, e2end) = edges(routingTree); e2 != e2end; ++e2) {
					if (c[e2->m_source] == c[e->m_source]) {
						int x1 = routingTree[e2->m_source].x;
						int y1 = routingTree[e2->m_source].y;
						int z1 = routingTree[e2->m_source].z;
						int x2 = routingTree[e2->m_target].x;
						int y2 = routingTree[e2->m_target].y;
						int z2 = routingTree[e2->m_target].z;
						if (x1 != x2 || y1 != y2) {
							assert(z1 == z2);
							for (int x = min(x1, x2); x <= max(x1, x2); x +=
									5) {
								for (int y = min(y1, y2); y <= max(y1, y2); y +=
										5) {
									int edgeId = findEdge(x, y, z1);
									if (edgeId != -1) {
										vp.gnetGrids.insert(_ep1[edgeId]);
										vp.gnetGrids.insert(_ep2[edgeId]);
									}
								}
							}
						} else if (z1 != z2) {
							assert(x1 == x2);
							assert(y1 == y2);
							for (int z = min(z1, z2); z <= max(z1, z2); ++z) {
								vp.gnetGrids.insert(Gcell(x1, y1, z));
							}
						}
					}
				}
				for (tie(v, vend) = vertices(routingTree); v != vend; ++v) {
					if (c[*v] == c[e->m_source]) {
						vp.gnetVertices.insert(
								Gcell(routingTree[*v].x, routingTree[*v].y,
										routingTree[*v].z));
						vp.gnetGrids.insert(
								Gcell(routingTree[*v].x, routingTree[*v].y,
										routingTree[*v].z));
						if (routingTree[*v].isRoot)
							vp.hasOriginalRoot = true;
					}

					if (c[*v] == c[e->m_source]
							&& routingTree[*v].realPinId != -1) {
						Gcell gcell = Gcell(routingTree[*v].x,
								routingTree[*v].y, routingTree[*v].z);
						// find pin type
						// find pin node
						Net net = gnet._net;
						assert(net._netPins.size() == net._netPinTypes.size());
						assert(
								net._netPins.size() == net._netPinNodeIDs.size());
						for (size_t i = 0; i < net._netPins.size(); ++i) {
							if (layout.getGcell(net._netPins[i]).overlapWith(
									gcell)) {
								if (vp.pinNodeIDs.insert(net._netPinNodeIDs[i]).second) {
									vp.pins.push_back(net._netPins[i]);
									vp.pinTypes.push_back(net._netPinTypes[i]);
									vp.cellAreas.push_back(
											layout._nodes[net._netPinNodeIDs[i]]._height
													* layout._nodes[net._netPinNodeIDs[i]]._width);
								}
							}
						}
					}
				}
				EdgeIter ewl, ewlend;
				int wl = 0;
				for (tie(ewl, ewlend) = edges(routingTree); ewl != ewlend;
						++ewl) {
					if (c[ewl->m_source] == c[e->m_source]
							&& c[ewl->m_target] == c[e->m_source]) {
						wl += Gcell(routingTree[ewl->m_source].x,
								routingTree[ewl->m_source].y,
								routingTree[ewl->m_source].z).viaWeightedDist(
								Gcell(routingTree[ewl->m_target].x,
										routingTree[ewl->m_target].y,
										routingTree[ewl->m_target].z));
					}
				}
				if (vp.pins.size() > 0) {
					++vpCount;
					vp.wlToL1 = wl;
					vpins.push_back(vp);
				}
			}
		}
		if (includeAllPIOs) {
			for (size_t id = 0; id < gnet._gWires.size(); id++) {
				auto &gWire = gnet._gWires[id];
				if (gWire._z > splitLayer && gWire._realPinId > -1) {
					Vpin vp;
					vp.vPinID = vpCountBase + vpCount;
					vp.gnetID = gnetID;
					vp.xCoord = gWire._x;
					vp.yCoord = gWire._y;
					vp.zCoord = gWire._z;
					vp.hasOriginalRoot = false;
					vp.isPIO = true;
					Gcell gcell = Gcell(gWire._x, gWire._y, gWire._z);
					// find pin type
					// find pin node
					Net net = gnet._net;
					for (size_t i = 0; i < net._netPins.size(); ++i) {
						if (layout.getGcell(net._netPins[i]).overlapWith(
								gcell)) {
							if (vp.pinNodeIDs.insert(net._netPinNodeIDs[i]).second) {
								vp.pins.push_back(net._netPins[i]);
								vp.pinTypes.push_back(net._netPinTypes[i]);
								vp.cellAreas.push_back(
										layout._nodes[net._netPinNodeIDs[i]]._height
												* layout._nodes[net._netPinNodeIDs[i]]._width);
							}
						}
					}
					++vpCount;
					++totalVpin;
					vp.wlToL1 = 0;
					vpins.push_back(vp);
				}
			}
		}
		// Find matching vpin
		if (twoCutNetsOnly) {
			assert(vpCount == 2);
			vpins[vpins.size() - 1].matchingVpinIdx = vpins.size() - 2;
			vpins[vpins.size() - 2].matchingVpinIdx = vpins.size() - 1;
		} else {
			int outputVpin = 0;
			assert(vpCount >= 2);
			for (int i = 1; i <= vpCount; ++i) {
				auto &vp = vpins[vpins.size() - i];
				for (size_t j = 0; j < vp.pinTypes.size(); ++j) {
					if (vp.pinTypes[j] == PI || vp.pinTypes[j] == CO) {
						outputVpin = i;
						break;
					}
				}
				if (outputVpin != 0)
					break;
			}
			assert(outputVpin != 0);
			if (vpCount == 2) {
				vpins[vpins.size() - 1].matchingVpinIdx = vpins.size() - 2;
				vpins[vpins.size() - 2].matchingVpinIdx = vpins.size() - 1;
			} else {
				for (int i = 1; i <= vpCount; ++i) {
					if (i != outputVpin)
						vpins[vpins.size() - i].matchingVpinIdx = vpins.size()
								- outputVpin;
					else
						vpins[vpins.size() - i].matchingVpinIdx = -1;
				}
			}
		}
		// Grouping vpins that share the same node.
		Graph vpGroup;
		for (int i = 0; i < vpCount; ++i)
			add_vertex(vpGroup);
		for (int i = 0; i < vpCount; ++i) {
			for (int j = i + 1; j < vpCount; ++j) {
				bool skip = false;
				for (auto nodeId1 : vpins[vpins.size() - vpCount + i].pinNodeIDs) {
					for (auto nodeId2 : vpins[vpins.size() - vpCount + j].pinNodeIDs) {
						if (nodeId1 == nodeId2) {
							add_edge(i, j, vpGroup);
							skip = true;
							break;
						}
					}
					if (skip)
						break;
				}
			}
		}
		std::vector<int> vpg(num_vertices(vpGroup));
		int groups = connected_components(vpGroup, &vpg[0]);
		for (int i = 0; i < vpCount; ++i) {
			vpins[vpins.size() - vpCount + i].vpGroupID = vpGroupBase + vpg[i];
		}
		vpGroupBase += groups;
		if (vpCount > 0) {
			beolNets++;
		}
	}

	cout << "# Vpins: " << vpins.size() << endl;
	cout << "# BEOL Nets: " << beolNets << endl;
	return vpins;
}

vector<Vpin> RoutingDB_DR::getVpins(const LayoutDR &layout, const int gnetID,
		const int splitLayer, bool twoCutNetsOnly, bool includeAllPIOs) const { // splitLayer: 0-based
	vector<Vpin> vpins;
	using namespace boost;
	const Gnet &gnet = _gnets[gnetID];
	bool noVpins = true;
	for (auto &gWire : gnet._gWires) {
		if (gWire._z > splitLayer) {
			noVpins = false;
			break;
		}
	}
	if (noVpins)
		return vpins;

	Graph routingTree;
	int totalVpin = 0;
	int vpCount = 0;
//vector<Vertex> vs;
	for (auto &gWire : gnet._gWires) {
		Vertex v = add_vertex(routingTree);
		routingTree[v].x = gWire._x;
		routingTree[v].y = gWire._y;
		routingTree[v].z = gWire._z;
		routingTree[v].realPinId = gWire._realPinId;
		routingTree[v].isRoot = (gWire._pWireId == -1 && gWire._realPinId >= 0);
	}
	for (size_t i = 0; i < gnet._gWires.size(); i++) {
		auto &gWire = gnet._gWires[i];
		if (gWire._pWireId >= 0
				&& !(gWire._z > splitLayer
						&& gnet._gWires[gWire._pWireId]._z > splitLayer)) {
			auto r = add_edge(gWire._pWireId, i, routingTree); // only connect if lower than or on splitLayer
			assert(r.second);
			EdgeType e = r.first;
			if (routingTree[e.m_source].z > splitLayer
					|| routingTree[e.m_target].z > splitLayer) {
				routingTree[e].isVpin = true;
				++vpCount;
				++totalVpin;
			}
		}
	}

	if (vpCount == 0 || (twoCutNetsOnly && vpCount != 2))
		return vpins;

	int vpCountBase = totalVpin - vpCount;
	vpCount = 0;
	EdgeIter ei, eiend;
	for (tie(ei, eiend) = edges(routingTree); ei != eiend; ++ei) { // for each vpin
		if (routingTree[*ei].isVpin) {
			Vpin vp;
			vp.vPinID = vpCountBase + vpCount;
			vp.gnetID = gnetID;
			vp.xCoord = routingTree[ei->m_source].x;
			vp.yCoord = routingTree[ei->m_source].y;
			vp.zCoord = splitLayer;
			vp.hasOriginalRoot = false;
			std::vector<int> c(num_vertices(routingTree));
			connected_components(routingTree, &c[0]);
			VertexIter vi, viend;
			EdgeIter e2, e2end;
			for (tie(e2, e2end) = edges(routingTree); e2 != e2end; ++e2) {
				if (c[e2->m_source] == c[ei->m_source]) {
					int x1 = routingTree[e2->m_source].x;
					int y1 = routingTree[e2->m_source].y;
					int z1 = routingTree[e2->m_source].z;
					int x2 = routingTree[e2->m_target].x;
					int y2 = routingTree[e2->m_target].y;
					int z2 = routingTree[e2->m_target].z;

					if (z1 != z2) {
						// assert(abs(x1 - x2) <= extValue[z1] + extValue[z2]);
						// assert(abs(y1 - y2) <= extValue[z1] + extValue[z2]);
						for (int z = min(z1, z2); z <= max(z1, z2); ++z) {
							for (int x = min(x1, x2); x <= max(x1, x2); x +=
									5) {
								for (int y = min(y1, y2); y <= max(y1, y2); y +=
										5) {
									if (findVia(x, y, z) != -1) {
										vp.gnetGrids.insert(Gcell(x, y, z));
									}
								}
							}
						}
					} else {
						for (int x = min(x1, x2); x <= max(x1, x2); x += 5) {
							for (int y = min(y1, y2); y <= max(y1, y2); y +=
									5) {
								if (findEdge(x, y, z1) != -1) {
									vp.gnetGrids.insert(Gcell(x, y, z1));
								}
							}
						}
					}
				}
			}
			for (tie(vi, viend) = vertices(routingTree); vi != viend; ++vi) {
				if (c[*vi] == c[ei->m_source]) {
					vp.gnetVertices.insert(
							Gcell(routingTree[*vi].x, routingTree[*vi].y,
									routingTree[*vi].z));
					vp.gnetGrids.insert(
							Gcell(routingTree[*vi].x, routingTree[*vi].y,
									routingTree[*vi].z));
					if (routingTree[*vi].isRoot)
						vp.hasOriginalRoot = true;
				}
				if (c[*vi] == c[ei->m_source]
						&& routingTree[*vi].realPinId != -1) {
					Gcell gcell = Gcell(routingTree[*vi].x, routingTree[*vi].y,
							routingTree[*vi].z);
					// find pin type
					// find pin node
					Net net = gnet._net;
					assert(net._netPins.size() == net._netPinTypes.size());
					assert(net._netPins.size() == net._netPinNodeIDs.size());
					for (size_t i = 0; i < net._netPins.size(); ++i) {
						if (layout.getGcell(net._netPins[i]).overlapWith(
								gcell)) {
							if (vp.pinNodeIDs.insert(net._netPinNodeIDs[i]).second) {
								vp.pins.push_back(net._netPins[i]);
								vp.pinTypes.push_back(net._netPinTypes[i]);
								vp.cellAreas.push_back(
										layout._nodes[net._netPinNodeIDs[i]]._height
												* layout._nodes[net._netPinNodeIDs[i]]._width);
							}
						}
					}
				}
			}
			EdgeIter ewl, ewlend;
			int wl = 0;
			for (tie(ewl, ewlend) = edges(routingTree); ewl != ewlend; ++ewl) {
				if (c[ewl->m_source] == c[ei->m_source]
						&& c[ewl->m_target] == c[ei->m_source]) {
					wl += Gcell(routingTree[ewl->m_source].x,
							routingTree[ewl->m_source].y,
							routingTree[ewl->m_source].z).viaWeightedDist(
							Gcell(routingTree[ewl->m_target].x,
									routingTree[ewl->m_target].y,
									routingTree[ewl->m_target].z));
				}
			}
			if (vp.pins.size() > 0) {
				vpCount++;
				vp.wlToL1 = wl;
				vpins.push_back(vp);
			}
		}
	}
	if (includeAllPIOs) {
		for (size_t id = 0; id < gnet._gWires.size(); id++) {
			auto &gWire = gnet._gWires[id];
			if (gWire._z > splitLayer && gWire._realPinId > -1) {
				Vpin vp;
				vp.vPinID = vpCountBase + vpCount;
				vp.gnetID = gnetID;
				vp.xCoord = gWire._x;
				vp.yCoord = gWire._y;
				vp.zCoord = gWire._z;
				vp.hasOriginalRoot = false;
				vp.isPIO = true;
				Gcell gcell = Gcell(gWire._x, gWire._y, gWire._z);
				// find pin type
				// find pin node
				Net net = gnet._net;
				for (size_t i = 0; i < net._netPins.size(); ++i) {
					if (layout.getGcell(net._netPins[i]).overlapWith(gcell)) {
						if (vp.pinNodeIDs.insert(net._netPinNodeIDs[i]).second) {
							vp.pins.push_back(net._netPins[i]);
							vp.pinTypes.push_back(net._netPinTypes[i]);
							vp.cellAreas.push_back(
									layout._nodes[net._netPinNodeIDs[i]]._height
											* layout._nodes[net._netPinNodeIDs[i]]._width);
						}
					}
				}
				++vpCount;
				++totalVpin;
				vp.wlToL1 = 0;
				vpins.push_back(vp);
			}
		}
	}

	if (twoCutNetsOnly && vpCount != 2) {
		while (vpCount--) {
			vpins.pop_back();
		}
	}
	if (twoCutNetsOnly) {
		assert(vpCount == 2);
		vpins[vpins.size() - 1].matchingVpinIdx = vpins.size() - 2;
		vpins[vpins.size() - 2].matchingVpinIdx = vpins.size() - 1;
	} else {
		int outputVpin = 0;
		assert(vpCount >= 2);
		for (int i = 1; i <= vpCount; ++i) {
			auto &vp = vpins[vpins.size() - i];
			for (size_t j = 0; j < vp.pinTypes.size(); ++j) {
				if (vp.pinTypes[j] == PI || vp.pinTypes[j] == CO) {
					outputVpin = i;
					break;
				}
			}
			if (outputVpin != 0)
				break;
		}
		assert(outputVpin != 0);
		if (vpCount == 2) {
			vpins[vpins.size() - 1].matchingVpinIdx = vpins.size() - 2;
			vpins[vpins.size() - 2].matchingVpinIdx = vpins.size() - 1;
		} else {
			for (int i = 1; i <= vpCount; ++i) {
				if (i != outputVpin)
					vpins[vpins.size() - i].matchingVpinIdx = vpins.size()
							- outputVpin;
				else
					vpins[vpins.size() - i].matchingVpinIdx = -1;
			}
		}
	}

// cout << "# Vpins: " << vpins.size() << endl;
	assert(
			vpins.size() != 2 || vpins[0].hasOriginalRoot != vpins[1].hasOriginalRoot);
	return vpins;
}

void RoutingDB::outputCSV(const Layout &layout, const int splitLayer,
		const string fileName) const {
// 0: Node name				1: numInputs			2: numOutputs
#if 0
	ofstream fout;
	fout.open(fileName, ofstream::out);
	fout << "Node name,numInputs,numOutputs,isTerminal" << endl;
	for (auto &node : layout._nodes) {
		fout << node._name << "," << node._numInputs << "," << node._numOutputs
				<< "," << node._isTerm << endl;
	}
	fout.close();
#endif
#if 1
// 0: VpinID				1: X coordinate			2: Y coordinate
// 3: WL visible			4: Pin Type 			5: NumberLayer One Pins
// 6: Ave Cell Area Input	7: Ave Cell Area Output	8: Ave Pin X coordinate
// 9: Ave Pin Y coordinate	10*: CongestionCrouting	11*: CongestionPlacement
// 12: Matching Vpin ID     13: gnetID              14: Output and PI Pin Count

	vector<Vpin> vpins = getVpins(layout, splitLayer, true, false);

	ofstream fout;
	fout.open(fileName, ofstream::out);
	fout
			<< "VpinID,X coordinate,Y coordinate,WL visible,Pin Type,NumberLayer One Pins,";
	fout
			<< "Ave Cell Area Input,Ave Cell Area Output,Ave Pin X coordinate,Ave Pin Y coordinate,";
	fout
			<< "CongestionCrouting,CongestionPlacement,Matching Vpin ID,gnetID,Output and PI Pin Count"
			<< endl;
	for (auto vp : vpins) {
		fout << "S" << vp.vPinID << "," << vp.xCoord * layout._cellWidth << ","
				<< vp.yCoord * layout._cellHeight << "," << vp.wlToL1 << ",";
		for (auto p : vp.pinTypes) {
			fout
					<< (p == CI ? "i" : p == CO ? "o" : p == PI ? "I" :
						p == PO ? "O" : "N");
		}
		fout << "," << vp.pinTypes.size() << ",";
		assert(vp.pinTypes.size() == vp.cellAreas.size());
		double totalInputArea = 0, totalOutputArea = 0, totalOutput_PIPinCount =
				0;
		int inputPinCount = 0, outputPinCount = 0;

		for (size_t i = 0; i < vp.pinTypes.size(); ++i) {
			if (vp.pinTypes[i] == CI || vp.pinTypes[i] == PI) {
				totalInputArea += vp.cellAreas[i];
				++inputPinCount;
			} else if (vp.pinTypes[i] == CO || vp.pinTypes[i] == PO) {
				totalOutputArea += vp.cellAreas[i];
				++outputPinCount;
			}
			if (vp.pinTypes[i] == PI || vp.pinTypes[i] == CO) {
				++totalOutput_PIPinCount;
			}
		}
		fout << (inputPinCount > 0 ? totalInputArea / inputPinCount : 0) << ",";
		fout << (outputPinCount > 0 ? totalOutputArea / outputPinCount : 0)
				<< ",";
		assert(vp.pinTypes.size() == vp.pins.size());
		double totalPinX = 0;
		double totalPinY = 0;
		for (size_t i = 0; i < vp.pins.size(); ++i) {
			totalPinX += ((int) (vp.pins[i]._x / layout._cellWidth))
					* layout._cellWidth;
			totalPinY += ((int) (vp.pins[i]._y / layout._cellHeight))
					* layout._cellHeight;
		}
		fout << (vp.pins.size() > 0 ? totalPinX / vp.pins.size() : 0) << ",";
		fout << (vp.pins.size() > 0 ? totalPinY / vp.pins.size() : 0) << ",";
		fout << "0,0,"; // congestions, n/a
		fout << "S" << vpins[vp.matchingVpinIdx].vPinID << "," << vp.gnetID << ",";
		fout << totalOutput_PIPinCount << endl;
	}
	fout.close();
#endif
}

void RoutingDB_DR::outputCSV(const LayoutDR &layout, const int splitLayer,
		const string fileName) const {
// 0: VpinID				1: X coordinate			2: Y coordinate
// 3: WL visible			4: Pin Type 			5: NumberLayer One Pins
// 6: Ave Cell Area Input	7: Ave Cell Area Output	8: Ave Pin X coordinate
// 9: Ave Pin Y coordinate	10*: CongestionCrouting	11*: CongestionPlacement
// 12: Matching Vpin ID     13: gnetID              14: Total output+PI Pin Count

	vector<Vpin> vpins = getVpins(layout, splitLayer, false, true);

	ofstream fout;
	fout.open(fileName, ofstream::out);
	fout
			<< "VpinID,X coordinate,Y coordinate,WL visible,Pin Type,NumberLayer One Pins,";
	fout
			<< "Ave Cell Area Input,Ave Cell Area Output,Ave Pin X coordinate,Ave Pin Y coordinate,";
	fout
			<< "CongestionCrouting,CongestionPlacement,Matching Vpin ID,Net Name,Output and PI Pin Count,GroupID"
			<< endl;
	for (auto vp : vpins) {
		fout << "S" << vp.vPinID << "," << vp.xCoord << "," << vp.yCoord << ","
				<< vp.wlToL1 << ",";
		for (auto p : vp.pinTypes) {
			fout
					<< (p == CI ? "i" : p == PI ? "I" : p == CO ? "o" :
						p == PO ? "O" : "N");
		}
		fout << "," << vp.pinTypes.size() << ",";
		assert(vp.pinTypes.size() == vp.cellAreas.size());
		double totalInputArea = 0, totalOutputArea = 0, totalOutput_PIPinCount =
				0;
		int inputPinCount = 0, outputPinCount = 0;

		for (size_t i = 0; i < vp.pinTypes.size(); ++i) {
			if (vp.pinTypes[i] == CI || vp.pinTypes[i] == PI) {
				totalInputArea += vp.cellAreas[i];
				++inputPinCount;
			} else if (vp.pinTypes[i] == CO || vp.pinTypes[i] == PO) {
				totalOutputArea += vp.cellAreas[i];
				++outputPinCount;
			}
			if (vp.pinTypes[i] == PI || vp.pinTypes[i] == CO) {
				++totalOutput_PIPinCount;
			}
		}
		fout << (inputPinCount > 0 ? totalInputArea / inputPinCount : 0) << ",";
		fout << (outputPinCount > 0 ? totalOutputArea / outputPinCount : 0)
				<< ",";
		assert(vp.pinTypes.size() == vp.pins.size());
		double totalPinX = 0;
		double totalPinY = 0;
		for (size_t i = 0; i < vp.pins.size(); ++i) {
			totalPinX += vp.pins[i]._x;
			totalPinY += vp.pins[i]._y;
		}
		fout << (vp.pins.size() > 0 ? totalPinX / vp.pins.size() : 0) << ",";
		fout << (vp.pins.size() > 0 ? totalPinY / vp.pins.size() : 0) << ",";
		fout << "0,0,"; // congestions, n/a
		fout << "S" << vpins[vp.matchingVpinIdx].vPinID << "," << _gnets[vp.gnetID].getName()
				<< ",";
		fout << totalOutput_PIPinCount << "," << vp.vpGroupID << endl;
	}
	fout.close();
}
