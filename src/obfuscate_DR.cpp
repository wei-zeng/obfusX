/*
 * obfuscate.cpp
 *
 *  Created on: Aug 28, 2019
 *      Author: wzeng
 */
#include "parser.h"
#include "grDB_DR.h"
#include <chrono>
#include <set>
#include <cassert>

#define NUM_OPTIONS_TO_SAVE 1
std::pair<ObfusOption, bool> RoutingDB_DR::traverseObfusOpts(
		const LayoutDR &layout, int layer, bool singleVpin, size_t sortedVpinID,
		int axis, const vector<double> &featVec, const SHAP &shap,
		const vector<Vpin> &vpins, int maxLayer) {
	assert(axis == 1 || axis == 2);
	vector<ObfusOption> opts;
	double thGain = 0;
	double thxcost = 0;
	double gain = 0;

	int wl_init = getTotalWL();

	size_t start = singleVpin ? sortedVpinID : 0;
	size_t end = singleVpin ? sortedVpinID + 1 : vpins.size();
	for (size_t vpinID = start; vpinID < end; ++vpinID) {
		if (opts.size() > 0 && opts[0].gain > gain) {
			//cout << DiMOid << "/" << vGnetID2DiMO.size() << endl;
			gain = opts[0].gain;
			cout << opts[0].toString() << "\t| " << opts.back().toString()
					<< ":" << opts.size();
			for (auto opt : opts) {
				cout << " " << opt.gain;
			}
			cout << endl;
		}

		std::unordered_map<Gcell, size_t, HashGcell3d> uMapVpin;
		for (size_t i = 0; i < vpins.size(); i++) {
			auto &vp = vpins[i];
			uMapVpin[Gcell(vp.xCoord, vp.yCoord, vp.zCoord)] = i;
		}
		int prev_wl = wl_init;
		auto vp = vpins[vpinID];
		if (vp.zCoord > layer - 1) { // PIO
			continue;
		}
		Gnet cleanGnet = _gnets[vp.gnetID];
		cout << "Vpin " << vpinID << "/" << vpins.size() << ": (" << vp.xCoord
				<< "," << vp.yCoord << "," << vp.zCoord << ")" << endl;
		//cout << "HasOrigRoot: " << vp.hasOriginalRoot << endl;
		//double originalWL = getWlByNet(vp.gnetID);
		int originalWLToL1 = vp.wlToL1; // backup originalWL for later use
		auto res = ripUpNet(layout, vp);
		//cout << "<After RU> HasOrigRoot: " << vp.hasOriginalRoot << endl;
		Gnet rippedUpGnet = _gnets[vp.gnetID];
		vector<double> new_featVec(featVec);
		vector<double> shap_values = shap.eval(featVec);
		double shap_base = shap.base_value();
		double mo_ = std::accumulate(shap_values.begin(), shap_values.end(),
				shap_base);
		cout << "MO_ = " << mo_ << endl;
		cout << "Original feature vector: ";
		for (size_t i = 0; i < featVec.size(); i++) {
			cout << "[" << i << "] " << featVec[i] << ", ";
		}
		cout << endl;
		Vpin vp_tmp(vp);
#define RADIUS 10000
		for (int margin = 10; margin <= RADIUS; margin += 10) {
			for (int d0 = -margin; d0 <= margin;
					d0 += (margin == 0 ? 1 : 2 * margin)) {
				if (findVia(vp.xCoord + (axis == 1 ? d0 : 0),
						vp.yCoord + (axis == 2 ? d0 : 0), vp.zCoord) == -1)
					continue;
				int occuGnetId = _occupiedGnetId[vp.zCoord].at(
						Gcell(vp.xCoord + (axis == 1 ? d0 : 0),
								vp.yCoord + (axis == 2 ? d0 : 0), vp.zCoord));
				if (occuGnetId != -1 && occuGnetId != vp.gnetID)
					continue;
				occuGnetId = _occupiedGnetId[vp.zCoord + 1].at(
						Gcell(vp.xCoord + (axis == 1 ? d0 : 0),
								vp.yCoord + (axis == 2 ? d0 : 0),
								vp.zCoord + 1));
				if (occuGnetId != -1 && occuGnetId != vp.gnetID)
					continue;
				vp_tmp.xCoord = vp.xCoord + (axis == 1 ? d0 : 0);
				vp_tmp.yCoord = vp.yCoord + (axis == 2 ? d0 : 0);
				double minmhd0 = INFINITY;
				double minmhd1 = INFINITY;

				auto goals0 = vp_tmp.gnetGrids;
				for (auto goal : goals0) {
					double mhd = fabs(vp_tmp.xCoord - goal._x)
							+ fabs(vp_tmp.yCoord - goal._y);
					for (int z = min(vp_tmp.zCoord, goal._z);
							z < max(vp_tmp.zCoord, goal._z); z++)
						mhd += Gcell(0, 0, z).viaWeights[z];
					if (mhd < minmhd0)
						minmhd0 = mhd;
				}

				auto goals1 = get<0>(res);
				for (auto goal : goals1) {
					double mhd = fabs(vp_tmp.xCoord - goal._x)
							+ fabs(vp_tmp.yCoord - goal._y);
					for (int z = min(vp_tmp.zCoord, goal._z);
							z < max(vp_tmp.zCoord, goal._z); z++)
						mhd += Gcell(0, 0, z).viaWeights[z];
					if (mhd < minmhd1)
						minmhd1 = mhd;
				}
				int minxcost = minmhd0 + minmhd1 - std::get<6>(res);
				if (axis == 1) {
					new_featVec[0] = fabs(
							vp_tmp.xCoord
									- vpins[vp_tmp.matchingVpinIdx].xCoord);
					new_featVec[4] = new_featVec[0] + new_featVec[1];
					new_featVec[6] += abs(d0); // temporary change in total FEOL wirelength to reflect the estimated extra cost
				} else if (axis == 2) {
					new_featVec[1] = fabs(
							vp_tmp.yCoord
									- vpins[vp_tmp.matchingVpinIdx].yCoord);
					new_featVec[4] = new_featVec[0] + new_featVec[1];
					new_featVec[6] += abs(d0);
				}
				shap_values = shap.eval(new_featVec);
				cout << d0 << ": New SHAP_" << ": [0] " << shap_values[0]
						<< " [1] " << shap_values[1] << " [4] "
						<< shap_values[4] << endl;
				double new_mo_ = std::accumulate(shap_values.begin(),
						shap_values.end(), shap_base);
				cout << "New MO_ = " << new_mo_ << endl;
				double dimo = mo_ - new_mo_;
				cout << dimo << " = (" << mo_ << ") - (" << new_mo_ << ")"
						<< endl;
				new_featVec[6] -= abs(d0); // Reverse: temporary change in total FEOL wirelength
				if (dimo <= 1e-6)
					continue;
				if (thGain > 1) {
					if (minxcost > 0)
						continue;
					if (1 + dimo < thGain)
						continue;
					if (1 + dimo == thGain && minxcost > thxcost)
						continue;
				} else if (thGain > 0) {
					if (minxcost > 0 && dimo / minxcost < thGain)
						continue;
				}
				auto res1 = rerouteNet(layout, vp_tmp,
						get<1>(res) /* otherVpinGridsOnBelowSL, banned */, 0,
						vp_tmp.zCoord, 7, false, false, !vp.hasOriginalRoot, 5, vpins[vp.matchingVpinIdx]);
				int margin0 = get<3>(res1);
				if (get<0>(res1) == INFINITY)
					continue;
				auto res2 = rerouteNet(layout, vp_tmp,
						get<0>(res) /* otherVpinGridsAboveSL */,
						get<2>(res) /* otherVpinVerticesAboveSL */, vp_tmp.zCoord + 1,
						maxLayer - 1, 7, false, false, 5, false, 1, vp_tmp.zCoord, vpins[vp.matchingVpinIdx]);
				int margin1 = get<3>(res2);
				double wlAdded = get<0>(res1) + get<0>(res2);
				cout << get<0>(res1) << " + " << get<0>(res2) << " - "
						<< get<6>(res) << endl;
				new_featVec[6] = featVec[6] - originalWLToL1 + vp_tmp.wlToL1
						+ get<0>(res1) + Gcell(0,0,vp_tmp.zCoord).viaWeights[0,0,vp_tmp.zCoord];
				cout << "-=" << originalWLToL1;
				cout << "+=" << vp_tmp.wlToL1 + get<0>(res1);
				shap_values = shap.eval(new_featVec);
				new_mo_ = std::accumulate(shap_values.begin(),
						shap_values.end(), shap_base);
				cout << "Real New feature vector: ";
				for (size_t i = 0; i < new_featVec.size(); i++) {
					cout << "[" << i << "] " << new_featVec[i] << ", ";
				}
				cout << endl;
				cout << d0 << ": Real New SHAP_" << ": [0] " << shap_values[0]
						<< " [1] " << shap_values[1] << " [4] "
						<< shap_values[4] << endl;
				cout << "Real New MO_ = " << new_mo_ << endl;
				dimo = mo_ - new_mo_;

				// updateGridOccupation(vp.gnetID, false);

				// int wl = getTotalWL();
				//if (wl > prev_wl + 2 * extValue[maxLayer - 1]) {
				//	throw("WL increased?");
				//	exit(-2);
				//}
				//prev_wl = wl;
				// update vpins
				//auto newVpins = getVpins(layout, vp.gnetID, layer - 1, false, true);
				//for (auto &nvp : newVpins) {
				//	size_t idx = uMapVpin.at(Gcell(nvp.xCoord, nvp.yCoord, nvp.zCoord));
				//	vpins[idx] = nvp;
				//}

				double xcost = wlAdded - std::get<6>(res);
				double gain = 0;
				if (dimo > 1e-6) {
					if (xcost <= 0) {
						gain = 1 + dimo;
					} else {
						gain = dimo / xcost;
					}
				}
				if (gain > 0) {
					ObfusOption opt;
					opt.gain = gain;
					opt.gnetID = vpinID;
					opt.DiMO = dimo;
					opt.xCost = xcost;
					opt.d0 = d0;
					opt.d1 = 0;
					opt.axis = axis;
					opt.margin0 = margin0;
					opt.margin1 = margin1;
					// opt.newGnet = tmpRtDB.getGnets()[gnetID];
					cout << "(" << (opt.axis == 1 ? "X" : "Y")
							<< (opt.d0 >= 0 ? "+" : "") << opt.d0 << ") "
							<< opt.gain << "=" << opt.DiMO << "/" << opt.xCost
							<< endl;
					size_t k;
					for (k = 0; k < opts.size(); ++k) { // replace the node with the same gnet, if better
						if (opt.gnetID == opts[k].gnetID) {
							if (opt > opts[k]) {
								opts[k] = opt;
								std::sort(opts.begin(), opts.end(),
										std::greater<ObfusOption>());
								thGain = opts.back().gain;
								thxcost = opts.back().xCost;
							}
							break;
						}
					}
					if (k == opts.size()) { // no same gnet in the list
						if (opts.size() < NUM_OPTIONS_TO_SAVE) {
							opts.push_back(opt);
							std::sort(opts.begin(), opts.end(),
									std::greater<ObfusOption>());
							thGain = opts.back().gain;
							thxcost = opts.back().xCost;
						} else if (opt > opts.back()) {
							opts.back() = opt;
							std::sort(opts.begin(), opts.end(),
									std::greater<ObfusOption>());
							thGain = opts.back().gain;
							thxcost = opts.back().xCost;
						}
					}
				} // if (gain > 0)
			} // for d0
		}
		if (isDirty()) {
			restoreNet(vp.gnetID, cleanGnet, _gnets[vp.gnetID]._cleanEdges,
					_gnets[vp.gnetID]._cleanVias, true);
		}
		assert(_gnets[vp.gnetID] == cleanGnet);
	} // for vp

	// cout << "R&R list:" << endl;
	for (auto opt : opts) {
		cout << opt.toString() << endl;
	}
	return opts.size() > 0 ?
			make_pair(opts[0], true) : make_pair(ObfusOption(), false);
}

std::pair<ObfusOption, bool> RoutingDB_DR::traverseLiftingOpts(
		const LayoutDR &layout, int layer, bool singleVpin, size_t sortedVpinID,
		int axis, const vector<double> &featVec, const SHAP &shap,
		const vector<Vpin> &vpins, int maxLayer, int splitLayer) {
	vector<ObfusOption> opts;
	double thGain = 0;
	double thxcost = 0;
	double gain = 0;

	int wl_init = getTotalWL();

	size_t start = singleVpin ? sortedVpinID : 0;
	size_t end = singleVpin ? sortedVpinID + 1 : vpins.size();
	for (size_t vpinID = start; vpinID < end; ++vpinID) {
		if (opts.size() > 0 && opts[0].gain > gain) {
			//cout << DiMOid << "/" << vGnetID2DiMO.size() << endl;
			gain = opts[0].gain;
			cout << opts[0].toString() << "\t| " << opts.back().toString()
					<< ":" << opts.size();
			for (auto opt : opts) {
				cout << " " << opt.gain;
			}
			cout << endl;
		}
		double shap_base = shap.base_value();
		std::unordered_map<Gcell, size_t, HashGcell3d> uMapVpin;
		for (size_t i = 0; i < vpins.size(); i++) {
			auto &vp = vpins[i];
			uMapVpin[Gcell(vp.xCoord, vp.yCoord, vp.zCoord)] = i;
		}
		int prev_wl = wl_init;
		auto vp = vpins[vpinID];
		if (vp.zCoord > layer - 1) { // PIO
			continue;
		}
		Gnet cleanGnet = _gnets[vp.gnetID];
		cout << "Vpin " << vpinID << "/" << vpins.size() << ": (" << vp.xCoord
				<< "," << vp.yCoord << "," << vp.zCoord << ")" << endl;
		//cout << "HasOrigRoot: " << vp.hasOriginalRoot << endl;
		//double originalWL = getWlByNet(vp.gnetID);
		int originalWLToL1 = vp.wlToL1; // backup originalWL for later use
		auto res = ripUpNet(layout, vp);
		//cout << "<After RU> HasOrigRoot: " << vp.hasOriginalRoot << endl;
		Gnet rippedUpGnet = _gnets[vp.gnetID];
		vector<double> new_featVec(featVec);
		vector<double> shap_values = shap.eval(featVec);
		double mo_ = std::accumulate(shap_values.begin(), shap_values.end(),
				shap_base);
		cout << "MO_ = " << mo_ << endl;
		cout << "Original feature vector: ";
		for (size_t i = 0; i < featVec.size(); i++) {
			cout << "[" << i << "] " << featVec[i] << ", ";
		}
		cout << endl;
		Vpin vp_tmp(vp);
#define RADIUS_LIFTING 1000
		for (int margin = 0; margin <= RADIUS_LIFTING; margin += 10) {
			for (int dx = -margin; dx <= margin; dx += 10) {
				for (int dy = -margin; dy <= margin; dy += 10) {
					if (dx != -margin && dx != margin && dy != -margin
							&& dy != -margin)
						continue;
					if (findVia(vp.xCoord + dx, vp.yCoord + dy, vp.zCoord)
							== -1)
						continue;
					int occuGnetId = _occupiedGnetId[vp.zCoord].at(
							Gcell(vp.xCoord + dx, vp.yCoord + dy, vp.zCoord));
					if (occuGnetId != -1 && occuGnetId != vp.gnetID)
						continue;
					occuGnetId = _occupiedGnetId[vp.zCoord + 1].at(
							Gcell(vp.xCoord + dx, vp.yCoord + dy,
									vp.zCoord + 1));
					if (occuGnetId != -1 && occuGnetId != vp.gnetID)
						continue;
					vp_tmp.xCoord = vp.xCoord + dx;
					vp_tmp.yCoord = vp.yCoord + dy;
					double minmhd0 = INFINITY;
					double minmhd1 = INFINITY;

					auto goals0 = vp_tmp.gnetGrids;
					for (auto goal : goals0) {
						double mhd = fabs(vp_tmp.xCoord - goal._x)
								+ fabs(vp_tmp.yCoord - goal._y);
						for (int z = min(vp_tmp.zCoord, goal._z);
								z < max(vp_tmp.zCoord, goal._z); z++)
							mhd += Gcell(0, 0, z).viaWeights[z];
						if (mhd < minmhd0)
							minmhd0 = mhd;
					}

					auto goals1 = get<0>(res);
					for (auto goal : goals1) {
						double mhd = fabs(vp_tmp.xCoord - goal._x)
								+ fabs(vp_tmp.yCoord - goal._y);
						for (int z = min(vp_tmp.zCoord, goal._z);
								z < max(vp_tmp.zCoord, goal._z); z++)
							mhd += Gcell(0, 0, z).viaWeights[z];
						if (mhd < minmhd1)
							minmhd1 = mhd;
					}
					int minxcost = minmhd0 + minmhd1 - std::get<6>(res);
					if (axis == 1) {
						new_featVec[0] = fabs(
								vp_tmp.xCoord
										- vpins[vp_tmp.matchingVpinIdx].xCoord);
						new_featVec[4] = new_featVec[0] + new_featVec[1];
						new_featVec[6] += abs(dx) + abs(dy); // temporary change in total FEOL wirelength to reflect the estimated extra cost
					} else if (axis == 2) {
						new_featVec[1] = fabs(
								vp_tmp.yCoord
										- vpins[vp_tmp.matchingVpinIdx].yCoord);
						new_featVec[4] = new_featVec[0] + new_featVec[1];
						new_featVec[6] += abs(dx) + abs(dy);
					}
					shap_values = shap.eval(new_featVec);
					cout << dx << " " << dy << ": New SHAP_" << ": [0] "
							<< shap_values[0] << " [1] " << shap_values[1]
							<< " [4] " << shap_values[4] << endl;
					double new_mo_ = std::accumulate(shap_values.begin(),
							shap_values.end(), shap_base);
					cout << "New MO_ = " << new_mo_ << endl;
					double dimo = 1 - new_mo_;
					cout << dimo << " = (" << 1 << ") - (" << new_mo_ << ")"
							<< endl;
					new_featVec[6] -= abs(dx) + abs(dy); // Reverse: temporary change in total FEOL wirelength
					if (dimo <= 1e-6)
						continue;
					if (thGain > 1) {
						if (minxcost > 0)
							continue;
						if (1 + dimo < thGain)
							continue;
						if (1 + dimo == thGain && minxcost > thxcost)
							continue;
					} else if (thGain > 0) {
						if (minxcost > 0 && dimo / minxcost < thGain)
							continue;
					}
					auto res1 = rerouteNet(layout, vp_tmp,
							get<1>(res) /* otherVpinGridsOnBelowSL, banned */,
							0, vp_tmp.zCoord, 7, false, false, !vp.hasOriginalRoot,
							5, vpins[vp.matchingVpinIdx]);
					int margin0 = get<3>(res1);
					if (get<0>(res1) == INFINITY)
						continue;
					auto res2 = rerouteNet(layout, vp_tmp,
							get<0>(res) /* otherVpinGridsAboveSL */,
							get<2>(res) /* otherVpinVerticesAboveSL */, vp_tmp.zCoord + 1,
							maxLayer - 1, 7, false, false, 5, true, 100, splitLayer, vpins[vp.matchingVpinIdx]);
					int margin1 = get<3>(res2);
					double wlAdded = get<0>(res1) + get<0>(res2);
					cout << get<0>(res1) << " + " << get<0>(res2) << " - "
							<< get<6>(res) << endl;
					new_featVec[6] = featVec[6] - originalWLToL1 + vp_tmp.wlToL1
							+ get<0>(res1);
					cout << "-=" << originalWLToL1;
					cout << "+=" << vp_tmp.wlToL1 + get<0>(res1);
					shap_values = shap.eval(new_featVec);
					new_mo_ = std::accumulate(shap_values.begin(),
							shap_values.end(), shap_base);
					cout << "Real New feature vector: ";
					for (size_t i = 0; i < new_featVec.size(); i++) {
						cout << "[" << i << "] " << new_featVec[i] << ", ";
					}
					cout << endl;
					cout << dx << " " << dy << ": Real New SHAP_" << ": [0] "
							<< shap_values[0] << " [1] " << shap_values[1]
							<< " [4] " << shap_values[4] << endl;
					cout << "Real New MO_ = " << new_mo_ << endl;
					dimo = 1 - new_mo_;

					// updateGridOccupation(vp.gnetID, false);

					// int wl = getTotalWL();
					//if (wl > prev_wl + 2 * extValue[maxLayer - 1]) {
					//	throw("WL increased?");
					//	exit(-2);
					//}
					//prev_wl = wl;
					// update vpins
					//auto newVpins = getVpins(layout, vp.gnetID, layer - 1, false, true);
					//for (auto &nvp : newVpins) {
					//	size_t idx = uMapVpin.at(Gcell(nvp.xCoord, nvp.yCoord, nvp.zCoord));
					//	vpins[idx] = nvp;
					//}

					double xcost = wlAdded - std::get<6>(res);
					double gain = 0;
					if (dimo > 1e-6) {
						if (xcost <= 0) {
							gain = 1 + dimo;
						} else {
							gain = dimo / xcost;
						}
					}
					if (gain > 0) {
						ObfusOption opt;
						opt.gain = gain;
						opt.gnetID = vpinID;
						opt.DiMO = dimo;
						opt.xCost = xcost;
						opt.d0 = dx;
						opt.d1 = dy;
						opt.axis = axis;
						opt.margin0 = margin0;
						opt.margin1 = margin1;
						// opt.newGnet = tmpRtDB.getGnets()[gnetID];
						cout << "(" << "X" << (opt.d0 >= 0 ? "+" : "") << opt.d0
								<< ", Y" << (opt.d1 >= 0 ? "+" : "") << opt.d1
								<< ") " << opt.gain << "=" << opt.DiMO << "/"
								<< opt.xCost << endl;
						size_t k;
						for (k = 0; k < opts.size(); ++k) { // replace the node with the same gnet, if better
							if (opt.gnetID == opts[k].gnetID) {
								if (opt > opts[k]) {
									opts[k] = opt;
									std::sort(opts.begin(), opts.end(),
											std::greater<ObfusOption>());
									thGain = opts.back().gain;
									thxcost = opts.back().xCost;
								}
								break;
							}
						}
						if (k == opts.size()) { // no same gnet in the list
							if (opts.size() < NUM_OPTIONS_TO_SAVE) {
								opts.push_back(opt);
								std::sort(opts.begin(), opts.end(),
										std::greater<ObfusOption>());
								thGain = opts.back().gain;
								thxcost = opts.back().xCost;
							} else if (opt > opts.back()) {
								opts.back() = opt;
								std::sort(opts.begin(), opts.end(),
										std::greater<ObfusOption>());
								thGain = opts.back().gain;
								thxcost = opts.back().xCost;
							}
						}
					} // if (gain > 0)
				} // for dy
			} // for dx
		}
		if (isDirty()) {
			restoreNet(vp.gnetID, cleanGnet, _gnets[vp.gnetID]._cleanEdges,
					_gnets[vp.gnetID]._cleanVias, true);
		}
		assert(_gnets[vp.gnetID] == cleanGnet);
	} // for vp

	// cout << "R&R list:" << endl;
	for (auto opt : opts) {
		cout << opt.toString() << endl;
	}
	return opts.size() > 0 ?
			make_pair(opts[0], true) : make_pair(ObfusOption(), false);
}

