/*
 * obfuscate.cpp
 *
 *  Created on: Aug 28, 2019
 *      Author: wzeng
 */
#include "parser.h"
#include "grDB.h"
#include <chrono>
#include <set>
#include <cassert>

bool ObfusOption::operator>(const ObfusOption &other) const {
	return (gain != other.gain) ? (gain > other.gain) :
			(xCost != other.xCost) ? (xCost < other.xCost) :
			(DiMO != other.DiMO) ?
					(DiMO > other.DiMO) :
					(fabs(d0) + fabs(d1) < fabs(other.d0) + fabs(other.d1));
}

std::string ObfusOption::toString() const {
	std::stringstream ss;
	ss << "[" << gnetID << "," << d0 << "," << d1 << "]\tgain=" << gain
			<< "\tDiMO=" << DiMO << "\txCost=" << xCost;
	return ss.str();
}

#define NUM_OPTIONS_TO_SAVE 1
std::pair<ObfusOption, bool> RoutingDB::traverseObfusOpts(const Layout &layout,
		int layer, vector<Vpin> &vps, size_t best_vpin,
		const vector<double> &featVec, const SHAP &shap, double shap_base,
		double mo_) {
	vector<ObfusOption> opts;
	RoutingDB &tmpRtDB = (*this);
	double thGain = 0;
	double thxcost = 0;
	int r0 = 0, r1 = 0, r2 = 0;
	double gain = 0;
	if (opts.size() > 0 && opts[0].gain > gain) {
		gain = opts[0].gain;
		cout << opts[0].toString() << "\t| " << opts.back().toString() << ":"
				<< opts.size();
		for (auto opt : opts) {
			cout << " " << opt.gain;
		}
		cout << endl;
	}
	int gnetID = vps[best_vpin].gnetID;
	Gnet cleanGnet = _gnets[gnetID];
	assert(cleanGnet.isGlobal());
	Vpin vp = vps[best_vpin];
	int originalWLToL1 = vp.wlToL1; // backup originalWL for later use
	auto res = tmpRtDB.ripUpPin(layout, vp, layer - 1);
	Gnet rippedUpGnet = tmpRtDB._gnets[gnetID];
#define RADIUS 3
	double wlAdded0[2 * RADIUS + 1] = { 0 };
	double minmhd0[2 * RADIUS + 1] = { 0 };
	int margin0[2 * RADIUS + 1] = { 0 };
	int margin2 = 0;

	for (int d0 = -RADIUS; d0 <= RADIUS; d0++) {
		auto vp0 = vps[best_vpin];
		vp0.yCoord += d0;
		minmhd0[d0 + RADIUS] = INFINITY;
		wlAdded0[d0 + RADIUS] = INFINITY;
		auto goals0 = vp0.gnetGrids;
		for (auto goal : goals0) {
			double mhd = fabs(vp0.xCoord - goal._x) + fabs(vp0.yCoord - goal._y)
					+ fabs(vp0.zCoord - goal._z);
			if (mhd < minmhd0[d0 + RADIUS])
				minmhd0[d0 + RADIUS] = mhd;
		}
	}
	for (int d0 = -RADIUS; d0 <= RADIUS; d0++) {
		auto vp0 = Vpin(vp);
		vp0.yCoord += d0;
		if (vp0.yCoord < 0
				|| vp0.yCoord > static_cast<int>(layout._numTilesY) - 1)
			continue;
		auto vp1 = Vpin(vps[vp0.matchingVpinIdx]);
		//vp1.yCoord += d1;
		//if (vp1.yCoord < 0
		//		|| vp1.yCoord
		//				> static_cast<int>(layout._numTilesY) - 1)
		//	continue;
		int xdiff = abs(vp0.yCoord - vp1.yCoord)
				- abs(vp0.yCoord - vp1.yCoord - d0);
		vector<double> new_featVec(featVec);
		new_featVec[1] = abs(vp0.yCoord - vp1.yCoord) * layout._cellHeight;
		new_featVec[4] = new_featVec[0] + new_featVec[1];
		//new_featVec[6] += abs(d0); // temp
		//auto new_shap_val = shap.eval(new_featVec);
		//double dimo = mo_
		//		- accumulate(new_shap_val.begin(), new_shap_val.end(),
		//				shap_base);
		//new_featVec[6] -= abs(d0);
		//if (dimo <= 1e-10)
		//	continue;
		double minxcost = minmhd0[d0 + RADIUS] + 1 - std::get<3>(res)
				+ (Gcell(vp0.xCoord, vp0.yCoord, vp0.zCoord + 1)
						- Gcell(vp1.xCoord, vp1.yCoord, vp1.zCoord + 1));
		if (thGain > 1) {
			if (minxcost > 0)
				continue;
			if (1 + 1 < thGain)
				continue;
			if (1 + 1 == thGain && minxcost > thxcost)
				continue;
		} else if (thGain > 0) {
			if (minxcost > 0 && 1 / minxcost < thGain)
				continue;
		}
		// Find root of vp0
		auto res2 = tmpRtDB.rerouteNet(layout, vp0,
				vps[vp0.matchingVpinIdx].gnetGrids, 0, layer - 1, 15, false,
				false, false);
		r0++;
		auto bannedPts = vp0.gnetGrids;
		for (auto pt : std::get<4>(res2)) {
			bannedPts.insert(pt);
		}
		/*cerr << d0 << "Vp1.bannedPts" << endl;
		 for (auto i : bannedPts) {
		 cerr << i._x << " " << i._y << " " << i._z << endl;
		 }*/
		if (std::get<0>(res2) != INFINITY) {
			wlAdded0[d0 + RADIUS] = std::get<0>(res2);
			margin0[d0 + RADIUS] = std::get<3>(res2);
		} else {
			wlAdded0[d0 + RADIUS] = INFINITY;
			continue;
		}
		//	cout << "(Reroute) Failed!" << endl;
		auto res3 = tmpRtDB.rerouteNet(layout, vp0, vps[vp0.matchingVpinIdx],
				layer, 8, 15, false, false);
		// tmpRtDB.getTotalWL();

		r2++;
		new_featVec[6] = featVec[6] - originalWLToL1 + vp0.wlToL1 + get<0>(res2)
				+ 1;
		double wlAdded;
		if (std::get<0>(res3) != INFINITY) {
			wlAdded = wlAdded0[d0 + RADIUS] + std::get<0>(res3);
			margin2 = std::get<3>(res3);
		} else {
			wlAdded = INFINITY;
			continue;
		}
		vector<double> new_shap_val = shap.eval(new_featVec);
		//cout << "From obfus: original " << featVec[1] << " " << featVec[4] << " " << featVec[6] << endl;
		//cout << "From obfus: " << new_featVec[1] << " " << new_featVec[4] << " " << new_featVec[6] << endl;
		double dimo = mo_
				- accumulate(new_shap_val.begin(), new_shap_val.end(),
						shap_base);

		double xcost = wlAdded - std::get<3>(res);
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
			opt.gnetID = gnetID;
			opt.DiMO = dimo;
			opt.xCost = xcost;
			opt.d0 = d0;
			opt.d1 = 0;
			opt.margin0 = margin0[d0 + RADIUS];
			opt.margin1 = 0;
			opt.margin2 = margin2;
			//opt.changeInEdgeDemand =
			//		changesInEdgeDemand[d0 + RADIUS][d1 + RADIUS];
			//opt.changeInViaDemand =
			//		changesInViaDemand[d0 + RADIUS][d1 + RADIUS];
			// opt.newGnet = tmpRtDB.getGnets()[gnetID];
			//cout << "(" << opt.d0 << "," << opt.d1 << ") "
			//		<< opt.gain << "=" << opt.DiMO << "/"
			//		<< opt.xCost << endl;
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
		  //} // for d1
		if (tmpRtDB.isDirty()) {
			tmpRtDB.restoreNet(gnetID, rippedUpGnet, _gnets[gnetID]._ruEdges,
					_gnets[gnetID]._ruVias, true);
			_dirty = true; // not restored to clean yet.
		}
	} // for d0
	if (tmpRtDB.isDirty()) {
		tmpRtDB.restoreNet(gnetID, cleanGnet, _gnets[gnetID]._cleanEdges,
				_gnets[gnetID]._cleanVias, true);
	}
	assert(_gnets[gnetID] == cleanGnet);
	/*if (opts.size() > 0) {
	 cout << opts[0].toString() << "\t| " << opts.back().toString()
	 << ":" << opts.size();
	 for (auto opt : opts) {
	 cout << " " << opt.gain;
	 }
	 cout << endl;
	 }*/
	//} // for gnet
	// cout << "R&R list:" << endl;
	for (auto opt : opts) {
		cout << opt.toString() << endl;
	}
	cout << " Reroute count: R0 = " << r0 << ", R1 = " << r1 << ", R2 = " << r2
			<< " Total = " << r0 + r1 + r2 << endl;
	return opts.size() > 0 ?
			make_pair(opts[0], true) : make_pair(ObfusOption(), false);
}

std::pair<ObfusOption, bool> RoutingDB::randomTraverse(const Layout &layout,
		int layer, vector<Vpin> &vps, size_t best_vpin, size_t maxTrials,
		double stdev, std::default_random_engine &rng) {
	vector<ObfusOption> opts;
	RoutingDB &tmpRtDB = (*this);
	double thGain = 0;
	double thxcost = 0;
	int r0 = 0, r1 = 0, r2 = 0;
	double gain = 0;
	if (opts.size() > 0 && opts[0].gain > gain) {
		gain = opts[0].gain;
		cout << opts[0].toString() << "\t| " << opts.back().toString() << ":"
				<< opts.size();
		for (auto opt : opts) {
			cout << " " << opt.gain;
		}
		cout << endl;
	}
	int gnetID = vps[best_vpin].gnetID;
	Gnet cleanGnet = _gnets[gnetID];
	assert(cleanGnet.isGlobal());
	Vpin vp = vps[best_vpin];
	int originalWLToL1 = vp.wlToL1; // backup originalWL for later use
	auto res = tmpRtDB.ripUpPin(layout, vp, layer - 1);
	Gnet rippedUpGnet = tmpRtDB._gnets[gnetID];
#define RADIUS 3
	double wlAdded0 = 0.0;
	double minmhd0 = 0.0;
	int margin0 = 0;
	int margin2 = 0;
	std::normal_distribution<double> randn(0.0, stdev);

	for (int trial = 0; trial < maxTrials; trial++) {
		auto vp0 = Vpin(vp);
		double rnd = randn(rng);
		int d0 = round(rnd / layout._cellHeight);
		vp0.yCoord += d0;
		minmhd0 = INFINITY;
		wlAdded0 = INFINITY;
		auto goals0 = vp0.gnetGrids;
		for (auto goal : goals0) {
			double mhd = fabs(vp0.xCoord - goal._x) + fabs(vp0.yCoord - goal._y)
					+ fabs(vp0.zCoord - goal._z);
			if (mhd < minmhd0)
				minmhd0 = mhd;
		}
		if (vp0.yCoord < 0
				|| vp0.yCoord > static_cast<int>(layout._numTilesY) - 1)
			continue;
		auto vp1 = Vpin(vps[vp0.matchingVpinIdx]);
		auto res2 = tmpRtDB.rerouteNet(layout, vp0,
				vps[vp0.matchingVpinIdx].gnetGrids, 0, layer - 1, 15, false,
				false, false);
		r0++;
		auto bannedPts = vp0.gnetGrids;
		for (auto pt : std::get<4>(res2)) {
			bannedPts.insert(pt);
		}
		if (std::get<0>(res2) != INFINITY) {
			wlAdded0 = std::get<0>(res2);
			margin0 = std::get<3>(res2);
		} else {
			wlAdded0 = INFINITY;
			continue;
		}
		//	cout << "(Reroute) Failed!" << endl;
		auto res3 = tmpRtDB.rerouteNet(layout, vp0, vps[vp0.matchingVpinIdx],
				layer, 8, 15, false, false);
		// tmpRtDB.getTotalWL();

		r2++;
		double wlAdded;
		if (std::get<0>(res3) != INFINITY) {
			wlAdded = wlAdded0 + std::get<0>(res3);
			margin2 = std::get<3>(res3);
		} else {
			wlAdded = INFINITY;
			continue;
		}

		double xcost = wlAdded - std::get<3>(res);
		ObfusOption opt;
		opt.gain = gain;
		opt.gnetID = gnetID;
		opt.DiMO = 0;
		opt.xCost = xcost;
		opt.d0 = d0;
		opt.d1 = 0;
		opt.margin0 = margin0;
		opt.margin1 = 0;
		opt.margin2 = margin2;
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
		if (tmpRtDB.isDirty()) {
			tmpRtDB.restoreNet(gnetID, rippedUpGnet, _gnets[gnetID]._ruEdges,
					_gnets[gnetID]._ruVias, true);
			_dirty = true; // not restored to clean yet.
		}
		break;
	} // for trial
	if (tmpRtDB.isDirty()) {
		tmpRtDB.restoreNet(gnetID, cleanGnet, _gnets[gnetID]._cleanEdges,
				_gnets[gnetID]._cleanVias, true);
	}
	assert(_gnets[gnetID] == cleanGnet);
	for (auto opt : opts) {
		cout << opt.toString() << endl;
	}
	cout << " Reroute count: R0 = " << r0 << ", R1 = " << r1 << ", R2 = " << r2
			<< " Total = " << r0 + r1 + r2 << endl;
	return opts.size() > 0 ?
			make_pair(opts[0], true) : make_pair(ObfusOption(), false);
}
