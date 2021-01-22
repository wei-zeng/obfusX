/*
 * =====================================================================================
 *
 *       Filename:  main.cpp
 *
 *    Description:  Main file
 *
 *        Version:  1.0
 *        Created:  11/01/2014 16:56:37
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Daohang Shi, Jackson Melchert, Wei Zeng
 *
 * =====================================================================================
 */
#include <cstdio>
#include <sys/param.h>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include "parser.h"
#include <map>
#include "lefdef/DefWriter.h"
#include "def/defwWriter.hpp"
#include "shap.h"

bool SortByMaxSHAP(std::tuple<size_t, vector<double>, vector<double>> vp1,
		std::tuple<size_t, vector<double>, vector<double>> vp2) {
	double max_shap1 = *max_element(get<1>(vp1).begin(), get<1>(vp1).end());
	double max_shap2 = *max_element(get<1>(vp2).begin(), get<1>(vp2).end());
	return max_shap1 > max_shap2;
}

std::map<size_t, vector<double>> readSHAP(string fileName) {
	ifstream fin;
	fin.open(fileName);
	size_t vpinID;
	string line;
	map<size_t, vector<double>> mapSHAP;
	char cm; // comma
	double shap0, shap1, shap2, shap3, shap4, shap5, shap6, shap7, shap8;
	while (getline(fin, line)) {
		istringstream iss(line);
		iss >> vpinID >> cm >> shap0 >> cm >> shap1 >> cm >> shap2 >> cm
				>> shap3 >> cm >> shap4 >> cm >> shap5 >> cm >> shap6 >> cm
				>> shap7 >> cm >> shap8;
		// cout << gnetID << " " << dimo6 << endl;
		std::vector<double> vec = { shap0, shap1, shap2, shap3, shap4, shap5,
				shap6, shap7, shap8 };
		mapSHAP[vpinID] = vec;
	}
	fin.close();
	return mapSHAP;
}

void writeSHAP(string fileName,
		vector<tuple<size_t, vector<double>, vector<double>>> tupleSHAP) {
	ofstream fout;
	fout.open(fileName);
	string line;
	for (auto &s : tupleSHAP) {
		fout << get<0>(s);
		auto &vec = get<1>(s);
		for (size_t i = 0; i < vec.size(); i++) {
			fout << "," << vec[i];
		}
		fout << endl;
	}
	fout.close();
}
#define VERSION "v1.1.210121"
int main(int argc, char **argv) {
	char *designName = nullptr;
	char *auxFile = nullptr;
	char *lefFile = nullptr;
	char *defFile = nullptr;
	char *rtFile = nullptr;
	char *shapFile = nullptr;
	char *jsonFile = nullptr;
	char *outputRtFile = nullptr;
	char *outputCsvFile = nullptr;
	char *outputDefFile = nullptr;
	double outputPerOH = 0.0;
	int i = 1;
	size_t vp_start = 0;
	size_t vp_end = 0;
	size_t ub_z = 10;
	int layer = 2;
	int maxLayer = 0;
	size_t maxIter = 0;
#ifdef LIFT
	cout << "ObfusX " << VERSION << "\nCompiled with -DLIFT\nPerforming wire lifting\n====================" << endl;
#else
	cout << "ObfusX " << VERSION << "\nCompiled without -DLIFT\nPerforming via perturbation\n====================" << endl;
#endif
	// Read input arguments
	while (i < argc) {
		if (!strcmp(argv[i], "-design")) {
			i++;
			assert(i < argc);
			designName = argv[i++];
		} else if (!strcmp(argv[i], "-layer")) {
			i++;
			assert(i < argc);
			layer = stoi(argv[i++]);
		} else if (!strcmp(argv[i], "-lefFile")) {
			i++;
			assert(i < argc);
			lefFile = argv[i++];
		} else if (!strcmp(argv[i], "-defFile")) {
			i++;
			assert(i < argc);
			defFile = argv[i++];
		} else if (!strcmp(argv[i], "-auxFile")) {
			i++;
			assert(i < argc);
			auxFile = argv[i++];
		} else if (!strcmp(argv[i], "-rtFile")) {
			i++;
			assert(i < argc);
			rtFile = argv[i++];
		} else if (!strcmp(argv[i], "-shap")) {
			i++;
			assert(i < argc);
			shapFile = argv[i++];
		} else if (!strcmp(argv[i], "-json")) {
			i++;
			assert(i < argc);
			jsonFile = argv[i++];
		} else if (!strcmp(argv[i], "-outputRT")) {
			i++;
			assert(i < argc);
			outputRtFile = argv[i++];
		} else if (!strcmp(argv[i], "-maxLayer")) {
			i++;
			assert(i < argc);
			maxLayer = stoi(argv[i++]);
		} else if (!strcmp(argv[i], "-maxIter")) {
			i++;
			assert(i < argc);
			maxIter = stoi(argv[i++]);
		} else if (!strcmp(argv[i], "-layer")) {
			i++;
			assert(i < argc);
			layer = stoi(argv[i++]);
		} else if (!strcmp(argv[i], "-ub_z")) {
			i++;
			assert(i < argc);
			ub_z = stoi(argv[i++]);
		} else if (!strcmp(argv[i], "-outputDEF")) {
			i++;
			assert(i < argc);
			outputDefFile = argv[i++];
		} else if (!strcmp(argv[i], "-outputCSV")) {
			i++;
			assert(i < argc);
			outputCsvFile = argv[i++];
		} else if (!strcmp(argv[i], "-outputPerOH")) {
			i++;
			assert(i < argc);
			outputPerOH = stod(argv[i++]);
		} else {
			cout
					<< "Usage for ISPD '11 design: " << argv[0] << " -design <design_name> -auxFile <aux_file> -rtFile <rt_file>\n";
			cout
					<< "Usage for ISCAS '85 design: " << argv[0] << " -design <design_name> -lefFile <lef_file> -defFile <def_file>\n";
			exit(1);
		}
	}

	if ((!designName || !auxFile || !rtFile)
			&& (!designName || !lefFile || !defFile)) {
		cout
				<< "Usage for ISPD '11 design: " << argv[0] << " -design <design_name> -auxFile <aux_file> -rtFile <rt_file>\n";
		cout
				<< "Usage for ISCAS '85 design: " << argv[0] << " -design <design_name> -lefFile <lef_file> -defFile <def_file>\n";
		exit(1);
	}

	if (designName)
		cout << "Design : " << designName << endl;

	Parser parser, parser1;
	Layout layout;
	LayoutDR layout1(maxLayer);
	if (auxFile) {
		layout.initDesignName(designName);
	} else if (lefFile && defFile) {
		layout1.initDesignName(designName);
	}

	// Read design
	if (auxFile) {
		parser.ReadAux(auxFile);
		parser.ReadNode(layout);
		parser.ReadPlace(layout);
		parser.ReadShape(layout);
		parser.ReadRouteConfig(layout);
		parser.ReadNet(layout);

		RoutingDB routingDB(designName, layout);

		routingDB.initGlobalNets(layout);

		routingDB.initEdges(layout);
		routingDB.initGlobalEdgeProfile(layout);

// Read routing file
		if (rtFile) {

			routingDB.readGlobalWires(layout, rtFile);

			routingDB.initRoutingTreeForAllGnets();

			routingDB.updateEdgeDemands();

			int init_wl = routingDB.getTotalWL();
			int nextFileMark = 1;
			routingDB.getTotalWLByDemand();
			if (outputCsvFile)
				routingDB.outputCSV(layout, layer - 1, outputCsvFile);

			auto vpins = routingDB.getVpins(layout, layer - 1, true, false);
			std::unordered_map<Gcell, size_t, HashGcell3d> uMapVpin;
			for (size_t i = 0; i < vpins.size(); i++) {
				auto &vp = vpins[i];
				uMapVpin[Gcell(vp.gnetID, vp.xCoord, vp.yCoord)] = i;
			}
			SHAP shap;
			shap.init();
			shap.read_model(jsonFile);
			double shap_base = shap.base_value();

			vector<bool> no_better_move;
			int settled = 0;
			no_better_move.assign(vpins.size(), false);
			vector<tuple<size_t, vector<double>, vector<double>>> vpinID2SHAP;
			// tuple: (index in vpins, shap vector, feature vector)

			// calc shap
			map<size_t, vector<double>> shap_vals; // precalculated SHAP values
			if (shapFile == nullptr) {
				cout << "Calculating Initial SHAP" << endl;
			} else {
				shap_vals = readSHAP(string(shapFile));
			}
			for (size_t i = 0; i < vpins.size(); i++) {
				if (shapFile == nullptr) {
					if ((i + 1) % (vpins.size() / 100) == 0) {
						cout << "." << flush;
					}
				}
				auto &vp = vpins[i];
				if (vp.matchingVpinIdx == -1)
					continue;
				if (vp.isPIO)
					continue;
				auto &mvp = vpins[vp.matchingVpinIdx];

				double diffVpinX = fabs(vp.xCoord - mvp.xCoord)
						* layout.getCellWidth();
				double diffVpinY = fabs(vp.yCoord - mvp.yCoord)
						* layout.getCellHeight();
				double manhattanVpin = diffVpinX + diffVpinY;
				double totalPinX = 0, totalPinY = 0;
				double totalInputArea = 0, totalOutputArea = 0;
				int inputPinCount = 0, outputPinCount = 0;
				for (size_t j = 0; j < vp.pins.size(); ++j) {
					totalPinX += vp.pins[j]._x;
					totalPinY += vp.pins[j]._y;
					if (vp.pinTypes[j] == CI || vp.pinTypes[j] == PI) {
						totalInputArea += vp.cellAreas[j];
						++inputPinCount;
					} else if (vp.pinTypes[j] == CO || vp.pinTypes[j] == PO) {
						totalOutputArea += vp.cellAreas[j];
						++outputPinCount;
					}
				}
				double vp_pinX = (
						vp.pins.size() > 0 ? totalPinX / vp.pins.size() : 0);
				double vp_pinY = (
						vp.pins.size() > 0 ? totalPinY / vp.pins.size() : 0);
				double vp_avgInputArea = (
						inputPinCount > 0 ? totalInputArea / inputPinCount : 0);
				double vp_avgOutputArea = (
						outputPinCount > 0 ?
								totalOutputArea / outputPinCount : 0);
				totalPinX = 0;
				totalPinY = 0;
				totalInputArea = 0;
				totalOutputArea = 0;
				inputPinCount = 0;
				outputPinCount = 0;
				for (size_t j = 0; j < mvp.pins.size(); ++j) {
					totalPinX += mvp.pins[j]._x;
					totalPinY += mvp.pins[j]._y;
					if (mvp.pinTypes[j] == CI || mvp.pinTypes[j] == PI) {
						totalInputArea += mvp.cellAreas[j];
						++inputPinCount;
					} else if (mvp.pinTypes[j] == CO || mvp.pinTypes[j] == PO) {
						totalOutputArea += mvp.cellAreas[j];
						++outputPinCount;
					}
				}
				double mvp_avgInputArea = (
						inputPinCount > 0 ? totalInputArea / inputPinCount : 0);
				double mvp_avgOutputArea = (
						outputPinCount > 0 ?
								totalOutputArea / outputPinCount : 0);
				double mvp_pinX = (
						mvp.pins.size() > 0 ? totalPinX / mvp.pins.size() : 0);
				double mvp_pinY = (
						mvp.pins.size() > 0 ? totalPinY / mvp.pins.size() : 0);

				double diffPinX = fabs(vp_pinX - mvp_pinX);
				double diffPinY = fabs(vp_pinY - mvp_pinY);
				double manhattanPin = diffPinX + diffPinY;
				double wlToL1 = vp.wlToL1 + mvp.wlToL1;
				double totalArea = vp_avgInputArea + vp_avgOutputArea
						+ mvp_avgInputArea + mvp_avgOutputArea;
				double diffArea = vp_avgOutputArea + mvp_avgOutputArea
						- vp_avgInputArea - mvp_avgInputArea;
				vector<double> featureVec( { diffVpinX, diffVpinY, diffPinX,
						diffPinY, manhattanVpin, manhattanPin, wlToL1,
						totalArea, diffArea });
				vector<double> shap_val;
				if (shapFile == nullptr) {
					shap_val = shap.eval(featureVec);
				} else {
					shap_val = shap_vals[i];
				}
				vpinID2SHAP.push_back(make_tuple(i, shap_val, featureVec));
			}
			if (shapFile == nullptr) {
				cout << endl;
				cout << "Writing SHAP file" << endl;
				writeSHAP(
						string(designName) + string("_") + to_string(layer)
								+ string(".shap"), vpinID2SHAP);
			}
			for (size_t iter = 0; iter < maxIter; iter++) {
				// find best vpin and axis to perturb
				auto sortedVpinIDbySHAP = vpinID2SHAP;
				sort(sortedVpinIDbySHAP.begin(), sortedVpinIDbySHAP.end(),
						SortByMaxSHAP);
				int best_vpin = -1;
				int axis = 0; // 0 : None, 1: X-axis, 2: Y-axis
				double mo_;
				vector<double> featVec; // feature vector of best candidate
				int matching_vpin = -1;

				cout << "<I> Iter " << iter + 1 << " Settled:" << settled << "/"
						<< vpins.size() << endl;
				for (size_t sorted_i = 0; sorted_i < sortedVpinIDbySHAP.size();
						sorted_i++) {
					size_t i = get<0>(sortedVpinIDbySHAP[sorted_i]);
					auto vp = vpins[i];
					auto &shap_val = get<1>(sortedVpinIDbySHAP[sorted_i]);
					double max_shap = *max_element(shap_val.begin(),
							shap_val.end());
					if (!no_better_move[i]
							&& !(max_shap > 0
									&& (max_shap == shap_val[0]
											|| max_shap == shap_val[1]
											|| max_shap == shap_val[4]))) {
						no_better_move[i] = true;
						settled++;
						continue;
					}
					if (!no_better_move[i]) {
						if (layer % 2 == 1 && shap_val[0] >= shap_val[1]) {
							axis = 1;
							cout << i << ": [0] " << shap_val[0] << " [1] "
									<< shap_val[1] << " [4] " << shap_val[4]
									<< endl;
						} else if (layer % 2 == 0
								&& shap_val[1] >= shap_val[0]) {
							axis = 2;
							cout << i << ": [0] " << shap_val[0] << " [1] "
									<< shap_val[1] << " [4] " << shap_val[4]
									<< endl;
						} else {
							no_better_move[i] = true;
							settled++;
							continue;
						}
						best_vpin = i;
						matching_vpin = vp.matchingVpinIdx;
						featVec = get<2>(sortedVpinIDbySHAP[sorted_i]);
						mo_ = std::accumulate(shap_val.begin(), shap_val.end(),
								shap_base);
					}
					if (best_vpin != -1)
						break;
				}
				if (best_vpin == -1)
					break;
				cout << "Best Vpin: " << best_vpin << "@"
						<< routingDB.getGnets()[vpins[best_vpin].gnetID].getName()
						<< " " << "Matching " << matching_vpin << ", " << " ("
						<< vpins[best_vpin].xCoord << ","
						<< vpins[best_vpin].yCoord << ","
						<< vpins[best_vpin].zCoord << ")" << endl;

				auto r0 = routingDB.traverseObfusOpts(layout, layer, vpins,
						best_vpin, featVec, shap, shap_base, mo_);
				auto r1 = routingDB.traverseObfusOpts(layout, layer, vpins,
						matching_vpin, featVec, shap, shap_base, mo_);
				if (r0.second && (!r1.second || !(r1.first > r0.first))) {
					auto gnetID = r0.first.gnetID;
					auto res = routingDB.ripUpPin(layout, vpins[best_vpin],
							layer - 1);
					auto &vp0 = vpins[best_vpin], &vp1 = vpins[matching_vpin];
					uMapVpin.erase(Gcell(vp0.gnetID, vp0.xCoord, vp0.yCoord));
					vp0.yCoord += r0.first.d0;
					uMapVpin[Gcell(vp0.gnetID, vp0.xCoord, vp0.yCoord)] =
							best_vpin;

					auto res0 = routingDB.rerouteNet(layout, vp0, vp1.gnetGrids,
							0, layer - 1, r0.first.margin0, true, true,
							!vp0.hasOriginalRoot);
					auto res2 = routingDB.rerouteNet(layout, vp0, vp1, layer, 8,
							r0.first.margin2, true, true);

					cout << "NewEntry: " << "Net " << vp0.gnetID << " "
							<< vp0.xCoord * layout.getCellWidth() << " "
							<< vp0.yCoord * layout.getCellHeight() << " "
							<< vp0.wlToL1 << endl;

					auto newVpins = routingDB.getVpins(layout, vp0.gnetID,
							vp0.zCoord, true, false);
					for (auto &nvp : newVpins) {
						size_t idx = uMapVpin.at(
								Gcell(vp0.gnetID, nvp.xCoord, nvp.yCoord));
						vpins[idx].wlToL1 = nvp.wlToL1;
						vpins[idx].gnetGrids = nvp.gnetGrids;
						vpins[idx].gnetVertices = nvp.gnetVertices;
						vpins[idx].hasOriginalRoot = nvp.hasOriginalRoot;

						auto &vp = vpins[idx];
						auto &mvp = vpins[vp.matchingVpinIdx];

						double diffVpinX = fabs(vp.xCoord - mvp.xCoord)
								* layout.getCellWidth();
						double diffVpinY = fabs(vp.yCoord - mvp.yCoord)
								* layout.getCellHeight();
						double manhattanVpin = diffVpinX + diffVpinY;
						double totalPinX = 0, totalPinY = 0;
						double totalInputArea = 0, totalOutputArea = 0;
						int inputPinCount = 0, outputPinCount = 0;
						for (size_t j = 0; j < vp.pins.size(); ++j) {
							totalPinX += vp.pins[j]._x;
							totalPinY += vp.pins[j]._y;
							if (vp.pinTypes[j] == CI || vp.pinTypes[j] == PI) {
								totalInputArea += vp.cellAreas[j];
								++inputPinCount;
							} else if (vp.pinTypes[j] == CO
									|| vp.pinTypes[j] == PO) {
								totalOutputArea += vp.cellAreas[j];
								++outputPinCount;
							}
						}
						double vp_pinX = (
								vp.pins.size() > 0 ?
										totalPinX / vp.pins.size() : 0);
						double vp_pinY = (
								vp.pins.size() > 0 ?
										totalPinY / vp.pins.size() : 0);
						double vp_avgInputArea = (
								inputPinCount > 0 ?
										totalInputArea / inputPinCount : 0);
						double vp_avgOutputArea = (
								outputPinCount > 0 ?
										totalOutputArea / outputPinCount : 0);
						totalPinX = 0;
						totalPinY = 0;
						totalInputArea = 0;
						totalOutputArea = 0;
						inputPinCount = 0;
						outputPinCount = 0;
						for (size_t j = 0; j < mvp.pins.size(); ++j) {
							totalPinX += mvp.pins[j]._x;
							totalPinY += mvp.pins[j]._y;
							if (mvp.pinTypes[j] == CI
									|| mvp.pinTypes[j] == PI) {
								totalInputArea += mvp.cellAreas[j];
								++inputPinCount;
							} else if (mvp.pinTypes[j] == CO
									|| mvp.pinTypes[j] == PO) {
								totalOutputArea += mvp.cellAreas[j];
								++outputPinCount;
							}
						}
						double mvp_avgInputArea = (
								inputPinCount > 0 ?
										totalInputArea / inputPinCount : 0);
						double mvp_avgOutputArea = (
								outputPinCount > 0 ?
										totalOutputArea / outputPinCount : 0);
						double mvp_pinX = (
								mvp.pins.size() > 0 ?
										totalPinX / mvp.pins.size() : 0);
						double mvp_pinY = (
								mvp.pins.size() > 0 ?
										totalPinY / mvp.pins.size() : 0);

						double diffPinX = fabs(vp_pinX - mvp_pinX);
						double diffPinY = fabs(vp_pinY - mvp_pinY);
						double manhattanPin = diffPinX + diffPinY;
						double wlToL1 = vp.wlToL1 + mvp.wlToL1;
						double totalArea = vp_avgInputArea + vp_avgOutputArea
								+ mvp_avgInputArea + mvp_avgOutputArea;
						double diffArea = vp_avgOutputArea + mvp_avgOutputArea
								- vp_avgInputArea - mvp_avgInputArea;
						vector<double> featureVec( { diffVpinX, diffVpinY,
								diffPinX, diffPinY, manhattanVpin, manhattanPin,
								wlToL1, totalArea, diffArea });
						vector<double> shap_val;
						shap_val = shap.eval(featureVec);
						// update shap values
						get<1>(vpinID2SHAP[idx]) = shap_val;
						// update feature vector
						get<2>(vpinID2SHAP[idx]) = featureVec;
						//cout << "After RRR: " << featureVec[1] << " " << featureVec[4] << " " << featureVec[6] << endl;
					}
					int wl = routingDB.getTotalWL();
					routingDB.getTotalWLByDemand();
					if (outputPerOH > 0
							&& wl / (double) init_wl
									>= 1 + outputPerOH * nextFileMark) {
						cout << "OutputFileMark: " << nextFileMark << endl;
						routingDB.outputCSV(layout, layer - 1,
								string(outputCsvFile) + to_string(nextFileMark)
										+ string(".csv"));
						nextFileMark =
								max(nextFileMark,
										(int) ((wl / (double) init_wl - 1)
												/ outputPerOH)) + 1;
					}
					auto &gnet = routingDB.getGnets()[vpins[best_vpin].gnetID];
					gnet.resetCleanEdge();
					gnet.resetCleanVia();
					gnet.resetRUEdge();
					gnet.resetRUVia();
				} // if valid obfus option
				else if (r1.second) { // matching vpin is better
					auto gnetID = r1.first.gnetID;
					auto res = routingDB.ripUpPin(layout, vpins[matching_vpin],
							layer - 1);
					// cout << "WL reduced = " << std::get<3>(res) << endl;
					auto &vp0 = vpins[matching_vpin], &vp1 = vpins[best_vpin];
					uMapVpin.erase(Gcell(vp0.gnetID, vp0.xCoord, vp0.yCoord));
					vp0.yCoord += r1.first.d0;
					uMapVpin[Gcell(vp0.gnetID, vp0.xCoord, vp0.yCoord)] =
							matching_vpin;

					auto res0 = routingDB.rerouteNet(layout, vp0, vp1.gnetGrids,
							0, layer - 1, r1.first.margin0, true, true,
							!vp0.hasOriginalRoot);
					auto res2 = routingDB.rerouteNet(layout, vp0, vp1, layer, 8,
							r1.first.margin2, true, true);

					cout << "NewEntry: " << "Net " << vp0.gnetID << " "
							<< vp0.xCoord * layout.getCellWidth() << " "
							<< vp0.yCoord * layout.getCellHeight() << " "
							<< vp0.wlToL1 << endl;

					auto newVpins = routingDB.getVpins(layout, vp0.gnetID,
							vp0.zCoord, true, false);
					for (auto &nvp : newVpins) {
						size_t idx = uMapVpin.at(
								Gcell(vp0.gnetID, nvp.xCoord, nvp.yCoord));
						vpins[idx].wlToL1 = nvp.wlToL1;
						vpins[idx].gnetGrids = nvp.gnetGrids;
						vpins[idx].gnetVertices = nvp.gnetVertices;
						vpins[idx].hasOriginalRoot = nvp.hasOriginalRoot;

						auto &vp = vpins[idx];
						auto &mvp = vpins[vp.matchingVpinIdx];

						double diffVpinX = fabs(vp.xCoord - mvp.xCoord)
								* layout.getCellWidth();
						double diffVpinY = fabs(vp.yCoord - mvp.yCoord)
								* layout.getCellHeight();
						double manhattanVpin = diffVpinX + diffVpinY;
						double totalPinX = 0, totalPinY = 0;
						double totalInputArea = 0, totalOutputArea = 0;
						int inputPinCount = 0, outputPinCount = 0;
						for (size_t j = 0; j < vp.pins.size(); ++j) {
							totalPinX += vp.pins[j]._x;
							totalPinY += vp.pins[j]._y;
							if (vp.pinTypes[j] == CI || vp.pinTypes[j] == PI) {
								totalInputArea += vp.cellAreas[j];
								++inputPinCount;
							} else if (vp.pinTypes[j] == CO
									|| vp.pinTypes[j] == PO) {
								totalOutputArea += vp.cellAreas[j];
								++outputPinCount;
							}
						}
						double vp_pinX = (
								vp.pins.size() > 0 ?
										totalPinX / vp.pins.size() : 0);
						double vp_pinY = (
								vp.pins.size() > 0 ?
										totalPinY / vp.pins.size() : 0);
						double vp_avgInputArea = (
								inputPinCount > 0 ?
										totalInputArea / inputPinCount : 0);
						double vp_avgOutputArea = (
								outputPinCount > 0 ?
										totalOutputArea / outputPinCount : 0);
						totalPinX = 0;
						totalPinY = 0;
						totalInputArea = 0;
						totalOutputArea = 0;
						inputPinCount = 0;
						outputPinCount = 0;
						for (size_t j = 0; j < mvp.pins.size(); ++j) {
							totalPinX += mvp.pins[j]._x;
							totalPinY += mvp.pins[j]._y;
							if (mvp.pinTypes[j] == CI
									|| mvp.pinTypes[j] == PI) {
								totalInputArea += mvp.cellAreas[j];
								++inputPinCount;
							} else if (mvp.pinTypes[j] == CO
									|| mvp.pinTypes[j] == PO) {
								totalOutputArea += mvp.cellAreas[j];
								++outputPinCount;
							}
						}
						double mvp_avgInputArea = (
								inputPinCount > 0 ?
										totalInputArea / inputPinCount : 0);
						double mvp_avgOutputArea = (
								outputPinCount > 0 ?
										totalOutputArea / outputPinCount : 0);
						double mvp_pinX = (
								mvp.pins.size() > 0 ?
										totalPinX / mvp.pins.size() : 0);
						double mvp_pinY = (
								mvp.pins.size() > 0 ?
										totalPinY / mvp.pins.size() : 0);

						double diffPinX = fabs(vp_pinX - mvp_pinX);
						double diffPinY = fabs(vp_pinY - mvp_pinY);
						double manhattanPin = diffPinX + diffPinY;
						double wlToL1 = vp.wlToL1 + mvp.wlToL1;
						double totalArea = vp_avgInputArea + vp_avgOutputArea
								+ mvp_avgInputArea + mvp_avgOutputArea;
						double diffArea = vp_avgOutputArea + mvp_avgOutputArea
								- vp_avgInputArea - mvp_avgInputArea;
						vector<double> featureVec( { diffVpinX, diffVpinY,
								diffPinX, diffPinY, manhattanVpin, manhattanPin,
								wlToL1, totalArea, diffArea });
						vector<double> shap_val;
						shap_val = shap.eval(featureVec);
						// update shap values
						get<1>(vpinID2SHAP[idx]) = shap_val;
						// update feature vector
						get<2>(vpinID2SHAP[idx]) = featureVec;
						//cout << "After RRR: " << featureVec[1] << " " << featureVec[4] << " " << featureVec[6] << endl;
					}
					int wl = routingDB.getTotalWL();
					routingDB.getTotalWLByDemand();
					if (outputPerOH > 0
							&& wl / (double) init_wl
									>= 1 + outputPerOH * nextFileMark) {
						cout << "OutputFileMark: " << nextFileMark << endl;
						routingDB.outputCSV(layout, layer - 1,
								string(outputCsvFile) + to_string(nextFileMark)
										+ string(".csv"));
						nextFileMark =
								max(nextFileMark,
										(int) ((wl / (double) init_wl - 1)
												/ outputPerOH)) + 1;
					}
					auto &gnet = routingDB.getGnets()[vpins[best_vpin].gnetID];
					gnet.resetCleanEdge();
					gnet.resetCleanVia();
					gnet.resetRUEdge();
					gnet.resetRUVia();
				} else { // no valid move
					if (!no_better_move[best_vpin]) {
						no_better_move[best_vpin] = true;
						settled++;
					}
					if (!no_better_move[matching_vpin]) {
						no_better_move[matching_vpin] = true;
						settled++;
					}
				}
			} // for iter
			if (outputRtFile) {
				routingDB.writeGlobalWires(layout, outputRtFile);
			}
			if (outputCsvFile)
				routingDB.outputCSV(layout, layer - 1,
						string(outputCsvFile) + string("final.csv"));

			shap.fin();
		} // if (rtFile)
	} else if (lefFile && defFile) {
		string defFile_str(defFile);
		parser1.ReadLEFDEF(layout1, lefFile, defFile);
		parser1.ReadNode(layout1);
		parser1.ReadRouteConfig(layout1, ub_z);
		parser1.ReadNet(layout1);

		RoutingDB_DR routingDB(designName, layout1);

		routingDB.initGlobalNets(layout1);

		routingDB.initEdges(layout1);
		routingDB.initGlobalEdgeProfile(layout1);
		routingDB.readGlobalWires(layout1);

		routingDB.initRoutingTreeForAllGnets();
		if (outputCsvFile)
			routingDB.outputCSV(layout1, layer - 1, outputCsvFile);
		routingDB.updateEdgeDemands(layout1);
		int init_wl = routingDB.getTotalWL();
		int nextFileMark = 1;
#ifdef LIFT
		auto vpins = routingDB.getVpins(layout1, layer - 2, false, true);
		auto vpins_orig = routingDB.getVpins(layout1, layer - 1, false, true);
		unordered_set<int> vpins_orig_gnetID;
		for (auto &vp : vpins_orig) {
			vpins_orig_gnetID.insert(vp.gnetID);
		}
#else
		auto vpins = routingDB.getVpins(layout1, layer - 1, false, true);
#endif
		std::unordered_map<Gcell, size_t, HashGcell3d> uMapVpin;
		for (size_t i = 0; i < vpins.size(); i++) {
			auto &vp = vpins[i];
			uMapVpin[Gcell(vp.xCoord, vp.yCoord, vp.zCoord)] = i;
		}
		// Initialize shap package
		SHAP shap;
		shap.init();
		shap.read_model(jsonFile);
		double shap_base = shap.base_value();

#ifdef LIFT
		vector<bool> lifted;
		lifted.assign(vpins.size(), false);
#else
		vector<bool> no_better_move;
		no_better_move.assign(vpins.size(), false);
#endif
		for (size_t iter = 0; iter < maxIter; iter++) {
			// find best vpin and axis to perturb
			int best_vpin = -1;
			int axis = 0; // 0 : None, 1: X-axis, 2: Y-axis
			vector<double> featVec; // feature vector of best candidate
			int matching_vpin = -1;

			cout << "<I> Iter " << iter + 1 << endl;
			double curr_max = -INFINITY; // for perturbing
			double curr_mo_ = INFINITY;
			double curr_gain_ = 0; // for lifting
			for (size_t i = 0; i < vpins.size(); i++) {
				auto &vp = vpins[i];
				if (vp.matchingVpinIdx == -1)
					continue;
				if (vp.isPIO)
					continue;
				auto &mvp = vpins[vp.matchingVpinIdx];

				double diffVpinX = fabs(vp.xCoord - mvp.xCoord);
				double diffVpinY = fabs(vp.yCoord - mvp.yCoord);
				double manhattanVpin = diffVpinX + diffVpinY;
				double totalPinX = 0, totalPinY = 0;
				double totalInputArea = 0, totalOutputArea = 0;
				int inputPinCount = 0, outputPinCount = 0;
				for (size_t j = 0; j < vp.pins.size(); ++j) {
					totalPinX += vp.pins[j]._x;
					totalPinY += vp.pins[j]._y;
					if (vp.pinTypes[j] == CI || vp.pinTypes[j] == PI) {
						totalInputArea += vp.cellAreas[j];
						++inputPinCount;
					} else if (vp.pinTypes[j] == CO || vp.pinTypes[j] == PO) {
						totalOutputArea += vp.cellAreas[j];
						++outputPinCount;
					}
				}
				double vp_pinX = (
						vp.pins.size() > 0 ? totalPinX / vp.pins.size() : 0);
				double vp_pinY = (
						vp.pins.size() > 0 ? totalPinY / vp.pins.size() : 0);
				double vp_avgInputArea = (
						inputPinCount > 0 ? totalInputArea / inputPinCount : 0);
				double vp_avgOutputArea = (
						outputPinCount > 0 ?
								totalOutputArea / outputPinCount : 0);
				totalPinX = 0;
				totalPinY = 0;
				totalInputArea = 0;
				totalOutputArea = 0;
				inputPinCount = 0;
				outputPinCount = 0;
				for (size_t j = 0; j < mvp.pins.size(); ++j) {
					totalPinX += mvp.pins[j]._x;
					totalPinY += mvp.pins[j]._y;
					if (mvp.pinTypes[j] == CI || mvp.pinTypes[j] == PI) {
						totalInputArea += mvp.cellAreas[j];
						++inputPinCount;
					} else if (mvp.pinTypes[j] == CO || mvp.pinTypes[j] == PO) {
						totalOutputArea += mvp.cellAreas[j];
						++outputPinCount;
					}
				}
				double mvp_avgInputArea = (
						inputPinCount > 0 ? totalInputArea / inputPinCount : 0);
				double mvp_avgOutputArea = (
						outputPinCount > 0 ?
								totalOutputArea / outputPinCount : 0);
				double mvp_pinX = (
						mvp.pins.size() > 0 ? totalPinX / mvp.pins.size() : 0);
				double mvp_pinY = (
						mvp.pins.size() > 0 ? totalPinY / mvp.pins.size() : 0);

				double diffPinX = fabs(vp_pinX - mvp_pinX);
				double diffPinY = fabs(vp_pinY - mvp_pinY);
				double manhattanPin = diffPinX + diffPinY;
				double wlToL1 = vp.wlToL1 + mvp.wlToL1;
				for (int z = vp.zCoord; z < layer - 1; z++) {
					wlToL1 += Gcell(0, 0, z).viaWeights[z];
				}
				double totalArea = vp_avgInputArea + vp_avgOutputArea
						+ mvp_avgInputArea + mvp_avgOutputArea;
				double diffArea = vp_avgOutputArea + mvp_avgOutputArea
						- vp_avgInputArea - mvp_avgInputArea;
				vector<double> featureVec( { diffVpinX, diffVpinY, diffPinX,
						diffPinY, manhattanVpin, manhattanPin, wlToL1,
						totalArea, diffArea });
				vector<double> shap_val = shap.eval(featureVec);
				double mo_ = std::accumulate(shap_val.begin(), shap_val.end(),
						shap_base); // suffix _ means it's an estimate
				double max_shap = *max_element(shap_val.begin(), shap_val.end());//max(shap_val[0],
						//max(shap_val[1], shap_val[4]));
#ifdef LIFT
				double xcost_ = 1; //0;
				//for (int z = vp.zCoord; z < layer - 1; z++) {
				//	xcost_ += Gcell(0, 0, z).viaWeights[z];
				//}
				if (!lifted[i]
				//&& vpins_orig_gnetID.count(vp.gnetID) == 0
						&& (1 - mo_) / xcost_ > curr_gain_) {
					curr_mo_ = mo_;
					curr_gain_ = (1 - mo_) / xcost_;
					best_vpin = i;
					matching_vpin = vp.matchingVpinIdx;
					featVec = featureVec;
				}
#else
				if (!no_better_move[i] && max_shap > 0 && max_shap > curr_max) {
					if (layer % 2 == 1 && shap_val[0] >= shap_val[1]) {
						axis = 1;
						cout << i << ": [0] " << shap_val[0] << " [1] "
								<< shap_val[1] << " [4] " << shap_val[4]
								<< endl;
						curr_max = max_shap; // preferred direction

					} else if (layer % 2 == 0 && shap_val[1] >= shap_val[0]) {
						axis = 2;
						cout << i << ": [0] " << shap_val[0] << " [1] "
								<< shap_val[1] << " [4] " << shap_val[4]
								<< endl;
						curr_max = max_shap; // preferred direction
					} else {
						continue;
					}
					best_vpin = i;
					matching_vpin = vp.matchingVpinIdx;
					featVec = featureVec;
				}
#endif
			}
			if (best_vpin == -1)
				break;
			// find amount to perturb
			cout << "Best Vpin: " << best_vpin << "@"
					<< routingDB.getGnets()[vpins[best_vpin].gnetID].getName()
					<< " " << "Matching " << matching_vpin << ", " << "MO_ = "
					<< curr_mo_ << " Gain_ = " << curr_gain_ << " ("
					<< vpins[best_vpin].xCoord << "," << vpins[best_vpin].yCoord
					<< "," << vpins[best_vpin].zCoord << ")" << endl;
#ifdef LIFT
			auto &vp = vpins[best_vpin];
			lifted[best_vpin] = true;
			auto res = routingDB.ripUpNet(layout1, vp);
			uMapVpin.erase(Gcell(vp.xCoord, vp.yCoord, vp.zCoord));
			uMapVpin[Gcell(vp.xCoord, vp.yCoord, vp.zCoord)] = best_vpin;
			auto res1 = routingDB.rerouteNet(layout1, vp,
					get<1>(res) /* otherVpinGridsOnBelowSL, banned */, 0,
					vp.zCoord, 7, false, true, !vp.hasOriginalRoot, 5,
					vpins[vp.matchingVpinIdx]);
			assert(get<0>(res1) < INFINITY);
			auto res2 = routingDB.rerouteNet(layout1, vp,
					get<0>(res) /* otherVpinGridsAboveSL */,
					get<2>(res) /* otherVpinVerticesAboveSL */, vp.zCoord + 1,
					maxLayer - 1, 7, false, true, 5, true, 100, layer - 1,
					vpins[vp.matchingVpinIdx]);
			assert(get<0>(res2) < INFINITY);
			routingDB.updateGridOccupation(vp.gnetID, false);
			auto newVpins = routingDB.getVpins(layout1, vp.gnetID, vp.zCoord,
					false, true);
			for (auto &nvp : newVpins) {
				size_t idx = uMapVpin.at(
						Gcell(nvp.xCoord, nvp.yCoord, nvp.zCoord));
				vpins[idx].wlToL1 = nvp.wlToL1;
				vpins[idx].gnetGrids = nvp.gnetGrids;
				vpins[idx].gnetVertices = nvp.gnetVertices;
				vpins[idx].hasOriginalRoot = nvp.hasOriginalRoot;
			}
			int wl = routingDB.getTotalWL();
#else
			auto r = routingDB.traverseObfusOpts(layout1, layer, true,
					best_vpin, axis, featVec, shap, vpins, maxLayer);
			if (r.second) {
				auto &vp = vpins[best_vpin];
				auto res = routingDB.ripUpNet(layout1, vp);
				uMapVpin.erase(Gcell(vp.xCoord, vp.yCoord, vp.zCoord));
				vp.xCoord += (axis == 1 ? r.first.d0 : 0);
				vp.yCoord += (axis == 2 ? r.first.d0 : 0);
				uMapVpin[Gcell(vp.xCoord, vp.yCoord, vp.zCoord)] = best_vpin;
				auto res1 = routingDB.rerouteNet(layout1, vp,
						get<1>(res) /* otherVpinGridsOnBelowSL, banned */, 0,
						vp.zCoord, r.first.margin0, true, true,
						!vp.hasOriginalRoot, 5, vpins[matching_vpin]);
				assert(get<0>(res1) < INFINITY);
				auto res2 = routingDB.rerouteNet(layout1, vp,
						get<0>(res) /* otherVpinGridsAboveSL */,
						get<2>(res) /* otherVpinVerticesAboveSL */,
						vp.zCoord + 1, maxLayer - 1, r.first.margin1, true,
						true, 5, false, 1, vp.zCoord, vpins[matching_vpin]);
				assert(get<0>(res2) < INFINITY);

				routingDB.updateGridOccupation(vp.gnetID, false);

				// update vpins
				auto newVpins = routingDB.getVpins(layout1, vp.gnetID,
						layer - 1, false, true);
				for (auto &nvp : newVpins) {
					size_t idx = uMapVpin.at(
							Gcell(nvp.xCoord, nvp.yCoord, nvp.zCoord));
					vpins[idx].wlToL1 = nvp.wlToL1;
					vpins[idx].gnetGrids = nvp.gnetGrids;
					vpins[idx].gnetVertices = nvp.gnetVertices;
					vpins[idx].hasOriginalRoot = nvp.hasOriginalRoot;
				}
			} else {
				no_better_move[best_vpin] = true;
			}
#endif
			routingDB.getGnets()[vpins[best_vpin].gnetID].resetCleanEdge();
			routingDB.getGnets()[vpins[best_vpin].gnetID].resetCleanVia();
			routingDB.getGnets()[vpins[best_vpin].gnetID].resetRUEdge();
			routingDB.getGnets()[vpins[best_vpin].gnetID].resetRUVia();
		}
		if (outputDefFile) {
			parser1.WriteDEF(layout1, routingDB, string(outputDefFile));
		}
		shap.fin();
		routingDB.getTotalWL();
	}
	return 0;
}
