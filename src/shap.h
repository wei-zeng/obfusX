/*
 * shap.h
 *
 *  Created on: Nov 9, 2019
 *      Author: Wei Zeng
 */

#ifndef SHAP_H_
#define SHAP_H_

#define PY_SSIZE_T_CLEAN
#include "Python.h"
#include "numpy/arrayobject.h"
#include <vector>
#include <string>

class SHAP {
public:
	SHAP() {
	}
	void init();
	void read_model(const std::string &jsonFile);
	std::vector<double> eval(const std::vector<double> &featureVec) const;
	double base_value() const;
	void fin();
private:
	PyObject* _expl_class;
	PyObject* _expl_shap_values_class;
	PyObject* _tree_class;
	PyObject* _forest;
	PyObject* _expl;
	PyObject* _json_load_class;
	PyObject* _json_model;
	PyObject* _np_array_class;

};

#endif /* SHAP_H_ */
