/*
 * shap.cpp
 *
 *  Created on: Nov 9, 2019
 *      Author: Wei Zeng
 */
#include <iostream>
#include "shap.h"

using namespace std;

void SHAP::init() {
	Py_Initialize();

	PyObject *module = PyImport_ImportModule("shap");
	assert(module != NULL);

	_expl_class = PyObject_GetAttrString(module, "explainers");
	assert(_expl_class != NULL);
	_expl_class = PyObject_GetAttrString(_expl_class, "tree");
	assert(_expl_class != NULL);
	_tree_class = PyObject_GetAttrString(_expl_class, "Tree");
	assert(_tree_class != NULL);
	_expl_class = PyObject_GetAttrString(_expl_class, "TreeExplainer");
	assert(_expl_class != NULL);

	module = PyImport_ImportModule("json");
	assert(module != NULL);
	_json_load_class = PyObject_GetAttrString(module, "load");
	assert(_json_load_class != NULL);

	module = PyImport_ImportModule("numpy");
	assert(module != NULL);
	_np_array_class = PyObject_GetAttrString(module, "array");
	assert(_np_array_class != NULL);

	cout << "Init successfully." << endl;
}

void SHAP::read_model(const std::string &jsonFile) {
	FILE *fin = fopen(jsonFile.c_str(), "r");
	PyObject *pyfd = PyFile_FromFd(fileno(fin), NULL, "r", -1, NULL, NULL, NULL,
			1);
	PyObject *args = PyTuple_Pack(1, pyfd);
	_json_model = PyObject_Call(_json_load_class, args, NULL);
	fclose(fin);
	assert(_json_model != NULL);
#if 0 // Implementing the following Python code:
	forest = []
	for i in range(len(d)):
	    dd = d[i]
	    forest.append(Tree({'children_left': np.array(dd['children_left']),
	      'children_right': np.array(dd['children_right']),
	      'children_default': np.array(dd['children_left']),
	      'feature': np.array(dd['feature']),
	      'threshold': np.array(dd['threshold']),
	      'value': np.array(dd['value']).reshape(-1,1),
	      'node_sample_weight': np.array(dd['node_sample_weight']),
	}, scaling=1.0/len(d)))
#endif

	Py_ssize_t len_d = PyList_Size(_json_model);
	_forest = PyList_New(len_d);
	assert(_forest != NULL);
	for (Py_ssize_t i = 0; i < len_d; ++i) {
		PyObject *dd = PyList_GetItem(_json_model, i); // a dict: d[i]
		assert(dd != NULL);
		PyObject *dd_chl = PyObject_CallObject(_np_array_class,
				PyTuple_Pack(1, PyDict_GetItemString(dd, "children_left")));
		assert(dd_chl != NULL);
		PyObject *dd_chr = PyObject_CallObject(_np_array_class,
				PyTuple_Pack(1, PyDict_GetItemString(dd, "children_right")));
		assert(dd_chr != NULL);
		PyObject *dd_feature = PyObject_CallObject(_np_array_class,
				PyTuple_Pack(1, PyDict_GetItemString(dd, "feature")));
		assert(dd_feature != NULL);
		PyObject *dd_threshold = PyObject_CallObject(_np_array_class,
				PyTuple_Pack(1, PyDict_GetItemString(dd, "threshold")));
		assert(dd_threshold != NULL);
		args = PyTuple_Pack(2, PyLong_FromLong(-1), PyLong_FromLong(1));
		PyObject *dd_value = PyObject_CallObject(
				PyObject_GetAttrString(
						PyObject_CallObject(_np_array_class,
								PyTuple_Pack(1,
										PyDict_GetItemString(dd, "value"))),
						"reshape"), args);
		assert(dd_value != NULL);
		PyObject *dd_spwt = PyObject_CallObject(_np_array_class,
				PyTuple_Pack(1,
						PyDict_GetItemString(dd, "node_sample_weight")));
		assert(dd_spwt != NULL);

		PyObject *args = PyDict_New(); // a dict: {'children_left' : ..., ...}
		assert(args != NULL);
		PyDict_SetItemString(args, "children_left", dd_chl);
		PyDict_SetItemString(args, "children_right", dd_chr);
		PyDict_SetItemString(args, "children_default", dd_chl);
		PyDict_SetItemString(args, "feature", dd_feature);
		PyDict_SetItemString(args, "threshold", dd_threshold);
		PyDict_SetItemString(args, "value", dd_value);
		PyDict_SetItemString(args, "node_sample_weight", dd_spwt);
		args = PyTuple_Pack(1, args);
		assert(args != NULL);

		PyObject *kwargs = PyDict_New(); // a dict: {'scaling': 1.0 / len_d}
		assert(kwargs != NULL);
		PyDict_SetItemString(kwargs, "scaling",
				PyFloat_FromDouble(1.0 / len_d));
		PyObject *tree = PyObject_Call(_tree_class, args, kwargs);
		assert(tree != NULL);
		PyList_SetItem(_forest, i, tree);
		assert(_forest != NULL);
	}
	_expl = PyObject_CallObject(_expl_class, PyTuple_Pack(1, _forest));
	assert(_expl != NULL);
	_expl_shap_values_class = PyObject_GetAttrString(_expl, "shap_values");
	assert(_expl_shap_values_class != NULL);
}

double SHAP::base_value() const {
	PyObject* expected_value = PyObject_GetAttrString(_expl, "expected_value");
	assert(expected_value != NULL);
	return 1 - PyFloat_AsDouble(expected_value);
}

vector<double> SHAP::eval(const vector<double> &featureVec) const {
	vector<double> sh_value_vec;
	PyObject *list = PyList_New(featureVec.size());
	for (size_t i = 0; i < featureVec.size(); i++) {
		PyList_SetItem(list, i, PyFloat_FromDouble(featureVec[i]));
	}
	PyObject *args = PyTuple_Pack(2, PyLong_FromLong(1),
			PyLong_FromLong(featureVec.size()));
	PyObject *np_array = PyObject_CallObject(
			PyObject_GetAttrString(
					PyObject_CallObject(_np_array_class, PyTuple_Pack(1, list)),
					"reshape"), args);
	PyObject *sh_array_obj = PyObject_CallObject(_expl_shap_values_class,
			PyTuple_Pack(1, np_array));
	for (size_t i = 0; i < featureVec.size(); i++) {
		PyObject *sh_value_obj = PyArray_GETITEM(sh_array_obj,
				PyArray_GETPTR2(sh_array_obj, 0, i));
		assert(sh_value_obj != NULL);
		sh_value_vec.push_back(-PyFloat_AsDouble(sh_value_obj));
	}
	return sh_value_vec;
}

void SHAP::fin() {
	if (Py_IsInitialized()) {
		Py_Finalize();
		cout << "Fin successfully." << endl;
	}
}
