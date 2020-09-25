# ObfusX
This is the source code for our ASPDAC '21 paper. Please kindly cite it if it is helpful for your academic research.

W. Zeng, A. Davoodi, and R. O. Topaloglu, "ObfusX: Routing Obfuscation with Explanatory Analysis of a Machine Learning Attack," in *Proc. 26th IEEE/ACM Asia and South Pacific Design Automation Conference (ASP-DAC)*, Jan. 2021.

The code is authored by Wei Zeng, based on ISPD'11 benchmark parser [here](https://jonathoncmagana.github.io/magana/Parser/), and LEF/DEF parser and writer from [jinwookjungs/lefdef_util](https://github.com/jinwookjungs/lefdef_util).

## File description:
`circuits`: contains routed ISCAS'85 designs from [seth-tamu/network_flow_attack](https://github.com/seth-tamu/network_flow_attack), and routed ISPD'11 designs with [NCTU-GR 2.0](https://people.cs.nctu.edu.tw/~whliu/NCTU-GR.htm) in Regular mode. We use [UMich-SimPnR](http://www.ispd.cc/contests/11/ISPD2011_Pl_Files/UMich_SimPLR.tar.bz2) placement for `superblue1`/`5`/`12`/`18` and [NTU-Radiant](http://www.ispd.cc/contests/11/ISPD2011_Pl_Files/NTU_Radiant.tar.bz2) placement for `superblue10`, so that all five designs can be routed overflow-free.  
`CMakeLists.txt`: Installation script for CMake. 
`ISCAS85_JSON`, `ISPD11_JSON`: converted JSON files of trained Bagging model from Weka. They are trained with v-pin pairs from other designs in the same benchmark suite with the same split layer. Available for split layers 3 and 4 for ISCAS '85 designs, and layers 6 and 8 for ISPD '11 designs.  
`lefdef_util`: LEF/DEF parser and writer.  
`src`: Source files related to ObfusX and ISPD'11 benchmark parser.  
`README.md`: This readme file.  
`bookshelf_parser_readme`: The original readme for ISPD '11 bookshelf parser, which ObfusX is based on.

## Installation:
Requirements:
- Linux OS  
- CMake >= 3.12  
- Python >= 3.5
- SHAP library: `pip install shap`

Steps:
```
mkdir build && cd build
cmake ..
make
```
By default, obfusX performs via perturbation. To perform wire lifting instead, add `-DLIFT` to the compiler flag by uncommenting the relevant line in `CMakeLists.txt` before compiling.

## Decompress superblue designs:
Due to large size, we compressed each routed superblue design into a tarball. To decompress, use, e.g.
```
cd circuits
tar xjvf superblue1.tar.bz2
```

## Launch:
-----
The main execuatable `main` takes the following parameters in the format of `./main <name1> <value1> <name2> <value2> ...`:  
`-design`: an identifier of the design, could be any string (required).  
`-layer`: the split layer. E.g. `4` for splitting between M4 and M5 (required).  
`-maxLayer`: the highest metal layer that can be used for lifting and rerouting (required for ISCAS '85 benchmark).  
`-json`: path to the trained Weka Bagging model as a JSON file (required, generated by weka2json in this repository).  
`-maxIter`: maximum number of iterations, (required, could be set to a large number for unlimited iterations).  
`-lefFile`: path to LEF (library) file (required for ISCAS '85 benchmark).  
`-defFile`: path to DEF (design) file (required for ISCAS '85 benchmark).  
`-outputDEF`: (optional) file to write the obfuscated design in DEF (for ISCAS '85 benchmark).  
`-auxFile`: path to aux file in bookshelf format (required for ISPD '11 benchmark).  
`-rtFile`: path to routed wires in the format from ISPD'11 contest (required for ISPD '11 benchmark).  
`-outputRT`: (optional) file to write the obfuscated design in RT (for ISPD '11 benchmark).  
`-shap`: (optional) path to cached initial SHAP values (for ISPD '11 benchmark'). If omitted, initial SHAP values will be calculated and cached to a file with name ending in `.shap` for later use.  
`-outputCSV`: (optional) file to write the information of all v-pins before and after obfuscation.  
-----
Minimal `-maxLayer` values for ISCAS '85 designs with routed layouts in this repo (from [seth-tamu/network_flow_attack](https://github.com/seth-tamu/network_flow_attack)):
|Design|Value|
|---|---|
|`c880` | 7 |
|`c2670`| 9 |
|`c3540`| 7 |
|`c5315` | 8 |
|`c6288` | 6 |
|`c7552` | 7 |
-----
**Examples:**
ISCAS '85 design:
```
./main -design c880 -maxLayer 7 -layer 4 -lefFile ../circuits/NangateOpenCellLibrary.lef -defFile ../circuits/c880_45nm_routing_layer6.def -json ../ISCAS85_JSON/bg_for_880_4.json -maxIter 1000 -outputDEF c880_obfus_M4_lift.def
```

ISPD '11 design:
```
./main -design sb18 -layer 6 -auxFile ../circuits/superblue18/superblue18.aux -rtFile ../circuits/superblue18/sb18.rt -json ../ISPD11_JSON/bg_for_18_6.json -maxIter 1000000 -outputRT sb18_obfus_M6.rt
```
