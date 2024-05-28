#pragma once

#include <Eigen/Core>
#include <vector>

#include <string>
#include <cstdio>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>

namespace qSkeletonLib
{
    inline bool writeGraphOBJ(const Eigen::MatrixXd & V_in, const Eigen::MatrixXi & E_in, const std::string obj_file_name){
        bool verbose = false;

        using namespace std;
        using namespace Eigen;
        assert(V_in.cols() == 3 && "V should have 3 columns");
        ofstream s(obj_file_name);
        if(!s.is_open())
        {
            fprintf(stderr,"IOError: writeOBJ() could not open %s\n",obj_file_name.c_str());
            return false;
        }
        s<<
        V_in.format(IOFormat(FullPrecision,DontAlignCols," ","\n","v ","","","\n"))<<
        (E_in.array()+1).format(IOFormat(FullPrecision,DontAlignCols," ","\n","l ","","","\n"));
        return true;
    };
}
 