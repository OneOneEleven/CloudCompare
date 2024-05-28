#pragma once

#include <iostream>
#include <Eigen/Core>


namespace qSkeletonLib {

    //https://stackoverflow.com/a/21068014/2562693
    template <typename T>
    inline void cast_to_matrix(std::vector< std::vector <T> > input, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& output)
    {
        const int numRows = input.size();
        const int numCols = input[0].size();

        //output = Eigen::Map<Eigen::Matrix<T, input.size(), input[0].size()>, Eigen::Unaligned>(input.data(), input.size());
        output.resize(numRows, numCols);

        for (int i = 0; i < numRows; i++)
            for (int j = 0; j < numCols; j++)
                output(i, j) = input[i][j];
    }

}



