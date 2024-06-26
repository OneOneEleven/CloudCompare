#pragma once

#include <Eigen/Core>
#include <limits> 
#include <iostream>

#include "eigen_tools/nanoflannWrapper.hpp"

namespace qSkeletonLib {


    int argMax(const Eigen::VectorXd& data)
    {
        int argmax = 0;
        int max_dim = std::max(data.rows(), data.cols());
        for (int i = 0; i < max_dim; i++)
            if (data(argmax) < data(i))
                argmax = i;
        return argmax;
    }

    inline bool farthest_sampling_by_sphere(Eigen::MatrixXd& in_cloud, double sample_radius, Eigen::MatrixXd& nodes, Eigen::VectorXi& correspondences)
    {

        correspondences = Eigen::VectorXi::Zero(in_cloud.rows());

        nanoflann_wrapper knn_search(in_cloud);
        std::vector<int> node_list;
        Eigen::VectorXd mindst = Eigen::VectorXd::Constant(in_cloud.rows(), -1); // used as NaN

        for (int i = 0; i < in_cloud.rows(); i++) {

            if (correspondences(i) == 0) {

                mindst(i) = std::numeric_limits<double>::infinity();

                while ((correspondences.array() == 0).any()) {

                    int maxId = argMax(mindst);

                    if (mindst(maxId) == 0)
                        break;

                    std::vector<int> neighbours_id;
                    std::vector<double> neighbours_distances;
                    knn_search.radius_search(in_cloud.row(maxId), sample_radius, neighbours_id, neighbours_distances);

                    bool all_corresp_marked = correspondences(neighbours_id[0]) != 0;
                    for (int j = 0; j < neighbours_id.size(); j++)
                        all_corresp_marked = all_corresp_marked & correspondences(neighbours_id[j]) != 0;

                    if (all_corresp_marked) {
                        mindst(maxId) = 0;
                        break;
                    }

                    node_list.push_back(maxId);
                    for (int j = 0; j < neighbours_id.size(); j++) {
                        if (mindst(neighbours_id[j]) > neighbours_distances[j] || mindst(neighbours_id[j]) == -1) {
                            mindst(neighbours_id[j]) = neighbours_distances[j];
                            correspondences(neighbours_id[j]) = node_list.size();
                        }
                    }
                }
            }
        }

        correspondences = correspondences.array() - 1;
        if ((correspondences.array() == -1).any())
        {
            std::cout << "point without correspondences!!!\n";
            std::cin.get();
        }

        nodes.resize(node_list.size(), 3);
        for (int i = 0; i < node_list.size(); i++)
            nodes.row(i) << in_cloud.row(node_list[i]);

        return true;
    };

    inline bool farthest_sampling_by_knn(Eigen::MatrixXd& in_cloud, int k, Eigen::MatrixXd& nodes, Eigen::VectorXi& correspondences)
    {

        correspondences = Eigen::VectorXi::Zero(in_cloud.rows());

        nanoflann_wrapper knn_search(in_cloud);
        std::vector<int> node_list;
        Eigen::VectorXd mindst = Eigen::VectorXd::Constant(in_cloud.rows(), -1); // used as NaN

        for (int i = 0; i < in_cloud.rows(); i++) {

            if (correspondences(i) == 0) {

                mindst(i) = std::numeric_limits<double>::infinity();

                while ((correspondences.array() == 0).any()) {

                    int maxId = argMax(mindst);

                    if (mindst(maxId) == 0)
                        break;

                    std::vector<int> neighbours_id;
                    std::vector<double> neighbours_distances;
                    //knn_search.radius_search(in_cloud.row(maxId), sample_radius, neighbours_id, neighbours_distances);
                    knn_search.return_k_closest_points(in_cloud.row(maxId), k, neighbours_id, neighbours_distances);

                    bool all_corresp_marked = correspondences(neighbours_id[0]) != 0;
                    for (int j = 0; j < neighbours_id.size(); j++)
                        all_corresp_marked = all_corresp_marked & correspondences(neighbours_id[j]) != 0;

                    if (all_corresp_marked) {
                        mindst(maxId) = 0;
                        break;
                    }

                    node_list.push_back(maxId);
                    for (int j = 0; j < neighbours_id.size(); j++) {
                        if (mindst(neighbours_id[j]) > neighbours_distances[j] || mindst(neighbours_id[j]) == -1) {
                            mindst(neighbours_id[j]) = neighbours_distances[j];
                            correspondences(neighbours_id[j]) = node_list.size();
                        }
                    }
                }
            }
        }

        correspondences = correspondences.array() - 1;
        if ((correspondences.array() == -1).any())
        {
            std::cout << "point without correspondences!!!\n";
            std::cin.get();
        }

        nodes.resize(node_list.size(), 3);
        for (int i = 0; i < node_list.size(); i++)
            nodes.row(i) << in_cloud.row(node_list[i]);

        return true;
    };
}

