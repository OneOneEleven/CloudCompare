#include "PointCloudSkeleton.h"
 
#include "graph/graph.hpp"
 
#include "farthest_sampling.hpp"
#include "connect_by_inherit_neigh.hpp"

#include "eigen_tools/nanoflannWrapper.hpp"
#include "eigen_tools/concatenate.hpp"
#include "eigen_tools/sparse.hpp"
#include "eigen_tools/minMax.hpp"

qSkeletonLib::PointCloudSkeleton::PointCloudSkeleton(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Options& opts)
    :_opts(opts),_pointcloud(V),_F(F)
{
}

void qSkeletonLib::PointCloudSkeleton::skeletonize()
{
    init();
    normalization();
    laplacian_contraction();
    contracted_cloud_to_skeleton();
    un_normalization();
}

bool qSkeletonLib::PointCloudSkeleton::init()
{
    // get the nearest neighbours
    nanoflann_wrapper knn_search(_pointcloud);
    int k_for_knn = 30;

    // get the one ring
    for (int i = 0; i < _pointcloud.rows(); i++) 
    {
        // (1) search for k closest points and (2) remove the point itself
        std::vector < int > neighbours_id;
        neighbours_id = knn_search.return_k_closest_points(_pointcloud.row(i), k_for_knn + 1);
        neighbours_id.erase(neighbours_id.begin());

        // look for the one_ring neighbors
        OneRing one_ring(i, neighbours_id, _pointcloud);
        _ringList.push_back(one_ring);
    }

    _initialized = true;
    return true;
}

void qSkeletonLib::PointCloudSkeleton::laplacian_contraction()
{
    if (!_initialized)
        init();

    // store the contraction for each step
    std::vector<double> contraction_history;

    // initialization of: WH, WL, and L
    Eigen::VectorXd ms = one_ring_size(_pointcloud, _ringList, "mean");
    double initWL = 1.0 / (5.0 * ms.mean());

    Eigen::SparseMatrix<double> WL = SparseDiagonalMatrix::Constant(_pointcloud.rows(), initWL);
    Eigen::SparseMatrix<double> WH = SparseDiagonalMatrix::Constant(_pointcloud.rows(), 1);

    // get laplacian
    Eigen::SparseMatrix<double> L = laplacian(_pointcloud, _ringList);

    std::cout << "contraction step : 0\n";
    _contractedPointcloud = solve_contraction(WH, WL, L, _pointcloud);

    // further contraction steps
    int iteration_time = _opts.iteration_time;
    double termination_criteria = _opts.termination_criteria;
    double sl = _opts.sl;
    double WC = _opts.WC;

    Eigen::VectorXd size = one_ring_size(_pointcloud, _ringList, "min");
    Eigen::VectorXd new_size = one_ring_size(_contractedPointcloud, _ringList, "min");
    double contraction_rate = new_size.mean() / size.mean();
    contraction_history.push_back(contraction_rate);
    std::cout << "contraction ratio : " << contraction_history.back() << "\n";
 
    for (int i = 0; i < iteration_time; i++)
    {
        std::cout << "contraction step : " << i + 1 << "\n";

        // update WL
        WL *= sl;

        // update WH
        for (int i = 0; i < _pointcloud.rows(); i++)
            if (WC * size(i) / new_size(i) < _opts.MAX_POSITION_CONSTRAINT_WEIGHT)
                WH.coeffRef(i, i) = WC * size(i) / new_size(i);
            else
            {
                std::cout << "Warning: reached upper limit for WH: " << WC * size(i) / new_size(i) << "\n";
                WH.coeffRef(i, i) = _opts.MAX_POSITION_CONSTRAINT_WEIGHT;
            }

        // set up upper limit
        if (WL.coeffRef(0, 0) * sl > _opts.MAX_LAPLACIAN_CONSTRAINT_WEIGHT)
        {
            std::cout << "Warning: reached upper limit for the WL: " << WL.coeffRef(0, 0) * sl << "\n";
            for (int i = 0; i < _pointcloud.rows(); i++)
                WL.coeffRef(i, i) = _opts.MAX_LAPLACIAN_CONSTRAINT_WEIGHT;
        }

        // recompute L
        L = laplacian(_contractedPointcloud, _ringList);

        // i^th +1 contraction
        Eigen::MatrixXd contracted_pointcloud_temp = solve_contraction(WH, WL, L, _contractedPointcloud);

        // store contraction history and update the _contractedPointcloud if the contraction_rate is good enough
        new_size = one_ring_size(contracted_pointcloud_temp, _ringList, "min");
        contraction_rate = new_size.mean() / size.mean();
        if (contraction_history.rbegin()[0] - contraction_rate < _opts.termination_criteria)
            break;

        contraction_history.push_back(contraction_rate);
        std::cout << "contraction ratio : " << contraction_history.back() << "\n";
        _contractedPointcloud = contracted_pointcloud_temp;
    }
}

bool qSkeletonLib::PointCloudSkeleton::contracted_cloud_to_skeleton()
{
    // turn into a Skeletonization part
    double scale;
    getScale(_pointcloud, scale);

    Eigen::MatrixXd nodes;
    Eigen::VectorXi correspondences;

    if (_opts.use_radius)
    {
        double sample_radius = scale * _opts.sample_radius;
        farthest_sampling_by_sphere(_contractedPointcloud, sample_radius, nodes, correspondences);
    }
    else
    {
        int k = round(_pointcloud.rows() / _opts.sample_ratio);
        farthest_sampling_by_knn(_contractedPointcloud, k, nodes, correspondences);
    }

    Eigen::MatrixXi adjacency_matrix;
    connect_by_inherit_neigh(_pointcloud, nodes, correspondences, _ringList, adjacency_matrix);


    _skeleton = new Graph(nodes, adjacency_matrix);
    _skeleton->init();

    Eigen::VectorXi updated_correspondences = Eigen::VectorXi::Constant(correspondences.rows(), -1);
    std::vector<std::vector <int> > merged_nodes;
    if (_opts.skeleton_editing != 0) {
        merged_nodes = _skeleton->make_tree();

        // update the point correspondence
        for (int k = 0; k < correspondences.rows(); k++)
            for (int i = 0; i < merged_nodes.size(); i++)
                for (int j = 0; j < merged_nodes[i].size(); j++)
                    if (correspondences(k) == merged_nodes[i][j])
                        updated_correspondences(k) = i;
    }
    else {
        updated_correspondences = correspondences;
    }
 
    if ((updated_correspondences.array() == -1).any())
        std::cout << "Warning: bad point correspondence\n'";

    _correspondences = updated_correspondences;

    return true;
}

inline bool qSkeletonLib::PointCloudSkeleton::normalization()
{
    // centre in zero
    _nomalizationDeplacement = (_pointcloud.colwise().minCoeff() + _pointcloud.colwise().maxCoeff()) / 2;
    _pointcloud.rowwise() -= _nomalizationDeplacement.transpose();

    // make the diagonal 1.6
    _normalizationScaling = 1.6 / (_pointcloud.colwise().maxCoeff() - _pointcloud.colwise().minCoeff()).maxCoeff();
    _pointcloud *= _normalizationScaling;

    return true;
}

inline bool qSkeletonLib::PointCloudSkeleton::un_normalization()
{
    _pointcloud /= _normalizationScaling;
    _pointcloud.rowwise() += _nomalizationDeplacement.transpose();

    _contractedPointcloud /= _normalizationScaling;
    _contractedPointcloud.rowwise() += _nomalizationDeplacement.transpose();

    _skeleton->transform(_normalizationScaling, _nomalizationDeplacement);

    return true;
}

qSkeletonLib::Graph qSkeletonLib::PointCloudSkeleton::get_skeleton()
{
    return *_skeleton;
}

inline Eigen::VectorXd qSkeletonLib::PointCloudSkeleton::one_ring_size(Eigen::MatrixXd& cloud, std::vector<OneRing>& one_ring_list, std::string distance_type)
{
    // sum all triangle (abc) 
    Eigen::VectorXd out = Eigen::VectorXd::Zero(cloud.rows());
    if (distance_type == "area")
    {


        for (int i = 0; i < cloud.rows(); i++) {

            Eigen::Vector3d a = cloud.row(i);

            std::vector<int> one_ring = one_ring_list[i].get_one_ring();
            for (int j = 0; j < one_ring.size(); j++) {

                Eigen::Vector3d d = cloud.row(one_ring[j]);

                std::vector<int> connected_element = one_ring_list[i].get_connected_components(j);

                for (int connected_element_it = 0; connected_element_it < connected_element.size(); connected_element_it++) {
                    Eigen::Vector3d b = cloud.row(connected_element[connected_element_it]);

                    Eigen::Vector3d ba = b - a;
                    Eigen::Vector3d bd = b - d;


                    out(i) += ((ba).cross(bd)).norm() * 1 / 4;
                }
            }
        }


    }
    else
    {
        for (int i = 0; i < cloud.rows(); i++) {
            // get the point:
            Eigen::Vector3d vertex;
            vertex = cloud.row(i);

            // get the corresponding one ring structure
            std::vector<int> one_ring = one_ring_list.at(i).get_one_ring();

            // store all the distances between the vertex and its neighbours
            Eigen::VectorXd temp(one_ring.size());
            for (int neighbour_it = 0; neighbour_it < one_ring.size(); neighbour_it++) {
                Eigen::Vector3d neighbour;
                neighbour = cloud.row(one_ring.at(neighbour_it));
                temp(neighbour_it) = (vertex - neighbour).norm();
            }

            // get the mean of all the distance to the point
            if (distance_type == "min")
                out(i) = temp.minCoeff();
            else if (distance_type == "mean")
                out(i) = temp.mean();
            else if (distance_type == "max")
                out(i) = temp.maxCoeff();
        }
    }

    return out;
}

inline double qSkeletonLib::PointCloudSkeleton::cotan(Eigen::Vector3d v, Eigen::Vector3d w)
{
    return((v.dot(w)) / ((v.cross(w)).norm()));
}

inline Eigen::SparseMatrix<double> qSkeletonLib::PointCloudSkeleton::laplacian(Eigen::MatrixXd& cloud, std::vector<OneRing>& one_ring_list)
{
    Eigen::SparseMatrix<double> L_2(cloud.rows(), cloud.rows());

    for (int i = 0; i < cloud.rows(); i++) {
        /* The laplacian operator is defined with the cotangent weights for each edge.
         * E.g., with the two triangles (abd) and (acd), for the edge (ad) we sum cotan(b) and cotan(c).
         *
         *    a---b
         *    | \ |
         *    c---d
         *
         */

        Eigen::Vector3d a = cloud.row(i);

        const std::vector<int>& one_ring = one_ring_list[i].get_one_ring();
        bool require_normalization = false;
        double max_cot_value = 0;

        for (int j = 0; j < one_ring.size(); j++) {

            double cot_theta = 0;

            Eigen::Vector3d d = cloud.row(one_ring[j]);

            const std::vector<int>& connected_element = one_ring_list[i].get_connected_components(j);

            for (int connected_element_it = 0; connected_element_it < connected_element.size(); connected_element_it++) {
                Eigen::Vector3d b = cloud.row(connected_element[connected_element_it]);

                Eigen::Vector3d ba = b - a;
                Eigen::Vector3d bd = b - d;

                cot_theta += cotan(ba, bd);//( ba.dot(bd) ) / ( (ba.cross(bd)).norm() )
            }

            if (cot_theta > _opts.laplacian_threshold) {
                require_normalization = true;
                max_cot_value = std::max(max_cot_value, cot_theta);
            }

            L_2.coeffRef(i, one_ring[j]) += cot_theta;
            L_2.coeffRef(i, i) -= cot_theta;
        }

        if (require_normalization)
        {
            //std::cout << "Warning: very high laplacian value reached: cot_theta=" << max_cot_value << std::endl;
            for (int j = 0; j < one_ring.size(); j++) {
                L_2.coeffRef(i, one_ring[j]) *= _opts.laplacian_threshold / max_cot_value;
            }
            L_2.coeffRef(i, i) *= _opts.laplacian_threshold / max_cot_value;
        }

    }

    return L_2;
}

inline Eigen::MatrixXd qSkeletonLib::PointCloudSkeleton::solve_contraction(Eigen::SparseMatrix<double> WH, Eigen::SparseMatrix<double> WL, Eigen::SparseMatrix<double> L, Eigen::MatrixXd points)
{
    Eigen::SparseMatrix<double> A, A2, WlL;
    Eigen::MatrixXd b, b2, WhP, zeros;

    WhP = WH * points;
    WlL = WL * L;
    zeros = Eigen::MatrixXd::Zero(WhP.rows(), 3);

    A = concatenate(WlL, WH, 1);
    b = concatenate(zeros, WhP, 1);
    A2 = A.transpose() * A;
    b2 = A.transpose() * b;

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
    solver.compute(A2);
    if (solver.info() != Eigen::Success) {
        // decomposition failed
        std::cout << "Error: solver failed\n";
    }
    return solver.solve(b2);
}
