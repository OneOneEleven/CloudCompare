#pragma once
#include <algorithm>           // std::all_of
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <limits>
#include <cmath>
#include <vector>
 
#include "OneRing.hpp"
#include "graph/graph.hpp"

namespace qSkeletonLib
{
    template<typename PointT>
    class IPointCloud
    {
    public:
        virtual size_t size() const = 0;
        virtual void getPoint(size_t index, PointT* coords) const = 0;
    };
 
    class Graph;
    class OneRing;
    class PointCloudSkeleton
    {
    public:
        struct Options
        {
            bool verbose = false;

            int skeleton_editing = 0;
            bool use_radius = true;
            double sample_radius = 0.002;
            double sample_ratio = 20;
            int iteration_time = 10;
            double termination_criteria = 0.01;
            int k_for_knn = 15;
            double sl = 3;
            double WC = 1;
            double laplacian_threshold = 10000;
            double MAX_POSITION_CONSTRAINT_WEIGHT = 10000;
            double MAX_LAPLACIAN_CONSTRAINT_WEIGHT = 2048;
            bool cloud_only = false;

            double nodes_ratio;
            double edges_ratio;
            double graph_res;
            std::vector<double> nodes_color;
            std::vector<double> edges_color;
        };

    public:
        PointCloudSkeleton(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Options& opts);
      
 
        // destructor
        ~PointCloudSkeleton() = default;


        void skeletonize();

        bool init();
        void laplacian_contraction();
        bool contracted_cloud_to_skeleton();
        inline bool normalization();
        inline bool un_normalization();

        Eigen::VectorXi get_correspondences()
        {
            return _correspondences;
        }

        Graph get_skeleton();
    private:
        // internal functions
        inline Eigen::VectorXd one_ring_size(Eigen::MatrixXd& cloud, std::vector< OneRing >& one_ring_list, std::string distance_type);
        inline double cotan(Eigen::Vector3d v, Eigen::Vector3d w);
        inline Eigen::SparseMatrix<double> laplacian(Eigen::MatrixXd& cloud, std::vector< OneRing >& one_ring_list);
        inline Eigen::MatrixXd solve_contraction(Eigen::SparseMatrix<double> WH,
            Eigen::SparseMatrix<double> WL,
            Eigen::SparseMatrix<double> L,
            Eigen::MatrixXd points);

    private:
        Options _opts;

        Eigen::MatrixXd _pointcloud;                      // pointcloud
        Eigen::MatrixXd _contractedPointcloud;            // contracted pointcloud
        Eigen::VectorXi _correspondences;                 // association between each point from pointcloud_ and the nodes of the skeleton

        Eigen::MatrixXi _F;                               // F: faces of the surface (for plots)

        std::vector<OneRing> _ringList;

        bool _initialized{ false };

        Graph* _skeleton;

        Eigen::Vector3d _nomalizationDeplacement;
        double _normalizationScaling;

    };

}
