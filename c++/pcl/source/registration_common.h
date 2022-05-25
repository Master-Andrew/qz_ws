#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

void DoICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
           pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f transformation,
           float max_cor_distance = 1.0, int max_iter = 50)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(1.0);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);

    icp.align(*aligen);

    transformation = icp.getFinalTransformation();
}

void DoNDT(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
           pcl::PointCloud<pcl::PointXYZ>::Ptr aligen, Eigen::Matrix4f transformation,
           float resolution = 1.0, int max_iter = 50)
{
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.1);
    ndt.setStepSize(0.5);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iter);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*aligen);
    transformation = ndt.getFinalTransformation();

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;
}

// Eigen::Matrix4f initial_estimate = init_transform.mat().cast<float>();

// http://pointclouds.org/documentation/classpcl_1_1registration_1_1_transformation_estimation_point_to_plane.html
Eigen::Matrix4f PclPointToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                const Eigen::Matrix4f &init_transform,
                                double dis_thres = 1.0,
                                int kMaxIteration = 20)
{

    //点云初始化
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr ref_cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*ref_cloud, *ref_cloud_normals);

    //法向估计
    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_estimation;
    normal_estimation.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(
        new pcl::search::KdTree<pcl::PointNormal>));

    normal_estimation.setKSearch(20);
    normal_estimation.setInputCloud(ref_cloud_normals);
    normal_estimation.setViewPoint(0, 0, 10);
    normal_estimation.compute(*ref_cloud_normals);
    cout << "normal_estimation" << endl;

    //对应关系
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    int iteration = 0;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // Convergence criteria

    //收敛准则初始化
    pcl::registration::DefaultConvergenceCriteria<float> convergence_criteria(
        iteration, transform, *correspondences);
    convergence_criteria.setMaximumIterations(kMaxIteration);
    convergence_criteria.setMaximumIterationsSimilarTransforms(5);
    convergence_criteria.setRelativeMSE(1e-5);
    cout << "convergence_criteria" << endl;

    // Correspondence estimation
    boost::shared_ptr<pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>>
        cor_est_closest_point(
            new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>());
    cor_est_closest_point->setInputTarget(ref_cloud);
    cout << "cor_est_closest_point" << endl;

    // Transformation estimation
    pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZ, pcl::PointNormal>
        transformation_estimation;

    pcl::transformPointCloud(*src_cloud, *source_aligned, init_transform);
    Eigen::Matrix4f final_transform = init_transform;
    cout << "transformPointCloud" << endl;

    const float correspondence_threshold = dis_thres;
    do
    {
        cor_est_closest_point->setInputSource(source_aligned);
        cor_est_closest_point->determineCorrespondences(*correspondences,
                                                        correspondence_threshold);

        transform.setIdentity();
        transformation_estimation.estimateRigidTransformation(
            *source_aligned, *ref_cloud_normals, *correspondences, transform);
        final_transform = transform * final_transform;

        pcl::transformPointCloud(*src_cloud, *source_aligned, final_transform);

        iteration++;
    } while (
        !convergence_criteria.hasConverged());

    cout << "done" << endl;
    return final_transform;
}