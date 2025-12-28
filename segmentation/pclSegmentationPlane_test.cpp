//
// Created by dzl on 2025/12/28.
//
#include <iostream>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/time.h>
// #include <pcl/segmentation/>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main() {
    std::cout << "hello pcl plane" << std::endl;
    // 1. 读取点云
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>("/home/dzl/PycharmProjects/coppeliasim/coppeliasim/test/01f.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read file.\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " data points." << std::endl;

    // 2. 初始化
    PointCloudT::Ptr cloud_filtered(new PointCloudT(*cloud));  // 剩余待处理点云
    std::vector<PointCloudT::Ptr> planes;                       // 存储分割出的各个平面
    std::vector<pcl::ModelCoefficients> coefficients_list;      // 平面参数 (ax+by+cz+d=0)

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);  // 距离阈值，决定点是否属于平面（单位：米）

    int max_iterations = 20;      // 最多提取多少个平面
    int min_inliers = 400;        // 平面最少内点数（可根据场景调整）

    pcl::console::TicToc tt;
    tt.tic();

    for (int i = 0; i < max_iterations && cloud_filtered->size() > min_inliers; ++i)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a plane model for iteration " << i << std::endl;
            break;
        }

        if (inliers->indices.size() < min_inliers)
        {
            std::cout << "Plane " << i << " has too few inliers (" << inliers->indices.size() << "), stopping." << std::endl;
            break;
        }

        // 保存当前平面
        PointCloudT::Ptr plane_cloud(new PointCloudT);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 提取平面点
        extract.filter(*plane_cloud);

        float nz = std::abs(coefficients->values[2]); // 法向 z 分量
        auto threshold = std::sin(pcl::deg2rad(20.0f));
        std:: cout << "nz " << nz << " threshold " << threshold << std::endl;
        if (nz < threshold) {
            planes.push_back(plane_cloud);
            coefficients_list.push_back(*coefficients);
            std::cout << "Plane " << i << ": "
                  << inliers->indices.size() << " inliers, coefficients: "
                  << coefficients->values[0] << ", "
                  << coefficients->values[1] << ", "
                  << coefficients->values[2] << ", "
                  << coefficients->values[3] << std::endl;
        }
        // 剩余点云 = 原始点云 - 当前平面点
        extract.setNegative(true);  // 提取非平面点
        PointCloudT::Ptr cloud_remainder(new PointCloudT);
        extract.filter(*cloud_remainder);
        cloud_filtered = cloud_remainder;
    }

    std::cout << "Segmented " << planes.size() << " planes in " << tt.toc() << " ms." << std::endl;

    // 3. 可选：保存每个平面为单独 ply 文件
    for (size_t i = 0; i < planes.size(); ++i)
    {
        std::string filename = "plane_" + std::to_string(i) + ".ply";
        // pcl::io::savePCDFileBinary(filename, *planes[i]);
        pcl::io::savePLYFile(filename, *planes[i]);
        std::cout << "Saved " << filename << std::endl;
    }



    return 0;
}
