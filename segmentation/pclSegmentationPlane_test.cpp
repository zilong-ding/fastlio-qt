//
// Created by dzl on 2025/12/28.
//
#include <iostream>
#include <memory>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/time.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <pcl/segmentation/>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::vector<PointCloudT::Ptr> extractPlanes(const PointCloudT::Ptr& cloud, int max_iterations = 20,
                                            int min_inliers = 400, float distance_threshold = 0.02f)
{
    PointCloudT::Ptr cloud_filtered = std::make_shared<PointCloudT>(*cloud);  // 剩余待处理点云
    std::vector<PointCloudT::Ptr> planes;                       // 存储分割出的各个平面
    std::vector<pcl::ModelCoefficients> coefficients_list;      // 平面参数 (ax+by+cz+d=0)

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();
    pcl::ModelCoefficients::Ptr coefficients = std::make_shared<pcl::ModelCoefficients>();
    pcl::ExtractIndices<PointT> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);  // 距离阈值，决定点是否属于平面（单位：米）

    pcl::console::TicToc tt;
    tt.tic();

    for (int i = 0; i < max_iterations && cloud_filtered->size() > min_inliers; ++i)
    {
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
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
        PointCloudT::Ptr plane_cloud = std::make_shared<PointCloudT>();
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 提取平面点
        extract.filter(*plane_cloud);

        float nz = std::abs(coefficients->values[2]); // 法向 z 分量
        float nx = std::abs(coefficients->values[0]);
        auto threshold = std::sin(pcl::deg2rad(20.0f));
        auto threshold_x = std::sin(pcl::deg2rad(45.0f));
        std:: cout << "nz " << nz << " threshold " << threshold << std::endl;
        if (nz < threshold && nx < threshold_x) {
            planes.push_back(plane_cloud);
            coefficients_list.push_back(*coefficients);
            std::cout << "Plane " << i << ": "
                  << inliers->indices.size() << " inliers\n coefficients: "
                  << coefficients->values[0] << "x+ "
                  << coefficients->values[1] << "y+ "
                  << coefficients->values[2] << "z+ "
                  << coefficients->values[3] << std::endl;
        }
        // 剩余点云 = 原始点云 - 当前平面点
        extract.setNegative(true);  // 提取非平面点
        PointCloudT::Ptr cloud_remainder = std::make_shared<PointCloudT>();
        extract.filter(*cloud_remainder);
        cloud_filtered = cloud_remainder;
    }

    std::cout << "Segmented " << planes.size() << " planes in " << tt.toc() << " ms." << std::endl;
    return planes;
}




int main() {
    std::cout << "hello pcl plane" << std::endl;
    // 1. 读取点云
    PointCloudT::Ptr cloud(new PointCloudT);
    std::string path0 = "/home/dzl/PycharmProjects/coppeliasim/coppeliasim/test/01f.pcd";
    std::string path1 = "/home/dzl/CLionProjects/fastlio-qt/segmentation/pointcloud_hdl64e3.ply";
    // if (pcl::io::loadPCDFile<PointT>(path0, *cloud) == -1)
    // {
    //     PCL_ERROR("Could not read file.\n");
    //     return -1;
    // }
    if (pcl::io::loadPLYFile<PointT>(path1, *cloud) == -1) {
        PCL_ERROR("Could not read file.\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " data points." << std::endl;

    // 2. 初始化

    // 在main函数中调用
    PointCloudT::Ptr cloud_filtered = std::make_shared<PointCloudT>(*cloud);
    std::vector<PointCloudT::Ptr> planes = extractPlanes(cloud_filtered);

    typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
    typedef K::Point_3                                             Point;
    std::vector<Point> points;

    // 3. 可选：保存每个平面为单独 ply 文件
    for (size_t i = 0; i < planes.size(); ++i)
    {
        std::string filename = "plane_" + std::to_string(i) + ".ply";
        // pcl::io::savePCDFileBinary(filename, *planes[i]);
        pcl::io::savePLYFile(filename, *planes[i]);
        std::cout << "Saved " << filename << std::endl;
        for (auto& p : planes[i]->points) {
            points.emplace_back(p.x, p.y, p.z);
        }
    }
    std::array<Point, 8> obb_points;
    CGAL::oriented_bounding_box(points, obb_points);
    std::cout <<
        "point[0] " << obb_points[0] << "\n" <<
        "point[1] " << obb_points[1] << "\n" <<
        "point[2] " << obb_points[2] << "\n" <<
        "point[3] " << obb_points[3] << "\n" <<
        "point[4] " << obb_points[4] << "\n" <<
        "point[5] " << obb_points[5] << "\n" <<
        "point[6] " << obb_points[6] << "\n" <<
        "point[7] " << obb_points[7] << "\n" <<
     std::endl;

    // auto obb = CGAL::optimal_bounding_box(points);



    return 0;
}
