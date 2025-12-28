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
    // 创建一个点云副本用于处理，初始化为输入的点云
    PointCloudT::Ptr cloud_filtered = std::make_shared<PointCloudT>(*cloud);
    // 存储检测到的平面点云
    std::vector<PointCloudT::Ptr> planes;
    // 存储每个平面的参数 (ax+by+cz+d=0)
    std::vector<pcl::ModelCoefficients> coefficients_list;

    // 创建平面分割对象
    pcl::SACSegmentation<PointT> seg;
    // 创建索引存储对象，用于存储属于平面的点的索引
    pcl::PointIndices::Ptr inliers = std::make_shared<pcl::PointIndices>();
    // 创建平面系数存储对象
    pcl::ModelCoefficients::Ptr coefficients = std::make_shared<pcl::ModelCoefficients>();
    // 创建点云提取对象
    pcl::ExtractIndices<PointT> extract;

    // 设置分割参数
    seg.setOptimizeCoefficients(true);                    // 优化系数
    seg.setModelType(pcl::SACMODEL_PLANE);               // 设置模型类型为平面
    seg.setMethodType(pcl::SAC_RANSAC);                  // 设置方法类型为RANSAC算法
    seg.setDistanceThreshold(distance_threshold);        // 设置距离阈值，决定点是否属于平面（单位：米）

    // 开始计时
    pcl::console::TicToc tt;
    tt.tic();

    // 循环分割多个平面，直到达到最大迭代次数或剩余点云数量小于最小内点数
    for (int i = 0; i < max_iterations && cloud_filtered->size() > min_inliers; ++i)
    {
        // 设置待分割的点云
        seg.setInputCloud(cloud_filtered);
        // 执行分割，获取内点索引和平面参数
        seg.segment(*inliers, *coefficients);

        // 检查是否成功分割到平面
        if (inliers->indices.empty())
        {
            std::cout << "Could not estimate a plane model for iteration " << i << std::endl;
            break;
        }

        // 检查内点数量是否满足最小要求
        if (inliers->indices.size() < min_inliers)
        {
            std::cout << "Plane " << i << " has too few inliers (" << inliers->indices.size() << "), stopping." << std::endl;
            break;
        }

        // 提取当前检测到的平面点云
        PointCloudT::Ptr plane_cloud = std::make_shared<PointCloudT>();
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);  // 提取平面点（即内点）
        extract.filter(*plane_cloud);

        // 进行平面方向筛选：检查平面法向量的z分量和x分量
        float nz = std::abs(coefficients->values[2]); // 法向量z分量的绝对值
        float nx = std::abs(coefficients->values[0]); // 法向量x分量的绝对值
        // 设置阈值，对应角度20度和45度
        auto threshold = std::sin(pcl::deg2rad(20.0f));  // 20度的正弦值，用于z分量（接近水平面）
        auto threshold_x = std::sin(pcl::deg2rad(45.0f)); // 45度的正弦值，用于x分量

        std::cout << "nz " << nz << " threshold " << threshold << std::endl;

        // 只保留接近水平面的平面（nz < threshold）且x方向分量不太大的平面(nx < threshold_x)
        if (nz < threshold && nx < threshold_x) {
            // 保存满足条件的平面点云和参数
            planes.push_back(plane_cloud);
            coefficients_list.push_back(*coefficients);
            std::cout << "Plane " << i << ": "
                  << inliers->indices.size() << " inliers\n coefficients: "
                  << coefficients->values[0] << "x+ "
                  << coefficients->values[1] << "y+ "
                  << coefficients->values[2] << "z+ "
                  << coefficients->values[3] << std::endl;
        }

        // 从原始点云中移除当前检测到的平面点，得到剩余点云
        extract.setNegative(true);  // 提取非平面点（即外点）
        PointCloudT::Ptr cloud_remainder = std::make_shared<PointCloudT>();
        extract.filter(*cloud_remainder);
        cloud_filtered = cloud_remainder;  // 更新待处理的点云
    }

    // 输出分割统计信息
    std::cout << "Segmented " << planes.size() << " planes in " << tt.toc() << " ms." << std::endl;
    return planes;  // 返回检测到的平面点云集合
}




int main() {
    std::cout << "hello pcl plane" << std::endl;

    // 1. 读取点云数据
    PointCloudT::Ptr cloud(new PointCloudT);
    // 定义点云文件路径
    std::string path0 = "/home/dzl/PycharmProjects/coppeliasim/coppeliasim/test/01f.pcd";
    std::string path1 = "/home/dzl/CLionProjects/fastlio-qt/segmentation/pointcloud_hdl64e3.ply";

    // 尝试加载PCD格式的点云文件
    if (pcl::io::loadPCDFile<PointT>(path0, *cloud) == -1)
    {
        PCL_ERROR("Could not read file.\n");
        return -1;
    }
    // 注释掉的PLY文件加载选项
    // if (pcl::io::loadPLYFile<PointT>(path1, *cloud) == -1) {
    //     PCL_ERROR("Could not read file.\n");
    //     return -1;
    // }
    std::cout << "Loaded " << cloud->size() << " data points." << std::endl;

    // 2. 初始化

    // 调用平面分割函数
    PointCloudT::Ptr cloud_filtered = std::make_shared<PointCloudT>(*cloud);
    std::vector<PointCloudT::Ptr> planes = extractPlanes(cloud_filtered);

    // 定义CGAL内核和点类型，用于计算有向包围盒
    typedef CGAL::Exact_predicates_inexact_constructions_kernel    K;
    typedef K::Point_3                                             Point;
    std::vector<Point> points;

    // 3. 保存每个分割出的平面为单独的PLY文件，并收集所有点用于包围盒计算
    for (size_t i = 0; i < planes.size(); ++i)
    {
        std::string filename = "plane_" + std::to_string(i) + ".ply";
        // 注释掉PCD保存选项
        // pcl::io::savePCDFileBinary(filename, *planes[i]);
        pcl::io::savePLYFile(filename, *planes[i]);
        std::cout << "Saved " << filename << std::endl;

        // 将当前平面的所有点转换为CGAL点类型并添加到点集合中
        for (auto& p : planes[i]->points) {
            points.emplace_back(p.x, p.y, p.z);
        }
    }

    // 计算所有平面点的有向包围盒(OBB)，返回8个顶点
    std::array<Point, 8> obb_points;
    CGAL::oriented_bounding_box(points, obb_points);

    // 输出包围盒的8个顶点坐标
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

    // 注释掉的最优包围盒计算选项
    // auto obb = CGAL::optimal_bounding_box(points);

    return 0;
}
