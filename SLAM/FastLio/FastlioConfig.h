//
// Created by dzl on 2025/12/1.
//

#ifndef FASTLIO_QT_MAINWORKERCONFIG_H
#define FASTLIO_QT_MAINWORKERCONFIG_H
#include <nlohmann/json.hpp>

// MainWorker.h ↑ 添加在 class MainWorker 之前
struct MainWorkerConfig {
    // —— 发布控制 ——
    bool path_en = true;
    bool scan_pub_en = true;
    bool dense_pub_en = true;
    bool scan_body_pub_en = true;
    bool pcd_save_en = false;
    int pcd_save_interval = -1;

    // —— 时间同步 ——
    bool time_sync_en = false;
    double time_diff_lidar_to_imu = 0.0;

    // —— 滤波参数 ——
    double filter_size_surf_min = 0.5;
    double filter_size_map_min = 0.5;
    double filter_size_corner_min = 0.5;  // NOTE: 实际未使用，可删或保留

    // —— 地图参数 ——
    double cube_len = 120.0;      // Local map 边长
    double DET_RANGE = 450.0;     // 有效探测距离
    double fov_deg = 90.0;        // LiDAR 竖直 FOV（用于视野分割）

    // —— IMU 噪声参数 ——
    double gyr_cov = 0.1;         // 角速度噪声 std²
    double acc_cov = 0.1;         // 加速度噪声 std²
    double b_gyr_cov = 1e-4;      // 陀螺零偏 std²
    double b_acc_cov = 1e-4;      // 加速度计零偏 std²

    // —— LiDAR 类型与预处理 ——
    int lidar_type = AVIA;        // enum: AVIA, VELO16, OUST64, MARSIM, etc.
    int N_SCANS = 6;              // 扫描线数
    int point_filter_num = 3;     // 每 n 点取 1（降采样）
    double blind = 4.0;           // 盲区距离（m）
    int time_unit = US;           // 时间单位：US=微秒, MS=毫秒
    int SCAN_RATE = 10;           // 扫描频率（Hz）

    // —— EKF & 优化 ——
    int NUM_MAX_ITERATIONS = 4;
    bool extrinsic_est_en = true; // 是否在线估计外参

    // —— 外参初值（仅用于初始化 IMUProcessor）——
    Eigen::Vector3d extrinsic_T{0.04165, 0.02326, -0.0284};
    Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();

    // —— 其他 ——
    bool runtime_pos_log = false;
};

// ↓ 放在 MainWorkerConfig 结构体定义之后 ↓
namespace nlohmann {
template<>
struct adl_serializer<MainWorkerConfig> {
    static void to_json(json& j, const MainWorkerConfig& c) {
        j = json{
            {"path_en", c.path_en},
            {"scan_pub_en", c.scan_pub_en},
            {"dense_pub_en", c.dense_pub_en},
            {"scan_body_pub_en", c.scan_body_pub_en},
            {"pcd_save_en", c.pcd_save_en},
            {"pcd_save_interval", c.pcd_save_interval},

            {"time_sync_en", c.time_sync_en},
            {"time_diff_lidar_to_imu", c.time_diff_lidar_to_imu},

            {"filter_size_surf_min", c.filter_size_surf_min},
            {"filter_size_map_min", c.filter_size_map_min},
            {"filter_size_corner_min", c.filter_size_corner_min},

            {"cube_len", c.cube_len},
            {"DET_RANGE", c.DET_RANGE},
            {"fov_deg", c.fov_deg},

            {"gyr_cov", c.gyr_cov},
            {"acc_cov", c.acc_cov},
            {"b_gyr_cov", c.b_gyr_cov},
            {"b_acc_cov", c.b_acc_cov},

            {"lidar_type", c.lidar_type},
            {"N_SCANS", c.N_SCANS},
            {"point_filter_num", c.point_filter_num},
            {"blind", c.blind},
            {"time_unit", c.time_unit},
            {"SCAN_RATE", c.SCAN_RATE},

            {"NUM_MAX_ITERATIONS", c.NUM_MAX_ITERATIONS},
            {"extrinsic_est_en", c.extrinsic_est_en},

            {"extrinsic_T", {c.extrinsic_T.x(), c.extrinsic_T.y(), c.extrinsic_T.z()}},
            {"extrinsic_R", {
                {c.extrinsic_R(0,0), c.extrinsic_R(0,1), c.extrinsic_R(0,2)},
                {c.extrinsic_R(1,0), c.extrinsic_R(1,1), c.extrinsic_R(1,2)},
                {c.extrinsic_R(2,0), c.extrinsic_R(2,1), c.extrinsic_R(2,2)}
            }},

            {"runtime_pos_log", c.runtime_pos_log}
        };
    }

    static void from_json(const json& j, MainWorkerConfig& c) {
        // 使用 value(key, default) 避免缺失字段崩溃
        c.path_en = j.value("path_en", true);
        c.scan_pub_en = j.value("scan_pub_en", true);
        c.dense_pub_en = j.value("dense_pub_en", true);
        c.scan_body_pub_en = j.value("scan_body_pub_en", true);
        c.pcd_save_en = j.value("pcd_save_en", false);
        c.pcd_save_interval = j.value("pcd_save_interval", -1);

        c.time_sync_en = j.value("time_sync_en", false);
        c.time_diff_lidar_to_imu = j.value("time_diff_lidar_to_imu", 0.0);

        c.filter_size_surf_min = j.value("filter_size_surf_min", 0.5);
        c.filter_size_map_min = j.value("filter_size_map_min", 0.5);
        c.filter_size_corner_min = j.value("filter_size_corner_min", 0.5);

        c.cube_len = j.value("cube_len", 120.0);
        c.DET_RANGE = j.value("DET_RANGE", 450.0);
        c.fov_deg = j.value("fov_deg", 90.0);

        c.gyr_cov = j.value("gyr_cov", 0.1);
        c.acc_cov = j.value("acc_cov", 0.1);
        c.b_gyr_cov = j.value("b_gyr_cov", 1e-4);
        c.b_acc_cov = j.value("b_acc_cov", 1e-4);

        c.lidar_type = j.value("lidar_type", AVIA);
        c.N_SCANS = j.value("N_SCANS", 6);
        c.point_filter_num = j.value("point_filter_num", 3);
        c.blind = j.value("blind", 4.0);
        c.time_unit = j.value("time_unit", US);
        c.SCAN_RATE = j.value("SCAN_RATE", 10);

        c.NUM_MAX_ITERATIONS = j.value("NUM_MAX_ITERATIONS", 4);
        c.extrinsic_est_en = j.value("extrinsic_est_en", true);

        c.runtime_pos_log = j.value("runtime_pos_log", false);

        // —— 外参 T ——
        if (j.contains("extrinsic_T") && j["extrinsic_T"].is_array() && j["extrinsic_T"].size() == 3) {
            auto& arr = j["extrinsic_T"];
            c.extrinsic_T = Eigen::Vector3d{
                arr[0].get<double>(),
                arr[1].get<double>(),
                arr[2].get<double>()
            };
        } else {
            c.extrinsic_T = Eigen::Vector3d{0.04165, 0.02326, -0.0284}; // default
        }

        // —— 外参 R ——
        if (j.contains("extrinsic_R") && j["extrinsic_R"].is_array() && j["extrinsic_R"].size() == 3) {
            auto& mat = j["extrinsic_R"];
            if (mat[0].size() == 3 && mat[1].size() == 3 && mat[2].size() == 3) {
                c.extrinsic_R = Eigen::Matrix3d{
                    {mat[0][0].get<double>(), mat[0][1].get<double>(), mat[0][2].get<double>()},
                    {mat[1][0].get<double>(), mat[1][1].get<double>(), mat[1][2].get<double>()},
                    {mat[2][0].get<double>(), mat[2][1].get<double>(), mat[2][2].get<double>()}
                };
            } else {
                c.extrinsic_R.setIdentity();
            }
        } else {
            c.extrinsic_R.setIdentity();
        }
    }
};
} // namespace nlohmann
#endif //FASTLIO_QT_MAINWORKERCONFIG_H