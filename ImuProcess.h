//
// Created by dzl on 2025/11/21.
//
#pragma once

#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
#include "ikfomDataTypes.h"
#include "dataTypes.h"

#ifndef FASTLIO_QT_IMUPROCESS_H
#define FASTLIO_QT_IMUPROCESS_H


#define MAX_INI_COUNT (10)
#define G_m_s2 (9.81)         // Gravaty const in GuangDong/China

inline bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

template<typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g, \
                const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return std::move(rot_kp);
}

class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess() = default;
    void Reset() ;
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4,4) &T);
    void set_gyr_cov(const V3D &scaler);
    void set_acc_cov(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
    void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

    std::ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;
    int lidar_type;

private:
    std::shared_ptr<IMU> last_imu_;
    PointCloudXYZI::Ptr cur_pcl_un_;
    std::deque<std::shared_ptr<IMU>> v_imu_;
    std::vector<Pose6D> IMUpose;
    M3D Lidar_R_wrt_IMU;
    V3D Lidar_T_wrt_IMU;
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;


    bool b_first_frame_;
    bool imu_need_init_;
    double start_timestamp_;
    double last_lidar_end_time_;
    int    init_iter_num = 1;
    Eigen::Matrix<double, 12, 12> Q;

};


#endif //FASTLIO_QT_IMUPROCESS_H