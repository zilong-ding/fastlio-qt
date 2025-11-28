//
// Created by dzl on 11/20/25.
//

#ifndef FASTLIO_QT_PREPROCESS_H
#define FASTLIO_QT_PREPROCESS_H
#include "dataTypes.h"

class Preprocess {
public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocess();
    ~Preprocess() = default;
    void process(std::shared_ptr<LidarFrame> &msg, PointCloudXYZI::Ptr &pcl_out);
    void set(bool feat_en, int lid_type, double bld, int pfilt_num);
    double blind;
    int N_SCANS;
    int time_unit;
    int lidar_type, point_filter_num, SCAN_RATE;
    bool feature_enabled, given_offset_time;
private:
    void avia_handler(std::shared_ptr<LidarFrame> &msg);
    void give_feature(PointCloudXYZI &pl, std::vector<orgtype> &types);
    int  plane_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types,
                        uint i, uint &i_nex, Eigen::Vector3d &curr_direct);


    PointCloudXYZI pl_full, pl_corn, pl_surf;
    PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
    std::vector<orgtype> typess[128]; //maximum 128 line lidar


    int group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;

    double vx, vy, vz;
};


#endif //FASTLIO_QT_PREPROCESS_H