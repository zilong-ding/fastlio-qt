//
// Created by dzl on 2025/12/1.
//

#ifndef FASTLIO_QT_FASTLIOMAIN_H
#define FASTLIO_QT_FASTLIOMAIN_H
#include "../AlgorithmMainBase.h"
#include "../../ikd-Tree/ikd_Tree.h"
#include "../../Msgs/dataTypes.h"
#include "ImuProcess.h"
#include "Preprocess.h"
#include "FastlioConfig.h"
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
#define LIDAR_SP_LEN    (2)
#define NUM_MATCH_POINTS    (5)
#define PI_M (3.14159265358)
class FastLioMain : public AlgorithmMainBase{
    Q_OBJECT
public:
    explicit FastLioMain(QObject *parent = nullptr);
    ~FastLioMain() override {

    };

    void initParams(const MainWorkerConfig& config) ;
    bool sync_packages(MeasureGroup &meas);
    void lasermap_fov_segment();
    static FastLioMain* instance;   // 单例指针
    static void h_share_model_proxy(
        state_ikfom &s,
        esekfom::dyn_share_datastruct<double> &data);
    void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
    void pointBodyToWorld(PointType const * const pi, PointType * const po);
    void RGBpointBodyToWorld(PointType const * const pi, PointType * const po);
    void points_cache_collect();
    void publish_odometry();
    void publish_path();
    void publish_frame_world();
    template<typename T>
    void set_posestamp(T & out)
    {
        out.pose.position.x = state_point.pos(0);
        out.pose.position.y = state_point.pos(1);
        out.pose.position.z = state_point.pos(2);
        out.pose.orientation.x = geoQuat.x;
        out.pose.orientation.y = geoQuat.y;
        out.pose.orientation.z = geoQuat.z;
        out.pose.orientation.w = geoQuat.w;

    }
    float calc_dist(PointType p1, PointType p2){
        float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return d;
    }
public slots:
    void lidarCallback(PointCloud2::Ptr msg) override;
    void imuCallback(IMU::Ptr msg_in) override;
    void loop() override;

    void map_incremental() ;

private:
    MainWorkerConfig m_config;
    int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
    // int process_increments = 0;
    double time_diff_lidar_to_imu = 0.0;
    double timediff_lidar_wrt_imu = 0.0;
    double lidar_mean_scantime = 0.0;
    int    scan_num = 0;
    bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
    bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
    bool   timediff_set_flg = false;
    bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
    // std::deque<double> time_buffer;
    // double s_plot11[MAXN];
    int lidar_type;
    /*** EKF inputs and output ***/
    MeasureGroup Measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom state_point;
    vect3 pos_lid;
    double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    // laserCloudValidNum = 0,
    int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,  pcd_save_interval = -1, pcd_index = 0;
    //
    double  res_mean_last = 0.05, total_residual = 0.0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;

    // lasermap_fov_segment
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;
    std::vector<BoxPointType> cub_needrm;
    float DET_RANGE = 300.0f;
    const float MOV_THRESHOLD = 1.5f;
    // ofstream fout_pre, fout_out, fout_dbg;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    KD_TREE<PointType> ikdtree;

    M3D Eye3d;
    M3F Eye3f;
    V3D Zero3d;
    V3F Zero3f;

    // V3F XAxisPoint_body;
    // V3F XAxisPoint_world;
    // V3D euler_cur;
    // V3D position_last;
    // V3D Lidar_T_wrt_IMU;
    // M3D Lidar_R_wrt_IMU;
    // std::vector<std::vector<int>>  pointSearchInd_surf;
    std::vector<PointVector>  Nearest_Points;
    Quaternion geoQuat;
    Odometry odomAftMapped;
    Path path;
    PoseStamped msg_body_pose;

    std::shared_ptr<Preprocess> p_pre;
    std::shared_ptr<ImuProcess> p_imu;
    // PointCloudXYZI::Ptr featsFromMap;
    PointCloudXYZI::Ptr feats_undistort;
    PointCloudXYZI::Ptr feats_down_body;
    PointCloudXYZI::Ptr feats_down_world;
    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corr_normvect;
    // PointCloudXYZI::Ptr _featsArray;
    bool   point_selected_surf[100000] = {0};
    float res_last[100000] = {0.0};
    double epsi[23] = {0.001};

    /*** variables definition ***/
    // effect_feat_num = 0,
    int  frame_num = 0;
    // deltaT, deltaR,
    double  aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    // bool flg_EKF_converged, EKF_stop_flg = 0;

};
template<typename T>
bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

#endif //FASTLIO_QT_FASTLIOMAIN_H