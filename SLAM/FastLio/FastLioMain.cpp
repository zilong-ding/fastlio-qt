//
// Created by dzl on 2025/12/1.
//

#include "FastLioMain.h"

// MainWorker.cpp ↑ 添加在文件顶部或类外
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// 保存配置到 JSON 文件
bool saveConfigToJson(const MainWorkerConfig& config, const std::string& filepath) {
    try {
        json j = config; // 自动调用 to_json
        std::ofstream f(filepath);
        if (!f.is_open()) return false;
        f << std::setw(4) << j << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[Config] Save to " << filepath << " failed: " << e.what() << std::endl;
        return false;
    }
}

// 从 JSON 文件加载配置
std::optional<MainWorkerConfig> loadConfigFromJson(const std::string& filepath) {
    try {
        std::ifstream f(filepath);
        if (!f.is_open()) {
            std::cerr << "[Config] File not found: " << filepath << std::endl;
            return std::nullopt;
        }
        json j;
        f >> j;
        MainWorkerConfig config = j.get<MainWorkerConfig>(); // 自动调用 from_json
        return config;
    } catch (const std::exception& e) {
        std::cerr << "[Config] Load from " << filepath << " failed: " << e.what() << std::endl;
        return std::nullopt;
    }
}
FastLioMain* FastLioMain::instance = nullptr;
FastLioMain::FastLioMain(QObject *parent) {
    instance = this;
    p_pre = std::make_shared<Preprocess>();
    p_imu = std::make_shared<ImuProcess>();
    feats_undistort = std::make_shared<PointCloudXYZI>();
    feats_down_body = std::make_shared<PointCloudXYZI>();
    feats_down_world = std::make_shared<PointCloudXYZI>();
    normvec = std::make_shared<PointCloudXYZI>(100000, 1);
    laserCloudOri = std::make_shared<PointCloudXYZI>(100000, 1);
    corr_normvect = std::make_shared<PointCloudXYZI>(100000, 1);
    Eye3d = M3D::Identity();
    Eye3f = M3F::Identity();
    Zero3d = V3D(0,0,0);
    Zero3f = V3F(0, 0, 0);
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model_proxy, NUM_MAX_ITERATIONS, epsi);

}

void FastLioMain::imuCallback(std::shared_ptr<IMU> msg_in) {
    publish_count ++;
    auto msg = std::make_shared<IMU>(*msg_in);
    msg->header.stamp = msg_in->header.stamp - time_diff_lidar_to_imu;
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = timediff_lidar_wrt_imu + msg_in->header.stamp;
    }
    double timestamp = msg->header.stamp;
    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        std::cout << "imu loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void FastLioMain::lidarCallback(PointCloud2::Ptr msg) {
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp < last_timestamp_lidar)
    {
        std::cout << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool FastLioMain::sync_packages(MeasureGroup &meas) {
    // std::cout << "sync_packages" << std::endl;
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();


        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cout << "Too few input point cloud!" << std::endl;
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp;
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp;
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void FastLioMain::loop()
{

    bool status = !stop;

    if (flg_exit) return;
    if(sync_packages(Measures))
    {

        if (flg_first_scan)
        {
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            return;
        }

        double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

        match_time = 0;
        kdtree_search_time = 0.0;
        solve_time = 0;
        solve_const_H_time = 0;
        svd_time   = 0;
        t0 = omp_get_wtime();

        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

        if (feats_undistort->empty() || (feats_undistort == NULL))
        {
            std::cout<<"feats_undistort is null, skip this scan!"<<std::endl;
            return;
        }


        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        t1 = omp_get_wtime();
        feats_down_size = feats_down_body->points.size();
        /*** initialize the map kdtree ***/
        if(ikdtree.Root_Node == nullptr)
        {
            if(feats_down_size > 5)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                ikdtree.Build(feats_down_world->points);
            }
            return;
        }
        // std::cout << "sync_packages" << std::endl;
        int featsFromMapNum = ikdtree.validnum();
        kdtree_size_st = ikdtree.size();

        // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size < 5)
        {
            // ROS_WARN("No point, skip this scan!\n");
            std::cout<<"No point, skip this scan!"<<std::endl;
            return;
        }

        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);

        // pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();

        /*** iterated state estimation ***/
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point = kf.get_x();
        // euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];

        double t_update_end = omp_get_wtime();

        /******* Publish odometry *******/
        publish_odometry();

        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        map_incremental();
        t5 = omp_get_wtime();

        /******* Publish points *******/
        if (path_en)                         publish_path();
        if (scan_pub_en || pcd_save_en)      publish_frame_world();

    }

    status = !stop;

}

void FastLioMain::initParams(const MainWorkerConfig& config)
{
    m_config = config;  // 保存当前配置

    // —— 发布控制 ——
    path_en = config.path_en;
    scan_pub_en = config.scan_pub_en;
    dense_pub_en = config.dense_pub_en;
    scan_body_pub_en = config.scan_body_pub_en;
    pcd_save_en = config.pcd_save_en;
    pcd_save_interval = config.pcd_save_interval;

    // —— 时间同步 ——
    time_sync_en = config.time_sync_en;
    time_diff_lidar_to_imu = config.time_diff_lidar_to_imu;

    // —— 滤波参数 ——
    filter_size_surf_min = config.filter_size_surf_min;
    filter_size_map_min = config.filter_size_map_min;
    filter_size_corner_min = config.filter_size_corner_min; // 若未用可删

    // —— 地图参数 ——
    cube_len = config.cube_len;
    DET_RANGE = config.DET_RANGE;
    fov_deg = config.fov_deg;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = std::cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    // —— IMU 噪声 ——
    gyr_cov = config.gyr_cov;
    acc_cov = config.acc_cov;
    b_gyr_cov = config.b_gyr_cov;
    b_acc_cov = config.b_acc_cov;

    // —— Preprocess 配置 ——
    p_pre->blind = config.blind;
    p_pre->N_SCANS = config.N_SCANS;
    p_pre->time_unit = config.time_unit;
    p_pre->SCAN_RATE = config.SCAN_RATE;
    p_pre->point_filter_num = config.point_filter_num;
    p_pre->feature_enabled = false; // 固定，FAST-LIO 不依赖 hand-crafted features

    // —— EKF & 外参 ——
    NUM_MAX_ITERATIONS = config.NUM_MAX_ITERATIONS;
    extrinsic_est_en = config.extrinsic_est_en;

    runtime_pos_log = config.runtime_pos_log;

    // —— Voxel 滤波器更新 ——
    downSizeFilterSurf.setLeafSize(config.filter_size_surf_min,
                                    config.filter_size_surf_min,
                                    config.filter_size_surf_min);
    downSizeFilterMap.setLeafSize(config.filter_size_map_min,
                                   config.filter_size_map_min,
                                   config.filter_size_map_min);

    // —— IMU Processor 配置 ——
    p_imu->set_extrinsic(config.extrinsic_T, config.extrinsic_R);
    p_imu->set_gyr_cov(V3D(config.gyr_cov, config.gyr_cov, config.gyr_cov));
    p_imu->set_acc_cov(V3D(config.acc_cov, config.acc_cov, config.acc_cov));
    p_imu->set_gyr_bias_cov(V3D(config.b_gyr_cov, config.b_gyr_cov, config.b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(config.b_acc_cov, config.b_acc_cov, config.b_acc_cov));
    p_imu->lidar_type = config.lidar_type;

    // —— Path header ——
    path.header.frame_id = "camera_init";

    // —— 初始化缓存 ——
    // _featsArray.reset(new PointCloudXYZI());
    std::fill(std::begin(point_selected_surf), std::end(point_selected_surf), true);
    std::fill(std::begin(res_last), std::end(res_last), -1000.0f);
}



void FastLioMain::lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    // X轴分界点转换到w系下，好像没有用到
    // pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void FastLioMain::h_share_model_proxy(
    state_ikfom &s,
    esekfom::dyn_share_datastruct<double> &data)
{
    instance->h_share_model(s, data);
}


void FastLioMain::h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        // ROS_WARN("No Effective Points! \n");
        std::cout << "No Effective Points!" << std::endl;
        return;
    }

    // res_mean_last = total_residual / effct_feat_num;
    // match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

void FastLioMain::pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void FastLioMain::RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void FastLioMain::points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);

}

void FastLioMain::publish_odometry()
{
    std::cout << "publish_odometry" << std::endl;
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = lidar_end_time;
    set_posestamp(odomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
    emit publishOdom(odomAftMapped);

}

void FastLioMain::publish_path()
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = lidar_end_time;
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        emit PathPublish(path);
    }
}
void FastLioMain::publish_frame_world()
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));


        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }
        PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI(size, 1));
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(laserCloudWorld); // 设置输入点云
        sor.setLeafSize(0.03f, 0.03f, 0.03f); // 设置体素大小
        sor.filter(*cloud_filtered); // 进行下采样

        PointCloudMsg laserCloudmsg;
        laserCloudmsg.pointCloud = cloud_filtered;
        // pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = lidar_end_time;
        laserCloudmsg.header.frame_id = "camera_init";
        emit PointCloudPublish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
}

void FastLioMain::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}