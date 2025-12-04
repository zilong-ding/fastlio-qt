//
// Created by dzl on 2025/11/29.
//

#ifndef FASTLIO_QT_LIOBASE_H
#define FASTLIO_QT_LIOBASE_H
#include <QObject>
#include "FastLio/FastLioMain.h"
#include "../Sensors/SensorType.h"
#include <QSharedPointer>
#include <utility>
#include "AlgorithmMainBase.h"
// #include
class SLAMBase : public QObject{
    Q_OBJECT
public:
    explicit SLAMBase(bool use_multi_thread =  true) : QObject()
    {
        _use_multi_thread = use_multi_thread;
        if (_use_multi_thread) {
            imu_thread = std::make_shared<QThread>();
            lidar_thread = std::make_shared<QThread>();
            camera_thread = std::make_shared<QThread>();
        }
        else {
            std::cout << "single thread mode" << std::endl;
        }
    }
    ~SLAMBase() {
        std::cout << "SLAMBase destructor" << std::endl;
        stop();
        if (_use_multi_thread) {
            imu_thread->quit();
            imu_thread->wait();
            lidar_thread->quit();
            lidar_thread->wait();
            camera_thread->quit();
            camera_thread->wait();
        }

    }
    void addIMUinstance(std::shared_ptr<ImuBase> imu) {
        _imu = std::move(imu);
        if (_use_multi_thread) {
            _imu->moveToThread(imu_thread.get());
            imu_thread->start();
        }
        auto res = connect(this, &SLAMBase::startImu, _imu.get(), &ImuBase::start);
        res = connect(this, &SLAMBase::stopImu, _imu.get(), &ImuBase::stop);
        // emit startImu(1);
        std::cout << "connect imu to main worker " << res << std::endl;
    }
    void addLidarInstance(std::shared_ptr<LidarBase> lidar) {
        _lidar = std::move(lidar);
        if (_use_multi_thread) {
            _lidar->moveToThread(lidar_thread.get());
            lidar_thread->start();
        }
        auto res = connect(this, &SLAMBase::startLidar, _lidar.get(), &LidarBase::start);
        res = connect(this, &SLAMBase::stopLidar, _lidar.get(), &LidarBase::stop);
        std::cout << "connect lidar to main worker " << res << std::endl;
    }
    void addAlgorithmInstance(std::shared_ptr<AlgorithmMainBase>  algorithm) {
        main_worker_instance = std::move(algorithm);
        // 🔗 连接 AlgorithmMainBase 的信号 → SLAMBase 的槽 → 再 emit 自身信号
        auto res1 = connect(main_worker_instance.get(), &AlgorithmMainBase::publishOdom,
                            this, &SLAMBase::getOdometry);
        auto res2 = connect(main_worker_instance.get(), &AlgorithmMainBase::PathPublish,
                            this, &SLAMBase::onPathPublish);
        auto res3 = connect(main_worker_instance.get(), &AlgorithmMainBase::PointCloudPublish,
                            this, &SLAMBase::onPointCloudPublish);

        std::cout << "connect main worker signals: odom=" << res1
                  << ", path=" << res2 << ", cloud=" << res3 << std::endl;
    }

    void addCameraInstance(std::shared_ptr<CameraBase> camera) {
        _camera = std::move(camera);
        if (_use_multi_thread) {
            _camera->moveToThread(camera_thread.get());
            camera_thread->start();
        }
        auto res = connect(this, &SLAMBase::startCamera, _camera.get(), &CameraBase::start);
        res = connect(this, &SLAMBase::stopCamera, _camera.get(), &CameraBase::stop);
        std::cout << "connect camera to main worker " << res << std::endl;
    }
    bool connectSlots() {
        bool res = false;
        if (_imu && main_worker_instance) res = connect(_imu.get(), &ImuBase::sendIMUData, main_worker_instance.get(), &AlgorithmMainBase::imuCallback);
        if (_lidar && main_worker_instance) res = connect(_lidar.get(), &LidarBase::sendLidarData, main_worker_instance.get(), &AlgorithmMainBase::lidarCallback);
        if (_camera && main_worker_instance) res = connect(_camera.get(), &CameraBase::sendCameraData, main_worker_instance.get(), &AlgorithmMainBase::imageCallback);
        return res;
    }
    bool start() {
        bool res = false;
        res = main_worker_instance->start(10);
        std::cout << "main worker start " << res << std::endl;
        if (_imu)
        {
            emit startImu(1);
        }
        if (_lidar)
        {
            emit startLidar(10);
        }
        if (_camera) {
            emit startCamera(1);
        }
        return res;
    }

    bool stop() {
        bool res = false;
        // res = main_worker_instance->setStop();
        if (_imu)
        {
            emit stopImu();
        }
        if (_lidar)
        {
            emit stopLidar();
        }
        if (_camera) {
            emit stopCamera();
        }
        return res;
    }

public slots:
    void getOdometry(Odometry odom) {
        emit odomUpdated(odom);  // ✅ 转发给 GUI
    }

    void onPathPublish(const Path& p) {
        emit pathUpdated(p);
    }         // 新增
    void onPointCloudPublish(const PointCloudMsg& p) {
        emit pointCloudUpdated(p);
    }

signals:
    void startImu(int intervalMs);
    void startLidar(int intervalMs);
    void startCamera(int intervalMs);
    void stopImu();
    void stopLidar();
    void stopCamera();

    void odomUpdated(const Odometry& odom);        // 位姿
    void pathUpdated(const Path& path);             // 轨迹
    void pointCloudUpdated(const PointCloudMsg& cloud); // 点云


private:
    std::shared_ptr<ImuBase> _imu =  nullptr;
    std::shared_ptr<LidarBase> _lidar = nullptr;
    std::shared_ptr<CameraBase> _camera = nullptr;
    std::shared_ptr<AlgorithmMainBase> main_worker_instance = nullptr;
    std::shared_ptr<QThread> imu_thread;
    std::shared_ptr<QThread> lidar_thread;
    std::shared_ptr<QThread> camera_thread;
    bool _use_multi_thread = true;

};


#endif //FASTLIO_QT_LIOBASE_H