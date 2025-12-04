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
    explicit SLAMBase(bool use_multi_thread =  true,QObject* parent =  nullptr) : QObject(parent)
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

    }
    void addIMUinstance(std::shared_ptr<ImuBase> imu) {
        _imu = std::move(imu);
        if (_use_multi_thread) _imu->moveToThread(imu_thread.get());
        auto res = connect(this, &SLAMBase::startImu, _imu.get(), &ImuBase::start);
        emit startImu(1);
        std::cout << "connect imu to main worker " << res << std::endl;
    }
    void addLidarInstance(std::shared_ptr<LidarBase> lidar) {
        _lidar = std::move(lidar);
        if (_use_multi_thread) _lidar->moveToThread(lidar_thread.get());
        auto res = connect(this, &SLAMBase::startLidar, _lidar.get(), &LidarBase::start);
        std::cout << "connect lidar to main worker " << res << std::endl;
    }
    void addAlgorithmInstance(std::shared_ptr<AlgorithmMainBase>  algorithm) {
        main_worker_instance = std::move(algorithm);
        auto res = connect(main_worker_instance.get(), &AlgorithmMainBase::publishOdom, this, &SLAMBase::getOdometry);
        std::cout << "connect main worker to odom " << res << std::endl;
    }

    void addCameraInstance(std::shared_ptr<CameraBase> camera) {
        _camera = std::move(camera);
        if (_use_multi_thread) _camera->moveToThread(camera_thread.get());
        auto res = connect(this, &SLAMBase::startCamera, _camera.get(), &CameraBase::start);
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
        res = main_worker_instance->start();
        std::cout << "main worker start " << res << std::endl;
        if (_imu)
        {
            emit startImu(1);
        }
        if (_lidar)
        {
            emit startLidar(1);
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
            if (imu_thread) {
                QMetaObject::invokeMethod(_imu.get(), "stop", Qt::QueuedConnection, Q_RETURN_ARG(bool, res));
            }
            else {
                _imu->stop();
            }
        }
        if (_lidar)
        {
            if (lidar_thread) {
                QMetaObject::invokeMethod(_lidar.get(), "stop", Qt::QueuedConnection, Q_RETURN_ARG(bool, res));
            }
            else {
                _lidar->stop();
            }
        }
        if (_camera) {
            if (camera_thread) {
                QMetaObject::invokeMethod(_camera.get(), "stop", Qt::QueuedConnection, Q_RETURN_ARG(bool, res));
            }
            else {
                _camera->stop();
            }
        }
        return res;
    }

public slots:
    void getOdometry( Odometry odom) {
        auto pose = odom.pose.pose;
        std::cout << "odom: " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
    }

signals:
    void startImu(int intervalMs);
    void startLidar(int intervalMs);
    void startCamera(int intervalMs);

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