//
// Created by dzl on 2025/11/29.
//

#ifndef FASTLIO_QT_LIOBASE_H
#define FASTLIO_QT_LIOBASE_H
#include <QObject>
#include "FastLio/MainWorker.h"
#include "../Sensors/SensorType.h"
#include <QSharedPointer>
#include <utility>
#include "AlgorithmMainBase.h"
// #include
class SLAMBase : public QObject{

public:
    explicit SLAMBase(bool use_multi_thread =  true,QObject* parent =  nullptr) : QObject(parent)
    {
        _use_multi_thread = use_multi_thread;
        if (_use_multi_thread) {
            imu_thread = QSharedPointer<QThread>();
            lidar_thread = QSharedPointer<QThread>();
            camera_thread = QSharedPointer<QThread>();
        }
        else {
            std::cout << "single thread mode" << std::endl;
        }
    }
    ~SLAMBase() {

    }
    void addIMUinstance(std::shared_ptr<ImuBase> imu) {
        _imu = std::move(imu);
        if (_use_multi_thread) _imu->moveToThread(imu_thread.get());
    }
    void addLidarInstance(std::shared_ptr<LidarBase> lidar) {
        _lidar = std::move(lidar);
        if (_use_multi_thread) _lidar->moveToThread(lidar_thread.get());
    }
    void addAlgorithmInstance(std::shared_ptr<AlgorithmMainBase>  algorithm) {
        main_worker_instance = std::move(algorithm);
        connect(main_worker_instance.get(), &AlgorithmMainBase::publishOdom, this, &SLAMBase::getOdometry);
    }

    void addCameraInstance(std::shared_ptr<CameraBase> camera) {
        _camera = std::move(camera);
        if (_use_multi_thread) _camera->moveToThread(camera_thread.get());
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
        if (_imu)
        {
            if (imu_thread) {
                QMetaObject::invokeMethod(_imu.get(), "start", Qt::QueuedConnection, Q_RETURN_ARG(bool, res));
            }
            else {
                _imu->start();
            }
        }
        if (_lidar)
        {
            if (lidar_thread) {
                QMetaObject::invokeMethod(_lidar.get(), "start", Qt::QueuedConnection, Q_RETURN_ARG(bool, res));
            }
            else {
                _lidar->start();
            }
        }
        if (_camera) {
            if (camera_thread) {
                QMetaObject::invokeMethod(_camera.get(), "start", Qt::QueuedConnection, Q_RETURN_ARG(bool, res));
            }
            else {
                _camera->start();
            }
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

private:
    std::shared_ptr<ImuBase> _imu =  nullptr;
    std::shared_ptr<LidarBase> _lidar = nullptr;
    std::shared_ptr<CameraBase> _camera = nullptr;
    std::shared_ptr<AlgorithmMainBase> main_worker_instance = nullptr;
    QSharedPointer<QThread> imu_thread;
    QSharedPointer<QThread> lidar_thread;
    QSharedPointer<QThread> camera_thread;
    bool _use_multi_thread = true;

};


#endif //FASTLIO_QT_LIOBASE_H