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
// #include
class LioBase {

public:
    LioBase() {
        imu_thread = QSharedPointer<QThread>();
        lidar_thread = QSharedPointer<QThread>();

    }
    ~LioBase() {

    }
    void addIMUinstance(std::shared_ptr<ImuBase> imu) {
        _imu = std::move(imu);
        _imu->moveToThread(imu_thread.get());
        main_worker_instance = std::make_shared<MainWorker>();

    }
    void addLidarInstance(std::shared_ptr<LidarBase> lidar) {
        _lidar = std::move(lidar);
        _lidar->moveToThread(lidar_thread.get());
    }



private:
    std::shared_ptr<ImuBase> _imu =  nullptr;
    std::shared_ptr<LidarBase> _lidar = nullptr;
    std::shared_ptr<MainWorker> main_worker_instance = nullptr;
    QSharedPointer<QThread> imu_thread;
    QSharedPointer<QThread> lidar_thread;

};


#endif //FASTLIO_QT_LIOBASE_H