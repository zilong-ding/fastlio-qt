//
// Created by dzl on 2025/11/30.
//

#ifndef FASTLIO_QT_ALGORITHMMAINBASE_H
#define FASTLIO_QT_ALGORITHMMAINBASE_H
#include <QObject>
#include <QtCore>
#include <QTimer>
#include <iostream>
#include <memory>
#include <mutex>
#include <deque>
#include <condition_variable>
// #include <shared_ptr>
struct IMU;
struct PointCloud2;
// struct MeasureGroup;
struct Path;
struct Odometry;
struct PointCloudMsg;

class AlgorithmMainBase : public QObject {
    Q_OBJECT
public:
    explicit AlgorithmMainBase(QObject *parent = nullptr)
        : QObject(parent),
          timer(new QTimer(this)),
          last_timestamp_lidar(0),
          last_timestamp_imu(-1.0),
          stop(false)
    {
        connect(timer, &QTimer::timeout, this, &AlgorithmMainBase::loop);
    }

    ~AlgorithmMainBase() override {
        if (timer) timer->stop();

        std::lock_guard<std::mutex> lk(mtx_buffer);
        imu_buffer.clear();
        lidar_buffer.clear();
        time_buffer.clear();
    }



public slots:
    virtual void imuCallback(std::shared_ptr<IMU> msg_in) = 0;
    virtual void lidarCallback(std::shared_ptr<PointCloud2> msg) = 0;
    virtual void loop() = 0;

    void setStop() {
        stop = true;
        if (timer) timer->stop();
        sig_buffer.notify_all();
    }

protected:
    bool stop;
    QTimer* timer;

    // 缓冲区：IMU + Lidar
    std::mutex mtx_buffer;
    std::deque<std::shared_ptr<IMU>> imu_buffer;
    // 🔥 恢复并加入 lidar_buffer
    std::deque<std::shared_ptr<LidarFrame>> lidar_buffer;

    // 时间缓存（用于 sync）
    std::deque<double> time_buffer;

    std::condition_variable sig_buffer;

    double last_timestamp_lidar;
    double last_timestamp_imu;

    Odometry odomAftMapped;
    Path path;

    signals:
    void publishOdom(Odometry odom);
    void PathPublish(Path p);
    void PointCloudPublish(PointCloudMsg p);
};


#endif //FASTLIO_QT_ALGORITHMMAINBASE_H