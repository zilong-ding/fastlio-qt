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
#include "../Sensors/SensorType.h"
// #include <shared_ptr>
// struct IMU;
// struct PointCloud2;
// // struct MeasureGroup;
// struct Path;
// struct Odometry;
// struct PointCloudMsg;

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
    bool start(int intervalMs = 10) {
        timer->start(intervalMs);
        std::cout << this->metaObject()->className() << " 开始运行！" << std::endl;
        return true;
    }



public slots:
    virtual void imuCallback(IMU::Ptr msg_in) {
        std::cout << "也许你忘记重载 imuCallback 函数了！" << std::endl;
    };
    virtual void lidarCallback(PointCloud2::Ptr msg) {
        std::cout << "也许你忘记重载 lidarCallback 函数了！" << std::endl;
    };
    virtual void imageCallback(Image::Ptr msg) {
        std::cout << "也许你忘记重载 imageCallback 函数了！" << std::endl;
    };
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
    std::deque<IMU::Ptr> imu_buffer;
    // 🔥 恢复并加入 lidar_buffer
    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<Image::Ptr> image_buffer;

    // 时间缓存（用于 sync）
    std::deque<double> time_buffer;

    std::condition_variable sig_buffer;

    double last_timestamp_lidar;
    double last_timestamp_imu;
    double last_timestamp_image;

    Odometry odomAftMapped;
    Path path;

signals:
    void publishOdom(Odometry odom);
    void PathPublish(Path p);
    void PointCloudPublish(PointCloudMsg p);
};


#endif //FASTLIO_QT_ALGORITHMMAINBASE_H