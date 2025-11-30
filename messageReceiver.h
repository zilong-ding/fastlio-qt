//
// Created by dzl on 11/20/25.
//

#ifndef FASTLIO_QT_MESSAGERECEIVER_H
#define FASTLIO_QT_MESSAGERECEIVER_H
#include <zmq.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
// #include <pcl_conversions/pcl_conversions.h>

#include <QThread>
#include "Msgs/dataTypes.h"
using json = nlohmann::json;

class messageReceiver:public QObject {
    Q_OBJECT
public slots:
    void doWork();
    void setStop() {
        std::cout << "messageReceiver setStop" << std::endl;
        stop = true;
        std::cout << "stop is: " << stop << std::endl;
    }

public:
    messageReceiver(QObject *parent = nullptr):QObject(parent) {
        std::cout << "messageReceiver create\n";
    };
    ~messageReceiver() = default;


signals:
    void sendImu(std::shared_ptr<IMU> imu_data);
    void sendLidar(std::shared_ptr<LidarFrame> lidar_data);





private:
    std::string address = "ipc:///tmp/livox_stream";
    bool stop = false;
};


#endif //FASTLIO_QT_MESSAGERECEIVER_H