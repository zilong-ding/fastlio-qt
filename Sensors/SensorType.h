//
// Created by dzl on 2025/11/29.
//

#ifndef FASTLIO_QT_SENSORTYPE_H
#define FASTLIO_QT_SENSORTYPE_H
#include <QObject>
#include "SensorBase.h"


/**
 * @brief IMU传感器基类，继承自SensorBase类
 *
 * 该类为IMU传感器提供了一个基础框架，定义了数据发送信号和默认的循环处理函数。
 * 用户需要继承此类并重写loop()函数来实现具体的IMU数据采集逻辑。
 */
class ImuBase : public SensorBase {
    Q_OBJECT
public:
    /**
     * @brief 构造函数
     * @param parent 父对象指针，默认为nullptr
     */
    explicit ImuBase(QObject* parent = nullptr) : SensorBase(parent) {
        std::cout << "ImuBase created" << std::endl;
    }

    /**
     * @brief 虚析构函数
     */
    ~ImuBase() override = default;

    void loop() override{
        std::cerr << "您必须重载ImuBase::loop函数来获取数据！" << std::endl;
    }

public slots:
    /**
     * @brief 循环处理函数，用于获取传感器数据
     *
     * 此函数提供了默认实现，会输出提示信息要求用户重载此函数。
     * 实际使用中应被派生类重写以实现具体的IMU数据采集逻辑。
     */


signals:
    /**
     * @brief 发送IMU数据的信号
     * @param data IMU数据指针
     */
    void sendIMUData(IMU::Ptr data);
};


/**
 * @brief 激光雷达传感器基类，继承自SensorBase类
 *
 * 该类为激光雷达传感器提供了一个基础框架，定义了点云数据发送信号和默认的循环处理函数。
 * 用户需要继承此类并重写loop()函数来实现具体的激光雷达数据采集逻辑。
 */
class LidarBase : public SensorBase {
    Q_OBJECT
public:
    /**
     * @brief 构造函数
     * @param parent 父对象指针，默认为nullptr
     */
    explicit LidarBase(QObject* parent = nullptr) : SensorBase(parent) {
        std::cout << "LidarBase created" << std::endl;
    }

    /**
     * @brief 虚析构函数
     */
    ~LidarBase() override = default;
    /**
     * @brief 循环处理函数，用于获取传感器数据
     *
     * 此函数提供了默认实现，会输出提示信息要求用户重载此函数。
     * 实际使用中应被派生类重写以实现具体的激光雷达数据采集逻辑。
     */
    void loop() override {
        std::cerr << "您必须重载LidarBase::loop函数来获取数据！" << std::endl;
    }

public slots:


signals:
    /**
     * @brief 发送激光雷达点云数据的信号
     * @param data 点云数据指针
     */
    void sendLidarData(PointCloud2::Ptr data);
};

/**
 * @brief 相机传感器基类，继承自SensorBase类
 *
 * 该类为相机传感器提供了一个基础框架，定义了默认的循环处理函数。
 * 用户需要继承此类并重写loop()函数来实现具体的相机数据采集逻辑。
 */
class CameraBase : public SensorBase {
    Q_OBJECT
public:
    /**
     * @brief 构造函数
     * @param parent 父对象指针，默认为nullptr
     */
    explicit CameraBase(QObject* parent = nullptr) : SensorBase(parent) {
        std::cout << "CameraBase created" << std::endl;
    }

    /**
     * @brief 虚析构函数
     */
    ~CameraBase() override = default;
    // std::vector<double> camera_matrix = std::vector<double>(9,0.0);
    // std::vector<double> distortion_coefficients = std::vector<double>(5,0.0);

public slots:
    /**
     * @brief 循环处理函数，用于获取传感器数据
     *
     * 此函数提供了默认实现，会输出提示信息要求用户重载此函数。
     * 实际使用中应被派生类重写以实现具体的相机数据采集逻辑。
     */
    void loop() override {
        std::cerr << "您必须重载CameraBase::loop函数来获取数据！" << std::endl;
    }

signals:
    void sendCameraData(Image::Ptr data);

};




#endif //FASTLIO_QT_SENSORTYPE_H