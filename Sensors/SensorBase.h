//
// Created by dzl on 2025/11/29.
//

#ifndef FASTLIO_QT_SENSORBASE_H
#define FASTLIO_QT_SENSORBASE_H

#include <QObject>
#include <QTimer>
#include "../Msgs/dataTypes.h"
#include <QDebug>
#include <QThread>
// template <typename T>
class SensorBase : public QObject {
    Q_OBJECT

public:
    explicit SensorBase(QObject* parent = nullptr)
        : QObject(parent)
        , timer(new QTimer(this))
    {
        auto res = connect(timer, &QTimer::timeout, this, &SensorBase::onTimeout);
        std::cout << "SensorBase::SensorBase"  << " 创建成功: " <<  res << std::endl;
    }

    ~SensorBase() override = default;

    // 纯虚函数：强制子类实现采样/处理逻辑
    virtual void loop() = 0;



public slots:
    void stop() {
        timer->stop();
    }

    void start(int intervalMs = 10) {
        qDebug() << "[SensorBase::start] ENTER"
             << "| class:" << metaObject()->className()
             << "| thread:" << QThread::currentThreadId()
             << "| interval:" << intervalMs;
        timer->setInterval(intervalMs);
        timer->start();
        // std::cout << this->metaObject()->className() << " 开始运行！" << std::endl;
        qDebug() << std::string("SensorBase::start") ;
        // return true;
    }

    void onTimeout() {
        auto start_time = std::chrono::high_resolution_clock::now();
        loop();  // ✅ 运行时调用子类实现，无纯虚函数取址风险
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        std::cout << metaObject()->className() << " 运行时间：" << elapsed.count()*1000 << " ms" << std::endl;
    }

protected:
    QTimer* timer;  // managed by QObject parent
};

#endif //FASTLIO_QT_SENSORBASE_H