//
// Created by dzl on 2025/11/29.
//

#ifndef FASTLIO_QT_SENSORBASE_H
#define FASTLIO_QT_SENSORBASE_H

#include <QObject>
#include <QTimer>
#include "../Msgs/dataTypes.h"

// template <typename T>
class SensorBase : public QObject {
    Q_OBJECT

public:
    explicit SensorBase(QObject* parent = nullptr)
        : QObject(parent)
        , timer(new QTimer(this))
    {
        connect(timer, &QTimer::timeout, this, &SensorBase::loop);
    }

    ~SensorBase() override = default;

    bool start(int intervalMs = 10) {
        timer->setInterval(intervalMs);
        timer->start();
        std::cout << this->metaObject()->className() << " 开始运行！" << std::endl;
        return true;
    }

    void stop() {
        timer->stop();
    }

public slots:
    // 纯虚函数：强制子类实现采样/处理逻辑
    virtual void loop() = 0;

    signals:
        // 泛化数据类型：T::Ptr（需 T 定义 using Ptr = std::shared_ptr<T>）
        // void dataReady(typename T::Ptr data);

protected:
    QTimer* timer;  // managed by QObject parent
};

#endif //FASTLIO_QT_SENSORBASE_H