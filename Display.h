//
// Created by dzl on 2025/11/24.
//

#ifndef FASTLIO_QT_DISPLAY_H
#define FASTLIO_QT_DISPLAY_H
#include <QWidget>
#include <QMainWindow>
// #include "widget3d/EzQVTKOpenGLNativeWidget.h"
#include <QVTKRenderWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "dataTypes.h"
#include "messageReceiver.h"
#include "MainWorker.h"
#include "QThread"
#include <QUuid>
#include <vtkGenericOpenGLRenderWindow.h>
QT_BEGIN_NAMESPACE
namespace Ui {
    class MainWindow;
}
QT_END_NAMESPACE

class Displayer: public QMainWindow {
    Q_OBJECT
public slots:
    void updateOdometry(Odometry odometry);
    void updatePath(Path path);
    void updatePointcloud(PointCloudMsg point_cloud_msg);

public:
    explicit Displayer(QWidget *parent = nullptr);
    ~Displayer() override;
    void initThread();
    void stopThread();
    void connectSolts();
    void paintEvent(QPaintEvent *event) override {
        updateviewer();
    }
    void updateviewer() {
        // viewer->spinOnce(50);
        viewer->spinOnce();
        // renderWindow->Render();
        // vtkWidget->update();
    }

signals:
    void beginLoop();
    void beginReceive();
    void sendStop();


private:

    Ui::MainWindow *ui;
    std::shared_ptr<QVTKRenderWidget> vtkWidget;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    std::shared_ptr<messageReceiver> dataReceiver;
    std::shared_ptr<MainWorker> mainWorker;
    std::shared_ptr<QThread> dataReceiver_thread;
    std::shared_ptr<QThread> mainWorker_thread;
    pcl::PointXYZ lastPoint;
    // QTimer* renderTimer = new QTimer(this);
    vtkNew<vtkRenderer> renderer ;
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow ;
};


#endif //FASTLIO_QT_DISPLAY_H