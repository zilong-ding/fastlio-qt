//
// Created by dzl on 2025/11/24.
//

#include <vtkPlaneSource.h>
#include "Display.h"
#include "ui_mainwindow.h"
#include <vtkGenericOpenGLRenderWindow.h>
vtkSmartPointer<vtkPolyData> CreateXYGridPlane(
    double width = 10.0,
    double height = 10.0,
    int nx = 10,    // X方向分段数
    int ny = 10,    // Y方向分段数
    double z = 0.0  // 平面 Z 高度
);
Displayer::Displayer(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    vtkWidget = std::make_shared<QVTKRenderWidget>();
    auto layout = ui->centralwidget->layout();
    layout->addWidget(vtkWidget.get());

    auto polyData = CreateXYGridPlane(
    500.0,
    500.0,
    10,    // X方向分段数
    10,    // Y方向分段数
    0.0  // 平面 Z 高度
    );

    // Create a mapper and actor
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    renderer = vtkNew<vtkRenderer>();
    renderWindow = vtkNew<vtkGenericOpenGLRenderWindow>();

    renderWindow->AddRenderer(renderer);
    renderer->AddActor(actor);
    viewer  = std::make_shared<pcl::visualization::PCLVisualizer>(renderer, renderWindow, "viewer", false);
    vtkWidget->setRenderWindow(viewer->getRenderWindow().Get());
    viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
    vtkWidget->update();
    viewer->addCoordinateSystem(10,"world");
    lastPoint = pcl::PointXYZ(0.0,0.0,0.0);

    initThread();
    connectSolts();
    std::cout << "Displayer created"<< std::endl;
    // dataReceiver->dumpObjectInfo();
    // mainWorker->dumpObjectInfo();
    emit beginLoop();
    sleep(1);
    emit beginReceive();

}

Displayer::~Displayer()
{
    // dataReceiver->setStop();
    // mainWorker->setStop();
    stopThread();
    delete ui;
    std::cout << "Displayer destroyed"<< std::endl;
}

void Displayer::connectSolts() {
    // connect(renderTimer, &QTimer::timeout, this, [=](){
    //     viewer->spinOnce();
    //     // vtkWidget->update();                   // 推荐
    //     // vtkWidget->renderWindow()->Render();
    // });
    // // renderTimer->start(100);
    // renderTimer->start(10); // 60 FPS
    connect(this,&Displayer::beginReceive,dataReceiver.get(),&messageReceiver::doWork);
    connect(this,&Displayer::sendStop,mainWorker.get(),&MainWorker::setStop);
    connect(this,&Displayer::sendStop,dataReceiver.get(),&messageReceiver::setStop);
    // connect(this,&Displayer::beginLoop,mainWorker.get(),&MainWorker::loop);

    connect(dataReceiver.get(),&messageReceiver::sendImu,mainWorker.get(),&MainWorker::imuCallback);
    connect(dataReceiver.get(),&messageReceiver::sendLidar,mainWorker.get(),&MainWorker::lidarCallback);

    connect(mainWorker.get(),&MainWorker::PathPublish,this,&Displayer::updatePath);
    connect(mainWorker.get(),&MainWorker::publish,this,&Displayer::updateOdometry);
    connect(mainWorker.get(),&MainWorker::PointCloudPublish,this,&Displayer::updatePointcloud);

}

void Displayer::updateOdometry(Odometry odometry) {
    // std::cout << "updateOdometry" << std::endl;
    // 1. 提取 position 和 orientation
    auto &pose = odometry.pose.pose;
    auto &position = pose.position;
    auto &quat = pose.orientation;
    // 2. 【关键】检查四元数是否有效（避免 NaN/Inf）
    double norm = std::sqrt(quat.x * quat.x + quat.y * quat.y +
                            quat.z * quat.z + quat.w * quat.w);
    if (norm < 1e-6 || std::isnan(norm) || std::isinf(norm)) {
        std::cerr << "[Displayer] Invalid quaternion in odometry, skipping update.\n";
        return;
    }

    // 3. 归一化四元数（Eigen 要求单位四元数）
    double inv_norm = 1.0 / norm;
    Eigen::Quaternionf q(
        static_cast<float>(quat.w * inv_norm),
        static_cast<float>(quat.x * inv_norm),
        static_cast<float>(quat.y * inv_norm),
        static_cast<float>(quat.z * inv_norm)
    );

    // 4. 构建 Eigen::Affine3f 变换
    Eigen::Affine3f transform = Eigen::Translation3f(
        static_cast<float>(position.x),
        static_cast<float>(position.y),
        static_cast<float>(position.z)
    ) * q;  // 先平移后旋转（等价于 T * R）

    // 5. 更新或添加坐标系
    try {
        if (viewer->contains("odom")) {
            viewer->updateCoordinateSystemPose("odom", transform);
        } else {
            // 第二个参数是坐标轴长度（1.0 表示 1 米长）
            viewer->addCoordinateSystem(5.0f, transform, "odom");
            // 可选：设置坐标轴线宽（默认较细，看不清可加粗）
            // viewer->setPointCloudRenderingProperties(
            //     pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "odom");
        }
    } catch (const std::exception& e) {
        std::cerr << "[Displayer] Error updating odom coordinate system: "
                  << e.what() << std::endl;
    }
    updateviewer();
    // vtkWidget->update();
}

void Displayer::updatePath(Path path) {
    auto pose = path.poses.back().pose.position;

    pcl::PointXYZ newPoint(pose.x,pose.y,pose.z);
    auto name = QUuid::createUuid().toString().toStdString();
    // viewer->addLine()
    viewer->addLine(lastPoint,newPoint,0.0,1.0,0.0,name);
    viewer->setShapeRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
        10.0,                      // 线宽（像素）
        name
        );
    viewer->spinOnce(30);
    // vtkWidget->update();
    lastPoint = newPoint;
}

void Displayer::updatePointcloud(PointCloudMsg point_cloud_msg) {
    auto &cloud_in = point_cloud_msg.pointCloud;
    auto name = QUuid::createUuid().toString().toStdString();
    std::cout << "point size: " << cloud_in->size() << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> fildColor(cloud_in,"intensity");
    viewer->addPointCloud<PointType>(cloud_in,fildColor, name);
    viewer->spinOnce(100);
    // vtkWidget->update();

}

void Displayer::initThread() {
    dataReceiver = std::make_shared<messageReceiver>();
    mainWorker = std::make_shared<MainWorker>();
    dataReceiver_thread = std::make_shared<QThread>();
    mainWorker_thread = std::make_shared<QThread>();
    dataReceiver->moveToThread(dataReceiver_thread.get());
    mainWorker->moveToThread(mainWorker_thread.get());
    dataReceiver_thread->start();
    mainWorker_thread->start();

}

void Displayer::stopThread() {
    emit sendStop();
    // emit sendStop();
    // 1. 停 mainWorker_thread
    mainWorker_thread->quit();
    if (!mainWorker_thread->wait(3000)) {  // 给足时间
        mainWorker_thread->terminate();    // 实在不退出再 terminate
        mainWorker_thread->wait();
    }
    mainWorker->deleteLater();
    mainWorker_thread->deleteLater();

    // 2. 停 dataReceiver_thread
    emit sendStop();
    dataReceiver_thread->quit();
    if (!dataReceiver_thread->wait(3000)) {
        dataReceiver->setStop();
        dataReceiver_thread->terminate();
        dataReceiver_thread->wait();
    }
    // 3. 销毁 worker 和 thread
    dataReceiver->deleteLater();
    dataReceiver_thread->deleteLater();
}

vtkSmartPointer<vtkPolyData> CreateXYGridPlane(
    double width ,
    double height ,
    int nx ,    // X方向分段数
    int ny ,    // Y方向分段数
    double z  // 平面 Z 高度
) {
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();

    double x0 = -width / 2.0, y0 = -height / 2.0;
    double dx = width / nx, dy = height / ny;

    // 生成点
    for (int j = 0; j <= ny; ++j)
        for (int i = 0; i <= nx; ++i)
            points->InsertNextPoint(x0 + i * dx, y0 + j * dy, z);

    // 水平线
    for (int j = 0; j <= ny; ++j)
        for (int i = 0; i < nx; ++i) {
            vtkIdType id0 = j * (nx + 1) + i;
            vtkIdType id1 = id0 + 1;
            lines->InsertNextCell(2);
            lines->InsertCellPoint(id0);
            lines->InsertCellPoint(id1);
        }

    // 垂直线
    for (int i = 0; i <= nx; ++i)
        for (int j = 0; j < ny; ++j) {
            vtkIdType id0 = j * (nx + 1) + i;
            vtkIdType id1 = id0 + (nx + 1);
            lines->InsertNextCell(2);
            lines->InsertCellPoint(id0);
            lines->InsertCellPoint(id1);
        }

    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    poly->SetLines(lines);
    return poly;
}