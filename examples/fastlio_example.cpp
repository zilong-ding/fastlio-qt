//
// Created by dzl on 2025/12/2.
//

#include "../Sensors/SensorType.h"
#include "../SLAM/FastLio/FastLioMain.h"
#include "../SLAM/FastLio/FastlioConfig.h"
#include "../SLAM/SLAMBase.h"
#include "../http/httpserver.h"
#include <zmq.hpp>
#include <QObject>
#include <QApplication>
class IMUReceiver:public ImuBase {
    Q_OBJECT
public:
    explicit IMUReceiver (QObject* parent = nullptr):ImuBase(parent) {
        sock = zmq::socket_t(ctx, ZMQ_SUB);
        sock.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
        sock.connect("ipc:///tmp/imu_stream");

        // sock.setsockopt(zmq::recv_flags::dontwait, 10) ;

        std::cout << "IMUReceiver started, waiting for messages..." << std::endl;
    }
    ~IMUReceiver() {
        sock.close();
    }
public slots:
    void loop() override{
        zmq::message_t msg;
        try {
            // if (!sock.recv(msg, zmq::recv_flags::none)) {
            //     return;
            // }
            sock.recv(msg, zmq::recv_flags::none);
            std::string payload(static_cast<char*>(msg.data()), msg.size());
            // Parse JSON
            auto j = json::parse(payload);
            // 🔍 Topic discrimination via top-level keys
            if (j.contains("orientation") && j.contains("angular_velocity"))
            {
                auto imuPtr = std::make_shared<IMU>();
                try {
                    j.get_to(*imuPtr);
                }
                catch (const json::exception& e)
                {
                    std::cerr << "[JSON] IMU parse fail: " << e.what() << "\n";
                    return;
                }
                emit sendIMUData(imuPtr);  // 安全：imuPtr 独占数据
            }
            else {
                // std::cerr << "[?] Unknown message type\n";
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }
private:
    zmq::context_t ctx;
    zmq::socket_t sock;
};

class LidarReceiver:public LidarBase {
    Q_OBJECT
public:
    explicit LidarReceiver(QObject* parent = nullptr):LidarBase(parent) {
        sock = zmq::socket_t(ctx, ZMQ_SUB);
        sock.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
        sock.connect("ipc:///tmp/lidar_stream");
        std::cout << "LidarReceiver started, waiting for messages..." << std::endl;
    }
    ~LidarReceiver() override {
        sock.close();
    }

    void loop() override {
        // std::cout << "LidarReceiver, waiting for messages..." << std::endl;
        zmq::message_t msg;
        try {
                // if (!sock.recv(msg, zmq::recv_flags::none)) {
                //     return;
                // }
                sock.recv(msg, zmq::recv_flags::none);
                std::string payload(static_cast<char*>(msg.data()), msg.size());
                // Parse JSON
                auto j = json::parse(payload);
                // 🔍 Topic discrimination via top-level keys
                if (j.contains("points") && j.contains("timebase")) {
                    auto framePtr = std::make_shared<LidarFrame>();
                    try {
                        j.get_to(*framePtr);
                    } catch (const json::exception& e) {
                        std::cerr << "[JSON] Lidar parse fail: " << e.what() << "\n";
                        return;
                    }
                    PointCloud2::Ptr data = std::make_shared<PointCloud2>();
                    LidarFrame2Pointcloud2(framePtr,data);
                    emit sendLidarData(data);
                }
                else {
                    // std::cerr << "[?] Unknown message type\n";
                }
            }
        catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }
public slots:
private:
    zmq::context_t ctx;
    zmq::socket_t sock;
};
#include <QMainWindow>
#include <QVTKRenderWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include "ui_mainwindow.h"
vtkSmartPointer<vtkPolyData> CreateXYGridPlane(
    double width = 10.0,
    double height = 10.0,
    int nx = 10,    // X方向分段数
    int ny = 10,    // Y方向分段数
    double z = 0.0  // 平面 Z 高度
);
QT_BEGIN_NAMESPACE
namespace Ui {
    class MainWindow;
}
QT_END_NAMESPACE
class SLAMDisplay: public QMainWindow{
    Q_OBJECT
public:
    explicit SLAMDisplay(QWidget *parent = nullptr):
    QMainWindow(parent),ui(new Ui::MainWindow)
    {
        ui->setupUi(this);
        auto vtkWidget = ui->openGLWidget1;

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
        renderWindow->Render();
        vtkWidget->update();
        viewer->addCoordinateSystem(10,"world");
        lastPoint = pcl::PointXYZ(0.0,0.0,0.0);


        http_server = std::make_shared<HttpServer>();
        server_thread = std::make_shared<QThread>();
        http_server->moveToThread(server_thread.get());
        server_thread->start();
    }
    ~SLAMDisplay() override {
        slam_base->stop();
    }
    void addSLAMBase(const std::shared_ptr<SLAMBase> &slambase) {
        slam_base = slambase;
        connect(slam_base.get(), &SLAMBase::odomUpdated, this, &SLAMDisplay::updateOdometry);
        connect(slam_base.get(), &SLAMBase::pathUpdated, this, &SLAMDisplay::updatePath);
        connect(slam_base.get(), &SLAMBase::pointCloudUpdated, this, &SLAMDisplay::updatePointcloud);
        connect(slambase.get(),&SLAMBase::odomUpdated,http_server.get(),&HttpServer::updateOdom);
        connect(this,&SLAMDisplay::startHttp,http_server.get(),&HttpServer::start);
        slam_base->start();
        emit startHttp();
    }
    void updateviewer() {
        // viewer->spinOnce(50);
        // viewer->spinOnce();
        renderWindow->Render();
        ui->openGLWidget1->update();
        // vtkWidget->update();
    }

    signals:
    void startHttp();

public slots:
    void updateOdometry(const Odometry& odom);
    void updatePath(const Path& path);
    void updatePointcloud(const PointCloudMsg& cloud);

private:
    Ui::MainWindow *ui;
    // std::shared_ptr<QVTKRenderWidget> vtkWidget;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointXYZ lastPoint;
    vtkNew<vtkRenderer> renderer ;
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow ;
    std::shared_ptr<SLAMBase> slam_base;
    std::shared_ptr<HttpServer> http_server;
    std::shared_ptr<QThread> server_thread;
};

void SLAMDisplay::updateOdometry(const Odometry& odom) {
    std::cout << "updateOdometry" << std::endl;
    // 1. 提取 position 和 orientation
    auto &pose = odom.pose.pose;
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

void SLAMDisplay::updatePath(const Path& path) {
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

void SLAMDisplay::updatePointcloud(const PointCloudMsg& cloud) {
    auto &cloud_in = cloud.pointCloud;
    auto name = QUuid::createUuid().toString().toStdString();
    std::cout << "point size: " << cloud_in->size() << std::endl;
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> fildColor(cloud_in,"intensity");
    viewer->addPointCloud<PointType>(cloud_in,fildColor, name);
    viewer->spinOnce(100);
    // vtkWidget->update();

}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWorkerConfig cfg;
    std::ifstream f("/home/dzl/CLionProjects/fastlio-qt/examples/fastlio.json");
    nlohmann::json j;
    f >> j;
    cfg = j.get<MainWorkerConfig>(); // 自动调用 from_json
    auto imu = std::make_shared<IMUReceiver>();
    auto lidar = std::make_shared<LidarReceiver>();
    auto fastlio = std::make_shared<FastLioMain>();
    fastlio->initParams(cfg);
    auto slam = std::make_shared<SLAMBase>();
    slam->addAlgorithmInstance(fastlio);
    slam->addIMUinstance(imu);
    slam->addLidarInstance(lidar);
    auto res = slam->connectSlots();
    auto displayer = SLAMDisplay();
    displayer.addSLAMBase(slam);
     // res &= slam->start();
    if (!res) {
        std::cout << "start failed" << std::endl;
        return -1;
    }
    std::cout << "hello fastlio_example" << std::endl;
    displayer.show();


    return app.exec();
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


#include "fastlio_example.moc"