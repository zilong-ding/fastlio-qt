//
// Created by dzl on 2025/12/27.
//

#ifndef FASTLIO_QT_HTTPSERVER_H
#define FASTLIO_QT_HTTPSERVER_H

#include <httplib.h>
#include <QObject>
#include "nlohmann/json.hpp"
#include "../Msgs/dataTypes.h"
struct detectionStatus {
        // ====== 平均距离 ======
    double front_left_mean = 0.0;
    double front_right_mean = 0.0;
    double back_left_mean = 0.0;
    double back_right_mean = 0.0;
    double front_dis_mean = 0.0;   // 原 front_dis

    // ====== 最小距离 + 时间戳（1~4）=====
    double min_distance1 = 0.0;
    int64_t timestamp1 = 0;  // 单位建议：微秒（us）

    double min_distance2 = 0.0;
    int64_t timestamp2 = 0;

    double min_distance3 = 0.0;
    int64_t timestamp3 = 0;

    double min_distance4 = 0.0;
    int64_t timestamp4 = 0;

    // ====== 导航时间戳 ======
    int64_t timestamp5 = 0;  // second nav front
    int64_t timestamp6 = 0;  // second nav back
    int64_t timestamp7 = 0;  // firstnav back left finish
    int64_t timestamp8 = 0;  // firstnav back right finish
    int64_t timestamp9 = 0;  // firstnav front finish

    // ====== 距离范围（10.31 新增）=====
    double front_left_disran = 0.0;   // distance range / std
    double front_right_disran = 0.0;
    double back_left_disran = 0.0;
    double back_right_disran = 0.0;
    double front_disran = 0.0;

    // ====== 首次导航距离（修复拼写：fitstnav → firstnav）=====
    double firstnav_frontleft = 0.0;   // 原 fitstnav_frontleft
    double firstnav_frontright = 0.0;
    double firstnav_backleft = 0.0;
    double firstnav_backright = 0.0;

    // ====== 障碍状态 ======
    bool if_front_blocked = false;
    bool if_back_blocked = false;
    bool if_inverted_blocked = false;
    bool if_left_blocked = false;
    bool if_right_blocked = false;

    // ====== 辅助时间戳 ======
    int64_t last_update_us = 0;

    // ====== JSON 序列化支持 ======
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        detectionStatus,
        front_left_mean, front_right_mean, back_left_mean, back_right_mean, front_dis_mean,
        min_distance1, timestamp1,
        min_distance2, timestamp2,
        min_distance3, timestamp3,
        min_distance4, timestamp4,
        timestamp5, timestamp6, timestamp7, timestamp8, timestamp9,
        front_left_disran, front_right_disran, back_left_disran, back_right_disran, front_disran,
        firstnav_frontleft, firstnav_frontright, firstnav_backleft, firstnav_backright,
        if_front_blocked, if_back_blocked, if_inverted_blocked, if_left_blocked, if_right_blocked,
        last_update_us
    )
};


class HttpServer:public QObject{
Q_OBJECT
public:
    HttpServer(int port = 8080,QObject *parent = nullptr);
    ~HttpServer();

public slots:
    void start();
    void stop();
    void updateAngles(detectionStatus  status);
    void updateOdom(const Odometry& odom);
private:
    std::shared_ptr<httplib::Server> server;
    std::thread server_thread;  // ← 新增成员
    detectionStatus status_;
    Odometry odom_;
    int port_;
    std::atomic<bool> running{false};
};

inline HttpServer::HttpServer(int port, QObject *parent) : QObject(parent){
    server = std::make_shared<httplib::Server>();
    port_ = port;
    // server->listen("0.0.0.0", 8080);
}

inline HttpServer::~HttpServer() {
    if (running) {
        server->stop();
        server.reset();
        if (server_thread.joinable()) {
            server_thread.join();  // 等待线程结束
        }
    }
}

inline void HttpServer::start() {
    std::cout << "HttpServer::start" << std::endl;
    server->Get("/left_and_right_distance_id_and_minidis", [&](const httplib::Request &req, httplib::Response &res) {
        auto j = nlohmann::json(status_);
        res.set_content(j.dump(), "application/json");
    });

    server->Get("/odom", [&](const httplib::Request &req, httplib::Response &res) {
        auto j = nlohmann::json(odom_);
        res.set_content(j.dump(), "application/json");
    });
    // server->listen("0.0.0.0", port_);
    // 在新线程中启动 server
    server_thread = std::thread([this]() {
        std::cout << "HTTP server listening on 0.0.0.0:8080" << std::endl;
        running = true;
        server->listen("0.0.0.0", port_);
        running = false;
    });

}

inline void HttpServer::stop() {
    server->stop();
}

inline void HttpServer::updateAngles(detectionStatus status) {
    status_ = status;
}

inline void HttpServer::updateOdom(const Odometry &odom) {
    // std::cout << "HttpServer::updateOdom" << std::endl;
    odom_ = odom;
}

#endif //FASTLIO_QT_HTTPSERVER_H