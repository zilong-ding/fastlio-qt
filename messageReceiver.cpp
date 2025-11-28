//
// Created by dzl on 11/20/25.
//

#include "messageReceiver.h"

void messageReceiver::doWork() {
    zmq::context_t ctx;
    zmq::socket_t sock(ctx, ZMQ_SUB);
    sock.set(zmq::sockopt::subscribe, ""); // Subscribe to all messages
    sock.connect("ipc:///tmp/livox_stream");
    std::cout << "C++ receiver started, waiting for messages...\n";
    auto status = !stop;
    while (status) {
        // std::cout << "status: " << status << std::endl;
        zmq::message_t msg;
        try {
            if (!sock.recv(msg, zmq::recv_flags::none)) {
                // sleep(1);
                QThread::msleep(100);
                status = !stop;
                break;
            }

            // Convert zmq message to string
            std::string payload(static_cast<char*>(msg.data()), msg.size());

            // Parse JSON
            auto j = json::parse(payload);

            // 🔍 Topic discrimination via top-level keys
            if (j.contains("orientation") && j.contains("angular_velocity")) {
                auto imuPtr = std::make_shared<IMU>();
                try {
                    j.get_to(*imuPtr);
                } catch (const json::exception& e) {
                    std::cerr << "[JSON] IMU parse fail: " << e.what() << "\n";
                    continue;
                }
                emit sendImu(imuPtr);  // 安全：imuPtr 独占数据

            } else if (j.contains("points") && j.contains("timebase")) {
                auto framePtr = std::make_shared<LidarFrame>();
                try {
                    j.get_to(*framePtr);
                } catch (const json::exception& e) {
                    std::cerr << "[JSON] Lidar parse fail: " << e.what() << "\n";
                    continue;
                }
                emit sendLidar(framePtr);
            }
            else {
                std::cerr << "[?] Unknown message type\n";
            }

        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            // Optionally skip malformed messages
        }
        status = !stop;
    }
    sock.close();
    std::cout << "messageReceiver stoped"<< std::endl;
    // return true;
}

