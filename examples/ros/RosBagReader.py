from rosbags.rosbag1 import Reader

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
from rosbags.typesys import get_types_from_idl, get_types_from_msg
import orjson  # 比 json 快 5~10 倍，支持 numpy
# protocol.py (Python & C++ 共享逻辑)
# 使用 msgpack 紧凑二进制序列化
from Msg import IMU,LidarFrame,CustomPoint
import msgpack
import numpy as np
import zmq
import time
context = zmq.Context()
# socket = context.socket(zmq.PUB)  # 或 PUB
# socket.bind("ipc:///tmp/livox_stream")  # 同机用 ipc://，跨机用 tcp://127.0.0.1:5555
imu_socket   = context.socket(zmq.PUB)
lidar_socket = context.socket(zmq.PUB)

imu_socket.bind("ipc:///tmp/imu_stream")
lidar_socket.bind("ipc:///tmp/lidar_stream")

msg_text = Path('/home/dzl/PycharmProjects/coppeliasim/ros/msg/CustomPoint').read_text()
msg_text1 = Path('/home/dzl/PycharmProjects/coppeliasim/ros/msg/CustomMsg').read_text()
msg_text2 = Path('/home/dzl/PycharmProjects/coppeliasim/ros/msg/ImuMsg').read_text()
add_types = {}

# Add definitions from one idl file to the dict.
# add_types.update(get_types_from_idl(idl_text))

# Add definitions from one msg file to the dict.
add_types.update(get_types_from_msg(msg_text, 'livox_ros_driver/msg/CustomPoint'))
add_types.update(get_types_from_msg(msg_text1, 'livox_ros_driver/msg/CustomMsg'))
# add_types.update(get_types_from_msg(msg_text2, 'sensor_msgs/msg/Imu'))

# bagpath = Path('/home/dzl/ros2_ws/outdoor_run_100Hz_2020-12-27-17-12-19.bag')
bagpath = Path('/home/dzl/ros2_ws/100hz_2021-02-05-19-50-54.bag')
# bagpath = Path('/home/dzl/ros2_ws/UAV_flip_over_0328.bag')
typestore = get_typestore(Stores.ROS1_NOETIC)
typestore.register(add_types)
# Create reader instance and open for reading.
# ---- 记录起始时间 ----
bag_start_time = None
replay_start_wall = None
with AnyReader([bagpath], default_typestore=typestore) as reader:
    # connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages():
        # --- timestamp 是 nanoseconds, 转成秒 ---
        t = timestamp * 1e-9

        # --- 第一次记录 ---
        if bag_start_time is None:
            bag_start_time = t
            replay_start_wall = time.time()

        # ---- 计算应当播放到的时间点 ----
        target_time = replay_start_wall + (t - bag_start_time)

        # ---- sleep 到该时间点 ----
        now = time.time()
        if target_time > now:
            time.sleep(target_time - now)
        if connection.topic == '/livox/lidar':
            # print(connection.msgtype)
            # print(timestamp,  connection.msgtype)
            msg = reader.deserialize(rawdata, connection.msgtype)
            # print(len(msg.points))
            frame = LidarFrame.from_ros_msg( msg)
            print("/livox/lidar pub: ", frame)
            lidar_socket.send(orjson.dumps(frame.to_dict()))  # 二进制 bytes
            # print("send one pointCloud")
        elif connection.topic == '/livox/imu':
            msg = reader.deserialize(rawdata, connection.msgtype)
            print(msg)
            imu = IMU.from_ros_msg(msg)
            print("/livox/imu pub: ", imu)
            imu_socket.send(orjson.dumps(imu.to_dict()))
            # print("send one imu")