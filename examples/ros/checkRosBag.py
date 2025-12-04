from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path
from datetime import datetime

# 配置路径
bagpath = Path('/home/dzl/ros2_ws/outdoor_run_100Hz_2020-12-27-17-12-19.bag')
typestore = get_typestore(Stores.ROS1_NOETIC)

# 读取 bag 信息
with AnyReader([bagpath], default_typestore=typestore) as reader:
    start_time_ns = reader.start_time
    end_time_ns = reader.end_time
    duration_ns = end_time_ns - start_time_ns
    message_count = reader.message_count
    topics = reader.topics

# 时间转换（纳秒 → 秒 + 可读格式）
start_time_s = start_time_ns / 1e9
end_time_s = end_time_ns / 1e9
duration_s = duration_ns / 1e9

start_dt = datetime.utcfromtimestamp(start_time_s)
end_dt = datetime.utcfromtimestamp(end_time_s)

# 输出概览
print("=" * 80)
print("ROS BAG SUMMARY")
print("=" * 80)
print(f"Path:          {bagpath}")
print(f"Start time:    {start_time_s:.3f} s  ({start_dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]} UTC)")
print(f"End time:      {end_time_s:.3f} s  ({end_dt.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]} UTC)")
print(f"Duration:      {duration_s:.3f} s  ({duration_s/60:.2f} min)")
print(f"Total messages:{message_count}")
print()

# 按主题输出详细信息
print("TOPICS:")
print("-" * 80)
# 表头
print(f"{'Topic':<40} {'Type':<35} {'Count':<8} {'Freq (Hz)':<10}")
print("-" * 80)

for topic_name, info in topics.items():
    count = info.msgcount
    freq = count / duration_s if duration_s > 0 else 0
    print(f"{topic_name:<40} {info.msgtype:<35} {count:<8} {freq:<10.2f}")

print()
print("MESSAGE DEFINITIONS (first 200 chars of each):")
print("-" * 80)
for topic_name, info in topics.items():
    msgdef_preview = info.msgdef.data
    # if len(info.msgdef.data) > 200:
    #     msgdef_preview += " ..."
    print(f"● {topic_name}")
    print(f"  Type: {info.msgtype}")
    print(f"  Def\n : {msgdef_preview}")
    print()