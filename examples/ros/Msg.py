from dataclasses import dataclass
from typing import List, Tuple,Optional
import struct
from datetime import datetime
import numpy as np
import open3d as o3d

def _to_serializable(x):
    """递归转换 numpy 标量/数组 为 Python 原生类型"""
    if isinstance(x, np.integer):
        return int(x)
    elif isinstance(x, np.floating):
        return float(x)
    elif isinstance(x, np.ndarray):
        return x.tolist()  # 自动递归转内部元素
    elif isinstance(x, (tuple, list)):
        return [_to_serializable(item) for item in x]
    elif isinstance(x, dict):
        return {k: _to_serializable(v) for k, v in x.items()}
    else:
        return x  # int, float, str, bool, None 等直接通过

# ------------------------------
# 基础嵌套消息类
# ------------------------------

@dataclass
class Header:
    seq: int          # uint32
    stamp_sec: int    # seconds since epoch
    stamp_nsec: int   # nanoseconds
    frame_id: str

    @property
    def stamp(self) -> float:
        """Unix timestamp in seconds (float, with ns precision)"""
        return self.stamp_sec + self.stamp_nsec * 1e-9

    @property
    def datetime(self) -> datetime:
        return datetime.utcfromtimestamp(self.stamp)

    @classmethod
    def from_ros_msg(cls, msg):
        # 兼容 rosbags 解析出的 ROS msg 对象（如 reader.message() 返回的）
        return cls(
            seq=msg.seq,
            stamp_sec=msg.stamp.sec,
            stamp_nsec=msg.stamp.nanosec,
            frame_id=str(msg.frame_id)
        )

    def to_dict(self):
        return {
            'seq': self.seq,
            'stamp': self.stamp,
            'frame_id': self.frame_id
        }


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.w)

    @classmethod
    def from_ros_msg(cls, msg):
        return cls(x=msg.x, y=msg.y, z=msg.z, w=msg.w)


@dataclass
class Vector3:
    x: float
    y: float
    z: float

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @classmethod
    def from_ros_msg(cls, msg):
        return cls(x=msg.x, y=msg.y, z=msg.z)

@dataclass
class IMU:
    header: Header
    orientation: Quaternion
    orientation_covariance: List[float]   # len=9
    angular_velocity: Vector3
    angular_velocity_covariance: List[float]  # len=9
    linear_acceleration: Vector3
    linear_acceleration_covariance: List[float]  # len=9

    # 方便使用的属性别名
    @property
    def timestamp(self) -> float:
        return self.header.stamp

    @property
    def time(self) -> datetime:
        return self.header.datetime

    @property
    def frame(self) -> str:
        return self.header.frame_id

    @classmethod
    def from_ros_msg(cls, msg):
        """从 rosbag 解析得到的原始 ROS Imu 消息构建 IMU 实例"""
        return cls(
            header=Header.from_ros_msg(msg.header),
            orientation=Quaternion.from_ros_msg(msg.orientation),
            orientation_covariance=list(msg.orientation_covariance),
            angular_velocity=Vector3.from_ros_msg(msg.angular_velocity),
            angular_velocity_covariance=list(msg.angular_velocity_covariance),
            linear_acceleration=Vector3.from_ros_msg(msg.linear_acceleration),
            linear_acceleration_covariance=list(msg.linear_acceleration_covariance)
        )

    # 转为 dict（方便 JSON 序列化）
    def to_dict(self):
        return _to_serializable({
            'header': {
                'seq': self.header.seq,
                'stamp': self.header.stamp,
                'frame_id': self.header.frame_id
            },
            'orientation': self.orientation.to_tuple(),
            'orientation_covariance': tuple(self.orientation_covariance),
            'angular_velocity': self.angular_velocity.to_tuple(),
            'angular_velocity_covariance': tuple(self.angular_velocity_covariance),
            'linear_acceleration': self.linear_acceleration.to_tuple(),
            'linear_acceleration_covariance': tuple(self.linear_acceleration_covariance)
        })

    def __repr__(self):
        return (f"IMU("
                f"t={self.timestamp:.6f}, "
                f"frame='{self.frame}', "
                f"acc=({self.linear_acceleration.x:.2f}, {self.linear_acceleration.y:.2f}, {self.linear_acceleration.z:.2f}), "
                f"gyro=({self.angular_velocity.x:.2f}, {self.angular_velocity.y:.2f}, {self.angular_velocity.z:.2f})"
                f")")


# ------------------------------
# Livox 点类（轻量，仅用于辅助；实际数据存于 LidarFrame.numpy）
# ------------------------------

@dataclass
class CustomPoint:
    offset_time: int  # uint32, ns
    x: float  # m
    y: float
    z: float
    reflectivity: int  # uint8
    tag: int  # uint8
    line: int  # uint8

    def to_dict(self):
        return {
            'offset_time': self.offset_time,
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'reflectivity': self.reflectivity,
            'tag': self.tag,
            'line': self.line
        }


# ------------------------------
# 主雷达帧类：LidarFrame
# ------------------------------

@dataclass
class LidarFrame:
    header: Header
    timebase: int  # uint64, ns —— 第一个点的绝对时间起点
    point_num: int  # uint32
    lidar_id: int  # uint8
    rsvd: Tuple[int, int, int]  # uint8[3]

    # 核心：用结构化 NumPy 数组存储点云，高效且内存连续
    points: np.ndarray  # dtype=CustomPointDType (see below)

    # ----- 预定义 dtype（关键！提升解析/访问效率） -----
    CustomPointDType = np.dtype([
        ('offset_time', np.uint32),  # ns
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('reflectivity', np.uint8),
        ('tag', np.uint8),
        ('line', np.uint8),
        ('_pad', np.uint8),  # padding to 16-byte alignment (optional but recommended)
    ], align=True)

    # ------------------------------
    # 属性快捷访问（返回 view，零拷贝）
    # ------------------------------
    @property
    def xyz(self) -> np.ndarray:
        """Shape: (N, 3), dtype=float32 —— 点云坐标"""
        return self.points[['x', 'y', 'z']].view(np.float32).reshape(-1, 3)

    @property
    def reflectivity(self) -> np.ndarray:
        """Shape: (N,), dtype=uint8"""
        return self.points['reflectivity']

    @property
    def timestamps(self) -> np.ndarray:
        """绝对时间戳（秒，float64），每个点的真实采集时刻"""
        return (self.timebase + self.points['offset_time']) * 1e-9  # ns → s

    @property
    def time_range(self) -> Tuple[float, float]:
        """(first_point_time, last_point_time) in seconds"""
        if self.point_num == 0:
            t = self.header.stamp
            return t, t
        t0 = (self.timebase + self.points['offset_time'][0]) * 1e-9
        t1 = (self.timebase + self.points['offset_time'][-1]) * 1e-9
        return t0, t1

    @property
    def duration(self) -> float:
        """扫描持续时间（秒）"""
        t0, t1 = self.time_range
        return t1 - t0

    @classmethod
    def from_ros_msg(cls, msg):
        """从 rosbag 反序列化的 Livox CustomMsg 构建 LidarFrame"""
        # Parse header
        header = Header.from_ros_msg(msg.header)

        # Pre-allocate numpy array
        n = msg.point_num
        points = np.empty(n, dtype=cls.CustomPointDType)

        # Batch-fill from list of points (rosbags returns list of point objects)
        # 注意：rosbags 解析后，msg.points 是 list[CustomPoint]
        for i, p in enumerate(msg.points):
            points[i] = (
                p.offset_time,
                p.x, p.y, p.z,
                p.reflectivity,
                p.tag,
                p.line,
                0  # padding
            )

        return cls(
            header=header,
            timebase=msg.timebase,
            point_num=n,
            lidar_id=msg.lidar_id,
            rsvd=tuple(msg.rsvd[:3]),
            points=points
        )

    def to_dict(self):
        return _to_serializable({
            'header': self.header.to_dict(),
            'timebase': self.timebase,
            'point_num': self.point_num,
            'lidar_id': self.lidar_id,
            'rsvd': list(self.rsvd),
            'points': self.points_to_dicts()
        })

    def points_to_dicts(self,
                        indices: Optional[np.ndarray] = None,
                        max_points: Optional[int] = None) -> list:
        """
        Convert point cloud (or subset) to list of dicts, like [CustomPoint.to_dict(), ...]

        Args:
            indices: Optional boolean or integer index array for filtering (e.g., mask, idxs).
            max_points: If given, limit total output count (applied after indices).

        Returns:
            List[dict]: Each dict has keys:
                ['offset_time', 'x', 'y', 'z', 'reflectivity', 'tag', 'line']
        """
        pts = self.points
        if indices is not None:
            pts = pts[indices]
        if max_points is not None and len(pts) > max_points:
            pts = pts[:max_points]

        # ✅ 核心：用 `pts[field].tolist()` 避免逐点循环
        # 注意：tolist() 自动将 np.uint32/np.float32 → int/float（JSON-safe）
        return [
            {
                'offset_time': ot,
                'x': x, 'y': y, 'z': z,
                'reflectivity': r,
                'tag': t,
                'line': l
            }
            for ot, x, y, z, r, t, l in zip(
                pts['offset_time'].tolist(),
                pts['x'].tolist(),
                pts['y'].tolist(),
                pts['z'].tolist(),
                pts['reflectivity'].tolist(),
                pts['tag'].tolist(),
                pts['line'].tolist()
            )
        ]

    def __repr__(self):
        return (f"LidarFrame("
                f"t={self.header.stamp:.6f}, "
                f"lidar_id={self.lidar_id}, "
                f"pts={self.point_num}, "
                f"frame='{self.header.frame_id}', "
                f"scan_dur={self.duration * 1000:.1f}ms"
                f")")

    # 可选：转为 Open3D 点云（需 `import open3d as o3d`）
    def to_open3d(self, with_intensity: bool = True):
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.xyz)
        if with_intensity and self.point_num > 0:
            # 归一化强度到 [0, 1]
            intensity = self.reflectivity.astype(np.float64) / 255.0
            colors = np.tile(intensity[:, None], (1, 3))  # gray scale
            pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd