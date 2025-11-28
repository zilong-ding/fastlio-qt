//
// Created by dzl on 11/20/25.
//

#ifndef FASTLIO_QT_DATATYPES_H
#define FASTLIO_QT_DATATYPES_H

#include <cstdint>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <array>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Eigen>
#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>
#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;
using json = nlohmann::json;
enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;
    orgtype()
    {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};
struct Header {
    uint32_t seq = 0;
    double stamp;          // seconds since epoch (float, ns precision)
    std::string frame_id;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Header, seq, stamp, frame_id)
};

struct Point {
    double x = 0, y = 0, z = 0;
    friend void from_json(const json& j, Point& p) {
        if (j.is_array()) {
            if (j.size() != 3) {
                throw std::runtime_error("Quaternion array must have 4 elements");
            }
            p.x = j[0].get<double>();
            p.y = j[1].get<double>();
            p.z = j[2].get<double>();
        }
        else if (j.is_object()) {
            p.x = j.value("x", 0.0);
            p.y = j.value("y", 0.0);
            p.z = j.value("z", 0.0);
        }
        else {
            throw std::runtime_error("Point must be array or object");
        }
    }
    friend void to_json(json& j, const Point& p) {
        j = json::array({p.x, p.y, p.z});  // match Python style
    }
};

// ——————— Quaternion: support [x,y,z,w] ———————
struct Quaternion {
    double x = 0, y = 0, z = 0, w = 1;

    // Custom from_json to accept both {"x":...} and [x,y,z,w]
    friend void from_json(const json& j, Quaternion& q) {
        if (j.is_array()) {
            if (j.size() != 4) {
                throw std::runtime_error("Quaternion array must have 4 elements");
            }
            q.x = j[0].get<double>();
            q.y = j[1].get<double>();
            q.z = j[2].get<double>();
            q.w = j[3].get<double>();
        } else if (j.is_object()) {
            q.x = j.value("x", 0.0);
            q.y = j.value("y", 0.0);
            q.z = j.value("z", 0.0);
            q.w = j.value("w", 1.0);
        } else {
            throw std::runtime_error("Quaternion must be array or object");
        }
    }

    // Optional: to_json (for symmetry)
    friend void to_json(json& j, const Quaternion& q) {
        j = json::array({q.x, q.y, q.z, q.w});  // match Python style
    }
};

struct Pose {
    Point position;
    Quaternion orientation;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        Pose,
        position,
        orientation
        )
};

struct PoseWithCovariance {
    Pose pose;
    std::vector<double> covariance = {
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0,
        0,0,0,0,0,0
    }; // len=36
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        PoseWithCovariance,
        pose,
        covariance
    )
};

// ——————— Vector3: support [x,y,z] ———————
struct Vector3 {
    double x = 0, y = 0, z = 0;

    friend void from_json(const json& j, Vector3& v) {
        if (j.is_array()) {
            if (j.size() != 3) {
                throw std::runtime_error("Vector3 array must have 3 elements");
            }
            v.x = j[0].get<double>();
            v.y = j[1].get<double>();
            v.z = j[2].get<double>();
        } else if (j.is_object()) {
            v.x = j.value("x", 0.0);
            v.y = j.value("y", 0.0);
            v.z = j.value("z", 0.0);
        } else {
            throw std::runtime_error("Vector3 must be array or object");
        }
    }

    friend void to_json(json& j, const Vector3& v) {
        j = json::array({v.x, v.y, v.z});
    }
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
    Twist,linear,angular
    )
};

struct TwistWithCovariance {
// This expresses velocity in free space with uncertainty.

    Twist twist;
/*
    # Row-major representation of the 6x6 covariance matrix
    # The orientation parameters use a fixed-axis representation.
    # In order, the parameters are:
    # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
*/
    std::vector<double> covariance; // len=36
};

struct IMU {
    Header header;
    Quaternion orientation;
    std::vector<double> orientation_covariance; // len=9, though not used in to_dict()
    Vector3 angular_velocity;
    std::vector<double> angular_velocity_covariance; // len=9
    Vector3 linear_acceleration;
    std::vector<double> linear_acceleration_covariance; // len=9

    // Helper accessors
    double timestamp() const { return header.stamp; }
    const std::string& frame() const { return header.frame_id; }

    // Only fields in to_dict() need deserialization; covariances are optional but kept for completeness
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        IMU,
        header,
        orientation,
        orientation_covariance,
        angular_velocity,
        angular_velocity_covariance,
        linear_acceleration,
        linear_acceleration_covariance
    )
};

struct LivoxPoint {
    uint32_t offset_time;   // ns
    float x, y, z;          // meters
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        LivoxPoint, offset_time, x, y, z, reflectivity, tag, line
    )
};

struct LidarFrame {
    Header header;
    uint64_t timebase;         // ns
    uint32_t point_num;
    uint8_t lidar_id;
    std::vector<uint8_t> rsvd; // size 3 (uint8_t[3])

    std::vector<LivoxPoint> points;

    // Derived properties (computed on demand)
    double scan_duration() const {
        if (points.empty()) return 0.0;
        double t0 = (timebase + points.front().offset_time) * 1e-9;
        double t1 = (timebase + points.back().offset_time) * 1e-9;
        return t1 - t0;
    }

    std::vector<double> absolute_timestamps() const {
        std::vector<double> ts;
        ts.reserve(points.size());
        for (const auto& p : points) {
            ts.push_back((timebase + p.offset_time) * 1e-9);
        }
        return ts;
    }

    // Extract XYZ as Nx3 vector (for e.g., Open3D/PCL conversion)
    std::vector<std::array<float, 3>> xyz() const {
        std::vector<std::array<float, 3>> out;
        out.reserve(points.size());
        for (const auto& p : points) {
            out.push_back({p.x, p.y, p.z});
        }
        return out;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        LidarFrame,
        header,
        timebase,
        point_num,
        lidar_id,
        rsvd,
        points
    )
};

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar;
    std::deque<std::shared_ptr<IMU>> imu;
};

struct Odometry {
    Header header;
    std::string child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        Odometry,
        child_frame_id,
        pose,
        twist
    )
};

struct PoseStamped {
    Header header;
    Pose pose;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        PoseStamped,
        header,
        pose
    )
};

struct Path {
    Header header;
    std::vector<PoseStamped> poses;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        Path,
        header,
        poses
    )
};

struct PointField {
    uint8_t INT8    = 1;
    uint8_t UINT8   = 2;
    uint8_t INT16   = 3;
    uint8_t UINT16  = 4;
    uint8_t INT32   = 5;
    uint8_t UINT32  = 6;
    uint8_t FLOAT32 = 7;
    uint8_t FLOAT64 = 8;
    std::string name ;     // Name of field
    uint32_t offset  ;  // Offset from start of point struct
    uint8_t  datatype ; // Datatype enumeration, see above
    uint32_t count  ;   // How many elements in the field
};

struct PointCloud2 {
// # The point cloud data may be organized 2d (image-like) or 1d
// # (unordered). Point clouds organized as 2d images may be produced by
// # camera depth sensors such as stereo or time-of-flight.
//
// # Time of sensor data acquisition, and the coordinate frame ID (for 3d
// # points).
    Header header;

    // # 2D structure of the point cloud. If the cloud is unordered, height is
    // # 1 and width is the length of the point cloud.
    uint32_t height;
    uint32_t width;

    // # Describes the channels and their layout in the binary data blob.
    std::vector<PointField> fields;

    bool    is_bigendian ;// Is this data bigendian?
    uint32_t  point_step;   // Length of a point in bytes
    uint32_t  row_step;     // Length of a row in bytes
    std::vector<uint8_t> data ;        // Actual point data, size is (row_step*height)

    bool is_dense;        // True if there are no invalid points
};

struct PointCloudMsg {
    Header header;
    PointCloudXYZI::Ptr pointCloud;
};


struct Pose6D {
// the preintegrated Lidar states at the time of IMU measurements in a frame
    double  offset_time = 0; // the offset time of IMU measurement w.r.t the first lidar point
    std::vector<double> acc = {0,0,0};      // the preintegrated total acceleration (global frame) at the Lidar origin
    std::vector<double> gyr = {0,0,0};       // the unbiased angular velocity (body frame) at the Lidar origin
    std::vector<double> vel  = {0,0,0};      // the preintegrated velocity (global frame) at the Lidar origin
    std::vector<double> pos  = {0,0,0};      // the preintegrated position (global frame) at the Lidar origin
    std::vector<double> rot  = {
        0,0,0,
        0,0,0,
        0,0,0
    };      // the preintegrated rotation (global frame) at the Lidar origin
};

#endif //FASTLIO_QT_DATATYPES_H