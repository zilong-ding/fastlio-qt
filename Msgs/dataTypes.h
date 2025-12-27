//
// Created by dzl on 11/20/25.
//

#ifndef FASTLIO_QT_DATATYPES_H
#define FASTLIO_QT_DATATYPES_H
// #include <pcl_>
// #include <pcl/conversions.h>
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
    std::vector<double> covariance = std::vector<double>(36,0.0); // len=36
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
    TwistWithCovariance,
    twist,
    covariance
    )
};

struct IMU {
    Header header;
    Quaternion orientation;
    std::vector<double> orientation_covariance = std::vector<double>(9,0.0); // len=9, though not used in to_dict()
    Vector3 angular_velocity;
    std::vector<double> angular_velocity_covariance = std::vector<double>(9,0.0); // len=9
    Vector3 linear_acceleration;
    std::vector<double> linear_acceleration_covariance = std::vector<double>(9,0.0); // len=9

    // Helper accessors
    double timestamp() const { return header.stamp; }
    const std::string& frame() const { return header.frame_id; }

    using Ptr = std::shared_ptr<IMU>;
    using ConstPtr = std::shared_ptr<const IMU>;
    using UniquePtr = std::unique_ptr<IMU>;

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
    std::vector<uint8_t> rsvd = std::vector<uint8_t>(3,0); // size 3 (uint8_t[3])

    std::vector<LivoxPoint> points;
    // === 智能指针别名（标准 ROS 风格）===
    using Ptr = std::shared_ptr<LidarFrame>;
    using ConstPtr = std::shared_ptr<const LidarFrame>;
    using UniquePtr = std::unique_ptr<LidarFrame>;

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
    double lidar_beg_time = 0.0;
    double lidar_end_time = 0.0;
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
        header,
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
// ========== PointField ==========
// 📌 移出常量定义到 namespace，避免每个实例带冗余数据
namespace PointFieldConst {
    constexpr uint8_t INT8    = 1;
    constexpr uint8_t UINT8   = 2;
    constexpr uint8_t INT16   = 3;
    constexpr uint8_t UINT16  = 4;
    constexpr uint8_t INT32   = 5;
    constexpr uint8_t UINT32  = 6;
    constexpr uint8_t FLOAT32 = 7;
    constexpr uint8_t FLOAT64 = 8;
}

struct PointField {
    std::string name;       // Name of field
    uint32_t offset = 0;    // Offset from start of point struct
    uint8_t datatype = 0;   // See PointFieldConst::*
    uint32_t count = 1;     // How many elements in the field

    // Optional helper: check if datatype is float/double etc.
    bool is_floating_point() const {
        return datatype == PointFieldConst::FLOAT32 || datatype == PointFieldConst::FLOAT64;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        PointField,
        name,
        offset,
        datatype,
        count
    )
};

// ========== PointCloud2 ==========
struct PointCloud2 {
    Header header;
    uint32_t height = 1;       // unordered → 1 row
    uint32_t width = 0;        // number of points = height * width
    std::vector<PointField> fields;
    bool is_bigendian = false;
    uint32_t point_step = 0;   // bytes per point
    uint32_t row_step = 0;     // bytes per row
    std::vector<uint8_t> data; // raw point data, size = row_step * height
    bool is_dense = true;      // true if no invalid (NaN/Inf) points

    // Optional: compute total point count
    size_t point_count() const { return static_cast<size_t>(height) * width; }

    // Optional: reserve data size (e.g., before filling)
    void reserve_data_bytes(size_t bytes) { data.reserve(bytes); }

    using Ptr = std::shared_ptr<PointCloud2>;
    using ConstPtr = std::shared_ptr<const PointCloud2>;
    using UniquePtr = std::unique_ptr<PointCloud2>;

    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        PointCloud2,
        header,
        height,
        width,
        fields,
        is_bigendian,
        point_step,
        row_step,
        data,
        is_dense
    )
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

struct Image {
    // === 核心字段（严格按 ROS sensor_msgs/Image 定义）===
    Header header;
    uint32_t height = 0;            // number of rows
    uint32_t width = 0;             // number of columns
    std::string encoding;           // e.g., "rgb8", "mono8", "bgr8"
    uint8_t is_bigendian = 0;       // 0 = little-endian (x86/Jetson), 1 = big-endian (rare)
    uint32_t step = 0;              // full row length in bytes (≥ width * channels * bytes_per_channel)
    std::vector<uint8_t> data;      // raw pixel data, size = step * height

    // === 辅助方法 ===
    double timestamp() const { return header.stamp; }
    const std::string& frame() const { return header.frame_id; }

    // === 常用工具函数 ===
    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }

    // Check if data size matches expected (step * height)
    bool validate() const {
        return data.size() == static_cast<size_t>(step) * height;
    }

    // Get number of channels (heuristic, based on encoding)
    int channels() const {
        if (encoding == "mono8" || encoding == "mono16") return 1;
        if (encoding == "rgb8" || encoding == "bgr8" ||
            encoding == "rgba8" || encoding == "bgra8") {
            return (encoding.find('a') != std::string::npos) ? 4 : 3;
        }
        if (encoding == "32FC1") return 1;
        if (encoding == "32FC3") return 3;
        // Add more as needed (see sensor_msgs/image_encodings.h)
        return -1; // unknown
    }

    // Get bytes per pixel (heuristic)
    int bytesPerPixel() const {
        if (encoding.find("8") != std::string::npos) return 1;
        if (encoding.find("16") != std::string::npos) return 2;
        if (encoding.find("32FC") != std::string::npos) return 4;
        return -1;
    }

    // === 智能指针别名（标准 ROS 风格）===
    using Ptr = std::shared_ptr<Image>;
    using ConstPtr = std::shared_ptr<const Image>;
    using UniquePtr = std::unique_ptr<Image>;

    // === JSON 序列化（nlohmann::json 兼容）===
    // Only serialize header + metadata (skip large 'data' by default)
    // To include data, use toJson(true)
    nlohmann::json toJson(bool include_data = false) const {
        nlohmann::json j;
        j["header"] = header;
        j["height"] = height;
        j["width"] = width;
        j["encoding"] = encoding;
        j["is_bigendian"] = is_bigendian;
        j["step"] = step;
        if (include_data) {
            // ⚠️ Warning: data can be huge! Only for debug/small images
            j["data"] = data;  // nlohmann auto-converts vector<uint8_t>
        } else {
            j["data_size"] = data.size();
        }
        return j;
    }

    // Static from JSON (deserialization)
    static Image fromJson(const nlohmann::json& j) {
        Image img;
        img.header = j.value("header", Header{});
        img.height = j.value("height", 0u);
        img.width = j.value("width", 0u);
        img.encoding = j.value("encoding", "");
        img.is_bigendian = j.value("is_bigendian", uint8_t(0));
        img.step = j.value("step", 0u);

        if (j.contains("data") && j["data"].is_array()) {
            img.data = j["data"].get<std::vector<uint8_t>>();
        }
        // Note: data_size is ignored on load
        return img;
    }

    // nlohmann macro for automatic from_json/to_json
    NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT(
        Image,
        header,
        height,
        width,
        encoding,
        is_bigendian,
        step
        // ⚠️ Exclude 'data' from default serialization to avoid huge JSON
    )
};

inline void LidarFrame2Pointcloud2(LidarFrame::Ptr& lidar_frame, PointCloud2::Ptr& pointcloud2)
{
    if (!pointcloud2) {
        pointcloud2 = std::make_shared<PointCloud2>();
    }

    // 空检查
    if (lidar_frame->points.empty()) {
        pointcloud2->width = 0;
        pointcloud2->row_step = 0;
        pointcloud2->data.clear();
        return;
    }

    // ------------ 1. 填写 header ------------
    pointcloud2->header = lidar_frame->header;

    // ------------ 2. 组织 pointcloud2 基本信息 ------------
    const size_t N = lidar_frame->points.size();
    pointcloud2->height = 1;
    pointcloud2->width = N;
    pointcloud2->is_dense = true;
    pointcloud2->is_bigendian = false;

    // 每个点大小 = LivoxPoint 实际字节大小
    const uint32_t POINT_SIZE = sizeof(LivoxPoint);   // 19 bytes
    pointcloud2->point_step = POINT_SIZE;
    pointcloud2->row_step = POINT_SIZE * N;

    // ------------ 3. 设置 fields ------------
    using PF = PointField;
    pointcloud2->fields.clear();

    pointcloud2->fields = {
        PF{"offset_time",  offsetof(LivoxPoint, offset_time), PointFieldConst::UINT32, 1},
        PF{"x",            offsetof(LivoxPoint, x),           PointFieldConst::FLOAT32, 1},
        PF{"y",            offsetof(LivoxPoint, y),           PointFieldConst::FLOAT32, 1},
        PF{"z",            offsetof(LivoxPoint, z),           PointFieldConst::FLOAT32, 1},
        PF{"reflectivity", offsetof(LivoxPoint, reflectivity),PointFieldConst::UINT8, 1},
        PF{"tag",          offsetof(LivoxPoint, tag),         PointFieldConst::UINT8, 1},
        PF{"line",         offsetof(LivoxPoint, line),        PointFieldConst::UINT8, 1},
    };

    // ------------ 4. 拷贝原始二进制点数据 ------------
    pointcloud2->data.resize(pointcloud2->row_step);

    // 直接按字节 memcpy，最高效，也确保字段正确对齐
    std::memcpy(pointcloud2->data.data(),
                lidar_frame->points.data(),
                pointcloud2->row_step);
}

inline bool Pointcloud2ToLidarFrame(const PointCloud2::ConstPtr& cloud, LidarFrame::Ptr& lidar_frame)
{
    if (!cloud || cloud->width == 0 || cloud->data.empty()) {
        return false;
    }

    // ---------------- 1. 填充 Header ----------------
    lidar_frame->header = cloud->header;

    // LidarFrame 里的时间基准（ns）仍由外部决定
    // 如果 PointCloud2 想要内置 timebase，可扩展 Header
    lidar_frame->timebase = 0;

    // ---------------- 2. 检查字段是否完整 ----------------
    // 需要的字段名称
    static const std::vector<std::string> required_fields = {
        "offset_time", "x", "y", "z", "reflectivity", "tag", "line"
    };

    // 查找 fields
    std::unordered_map<std::string, const PointField*> field_map;
    for (const auto& f : cloud->fields) {
        field_map[f.name] = &f;
    }

    // 确认字段齐全
    for (const auto& name : required_fields) {
        if (field_map.find(name) == field_map.end()) {
            std::cerr << "Missing PointField: " << name << std::endl;
            return false;
        }
    }

    // ---------------- 3. 准备输出点数组 ----------------
    const size_t N = cloud->width * cloud->height;
    lidar_frame->points.resize(N);
    lidar_frame->point_num = N;
    lidar_frame->lidar_id = 0;  // 可根据需要填写
    lidar_frame->rsvd = {0, 0, 0};

    const uint8_t* data_ptr = cloud->data.data();
    const uint32_t step = cloud->point_step;

    // ---------------- 4. 遍历每个点，逐字段复制 ----------------
    for (size_t i = 0; i < N; ++i)
    {
        const uint8_t* p = data_ptr + i * step;
        LivoxPoint& dst = lidar_frame->points[i];

        // 必须使用 memcpy，因为 PointCloud2 不保证结构体对齐
        memcpy(&dst.offset_time, p + field_map["offset_time"]->offset, sizeof(uint32_t));
        memcpy(&dst.x,           p + field_map["x"]->offset,           sizeof(float));
        memcpy(&dst.y,           p + field_map["y"]->offset,           sizeof(float));
        memcpy(&dst.z,           p + field_map["z"]->offset,           sizeof(float));
        memcpy(&dst.reflectivity,p + field_map["reflectivity"]->offset,sizeof(uint8_t));
        memcpy(&dst.tag,         p + field_map["tag"]->offset,         sizeof(uint8_t));
        memcpy(&dst.line,        p + field_map["line"]->offset,        sizeof(uint8_t));
    }

    return true;
}

inline bool Pointcloud2ToPCL(const PointCloud2::ConstPtr& cloud, PointCloudXYZI::Ptr& pcl_cloud)
{
    if (!cloud || cloud->width == 0 || cloud->data.empty()) {
        return false;
    }

    if (!pcl_cloud) {
        pcl_cloud = std::make_shared<PointCloudXYZI>();
    }

    pcl_cloud->clear();
    pcl_cloud->header.frame_id = cloud->header.frame_id;
    pcl_cloud->height = 1;
    pcl_cloud->width = cloud->width * cloud->height;
    pcl_cloud->is_dense = cloud->is_dense;
    pcl_cloud->points.resize(pcl_cloud->width);

    // ---------------- 1. 查找 PointField 的 offset ----------------
    const PointField* f_x = nullptr;
    const PointField* f_y = nullptr;
    const PointField* f_z = nullptr;
    const PointField* f_reflect = nullptr;
    const PointField* f_offset_time = nullptr;
    const PointField* f_tag = nullptr;
    const PointField* f_line = nullptr;

    for (const auto& f : cloud->fields) {
        if (f.name == "x") f_x = &f;
        else if (f.name == "y") f_y = &f;
        else if (f.name == "z") f_z = &f;
        else if (f.name == "reflectivity") f_reflect = &f;
        else if (f.name == "offset_time") f_offset_time = &f;
        else if (f.name == "tag") f_tag = &f;
        else if (f.name == "line") f_line = &f;
        else {
            std::cerr << "Warning: Unknown point field: " << f.name << std::endl;
        }
    }

    if (!f_x || !f_y || !f_z || !f_reflect || !f_offset_time || !f_tag || !f_line) {
        std::cerr << "Error: Missing required point fields\n";
        return false;
    }

    // ---------------- 2. 读取点数据 ----------------
    const uint8_t* data_ptr = cloud->data.data();
    const uint32_t step = cloud->point_step;
    const size_t N = pcl_cloud->points.size();

    for (size_t i = 0; i < N; ++i)
    {
        const uint8_t* p = data_ptr + i * step;
        PointType& dst = pcl_cloud->points[i];

        // x,y,z
        memcpy(&dst.x, p + f_x->offset, sizeof(float));
        memcpy(&dst.y, p + f_y->offset, sizeof(float));
        memcpy(&dst.z, p + f_z->offset, sizeof(float));

        // reflectivity → intensity
        uint8_t reflectivity_uint8;
        memcpy(&reflectivity_uint8, p + f_reflect->offset, sizeof(uint8_t));
        dst.intensity = static_cast<float>(reflectivity_uint8);

        // offset_time-> curvature
        if (f_offset_time != nullptr) {
            uint32_t offset_time;
            memcpy(&offset_time, p + f_offset_time->offset, sizeof(uint32_t));
            dst.curvature = static_cast<float>(offset_time) / float(1000000); // ns -> ms
        }
        else {
            dst.curvature = 0.0f;
        }

        // tag, line
        if (f_tag) {
            uint16_t tag_id;
            memcpy(&tag_id, p + f_tag->offset, sizeof(uint8_t));
            dst.normal_y = static_cast<float>(tag_id);
        } else {
            dst.normal_y = 0.f;
        }
        if (f_line) {
            uint16_t line_id;
            memcpy(&line_id, p + f_line->offset, sizeof(uint8_t));
            dst.normal_x = static_cast<float>(line_id);
        } else {
            dst.normal_x = 0.f;
        }


        // PCL 的 normal 全部置 0
        // dst.normal_x = 0.0f;
        // dst.normal_y = 0.0f;
        dst.normal_z = 0.0f;
    }
    return true;
}


#endif //FASTLIO_QT_DATATYPES_H