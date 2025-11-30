//
// Created by dzl on 2025/11/29.
//

#ifndef FASTLIO_QT_UTILLS_H
#define FASTLIO_QT_UTILLS_H
#include <string>
#include <vector>
#include "../Msgs/dataTypes.h"
// class PointField;
// Helper: add a field to fields vector
inline void addPointField(std::vector<PointField>& fields,
                          const std::string& name,
                          uint32_t offset,
                          uint8_t datatype,
                          uint32_t count = 1) {
    PointField pf;
    pf.name = name;
    pf.offset = offset;
    pf.datatype = datatype;
    pf.count = count;
    fields.push_back(pf);
}

// 🌟 Main conversion function
inline PointCloud2 lidarFrameToPointCloud2(const LidarFrame& frame) {
    PointCloud2 pc2;

    // --- Header ---
    pc2.header = frame.header;
    pc2.height = 1; // unordered cloud
    pc2.width = static_cast<uint32_t>(frame.points.size());
    pc2.is_dense = true; // assume no NaN

    // --- Define fields ---
    // We'll serialize as: x, y, z, intensity, time (optional), normal_x, normal_y, normal_z, curvature
    std::vector<PointField>& fields = pc2.fields;

    uint32_t offset = 0;

    // float32 x, y, z
    addPointField(fields, "x", offset, PointFieldConst::FLOAT32); offset += 4;
    addPointField(fields, "y", offset, PointFieldConst::FLOAT32); offset += 4;
    addPointField(fields, "z", offset, PointFieldConst::FLOAT32); offset += 4;

    // float32 intensity (reflectivity)
    addPointField(fields, "intensity", offset, PointFieldConst::FLOAT32); offset += 4;

    // ⏱️ Optional: add relative time (in seconds, float32 or float64)
    // Here we use 't' as float32 (common in Livox drivers)
    addPointField(fields, "t", offset, PointFieldConst::FLOAT32); offset += 4;

    // Normal + curvature (if you use PointXYZINormal)
    addPointField(fields, "normal_x", offset, PointFieldConst::FLOAT32); offset += 4;
    addPointField(fields, "normal_y", offset, PointFieldConst::FLOAT32); offset += 4;
    addPointField(fields, "normal_z", offset, PointFieldConst::FLOAT32); offset += 4;
    addPointField(fields, "curvature", offset, PointFieldConst::FLOAT32); offset += 4;

    pc2.point_step = offset;          // bytes per point = 4 * 9 = 36
    pc2.row_step = pc2.point_step * pc2.width;
    pc2.data.resize(pc2.row_step);   // pre-allocate

    // --- Fill binary data ---
    uint8_t* ptr = pc2.data.data();
    double timebase_sec = frame.timebase * 1e-9; // ns → s

    for (const auto& pt : frame.points) {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;
        float intensity = static_cast<float>(pt.reflectivity);
        float t = static_cast<float>((frame.timebase + pt.offset_time) * 1e-9 - timebase_sec); // relative time, seconds
        // Or use absolute: float t = (frame.timebase + pt.offset_time) * 1e-9f;

        float nx = 0.0f, ny = 0.0f, nz = 0.0f, curvature = 0.0f; // set to zero if not computed

        // ⚡ Efficient: memcpy 9 floats at once
        float point_data[9] = {x, y, z, intensity, t, nx, ny, nz, curvature};
        std::memcpy(ptr, point_data, sizeof(point_data));
        ptr += sizeof(point_data); // 36 bytes
    }

    // endian: assume little-endian (x86/ARM), set accordingly
    pc2.is_bigendian = false;

    return pc2;
}
#endif //FASTLIO_QT_UTILLS_H