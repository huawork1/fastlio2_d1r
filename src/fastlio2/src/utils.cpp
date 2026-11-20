#include "utils.h"
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace {
size_t fieldOffset(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name,
                   uint8_t type, size_t size) {
    for (const auto &field : msg.fields) {
        if (field.name == name && field.datatype == type && field.count == 1 &&
            field.offset + size <= msg.point_step) return field.offset;
    }
    throw std::runtime_error("Missing or invalid field: " + name);
}
template<typename T> T readField(const uint8_t *point, size_t offset) {
    T value;
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::seyond2PCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num,
    double min_range, double max_range) {
    using Field = sensor_msgs::msg::PointField;
    auto cloud = pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    if (msg->is_bigendian || filter_num < 1 || msg->point_step == 0 ||
        uint64_t(msg->row_step) < uint64_t(msg->width) * msg->point_step ||
        uint64_t(msg->data.size()) < uint64_t(msg->row_step) * msg->height)
        throw std::runtime_error("Invalid PointCloud2 layout or filter step");
    const auto x_off = fieldOffset(*msg, "x", Field::FLOAT32, sizeof(float));
    const auto y_off = fieldOffset(*msg, "y", Field::FLOAT32, sizeof(float));
    const auto z_off = fieldOffset(*msg, "z", Field::FLOAT32, sizeof(float));
    const auto i_off = fieldOffset(*msg, "intensity", Field::FLOAT32, sizeof(float));
    const auto t_off = fieldOffset(*msg, "timestamp", Field::FLOAT64, sizeof(double));
    const double start = getSec(msg->header);
    const double min_sq = min_range * min_range, max_sq = max_range * max_range;
    cloud->reserve(size_t(msg->width) * msg->height / filter_num + 1);
    for (size_t i = 0, count = size_t(msg->width) * msg->height; i < count; i += filter_num) {
        const auto *raw = msg->data.data() + (i / msg->width) * msg->row_step +
                          (i % msg->width) * msg->point_step;
        const float x = readField<float>(raw, x_off), y = readField<float>(raw, y_off),
                    z = readField<float>(raw, z_off), intensity = readField<float>(raw, i_off);
        const double relative_time = readField<double>(raw, t_off) - start;
        const double range_sq = double(x) * x + double(y) * y + double(z) * z;
        // Supplied SDK: point timestamps in seconds, header at scan start.
        // Reject invalid timestamps instead of inventing timing for deskewing.
        if (!std::isfinite(range_sq) || !std::isfinite(relative_time) ||
            relative_time < -1e-6 || relative_time > 0.2 ||
            range_sq < min_sq || range_sq > max_sq) continue;
        pcl::PointXYZINormal point{};
        point.x = x; point.y = y; point.z = z;
        point.intensity = std::isfinite(intensity) ? intensity : 0.0f;
        point.curvature = float(std::max(0.0, relative_time) * 1000.0);
        cloud->push_back(point);
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header) {
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec) {
    builtin_interfaces::msg::Time msg;
    const auto ns = static_cast<int64_t>(std::llround(sec * 1e9));
    msg.sec = static_cast<int32_t>(ns / 1000000000);
    msg.nanosec = static_cast<uint32_t>(ns % 1000000000);
    return msg;
}
