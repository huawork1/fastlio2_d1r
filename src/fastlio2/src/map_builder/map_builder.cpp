#include "map_builder.h"
#include <stdexcept>

MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
    m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::replaceMap(CloudType::Ptr replacement)
{
    if (m_status != BuilderStatus::MAPPING)
        throw std::logic_error("Map replacement requires an initialized LIO");
    m_lidar_processor->replaceCloudMap(replacement, true);
}

void MapBuilder::applyFeedback(const M3D &rotation, const V3D &translation,
                               CloudType::Ptr replacement, bool transformExisting)
{
    if (m_status != BuilderStatus::MAPPING)
        throw std::logic_error("Feedback requires an initialized LIO");
    if (!rotation.allFinite() || !translation.allFinite() ||
        !(rotation.transpose() * rotation).isApprox(M3D::Identity(), 1e-6) ||
        std::abs(rotation.determinant() - 1.0) > 1e-6)
        throw std::invalid_argument("Feedback must be a finite proper rigid transform");
    if (replacement && transformExisting)
        throw std::invalid_argument("Choose replacement or transformed existing map, not both");

    State corrected = m_kf->x();
    corrected.r_wi = rotation * corrected.r_wi;
    corrected.r_wi = Eigen::Quaterniond(corrected.r_wi).normalized().toRotationMatrix();
    corrected.t_wi = rotation * corrected.t_wi + translation;
    corrected.v = rotation * corrected.v;
    corrected.g = rotation * corrected.g;
    // Right-multiplicative rotation error is invariant under a common left
    // rotation. Only additive WORLD position/velocity error blocks rotate.
    // Extrinsics and body-frame gyro/accelerometer biases remain unchanged.
    M21D jacobian = M21D::Identity();
    jacobian.block<3, 3>(3, 3) = rotation;
    jacobian.block<3, 3>(12, 12) = rotation;
    M21D covariance = jacobian * m_kf->P() * jacobian.transpose();
    covariance = (0.5 * (covariance + covariance.transpose())).eval();
    if (!covariance.allFinite())
        throw std::runtime_error("Feedback covariance is nonfinite");

    // Build/validate the replacement before changing the filter. The wrapper
    // excludes process(), so no scan sees a partially applied correction.
    if (replacement)
        m_lidar_processor->replaceCloudMap(replacement, true);
    else if (transformExisting)
    {
        auto original = m_lidar_processor->exportCloud();
        auto transformed = LidarProcessor::transformCloud(original, rotation, translation);
        // Preserve the existing valid-point set for the rigid gauge control.
        m_lidar_processor->replaceCloudMap(transformed, false);
    }
    m_kf->x() = corrected;
    m_kf->P() = covariance;
    m_imu_processor->reframeWorld(rotation);
}

void MapBuilder::process(SyncPackage &package)
{
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (m_imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return;
    }

    m_imu_processor->undistort(package);

    if (m_status == BuilderStatus::MAP_INIT)
    {
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_status = BuilderStatus::MAPPING;
        return;
    }
    
    m_lidar_processor->process(package);
}
