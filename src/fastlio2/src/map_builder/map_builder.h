#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }    
    std::shared_ptr<LidarProcessor> lidar_processor(){return m_lidar_processor;}

    // Experiment-only. Call after process() has completed and before the next
    // process(). R,t are the LEFT world correction at the current scan time.
    // A supplied replacement is in corrected world coordinates, not body frame.
    // transformExisting instead rigidly reexpresses the native tree (gauge arm).
    void applyFeedback(const M3D &rotation, const V3D &translation,
                       CloudType::Ptr replacement = nullptr, bool transformExisting = false);
    void replaceMap(CloudType::Ptr replacement);
    void applyMapOnly(CloudType::Ptr replacement) { replaceMap(replacement); }
    CloudType::Ptr exportCloud() { return m_lidar_processor->exportCloud(); }

private:
    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IMUProcessor> m_imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
};
