#pragma once
#include "ieskf.h"
#include "commons.h"

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

    // Experiment-only: call between completed scans, never during undistort().
    // Raw IMU and gyro/bias quantities remain in the IMU frame.
    void reframeWorld(const M3D &rotation);

private:
    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    double m_last_propagate_end_time;
    Vec<IMUData> m_imu_cache;
    Vec<Pose> m_poses_cache;
    V3D m_last_acc;
    V3D m_last_gyro;
    M12D m_Q;
    IMUData m_last_imu;
};
