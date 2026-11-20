#pragma once
#include "commons.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <fstream>
#include <string>

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score;
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 1.0;
    double loop_time_tresh = 60.0;
    double loop_score_tresh = 0.15;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;
    bool loop_view_gate_enabled = false;
    double loop_view_min_fraction = 0.5;
    double loop_view_max_axis_angle_deg = 90.0;
    double view_horizontal_fov_deg = 140.0;
    double view_vertical_fov_deg = 100.0;
    double view_min_range_m = 0.1;
    double view_max_range_m = 50.0;
    M3D view_r_il = M3D::Identity();
    V3D view_t_il = V3D::Zero();
};

struct ViewGateResult
{
    bool pass = false;
    std::string reason = "invalid";
    int first = -1, last = -1;
    size_t finite_source_points = 0, inside_points = 0;
    double fraction = 0.0, axis_angle_deg = 0.0;
};

struct PairRegistrationResult
{
    ViewGateResult view;
    bool radius_pass = false, time_pass = false;
    bool icp_attempted = false, converged = false, score_pass = false, accepted = false;
    double distance_m = 0, time_gap_s = 0, fitness = -1;
    double source_center_shift_m = 0, rotation_deg = 0;
    size_t source_points = 0, target_points = 0;
    M4F delta = M4F::Identity();
    LoopPair measurement;
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    ViewGateResult evaluateViewCandidate(int target_idx) const;

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }
    const std::vector<LoopPair> &pendingPairs() const { return m_cache_pairs; }
    const Config &config() const { return m_config; }
    void addFrozenLoop(const LoopPair &pair);
    PairRegistrationResult probePair(size_t source_idx, size_t target_idx);

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

private:
    ViewGateResult evaluateViewPair(size_t source_idx, int target_idx) const;
    void auditView(const char *stage, int target_idx, const ViewGateResult &view,
                   bool converged = false, double fitness = -1., const M4F *delta = nullptr,
                   size_t icp_source_points = 0, size_t icp_target_points = 0);
    void saveViewSnapshot(const char *stage, size_t loop_index) const;
    std::ofstream m_view_audit;
    std::string m_view_audit_path;
    Config m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    pcl::IterativeClosestPoint<PointType, PointType> m_icp;
};
