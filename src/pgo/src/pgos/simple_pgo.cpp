#include "simple_pgo.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <stdexcept>
#include "online_direction.h"

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    if (m_config.loop_view_gate_enabled)
    {
        const auto &c = m_config;
        if (!(std::isfinite(c.loop_view_min_fraction) && c.loop_view_min_fraction >= 0. && c.loop_view_min_fraction <= 1.) ||
            !(std::isfinite(c.loop_view_max_axis_angle_deg) && c.loop_view_max_axis_angle_deg >= 0. && c.loop_view_max_axis_angle_deg <= 180.) ||
            !(std::isfinite(c.view_horizontal_fov_deg) && c.view_horizontal_fov_deg > 0. && c.view_horizontal_fov_deg <= 360.) ||
            !(std::isfinite(c.view_vertical_fov_deg) && c.view_vertical_fov_deg > 0. && c.view_vertical_fov_deg <= 180.) ||
            !(std::isfinite(c.view_min_range_m) && std::isfinite(c.view_max_range_m) && c.view_min_range_m >= 0. && c.view_max_range_m > c.view_min_range_m) ||
            !c.view_r_il.allFinite() || !c.view_t_il.allFinite() ||
            (c.view_r_il.transpose()*c.view_r_il-M3D::Identity()).norm() > 1e-6 || std::abs(c.view_r_il.determinant()-1.) > 1e-6)
            throw std::invalid_argument("Invalid LiDAR view-gate configuration");
    }
    if (const char *path = std::getenv("PGO_VIEW_AUDIT_PATH"))
    {
        m_view_audit_path = path;
        m_view_audit.open(path, std::ios::out | std::ios::app);
        if (!m_view_audit) throw std::runtime_error("Cannot open PGO_VIEW_AUDIT_PATH");
        m_view_audit << std::setprecision(17);
    }
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    m_icp.setMaximumIterations(50);
    m_icp.setMaxCorrespondenceDistance(10);
    m_icp.setTransformationEpsilon(1e-6);
    m_icp.setEuclideanFitnessEpsilon(1e-6);
    m_icp.setRANSACIterations(0);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

ViewGateResult SimplePGO::evaluateViewCandidate(int target_idx) const
{
    ViewGateResult result;
    if (m_key_poses.empty() || target_idx < 0 || target_idx >= int(m_key_poses.size())-1)
        return result;
    const int current_idx = int(m_key_poses.size())-1;
    result.first = std::max(0, target_idx-m_config.loop_submap_half_range);
    result.last = std::min(current_idx, target_idx+m_config.loop_submap_half_range);
    // Gate and ICP must use exactly the same target set. Reject self-inclusion.
    if (result.last >= current_idx) { result.reason = "source_in_target_submap"; return result; }
    const auto &source = m_key_poses.back();
    const auto &central = m_key_poses[target_idx];
    if (!source.body_cloud || source.body_cloud->empty() || !source.r_global.allFinite() || !source.t_global.allFinite())
    { result.reason = "invalid_source"; return result; }
    for (int j=result.first; j<=result.last; ++j)
    {
        if (!m_key_poses[j].r_global.allFinite() || !m_key_poses[j].t_global.allFinite() ||
            !m_key_poses[j].body_cloud || m_key_poses[j].body_cloud->empty())
        { result.reason = "invalid_target_pose"; return result; }
        for (const auto &point : *m_key_poses[j].body_cloud)
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            { result.reason = "nonfinite_target_point"; return result; }
    }
    const V3D forward_source = (source.r_global*m_config.view_r_il.col(0)).normalized();
    const V3D forward_target = (central.r_global*m_config.view_r_il.col(0)).normalized();
    result.axis_angle_deg = std::acos(std::clamp(forward_source.dot(forward_target), -1., 1.))*180./M_PI;
    const double max_az = m_config.view_horizontal_fov_deg*M_PI/360.;
    const double max_el = m_config.view_vertical_fov_deg*M_PI/360.;
    for (const auto &point : *source.body_cloud)
    {
        const V3D body(point.x, point.y, point.z);
        if (!body.allFinite()) { result.reason = "nonfinite_source_point"; return result; }
        ++result.finite_source_points;
        // body_cloud is already in IMU/body coordinates; do not apply R_IL twice.
        const V3D world = source.r_global*body+source.t_global;
        for (int j=result.first; j<=result.last; ++j)
        {
            const auto &historical = m_key_poses[j];
            const V3D lidar = m_config.view_r_il.transpose()*(historical.r_global.transpose()*(world-historical.t_global)-m_config.view_t_il);
            const double range = lidar.norm();
            if (range < m_config.view_min_range_m || range > m_config.view_max_range_m) continue;
            if (std::abs(std::atan2(lidar.y(),lidar.x())) <= max_az &&
                std::abs(std::atan2(lidar.z(),std::hypot(lidar.x(),lidar.y()))) <= max_el)
            { ++result.inside_points; break; } // union membership, count each source point once
        }
    }
    if (!result.finite_source_points) { result.reason = "no_finite_source_points"; return result; }
    result.fraction = double(result.inside_points)/result.finite_source_points;
    const bool direction_ok = result.axis_angle_deg <= m_config.loop_view_max_axis_angle_deg;
    const bool fraction_ok = result.fraction >= m_config.loop_view_min_fraction;
    result.pass = direction_ok && fraction_ok;
    result.reason = result.pass ? "pass" : (!direction_ok && !fraction_ok ? "direction_and_fraction" : (!direction_ok ? "direction" : "fraction"));
    return result;
}

void SimplePGO::auditView(const char *stage, int target_idx, const ViewGateResult &view,
                        bool converged, double fitness, const M4F *delta,
                        size_t icp_source_points, size_t icp_target_points)
{
    if (!m_view_audit.is_open()) return;
    const auto &s=m_key_poses.back(), &t=m_key_poses[target_idx];
    m_view_audit << "{\"stage\":\"" << stage << "\",\"source_id\":" << m_key_poses.size()-1
        << ",\"target_id\":" << target_idx << ",\"source_time\":" << s.time << ",\"target_time\":" << t.time
        << ",\"center_distance_m\":";
    if (std::isfinite((s.t_global-t.t_global).norm())) m_view_audit << (s.t_global-t.t_global).norm(); else m_view_audit << "null";
    m_view_audit << ",\"gate_enabled\":" << (m_config.loop_view_gate_enabled?"true":"false")
        << ",\"gate_pass\":" << (view.pass?"true":"false") << ",\"reason\":\"" << view.reason
        << "\",\"target_first\":" << view.first << ",\"target_last\":" << view.last
        << ",\"finite_source_points\":" << view.finite_source_points << ",\"inside_points\":" << view.inside_points
        << ",\"view_fraction\":" << view.fraction << ",\"axis_angle_deg\":";
    if (std::isfinite(view.axis_angle_deg)) m_view_audit << view.axis_angle_deg; else m_view_audit << "null";
    m_view_audit << ",\"full_source_points\":" << (s.body_cloud?s.body_cloud->size():0)
        << ",\"has_converged\":";
    if (delta) m_view_audit << (converged?"true":"false"); else m_view_audit << "null";
    m_view_audit << ",\"fitness_m2\":";
    if (delta && std::isfinite(fitness)) m_view_audit << fitness; else m_view_audit << "null";
    if (delta)
    {
        const M3D dr=delta->block<3,3>(0,0).cast<double>();
        const V3D shift=dr*s.t_global+delta->block<3,1>(0,3).cast<double>()-s.t_global;
        const double rotation_deg=Eigen::Quaterniond(dr).normalized().angularDistance(Eigen::Quaterniond::Identity())*180./M_PI;
        m_view_audit << ",\"icp_rotation_deg\":";
        if (std::isfinite(rotation_deg)) m_view_audit << rotation_deg; else m_view_audit << "null";
        m_view_audit << ",\"icp_sensor_center_shift_m\":";
        if (std::isfinite(shift.norm())) m_view_audit << shift.norm(); else m_view_audit << "null";
        m_view_audit << ",\"icp_source_points\":" << icp_source_points << ",\"icp_target_points\":" << icp_target_points;
    }
    m_view_audit << "}\n"; m_view_audit.flush();
}

void SimplePGO::saveViewSnapshot(const char *stage, size_t loop_index) const
{
    if (m_view_audit_path.empty()) return;
    std::ofstream out(m_view_audit_path+"."+stage+"_loop_"+std::to_string(loop_index)+".txt");
    if (!out) throw std::runtime_error("Cannot write view-gate pose snapshot");
    out << std::setprecision(17);
    for (size_t i=0; i<m_key_poses.size(); ++i)
    {
        const auto &p=m_key_poses[i]; const Eigen::Quaterniond q(p.r_global);
        out << i << ".pcd " << p.t_global.x() << ' ' << p.t_global.y() << ' ' << p.t_global.z()
            << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << '\n';
    }
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    pcl::PointXYZ last_pose_pt;
    last_pose_pt.x = last_item.t_global(0);
    last_pose_pt.y = last_item.t_global(1);
    last_pose_pt.z = last_item.t_global(2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < m_key_poses.size() - 1; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
    if (neighbors == 0)
        return;

    // Isolated experiment: preserve the original first-eligible-candidate
    // behavior unless explicitly enabled. No ranking or threshold changes.
    const char *retry_env = std::getenv("PGO_RETRY_CANDIDATES");
    const bool retry_candidates = retry_env && std::string(retry_env) == "1";
    if (retry_env && std::string(retry_env) != "0" && std::string(retry_env) != "1")
        throw std::invalid_argument("PGO_RETRY_CANDIDATES must be 0 or 1");
    int loop_idx = -1;
    M4F loop_transform;
    ViewGateResult selected_view;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int idx = ids[i];
        if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh)
        {
            if (m_config.loop_view_gate_enabled)
            {
                const auto view = evaluateViewCandidate(idx);
                auditView("candidate", idx, view);
                if (!view.pass) continue;
                selected_view = view;
            }
            CloudType::Ptr target_cloud = getSubMap(idx, m_config.loop_submap_half_range, m_config.submap_resolution);
            CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
            CloudType::Ptr align_cloud(new CloudType);

            m_icp.setInputSource(source_cloud);
            m_icp.setInputTarget(target_cloud);
            m_icp.align(*align_cloud);

            const bool icp_converged = m_icp.hasConverged();
            const double icp_score = m_icp.getFitnessScore();
            const M4F audited_transform = m_icp.getFinalTransformation();
            const bool accepted = icp_converged && icp_score <= m_config.loop_score_tresh;
            auditView(accepted ? "icp_accepted" : "icp_rejected", idx, selected_view, icp_converged, icp_score, &audited_transform, source_cloud->size(), target_cloud->size());
            if (!accepted)
            {
                if (retry_candidates) continue;
                return;
            }
            // Registration and its original acceptance are fixed before geometry inspection.
            // A weak-mode geometry failure drops this source proposal; it does not select a new target.
            if (online_direction::enabled())
            {
                const M3D refined_r = audited_transform.block<3,3>(0,0).cast<double>() * last_item.r_global;
                if (!online_direction::prepare(this,cur_idx,idx,icp_score,align_cloud,target_cloud,refined_r,audited_transform))
                    return;
            }
            loop_idx = idx;
            loop_transform = m_icp.getFinalTransformation();
            break;
        }
    }

    if (loop_idx == -1)
        return;

    saveViewSnapshot("before", m_history_pairs.size()+1);

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           online_direction::noise(this,pair)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
    if (has_loop) saveViewSnapshot("after", m_history_pairs.size());
}

// Isolated ablation/probe helpers; original automatic search above is unchanged.
void SimplePGO::addFrozenLoop(const LoopPair &pair)
{
    if (m_key_poses.empty() || pair.source_id != m_key_poses.size()-1 || pair.target_id >= pair.source_id)
        throw std::invalid_argument("Frozen factor must arrive exactly at its source keyframe");
    if (!std::isfinite(pair.score) || pair.score <= 0 || pair.score > m_config.loop_score_tresh ||
        !pair.r_offset.allFinite() || !pair.t_offset.allFinite() ||
        (pair.r_offset.transpose()*pair.r_offset-M3D::Identity()).norm() > 1e-3 ||
        std::abs(pair.r_offset.determinant()-1.0) > 1e-3)
        throw std::invalid_argument("Invalid frozen loop measurement/score");
    if (!m_cache_pairs.empty()) throw std::runtime_error("Unexpected pending loop before frozen factor");
    saveViewSnapshot("before", m_history_pairs.size()+1);
    m_cache_pairs.push_back(pair);
    m_history_pairs.emplace_back(pair.target_id, pair.source_id);
}

PairRegistrationResult SimplePGO::probePair(size_t source_idx, size_t target_idx)
{
    if (source_idx >= m_key_poses.size() || target_idx >= source_idx)
        throw std::invalid_argument("Probe requires an existing source and earlier target keyframe");
    PairRegistrationResult result;
    result.measurement.source_id = source_idx;
    result.measurement.target_id = target_idx;
    result.measurement.r_offset = M3D::Identity();
    result.measurement.t_offset = V3D::Zero();
    result.measurement.score = -1;
    const auto &source = m_key_poses[source_idx];
    const auto &target = m_key_poses[target_idx];
    result.distance_m = (source.t_global-target.t_global).norm();
    result.time_gap_s = std::abs(source.time-target.time);
    result.time_pass = result.time_gap_s > m_config.loop_time_tresh;
    // Use the actual original float KD-tree radius test, not a new distance gate.
    pcl::PointCloud<pcl::PointXYZ>::Ptr poses(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<source_idx; ++i)
    {
        pcl::PointXYZ p;
        p.x=m_key_poses[i].t_global.x(); p.y=m_key_poses[i].t_global.y(); p.z=m_key_poses[i].t_global.z();
        poses->push_back(p);
    }
    pcl::PointXYZ query;
    query.x=source.t_global.x(); query.y=source.t_global.y(); query.z=source.t_global.z();
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(poses);
    std::vector<int> ids; std::vector<float> sqdists;
    tree.radiusSearch(query, m_config.loop_search_radius, ids, sqdists);
    result.radius_pass = std::find(ids.begin(),ids.end(),static_cast<int>(target_idx)) != ids.end();
    result.view = evaluateViewPair(source_idx, static_cast<int>(target_idx));
    // Never permit self/future inclusion or malformed source/target geometry.
    if (result.view.reason == "source_in_target_submap" || result.view.reason == "invalid" ||
        result.view.reason == "invalid_source" || result.view.reason == "invalid_target_pose" ||
        result.view.reason == "nonfinite_target_point" || result.view.reason == "nonfinite_source_point" ||
        result.view.reason == "no_finite_source_points") return result;
    CloudType::Ptr target_cloud = getSubMap(target_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(source_idx, 0, m_config.submap_resolution);
    CloudType::Ptr aligned(new CloudType);
    // Identical parameters, separate instance: probing does not alter the
    // automatic ICP object's persistent state or create any graph constraint.
    pcl::IterativeClosestPoint<PointType,PointType> icp;
    icp.setMaximumIterations(50);
    icp.setMaxCorrespondenceDistance(10);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(source_cloud); icp.setInputTarget(target_cloud);
    icp.align(*aligned);
    result.icp_attempted = true;
    result.source_points = source_cloud->size(); result.target_points = target_cloud->size();
    result.converged = icp.hasConverged();
    result.fitness = icp.getFitnessScore();
    result.delta = icp.getFinalTransformation();
    result.score_pass = result.fitness <= m_config.loop_score_tresh;
    result.accepted = result.radius_pass && result.time_pass &&
        (!m_config.loop_view_gate_enabled || result.view.pass) && result.converged && result.score_pass;
    const M3D r_refined = result.delta.block<3,3>(0,0).cast<double>()*source.r_global;
    const V3D t_refined = result.delta.block<3,3>(0,0).cast<double>()*source.t_global+
                         result.delta.block<3,1>(0,3).cast<double>();
    result.measurement.r_offset = target.r_global.transpose()*r_refined;
    result.measurement.t_offset = target.r_global.transpose()*(t_refined-target.t_global);
    result.measurement.score = result.fitness;
    result.source_center_shift_m = (t_refined-source.t_global).norm();
    const M3D delta_r = result.delta.block<3,3>(0,0).cast<double>();
    result.rotation_deg = std::acos(std::clamp((delta_r.trace()-1.0)/2.0,-1.0,1.0))*180.0/M_PI;
    return result;
}

ViewGateResult SimplePGO::evaluateViewPair(size_t source_idx, int target_idx) const
{
    ViewGateResult result;
    if (source_idx >= m_key_poses.size() || target_idx < 0 || target_idx >= int(source_idx))
        return result;
    const int current_idx = int(source_idx);
    result.first = std::max(0, target_idx-m_config.loop_submap_half_range);
    result.last = std::min(current_idx, target_idx+m_config.loop_submap_half_range);
    // Gate and ICP must use exactly the same target set. Reject self-inclusion.
    if (result.last >= current_idx) { result.reason = "source_in_target_submap"; return result; }
    const auto &source = m_key_poses[source_idx];
    const auto &central = m_key_poses[target_idx];
    if (!source.body_cloud || source.body_cloud->empty() || !source.r_global.allFinite() || !source.t_global.allFinite())
    { result.reason = "invalid_source"; return result; }
    for (int j=result.first; j<=result.last; ++j)
    {
        if (!m_key_poses[j].r_global.allFinite() || !m_key_poses[j].t_global.allFinite() ||
            !m_key_poses[j].body_cloud || m_key_poses[j].body_cloud->empty())
        { result.reason = "invalid_target_pose"; return result; }
        for (const auto &point : *m_key_poses[j].body_cloud)
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            { result.reason = "nonfinite_target_point"; return result; }
    }
    const V3D forward_source = (source.r_global*m_config.view_r_il.col(0)).normalized();
    const V3D forward_target = (central.r_global*m_config.view_r_il.col(0)).normalized();
    result.axis_angle_deg = std::acos(std::clamp(forward_source.dot(forward_target), -1., 1.))*180./M_PI;
    const double max_az = m_config.view_horizontal_fov_deg*M_PI/360.;
    const double max_el = m_config.view_vertical_fov_deg*M_PI/360.;
    for (const auto &point : *source.body_cloud)
    {
        const V3D body(point.x, point.y, point.z);
        if (!body.allFinite()) { result.reason = "nonfinite_source_point"; return result; }
        ++result.finite_source_points;
        // body_cloud is already in IMU/body coordinates; do not apply R_IL twice.
        const V3D world = source.r_global*body+source.t_global;
        for (int j=result.first; j<=result.last; ++j)
        {
            const auto &historical = m_key_poses[j];
            const V3D lidar = m_config.view_r_il.transpose()*(historical.r_global.transpose()*(world-historical.t_global)-m_config.view_t_il);
            const double range = lidar.norm();
            if (range < m_config.view_min_range_m || range > m_config.view_max_range_m) continue;
            if (std::abs(std::atan2(lidar.y(),lidar.x())) <= max_az &&
                std::abs(std::atan2(lidar.z(),std::hypot(lidar.x(),lidar.y()))) <= max_el)
            { ++result.inside_points; break; } // union membership, count each source point once
        }
    }
    if (!result.finite_source_points) { result.reason = "no_finite_source_points"; return result; }
    result.fraction = double(result.inside_points)/result.finite_source_points;
    const bool direction_ok = result.axis_angle_deg <= m_config.loop_view_max_axis_angle_deg;
    const bool fraction_ok = result.fraction >= m_config.loop_view_min_fraction;
    result.pass = direction_ok && fraction_ok;
    result.reason = result.pass ? "pass" : (!direction_ok && !fraction_ok ? "direction_and_fraction" : (!direction_ok ? "direction" : "fraction"));
    return result;
}
