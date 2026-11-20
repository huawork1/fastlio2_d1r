#include "lidar_processor.h"
#include "match_audit.h"
#include "observation_sampling.h"
#include <Eigen/Eigenvalues>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

// This translation-unit-local observer does not change class layouts, matching,
// filtering, state, covariance, solver inputs, or the order of original math.
// A process runs one LidarProcessor; process()/updateLossFunc() are serial.
namespace
{
class FrontendDiagnostics
{
public:
    FrontendDiagnostics()
    {
        const char *directory = std::getenv("LIO_DIAG_DIR");
        if (!directory || !*directory)
            return;
        const std::filesystem::path root(directory);
        std::filesystem::create_directories(root);
        const auto iterations = root / "iter.csv";
        const auto frames = root / "frame.csv";
        if (std::filesystem::exists(iterations) || std::filesystem::exists(frames))
            throw std::runtime_error("Refusing to overwrite frontend diagnostics");
        iterations_.open(iterations);
        frames_.open(frames);
        if (!iterations_ || !frames_)
            throw std::runtime_error("Cannot open frontend diagnostic files");
        iterations_ << std::setprecision(17);
        frames_ << std::setprecision(17);
        iterations_ << "mapping_index,cloud_start_time,cloud_end_time,iteration,valid,voxel_points,effective_points,inlier_ratio,residual_mean_m,residual_abs_mean_m,residual_rms_m,residual_abs_max_m,translation_h_trace,h_xx,h_xy,h_xz,h_yy,h_yz,h_zz,eig_min,eig_mid,eig_max,eig_min_over_trace,eig_mid_over_trace,eig_max_over_trace,weak_world_x,weak_world_y,weak_world_z\n";
        frames_ << "mapping_index,cloud_start_time,cloud_end_time,raw_deskewed_points,voxel_points,loss_calls,valid_loss_calls,final_effective_points";
        for (const char *prefix : {"prior", "post"})
            for (const char *field : {"px", "py", "pz", "vx", "vy", "vz", "bgx", "bgy", "bgz", "bax", "bay", "baz", "qx", "qy", "qz", "qw"})
                frames_ << ',' << prefix << '_' << field;
        frames_ << ",correction_translation_m,correction_rotation_rad,correction_velocity_m_s,correction_gyro_bias_rad_s,correction_accel_bias_m_s2\n";
        enabled_ = true;
    }

    void begin(const SyncPackage &package, std::size_t voxel_points, const State &prior)
    {
        if (!enabled_)
            return;
        start_ = package.cloud_start_time;
        end_ = package.cloud_end_time;
        raw_points_ = package.cloud->size();
        voxel_points_ = voxel_points;
        prior_ = prior; // The IMU-propagated state immediately before LiDAR update.
        loss_calls_ = valid_calls_ = final_effective_ = 0;
    }

    void iteration(const SharedState &data, int effective_points, const CloudType &normals)
    {
        if (!enabled_)
            return;
        ++loss_calls_;
        final_effective_ = effective_points;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        double mean = nan, abs_mean = nan, rms = nan, abs_max = nan;
        M3D h = M3D::Zero();
        V3D eigenvalues = V3D::Constant(nan), normalized = V3D::Constant(nan);
        V3D weak = V3D::Constant(nan);
        if (data.valid && effective_points > 0)
        {
            ++valid_calls_;
            mean = abs_mean = rms = abs_max = 0;
            for (int i = 0; i < effective_points; ++i)
            {
                const double residual = normals.points[i].intensity;
                mean += residual;
                abs_mean += std::abs(residual);
                rms += residual * residual;
                abs_max = std::max(abs_max, std::abs(residual));
            }
            mean /= effective_points;
            abs_mean /= effective_points;
            rms = std::sqrt(rms / effective_points);
            // H is read only after the original complete accumulation. This is
            // the world-translation normal block, not the full pose information
            // or posterior Hessian. There is no prior or rotation elimination.
            h = data.H.block<3, 3>(3, 3);
            if (h.allFinite())
            {
                Eigen::SelfAdjointEigenSolver<M3D> solver(h);
                if (solver.info() == Eigen::Success)
                {
                    eigenvalues = solver.eigenvalues(); // Ascending order.
                    if (h.trace() > 0)
                        normalized = eigenvalues / h.trace();
                    weak = solver.eigenvectors().col(0); // Sign is arbitrary.
                }
            }
        }
        // For invalid loss calls H is never read: upstream may leave it unset.
        iterations_ << index_ << ',' << start_ << ',' << end_ << ',' << data.iter_num << ','
                    << int(data.valid) << ',' << voxel_points_ << ',' << effective_points << ','
                    << (voxel_points_ ? double(effective_points) / voxel_points_ : nan) << ','
                    << mean << ',' << abs_mean << ',' << rms << ',' << abs_max << ',' << h.trace() << ','
                    << h(0, 0) << ',' << h(0, 1) << ',' << h(0, 2) << ','
                    << h(1, 1) << ',' << h(1, 2) << ',' << h(2, 2);
        for (const auto *v : {&eigenvalues, &normalized, &weak})
            for (int i = 0; i < 3; ++i)
                iterations_ << ',' << (*v)(i);
        iterations_ << '\n';
    }

    void end(const State &post)
    {
        if (!enabled_)
            return;
        frames_ << index_ << ',' << start_ << ',' << end_ << ',' << raw_points_ << ',' << voxel_points_
                << ',' << loss_calls_ << ',' << valid_calls_ << ',' << final_effective_;
        appendState(prior_);
        appendState(post);
        // AngleAxis reads the relative rotation; it does not write to Sophus or
        // the filter. Values are aggregate prior-to-posterior update magnitudes.
        const double angle = Eigen::AngleAxisd(prior_.r_wi.transpose() * post.r_wi).angle();
        frames_ << ',' << (post.t_wi - prior_.t_wi).norm() << ',' << angle
                << ',' << (post.v - prior_.v).norm() << ',' << (post.bg - prior_.bg).norm()
                << ',' << (post.ba - prior_.ba).norm() << '\n';
        iterations_.flush();
        frames_.flush();
        if (!iterations_ || !frames_)
            throw std::runtime_error("Frontend diagnostic write failed");
        ++index_;
    }

private:
    void appendState(const State &state)
    {
        for (const auto *v : {&state.t_wi, &state.v, &state.bg, &state.ba})
            for (int i = 0; i < 3; ++i)
                frames_ << ',' << (*v)(i);
        const Eigen::Quaterniond q(state.r_wi);
        frames_ << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
    }
    bool enabled_ = false;
    std::ofstream iterations_, frames_;
    std::size_t index_ = 0, raw_points_ = 0, voxel_points_ = 0;
    int loss_calls_ = 0, valid_calls_ = 0, final_effective_ = 0;
    double start_ = 0, end_ = 0;
    State prior_;
};

FrontendDiagnostics &frontendDiagnostics()
{
    static FrontendDiagnostics diagnostics;
    return diagnostics;
}

class ObservationSampler
{
public:
    ObservationSampler()
    {
        const char *mode = std::getenv("LIO_SAMPLE_MODE");
        mode_name_ = mode && *mode ? mode : "all";
        mode_ = lidar_observation_sampling::parseMode(mode_name_);
        const char *budget = std::getenv("LIO_SAMPLE_BUDGET");
        if (budget && *budget)
        {
            std::size_t consumed = 0;
            const auto parsed = std::stoll(budget, &consumed);
            if (consumed != std::string(budget).size() || parsed <= 0)
                throw std::runtime_error("Invalid LIO_SAMPLE_BUDGET");
            budget_ = static_cast<std::size_t>(parsed);
        }
        const char *directory = std::getenv("LIO_SAMPLE_DIAG_DIR");
        if (!directory || !*directory) directory = std::getenv("LIO_DIAG_DIR");
        if (!directory || !*directory) return;
        const std::filesystem::path root(directory);
        std::filesystem::create_directories(root);
        const auto csv = root / "sampling_iter.csv";
        const auto bin = root / "sampling_selected_indices.bin";
        if (std::filesystem::exists(csv) || std::filesystem::exists(bin))
            throw std::runtime_error("Refusing to overwrite observation sampling audit");
        csv_.open(csv); selected_indices_.open(bin, std::ios::binary);
        if (!csv_ || !selected_indices_) throw std::runtime_error("Cannot open sampling audit");
        csv_ << std::setprecision(17);
        csv_ << "mapping_index,cloud_start_time,cloud_end_time,relative_s,iteration,loss_call,mode,budget,candidate_count,selected_count,indices_offset_bytes,selection_ms,diagnostics_ms";
        for (const char *prefix : {"candidate", "selected"})
        {
            csv_ << ',' << prefix << "_h_trace";
            for (int i = 0; i < 6; ++i) csv_ << ',' << prefix << "_eig_" << i;
            for (int i = 0; i < 6; ++i) csv_ << ',' << prefix << "_eig_fraction_" << i;
            for (int i = 0; i < lidar_observation_sampling::NORMAL_BINS; ++i)
                csv_ << ',' << prefix << "_normal_bin_" << i;
        }
        csv_ << '\n';
        logging_ = true;
    }
    bool needsCandidates() const
    { return mode_ != lidar_observation_sampling::Mode::All || logging_; }
    void begin(const SyncPackage &p)
    {
        frame_ = next_frame_++;
        start_ = p.cloud_start_time; end_ = p.cloud_end_time; loss_call_ = 0;
    }
    void sample(const std::vector<lidar_observation_sampling::Candidate> &c,
                int iteration, std::size_t point_count, std::vector<unsigned char> &mask)
    {
        using namespace lidar_observation_sampling;
        const auto started = std::chrono::steady_clock::now();
        for (const auto &p : c)
            if (!p.body_point.allFinite() || !p.body_normal.allFinite() || !p.normalized_j.allFinite())
                throw std::runtime_error("Nonfinite eligible sampling candidate");
        const auto selected = select(c, mode_, budget_);
        mask.assign(point_count, 0);
        for (auto i : selected) mask[c[i].query_index] = 1;
        const auto sampled = std::chrono::steady_clock::now();
        if (!logging_) return;
        std::vector<std::size_t> all(c.size()); std::iota(all.begin(), all.end(), 0);
        const std::array<M6, 2> hs{{information(c, all), information(c, selected)}};
        std::array<V6, 2> eigenvalues;
        std::array<std::array<std::size_t, NORMAL_BINS>, 2> bins{};
        for (int k = 0; k < 2; ++k)
        {
            Eigen::SelfAdjointEigenSolver<M6> eig(hs[k]);
            if (eig.info() != Eigen::Success) throw std::runtime_error("Sampling audit H eigensolve failed");
            eigenvalues[k] = eig.eigenvalues();
        }
        for (auto i : all) ++bins[0][normalBin(c[i].body_normal)];
        for (auto i : selected) ++bins[1][normalBin(c[i].body_normal)];
        const auto diagnosed = std::chrono::steady_clock::now();
        const auto offset = selected_indices_.tellp();
        for (auto i : selected)
        {
            const uint32_t q = c[i].query_index;
            selected_indices_.write(reinterpret_cast<const char *>(&q), sizeof(q));
        }
        csv_ << frame_ << ',' << start_ << ',' << end_ << ',' << end_-lidar_match_audit::BAG_ORIGIN
             << ',' << iteration << ',' << loss_call_++ << ',' << mode_name_ << ',' << budget_
             << ',' << c.size() << ',' << selected.size() << ',' << static_cast<uint64_t>(offset)
             << ',' << std::chrono::duration<double, std::milli>(sampled-started).count()
             << ',' << std::chrono::duration<double, std::milli>(diagnosed-sampled).count();
        for (int k = 0; k < 2; ++k)
        {
            const double trace = hs[k].trace();
            csv_ << ',' << trace;
            for (int d = 0; d < 6; ++d) csv_ << ',' << eigenvalues[k](d);
            for (int d = 0; d < 6; ++d)
                csv_ << ',' << (trace > 0 ? eigenvalues[k](d)/trace : 0.0);
            for (auto n : bins[k]) csv_ << ',' << n;
        }
        csv_ << '\n';
        csv_.flush(); selected_indices_.flush();
        if (!csv_ || !selected_indices_) throw std::runtime_error("Sampling audit write failed");
    }
private:
    lidar_observation_sampling::Mode mode_ = lidar_observation_sampling::Mode::All;
    std::string mode_name_;
    std::size_t budget_ = 1500, frame_ = 0, next_frame_ = 0;
    int loss_call_ = 0;
    double start_ = 0, end_ = 0;
    bool logging_ = false;
    std::ofstream csv_, selected_indices_;
};

ObservationSampler &observationSampler()
{
    static ObservationSampler sampler;
    return sampler;
}
} // namespace


LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
}

void LidarProcessor::trimCloudMap()
{
    m_local_map.cub_to_rm.clear();
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);

    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    auto &match_audit = lidar_match_audit::observer();
    match_audit.beginCandidates();
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            match_audit.candidate(i, m_cloud_down_world->points[i], m_point_selected_flag[i], 0);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            match_audit.candidate(i, m_cloud_down_world->points[i], m_point_selected_flag[i], 1);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            match_audit.candidate(i, m_cloud_down_world->points[i], m_point_selected_flag[i], 0);
        }
    }
    match_audit.candidates(point_to_add);
    match_audit.candidates(point_no_need_downsample);
    match_audit.endCandidates();
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
    lidar_match_audit::observer().built(point_vec, false);
}

CloudType::Ptr LidarProcessor::exportCloud()
{
    CloudType::Ptr cloud(new CloudType);
    int tree_size = 0, invalid_points = 0;
    snapshotCloudMap(cloud->points, tree_size, invalid_points);
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

void LidarProcessor::replaceCloudMap(CloudType::Ptr replacement, bool downsample)
{
    if (!replacement || replacement->empty())
        throw std::invalid_argument("Replacement map must contain points");
    for (const auto &p : replacement->points)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            throw std::invalid_argument("Replacement map contains nonfinite XYZ");

    CloudType filtered;
    if (downsample && m_config.map_resolution > 0.0)
    {
        pcl::VoxelGrid<PointType> filter;
        filter.setLeafSize(m_config.map_resolution, m_config.map_resolution, m_config.map_resolution);
        filter.setInputCloud(replacement);
        filter.filter(filtered);
    }
    else
        filtered = *replacement;
    if (filtered.empty())
        throw std::invalid_argument("Replacement map is empty after filtering");

    // Build in an independent tree. Destroying the old tree stops/joins its
    // background worker before disposing of its storage.
    auto next_tree = std::make_shared<KD_TREE<PointType>>();
    next_tree->set_downsample_param(m_config.map_resolution);
    next_tree->Build(filtered.points);
    m_ikdtree.swap(next_tree);
    lidar_match_audit::observer().built(filtered.points, true);
    m_local_map = LocalMap{};
    for (auto &points : m_nearest_points)
        points.clear();
    std::fill(m_point_selected_flag.begin(), m_point_selected_flag.end(), 0);
}

void LidarProcessor::process(SyncPackage &package)
{
    // m_kf->setLossFunction([&](State &s, SharedState &d)
    //                       { updateLossFunc(s, d); });
    // m_kf->setStopFunction([&](const V21D &delta) -> bool
    //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
    //                         V3D t_delta = delta.block<3, 1>(3, 0);
    //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }
    // All per-point buffers must fit the actual downsampled frame.
    const auto count = m_cloud_down_lidar->size();
    m_cloud_down_world->resize(count);
    m_norm_vec->resize(count);
    m_effect_cloud_lidar->resize(count);
    m_effect_norm_vec->resize(count);
    m_nearest_points.resize(count);
    m_point_selected_flag.resize(count, 0);
    trimCloudMap();
    frontendDiagnostics().begin(package, m_cloud_down_lidar->size(), m_kf->x());
    lidar_match_audit::observer().begin(package, m_kf->P());
    observationSampler().begin(package);
    m_kf->update();
    incrCloudMap();
    frontendDiagnostics().end(m_kf->x());
    lidar_match_audit::observer().end(m_kf->P());
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    int size = m_cloud_down_lidar->size();
    auto &match_audit = lidar_match_audit::observer();
    match_audit.beginIteration(size, m_config.near_search_num);
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++)
    {
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        match_audit.searched(i, point_body, point_world, point_world_vec, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Vector4d pabcd;
        m_point_selected_flag[i] = false;
        if (esti_plane(points_near, 0.1, pabcd))
        {
            match_audit.plane(i, pabcd);
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            match_audit.residual(i, pd2, s);
            if (s > 0.9)
            {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = pabcd(0);
                m_norm_vec->points[i].y = pabcd(1);
                m_norm_vec->points[i].z = pabcd(2);
                m_norm_vec->points[i].intensity = pd2;
                match_audit.accepted(i);
            }
        }
        else
            match_audit.plane(i, pabcd);
    }

    // Selection happens only after the existing KNN/plane/residual eligibility
    // checks. It cannot change m_point_selected_flag or map insertion decisions.
    auto &sampler = observationSampler();
    const bool use_sampling_mask = sampler.needsCandidates();
    if (use_sampling_mask)
    {
        using namespace lidar_observation_sampling;
        std::vector<Candidate> candidates;
        candidates.reserve(size);
        for (int i = 0; i < size; ++i)
        {
            if (!m_point_selected_flag[i]) continue;
            const auto &p = m_cloud_down_lidar->points[i];
            const auto &n = m_norm_vec->points[i];
            const V3D point_l(p.x, p.y, p.z), normal_w(n.x, n.y, n.z);
            const V3D point_i = state.r_il * point_l + state.t_il;
            Candidate c;
            c.query_index = static_cast<uint32_t>(i);
            c.body_point = point_i;
            c.body_normal = state.r_wi.transpose() * normal_w;
            const Eigen::Matrix<double, 1, 3> b = -normal_w.transpose() * state.r_wi * Sophus::SO3d::hat(point_i);
            c.normalized_j.head<3>() = b.transpose()/ROTATION_LENGTH_M;
            c.normalized_j.tail<3>() = normal_w;
            candidates.push_back(c);
        }
        sampler.sample(candidates, share_data.iter_num, size, m_loss_sampling_mask);
    }

    int effect_feat_num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!m_point_selected_flag[i])
            continue;
        if (use_sampling_mask && !m_loss_sampling_mask[i])
            continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        match_audit.effectiveIndex(i);
        effect_feat_num++;
    }
    if (effect_feat_num < 1)
    {
        share_data.valid = false;
        frontendDiagnostics().iteration(share_data, effect_feat_num, *m_effect_norm_vec);
        match_audit.iteration(state, share_data, effect_feat_num, m_config.lidar_cov_inv, m_config.near_search_num);
        std::cerr << "NO Effective Points!" << std::endl;
        return;
    }
    share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &norm_p = m_effect_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_il);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        share_data.H += J.transpose() * m_config.lidar_cov_inv * J;
        share_data.b += J.transpose() * m_config.lidar_cov_inv * norm_p.intensity;
        match_audit.jacobian(i, J);
    }
    frontendDiagnostics().iteration(share_data, effect_feat_num, *m_effect_norm_vec);
    match_audit.iteration(state, share_data, effect_feat_num, m_config.lidar_cov_inv, m_config.near_search_num);
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}
