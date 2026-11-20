#pragma once
// Experiment-only, read-only observer. Nothing from this class is supplied to
// nearest-neighbor search, plane fitting, the ESKF, or map insertion.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace lidar_match_audit
{
constexpr double BAG_ORIGIN = 1792128572.485967625;
constexpr double START_REL = 530.0;
constexpr double END_REL = 600.0;

// Native little-endian IEEE754; explicit ordering/padding has a checked layout.
struct PointRecord
{
    double world_f64[3];
    double plane[4];
    double residual;
    double score;
    double neighbor_first_rel[5];
    double jacobian6[6];
    float lidar_xyz[3];
    float world_search_xyz[3];
    float neighbor_xyz[15];
    float neighbor_sq_dist[5];
    uint32_t query_index;
    uint32_t neighbor_count;
    uint32_t reason;
    uint8_t neighbor_source[5];
    uint8_t padding[7];
};
static_assert(sizeof(PointRecord) == 288, "Unexpected audit record layout");
static_assert(offsetof(PointRecord, lidar_xyz) == 160, "Unexpected audit offsets");
static_assert(offsetof(PointRecord, query_index) == 264, "Unexpected audit offsets");
static_assert(offsetof(PointRecord, neighbor_source) == 276, "Unexpected audit offsets");

struct CandidateRecord
{
    float world_xyz[3];
    uint32_t query_index;
    uint32_t final_loss_accepted;
    uint32_t add_kind; // 0 Add_Points(...,true); 1 Add_Points(...,false)
    uint32_t final_loss_reason;
    uint32_t reserved;
};
static_assert(sizeof(CandidateRecord) == 32, "Unexpected candidate layout");

struct Key
{
    std::array<uint32_t, 3> bits;
    bool operator==(const Key &other) const { return bits == other.bits; }
};
struct KeyHash
{
    std::size_t operator()(const Key &key) const
    {
        std::size_t h = 1469598103934665603ULL;
        for (auto v : key.bits)
            h = (h ^ v) * 1099511628211ULL;
        return h;
    }
};
struct Origin
{
    double first_rel;
    uint8_t source;
};

class Observer
{
public:
    Observer()
    {
        const char *directory = std::getenv("LIO_MATCH_AUDIT_DIR");
        if (!directory || !*directory)
            directory = std::getenv("LIO_DIAG_DIR");
        if (!directory || !*directory)
            return;
        const uint16_t endian = 1;
        if (*reinterpret_cast<const uint8_t *>(&endian) != 1)
            throw std::runtime_error("Match audit requires little-endian platform");
        const std::filesystem::path root(directory);
        std::filesystem::create_directories(root);
        for (const char *name : {"match_points.bin", "match_iter.csv", "match_frame.csv", "match_covariance.bin", "match_candidates.bin", "match_candidates.csv"})
            if (std::filesystem::exists(root / name))
                throw std::runtime_error("Refusing to overwrite match audit");
        points_.open(root / "match_points.bin", std::ios::binary);
        covariances_.open(root / "match_covariance.bin", std::ios::binary);
        candidate_bin_.open(root / "match_candidates.bin", std::ios::binary);
        candidate_csv_.open(root / "match_candidates.csv");
        iterations_.open(root / "match_iter.csv");
        frames_.open(root / "match_frame.csv");
        if (!points_ || !covariances_ || !iterations_ || !frames_ || !candidate_bin_ || !candidate_csv_)
            throw std::runtime_error("Cannot open match audit outputs");
        iterations_ << std::setprecision(17);
        frames_ << std::setprecision(17);
        candidate_csv_ << std::setprecision(17);
        iterations_ << "mapping_index,cloud_start_time,cloud_end_time,relative_s,iteration,loss_call,valid,effective_points,point_count,offset_bytes,epoch,lidar_cov_inv,near_search_num";
        for (const char *name : {"R_wi", "R_il"})
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    iterations_ << ',' << name << '_' << i << j;
        for (const char *name : {"p_wi", "t_il"})
            for (int i = 0; i < 3; ++i)
                iterations_ << ',' << name << '_' << i;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                iterations_ << ",H_" << i << j;
        for (int i = 0; i < 6; ++i)
            iterations_ << ",b_" << i;
        iterations_ << '\n';
        frames_ << "mapping_index,cloud_start_time,cloud_end_time,relative_s,cov_offset_bytes,loss_calls,epoch,registry_size\n";
        candidate_csv_ << "mapping_index,cloud_end_time,relative_s,offset_bytes,point_count,epoch\n";
        enabled_ = true;
    }

    bool active() const { return active_; }

    void begin(const SyncPackage &package, const M21D &prior_p)
    {
        if (!enabled_)
            return;
        start_ = package.cloud_start_time;
        end_ = package.cloud_end_time;
        relative_ = end_ - BAG_ORIGIN;
        active_ = relative_ >= START_REL && relative_ <= END_REL;
        loss_calls_ = 0;
        if (active_)
            prior_p_ = prior_p;
    }

    void beginIteration(std::size_t count, int near_count)
    {
        if (!active_)
            return;
        if (near_count != 5)
            throw std::runtime_error("Match audit record requires five-neighbor configuration");
        records_.resize(count);
        accepted_indices_.clear();
        accepted_indices_.reserve(count);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const float fnan = std::numeric_limits<float>::quiet_NaN();
        for (std::size_t i = 0; i < count; ++i)
        {
            auto &r = records_[i];
            std::memset(&r, 0, sizeof(r));
            for (auto &v : r.world_f64) v = nan;
            for (auto &v : r.plane) v = nan;
            r.residual = r.score = nan;
            for (auto &v : r.neighbor_first_rel) v = nan;
            for (auto &v : r.jacobian6) v = nan;
            for (auto &v : r.lidar_xyz) v = fnan;
            for (auto &v : r.world_search_xyz) v = fnan;
            for (auto &v : r.neighbor_xyz) v = fnan;
            for (auto &v : r.neighbor_sq_dist) v = fnan;
            r.query_index = static_cast<uint32_t>(i);
            r.reason = 5; // not processed; should never remain in a complete row
        }
    }

    // Called by independent OpenMP point workers. Registry is read-only during
    // the loss call; each worker writes only its own already allocated record.
    void searched(int i, const PointType &lidar, const PointType &world,
                  const V3D &world_double, const PointVec &neighbors,
                  const std::vector<float> &sq_distances)
    {
        if (!active_) return;
        auto &r = records_[i];
        r.lidar_xyz[0] = lidar.x; r.lidar_xyz[1] = lidar.y; r.lidar_xyz[2] = lidar.z;
        r.world_search_xyz[0] = world.x; r.world_search_xyz[1] = world.y; r.world_search_xyz[2] = world.z;
        for (int k = 0; k < 3; ++k) r.world_f64[k] = world_double(k);
        r.neighbor_count = static_cast<uint32_t>(neighbors.size());
        r.reason = neighbors.size() < 5 ? 1 : (!(sq_distances[4] <= 5) ? 2 : 3);
        for (std::size_t j = 0; j < neighbors.size() && j < 5; ++j)
        {
            const auto &p = neighbors[j];
            r.neighbor_xyz[3*j] = p.x; r.neighbor_xyz[3*j+1] = p.y; r.neighbor_xyz[3*j+2] = p.z;
            if (j < sq_distances.size()) r.neighbor_sq_dist[j] = sq_distances[j];
            const auto it = registry_.find(key(p));
            if (it != registry_.end())
            {
                r.neighbor_first_rel[j] = it->second.first_rel;
                r.neighbor_source[j] = it->second.source;
            }
        }
    }

    void plane(int i, const Eigen::Vector4d &pabcd)
    {
        if (!active_) return;
        for (int k = 0; k < 4; ++k) records_[i].plane[k] = pabcd(k);
    }
    void residual(int i, double pd2, double score)
    {
        if (!active_) return;
        records_[i].residual = pd2; // original double; solver uses float(pd2)
        records_[i].score = score;
        records_[i].reason = 4;
    }
    void accepted(int i)
    {
        if (!active_) return;
        records_[i].reason = 0;
    }
    void effectiveIndex(int i)
    {
        if (active_) accepted_indices_.push_back(i);
    }
    template <typename Derived>
    void jacobian(int effective_index, const Eigen::MatrixBase<Derived> &j)
    {
        if (!active_) return;
        auto &r = records_[accepted_indices_[effective_index]];
        for (int k = 0; k < 6; ++k) r.jacobian6[k] = j(k);
    }

    void iteration(const State &state, const SharedState &data, int effective,
                   double weight, int near_count)
    {
        if (!active_) return;
        const auto offset = points_.tellp();
        points_.write(reinterpret_cast<const char *>(records_.data()), records_.size() * sizeof(PointRecord));
        iterations_ << index_ << ',' << start_ << ',' << end_ << ',' << relative_ << ','
                    << data.iter_num << ',' << loss_calls_++ << ',' << int(data.valid) << ','
                    << effective << ',' << records_.size() << ',' << static_cast<uint64_t>(offset)
                    << ',' << epoch_ << ',' << weight << ',' << near_count;
        for (const auto *r : {&state.r_wi, &state.r_il})
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    iterations_ << ',' << (*r)(i, j);
        for (const auto *v : {&state.t_wi, &state.t_il})
            for (int i = 0; i < 3; ++i)
                iterations_ << ',' << (*v)(i);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                iterations_ << ',' << (data.valid ? data.H(i,j) : nan);
        for (int i = 0; i < 6; ++i)
            iterations_ << ',' << (data.valid ? data.b(i) : nan);
        iterations_ << '\n';
    }

    void candidates(const PointVec &points)
    {
        if (!enabled_ || relative_ > END_REL) return;
        const uint8_t source = relative_ < START_REL ? 2 : 3;
        for (const auto &p : points)
            registry_.emplace(key(p), Origin{relative_, source});
    }
    void beginCandidates()
    {
        if (active_) candidate_records_.clear();
    }
    void candidate(int query_index, const PointType &p, bool accepted, uint32_t kind)
    {
        if (!active_) return;
        CandidateRecord r{};
        r.world_xyz[0] = p.x; r.world_xyz[1] = p.y; r.world_xyz[2] = p.z;
        r.query_index = query_index;
        r.final_loss_accepted = accepted;
        r.add_kind = kind;
        r.final_loss_reason = records_[query_index].reason;
        candidate_records_.push_back(r);
    }
    void endCandidates()
    {
        if (!active_) return;
        const auto offset = candidate_bin_.tellp();
        candidate_bin_.write(reinterpret_cast<const char *>(candidate_records_.data()), candidate_records_.size() * sizeof(CandidateRecord));
        candidate_csv_ << index_ << ',' << end_ << ',' << relative_ << ','
                       << static_cast<uint64_t>(offset) << ',' << candidate_records_.size()
                       << ',' << epoch_ << '\n';
    }
    void built(const PointVec &points, bool replacement)
    {
        if (!enabled_) return;
        registry_.clear();
        ++epoch_;
        const uint8_t source = replacement ? 4 : 1;
        // Rebuild timestamp is not the source scans' timestamps.
        const double time = replacement ? relative_ : std::numeric_limits<double>::quiet_NaN();
        for (const auto &p : points)
            registry_.emplace(key(p), Origin{time, source});
    }
    void end(const M21D &post_p)
    {
        if (!enabled_) return;
        if (active_)
        {
            const auto offset = covariances_.tellp();
            const M21D *covariances[] = {&prior_p_, &post_p};
            for (const auto *p : covariances)
                for (int i = 0; i < 21; ++i)
                    for (int j = 0; j < 21; ++j)
                    {
                        const double value = (*p)(i, j);
                        covariances_.write(reinterpret_cast<const char *>(&value), sizeof(value));
                    }
            frames_ << index_ << ',' << start_ << ',' << end_ << ',' << relative_ << ','
                    << static_cast<uint64_t>(offset) << ',' << loss_calls_ << ',' << epoch_
                    << ',' << registry_.size() << '\n';
            points_.flush(); covariances_.flush(); iterations_.flush(); frames_.flush();
            candidate_bin_.flush(); candidate_csv_.flush();
            if (!points_ || !covariances_ || !iterations_ || !frames_ || !candidate_bin_ || !candidate_csv_)
                throw std::runtime_error("Match audit write failed");
        }
        ++index_;
        active_ = false;
    }

private:
    static Key key(const PointType &p)
    {
        Key result;
        std::memcpy(&result.bits[0], &p.x, sizeof(float));
        std::memcpy(&result.bits[1], &p.y, sizeof(float));
        std::memcpy(&result.bits[2], &p.z, sizeof(float));
        return result;
    }
    bool enabled_ = false, active_ = false;
    std::size_t index_ = 0;
    uint64_t epoch_ = 0;
    unsigned loss_calls_ = 0;
    double start_ = 0, end_ = 0, relative_ = -std::numeric_limits<double>::infinity();
    M21D prior_p_;
    std::vector<PointRecord> records_;
    std::vector<CandidateRecord> candidate_records_;
    std::vector<int> accepted_indices_;
    std::unordered_map<Key, Origin, KeyHash> registry_;
    std::ofstream points_, covariances_, iterations_, frames_;
    std::ofstream candidate_bin_, candidate_csv_;
};

inline Observer &observer()
{
    static Observer instance;
    return instance;
}
} // namespace lidar_match_audit
