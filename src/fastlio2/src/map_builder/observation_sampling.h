#pragma once
// Experiment-only observation selection. No point/map ownership or filtering.
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace lidar_observation_sampling
{
using V6 = Eigen::Matrix<double, 6, 1>;
using M6 = Eigen::Matrix<double, 6, 6>;
constexpr int NORMAL_BINS = 42;
constexpr double ROTATION_LENGTH_M = 3.0;
constexpr double SPATIAL_CELL_M = 0.5;

enum class Mode { All, Uniform, Normal, Geometry };
inline Mode parseMode(const std::string &s)
{
    if (s == "all") return Mode::All;
    if (s == "uniform") return Mode::Uniform;
    if (s == "normal") return Mode::Normal;
    if (s == "geometry") return Mode::Geometry;
    throw std::runtime_error("LIO_SAMPLE_MODE must be all/uniform/normal/geometry");
}
struct Candidate
{
    uint32_t query_index;
    Eigen::Vector3d body_point;
    Eigen::Vector3d body_normal;
    V6 normalized_j; // actual [rotation/3 m, world translation] Jacobian
};
inline uint64_t hash64(uint64_t x)
{
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}
inline const std::array<Eigen::Vector3d, NORMAL_BINS> &normalAxes()
{
    // Equal-area Fibonacci samples on one hemisphere represent unoriented axes.
    static const auto axes = [] {
        std::array<Eigen::Vector3d, NORMAL_BINS> a;
        constexpr double golden_angle = 2.39996322972865332223;
        for (int i = 0; i < NORMAL_BINS; ++i)
        {
            const double z = (i + 0.5) / NORMAL_BINS;
            const double r = std::sqrt(1.0 - z*z), phi = golden_angle*i;
            a[i] = Eigen::Vector3d(r*std::cos(phi), r*std::sin(phi), z);
        }
        return a;
    }();
    return axes;
}
inline int normalBin(const Eigen::Vector3d &n)
{
    int best = 0;
    double score = -1;
    for (int b = 0; b < NORMAL_BINS; ++b)
    {
        const double s = std::abs(n.dot(normalAxes()[b]));
        if (s > score) { score = s; best = b; }
    }
    return best;
}
struct Cell
{
    std::array<int64_t, 3> xyz;
    bool operator==(const Cell &b) const { return xyz == b.xyz; }
    bool operator<(const Cell &b) const { return xyz < b.xyz; }
};
struct CellHash
{
    std::size_t operator()(const Cell &c) const
    {
        uint64_t h = 0;
        for (auto x : c.xyz) h = hash64(h ^ hash64(static_cast<uint64_t>(x)));
        return static_cast<std::size_t>(h);
    }
};
inline Cell cellOf(const Eigen::Vector3d &p)
{
    return {{{static_cast<int64_t>(std::floor(p.x()/SPATIAL_CELL_M)),
              static_cast<int64_t>(std::floor(p.y()/SPATIAL_CELL_M)),
              static_cast<int64_t>(std::floor(p.z()/SPATIAL_CELL_M))}}};
}
inline std::vector<std::size_t> spatialOrder(const std::vector<Candidate> &c,
                                            const std::vector<std::size_t> &ids)
{
    // Round-robin occupied body-frame cells; dense cells cannot dominate the
    // prefix. Hashing removes an arbitrary fixed spatial sweep direction.
    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells;
    for (auto i : ids) cells[cellOf(c[i].body_point)].push_back(i);
    std::vector<Cell> keys;
    keys.reserve(cells.size());
    for (const auto &kv : cells) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end(), [](const Cell &a, const Cell &b) {
        const auto ha = CellHash{}(a), hb = CellHash{}(b);
        return ha != hb ? ha < hb : a < b;
    });
    for (const auto &key : keys)
    {
        auto &v = cells[key];
        std::sort(v.begin(), v.end(), [&](std::size_t a, std::size_t b) {
            const auto ha = hash64(c[a].query_index), hb = hash64(c[b].query_index);
            return ha != hb ? ha < hb : c[a].query_index < c[b].query_index;
        });
    }
    std::vector<std::size_t> order;
    order.reserve(ids.size());
    for (std::size_t level = 0; order.size() < ids.size(); ++level)
        for (const auto &key : keys)
        {
            const auto &v = cells[key];
            if (level < v.size()) order.push_back(v[level]);
        }
    return order;
}
inline M6 information(const std::vector<Candidate> &c,
                       const std::vector<std::size_t> &indices)
{
    M6 h = M6::Zero();
    for (auto i : indices) h.noalias() += c[i].normalized_j*c[i].normalized_j.transpose();
    return h;
}
inline std::vector<std::size_t> select(const std::vector<Candidate> &c,
                                     Mode mode, std::size_t budget)
{
    if (!budget) throw std::runtime_error("LIO_SAMPLE_BUDGET must be positive");
    std::vector<std::size_t> all(c.size());
    std::iota(all.begin(), all.end(), 0);
    if (mode == Mode::All || c.size() <= budget) return all;
    const auto spatial = spatialOrder(c, all);
    std::vector<std::size_t> chosen;
    chosen.reserve(budget);
    if (mode == Mode::Uniform)
        chosen.assign(spatial.begin(), spatial.begin()+budget);
    else if (mode == Mode::Normal)
    {
        std::array<std::vector<std::size_t>, NORMAL_BINS> buckets;
        for (auto i : all) buckets[normalBin(c[i].body_normal)].push_back(i);
        for (auto &v : buckets) v = spatialOrder(c, v);
        for (std::size_t level = 0; chosen.size() < budget; ++level)
            for (const auto &v : buckets)
            {
                if (level < v.size()) chosen.push_back(v[level]);
                if (chosen.size() == budget) break;
            }
    }
    else
    {
        // Gelfand-inspired eigen-direction balancing, not a reproduction of
        // the original paper. One eigensolve and six sorts per IEKF iteration.
        Eigen::SelfAdjointEigenSolver<M6> eig(information(c, all));
        if (eig.info() != Eigen::Success)
            throw std::runtime_error("Sampling H eigendecomposition failed");
        std::vector<std::size_t> rank(c.size());
        for (std::size_t k = 0; k < spatial.size(); ++k) rank[spatial[k]] = k;
        std::array<std::vector<std::size_t>, 6> queues;
        std::array<std::vector<double>, 6> scores;
        const double active_floor = std::max(1e-12, eig.eigenvalues().maxCoeff()*1e-10);
        for (int d = 0; d < 6; ++d)
        {
            if (eig.eigenvalues()(d) <= active_floor) continue;
            auto &s = scores[d]; s.resize(c.size());
            for (auto i : all)
            {
                const double p = eig.eigenvectors().col(d).dot(c[i].normalized_j);
                // Genuine contribution in the consistently scaled pose space.
                s[i] = p*p;
            }
            queues[d] = all;
            std::sort(queues[d].begin(), queues[d].end(), [&](std::size_t a, std::size_t b) {
                return s[a] != s[b] ? s[a] > s[b] : rank[a] < rank[b];
            });
        }
        std::array<std::size_t, 6> cursor{};
        std::vector<unsigned char> used(c.size(), 0);
        while (chosen.size() < budget)
        {
            bool progressed = false;
            for (int d = 0; d < 6 && chosen.size() < budget; ++d)
            {
                const auto &q = queues[d];
                while (cursor[d] < q.size() && used[q[cursor[d]]]) ++cursor[d];
                if (cursor[d] == q.size()) continue;
                const auto i = q[cursor[d]++];
                used[i] = 1; chosen.push_back(i); progressed = true;
            }
            if (!progressed)
            {
                // Zero-information input is not expected after plane fitting;
                // preserve a deterministic, budget-sized selection if it occurs.
                for (auto i : spatial)
                    if (!used[i] && chosen.size() < budget)
                    { used[i] = 1; chosen.push_back(i); }
                break;
            }
        }
    }
    // Preserve the original numerical accumulation order of retained queries.
    std::sort(chosen.begin(), chosen.end(), [&](std::size_t a, std::size_t b) {
        return c[a].query_index < c[b].query_index;
    });
    return chosen;
}
} // namespace lidar_observation_sampling
