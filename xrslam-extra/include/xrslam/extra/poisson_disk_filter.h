#ifndef XRSLAM_EXTRA_POISSON_DISK_FILTER_H
#define XRSLAM_EXTRA_POISSON_DISK_FILTER_H

#include <unordered_map>
#include <xrslam/xrslam.h>

namespace xrslam::extra {

template <size_t dimension> class PoissonDiskFilter {
  public:
    typedef vector<dimension> point_type;
    typedef std::array<int, dimension> grid_index_type;

    PoissonDiskFilter(double radius)
        : radius(radius), radius_squared(radius * radius),
          grid_size(radius / sqrt(double(dimension))),
          grid_span((int)ceil(sqrt((double)dimension))) {}

    void clear() {
        points.clear();
        sparse_grid.clear();
    }

    void preset_point(const point_type &point) {
        grid_index_type index = to_index(point);
        sparse_grid[index] = points.size();
        points.emplace_back(point);
    }

    void preset_points(const std::vector<point_type> &points) {
        for (const auto &p : points) {
            preset_point(p);
        }
    }

    bool permit_point(const point_type &point) const {
        grid_index_type index;
        return test_point(point, index);
    }

    bool insert_point(const point_type &point) {
        if (grid_index_type index; test_point(point, index)) {
            sparse_grid[index] = points.size();
            points.emplace_back(point);
            return true;
        } else {
            return false;
        }
    }

    void insert_points(std::vector<point_type> &candidate_points) {
        size_t n_point_before = points.size();
        for (const auto &p : candidate_points) {
            if (grid_index_type index; test_point(p, index)) {
                sparse_grid[index] = points.size();
                points.emplace_back(p);
            }
        }
        std::vector<point_type>(points.begin() + n_point_before, points.end())
            .swap(candidate_points);
    }

    const std::vector<point_type> &get_points() const { return points; }

  private:
    grid_index_type to_index(const point_type &p) const {
        grid_index_type index;
        for (size_t i = 0; i < dimension; ++i) {
            index[i] = int(floor(p(i) / grid_size));
        }
        return index;
    }

    bool test_point(const point_type &p, grid_index_type &index) const {
        index = to_index(p);
        grid_index_type ibegin = index, iend = index;
        for (size_t i = 0; i < dimension; ++i) {
            ibegin[i] -= grid_span;
            iend[i] += grid_span;
        }
        grid_index_type icurr = ibegin;
        while (icurr[dimension - 1] <= iend[dimension - 1]) {
            icurr[0]++;
            for (int i = 0; icurr[i] > iend[i] && i < dimension - 1; ++i) {
                icurr[i] = ibegin[i];
                icurr[i + 1]++;
            }
            if (auto it = sparse_grid.find(icurr); it != sparse_grid.end()) {
                if ((p - points[it->second]).squaredNorm() < radius_squared) {
                    return false;
                }
            }
        }
        return true;
    }

    double radius, radius_squared;
    double grid_size;
    int grid_span;

    struct IndexHash {
        std::size_t operator()(const grid_index_type &index) const {
            size_t seed = 0;
            for (size_t i = 0; i < dimension; i++) {
                seed ^= std::hash<int>()(index[i]) + 0x9e3779b9 + (seed << 6) +
                        (seed >> 2);
            }
            return seed;
        }
    };

    std::vector<point_type> points;
    std::unordered_map<grid_index_type, size_t, IndexHash> sparse_grid;
};

} // namespace xrslam::extra

#endif // XRSLAM_EXTRA_POISSON_DISK_FILTER_H
