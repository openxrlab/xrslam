#ifndef XRSLAM_RANSAC_H
#define XRSLAM_RANSAC_H

#include <xrslam/common.h>
#include <xrslam/utility/random.h>

namespace xrslam {

template <size_t ModelDoF, typename ModelType, typename ModelSolver,
          typename ModelEvaluator>
struct Ransac {
    double threshold;
    double confidence;
    size_t max_iteration;
    int seed;

    ModelType model;
    size_t inlier_count;
    std::vector<char> inlier_mask;

    Ransac(double threshold, double confidence = 0.999,
           size_t max_iteration = 1000, int seed = 0)
        : threshold(threshold), confidence(confidence),
          max_iteration(max_iteration), seed(seed) {}

    template <typename... DataTypes>
    ModelType solve(const std::vector<DataTypes> &...data) {
        std::tuple<const std::vector<DataTypes> &...> tdata =
            std::make_tuple(std::cref(data)...);
        size_t size = std::get<0>(tdata).size();

        LotBox lotbox(size);
        lotbox.seed(seed);

        double K = log(std::max(1 - confidence, 1.0e-5));

        inlier_count = 0;

        if (size < ModelDoF) {
            std::vector<char> _(size, 0);
            inlier_mask.swap(_);
            return model;
        }

        size_t iter_max = max_iteration;
        for (size_t iter = 0; iter < iter_max; ++iter) {
            std::tuple<std::array<DataTypes, ModelDoF>...> tsample;

            lotbox.refill_all();
            for (size_t si = 0; si < ModelDoF; ++si) {
                size_t sample_index = lotbox.draw_without_replacement();
                make_sample(tdata, tsample, sample_index, si);
            }

            std::vector<ModelType> models{apply(ModelSolver(), tsample)};
            for (const auto &current_model : models) {
                size_t current_inlier_count = 0;
                std::vector<char> current_inlier_mask(size, 0);
                ModelEvaluator eval(current_model);
                for (size_t i = 0; i < size; ++i) {
                    double error = eval(data[i]...);
                    if (error <= threshold) {
                        current_inlier_count++;
                        current_inlier_mask[i] = 1;
                    }
                }

                if (current_inlier_count > inlier_count) {
                    model = current_model;
                    inlier_count = current_inlier_count;
                    inlier_mask.swap(current_inlier_mask);

                    double inlier_ratio = inlier_count / (double)size;
                    double N = K / log(1 - pow(inlier_ratio, 5));
                    if (N < (double)iter_max) {
                        iter_max = (size_t)ceil(N);
                    }
                }
            }
        }

        return model;
    }

  private:
    template <class Data, class Sample, size_t... I>
    static void make_sample_impl(Data &&data, Sample &&sample, size_t idata,
                                 size_t isample, std::index_sequence<I...>) {
        [[maybe_unused]] auto ret = {((void *)&(
            std::get<I>(sample)[isample] = std::get<I>(data)[idata]))...};
    }

    template <class Data, class Sample>
    static void make_sample(Data &&data, Sample &&sample, size_t idata,
                            size_t isample) {
        make_sample_impl(
            std::forward<Data>(data), std::forward<Sample>(sample), idata,
            isample,
            std::make_index_sequence<std::tuple_size<
                typename std::remove_reference<Data>::type>::value>{});
    }
};

} // namespace xrslam

#endif // XRSLAM_RANSAC_H
