#ifndef XRSLAM_IMU_PARSAC_H
#define XRSLAM_IMU_PARSAC_H

#include <stdlib.h>
#include <xrslam/common.h>
#include <xrslam/utility/random.h>

namespace xrslam {

template <size_t ModelDoF, typename ModelType, typename ModelSolver,
          typename ModelEvaluator>
struct IMU_Parsac {
    double threshold;
    double confidence;
    size_t max_iteration;
    int seed;

    ModelType model;
    size_t inlier_count;
    std::vector<char> inlier_mask;

    IMU_Parsac(double threshold, double confidence = 0.999,
               size_t max_iteration = 1000, int seed = 0)
        : threshold(threshold), confidence(confidence),
          max_iteration(max_iteration), seed(seed),
          m_parsacMinPriorBinConfidence(0.5) {}

    template <typename... DataTypes>
    ModelType solve(std::vector<float> &binConfidences,
                    const std::vector<DataTypes> &...data) {
        std::tuple<const std::vector<DataTypes> &...> tdata =
            std::make_tuple(std::cref(data)...);
        size_t size = std::get<0>(tdata).size();

        auto &pts1 = std::get<0>(tdata);
        auto &pts2 = std::get<1>(tdata);

        LotBox lotbox(size);
        lotbox.seed(seed);

        if (m_verbose) {
            std::cout << "============== IMU PARSAC ================"
                      << std::endl;
            std::cout << "-- Matches Size: " << pts1.size() << std::endl;
        }

        double K = log(std::max(1 - confidence, 1.0e-5));

        inlier_count = 0;

        if (size < ModelDoF) {
            std::vector<char> _(size, 0);
            inlier_mask.swap(_);
            return model;
        }

        SetBins(20, 20);
        if (m_verbose)
            std::cout << "-- SetBins()" << std::endl;

        CreateBucket();
        if (m_verbose)
            std::cout << "-- CreateBucket()" << std::endl;

        BucketData(pts2);
        if (m_verbose)
            std::cout << "-- BucketData()" << std::endl;

        ConvertConfidencesBinToValidBin(binConfidences,
                                        m_validBinConfidencesPrior);
        if (m_verbose)
            std::cout << "-- ConvertConfidencesBinToValidBin()" << std::endl;

        ThresholdAndNormalizeConfidences(m_validBinConfidencesPrior);
        if (m_verbose)
            std::cout << "-- ThresholdAndNormalizeConfidences()" << std::endl;

        AccumulateConfidences(m_validBinConfidencesPrior,
                              m_validBinConfidencesAccumulatedPrior);
        if (m_verbose)
            std::cout << "-- AccumulateConfidences()" << std::endl;

        Sampler sampler(m_validBinConfidencesAccumulatedPrior);
        if (m_verbose)
            std::cout << "-- sampler()" << std::endl;

        if (!ComputePriorDistribution(pts1, pts2)) {
            if (m_verbose)
                std::cout << "-- ComputePriorDistribution()" << std::endl;
            // std::cout << "IMU Process" << std::endl;
            inlier_mask = std::vector<char>(size, 1);
            return matrix<4>::Identity();
        }

        std::vector<std::vector<size_t>> validBinInliersBest;
        size_t iter_max = max_iteration;
        float scoreMax = -FLT_MAX, score;
        for (size_t iter = 0; iter < iter_max; ++iter) {
            // if(m_verbose) std::cout << "iters: " << iter << " " << iter_max
            // << std::endl;
            std::tuple<std::array<DataTypes, ModelDoF>...> tsample;

            lotbox.refill_all();
            sampler.refill_all();

            for (size_t si = 0; si < ModelDoF; ++si) {
                size_t sample_index;
                if (m_nValidBins > 20)
                    sample_index = sampler.draw_by_weight();
                else
                    sample_index = lotbox.draw_without_replacement();
                make_sample(tdata, tsample, sample_index, si);
            }

            std::vector<ModelType> models{apply(ModelSolver(), tsample)};
            int cnt = 0;
            for (const auto &current_model : models) {
                size_t current_inlier_count = 0;
                size_t current_outlier_count = 0;
                std::vector<char> current_inlier_mask(size, 0);
                std::vector<char> current_outlier_mask(size, 0);
                ModelEvaluator eval(current_model);
                for (size_t i = 0; i < size; ++i) {
                    double error = eval(data[i]...);
                    if (error <= threshold) {
                        current_inlier_count++;
                        current_inlier_mask[i] = 1;
                    } else {
                        current_outlier_mask[i] = 1;
                        current_outlier_count++;
                    }
                }

                size_t overlap_inlier_count = 0;
                std::vector<char> overlap_inlier_mask(size, 0);
                for (int i = 0; i < size; i++) {
                    if (m_prior_inliers_mask[i] && current_inlier_mask[i]) {
                        overlap_inlier_mask[i] = 1;
                        overlap_inlier_count++;
                    }
                }

                if (overlap_inlier_count < ModelDoF)
                    continue;

                std::vector<std::vector<size_t>> validBinInliers;
                std::vector<std::vector<size_t>> validBinOutliers;
                ConvertInliersListToValidBin(current_inlier_mask,
                                             validBinInliers);
                // if(m_verbose) std::cout << "  --
                // ConvertInliersListToValidBin()" << std::endl;
                score = ComputeScore(validBinInliers, m_validBinConfidences);

                // if(m_verbose) std::cout << "  -- ComputeScore()" <<
                // std::endl;
                if (score > scoreMax ||
                    score == scoreMax &&
                        (overlap_inlier_count > inlier_count)) {
                    scoreMax = score;
                    model = current_model;
                    inlier_count = overlap_inlier_count;
                    validBinInliersBest = validBinInliers;
                    // inlier_mask.swap(overlap_inlier_mask);
                    inlier_mask.swap(current_inlier_mask);
                    double inlier_ratio = inlier_count / (double)size;
                    double N = K / log(1 - pow(inlier_ratio, 5));
                    if (N < (double)iter_max) {
                        iter_max = (size_t)ceil(N);
                    }
                }
                // if(m_verbose) std::cout << "  -- finish update" << std::endl;
            }
            // if(m_verbose) std::cout << "  -- finish iterate" << std::endl;
        }

        if (inlier_count < ModelDoF) {
            inlier_mask = std::vector<char>(size, 1);
            return matrix<4>::Identity();
        }

        if (m_verbose)
            std::cout << "-- Before ComputeScore()" << std::endl;
        ComputeScore(validBinInliersBest, m_validBinConfidences);
        if (m_verbose)
            std::cout << "-- ComputeScore()" << std::endl;
        ConvertConfidencesValidBinToBin(m_validBinConfidences, binConfidences);
        if (m_verbose)
            std::cout << "-- ConvertConfidencesValidBinToBin()" << std::endl;

        return model;
    }

    matrix<4> prior_model;
    std::vector<size_t> m_lens;
    std::vector<char> m_prior_inliers_mask;

    void SetLens(const std::vector<size_t> &lens) { m_lens = lens; }

    void SetPriorPose(matrix<3> R, vector<3> t) {
        prior_model = matrix<4>::Identity();
        prior_model.block<3, 3>(0, 0) = R;
        prior_model.block<3, 1>(0, 3) = t;
    }

    void SetNormScale(double scale = 1.0) { m_norm_scale = scale; }
    template <typename... DataTypes>
    bool ComputePriorDistribution(const std::vector<DataTypes> &...data) {

        std::tuple<const std::vector<DataTypes> &...> tdata =
            std::make_tuple(std::cref(data)...);
        size_t size = std::get<0>(tdata).size();

        auto &pts1 = std::get<0>(tdata);
        auto &pts2 = std::get<1>(tdata);

        size_t inlier_count = 0;
        m_prior_inliers_mask = std::vector<char>(size, 0);

        ModelEvaluator eval(prior_model);
        for (size_t i = 0; i < size; ++i) {
            double error = eval(data[i]...);
            if (error <= threshold * 2.0) {
                inlier_count++;
                m_prior_inliers_mask[i] = 1;
            }
        }

        // if(inlier_count < 20 ) //|| inlier_count * 1.0 / size < 0.2
        if ((double)inlier_count / size < 0.15 ||
            inlier_count < 20) //|| inlier_count * 1.0 / size < 0.2
            return false;

        return true;
    }

    void SetDynamicProbability(const double &p) { m_dynamic_probability = p; }

  private:
    size_t m_nBinsX, m_nBinsY, m_nBins;
    std::vector<vector<2>> m_binLocations;
    std::vector<size_t> m_mapBinToValidBin;
    std::vector<size_t> m_mapValidBinToBin;
    std::vector<size_t> m_mapDataToValidBin;
    std::vector<std::vector<size_t>> m_validBinData;
    std::vector<size_t> m_validBinDataSizes;
    std::vector<float> m_validBinLens;

    std::vector<float> m_validBinConfidences;
    std::vector<float> m_validBinConfidencesPrior;
    std::vector<float> m_validBinConfidencesAccumulatedPrior;
    float m_parsacMinPriorBinConfidence;

    size_t m_nValidBins;
    float m_BinHeight;
    float m_BinWidth;
    bool m_verbose = false;

    double m_dynamic_probability = 0;
    double m_norm_scale = 1.0;

    float ComputeScore(std::vector<std::vector<size_t>> &validBinInliers,
                       std::vector<float> &validBinConfidences) {

        validBinConfidences.resize(m_nValidBins);

        float validBinConfidenceSum = 0, validBinConfidenceSqSum = 0;
        vector<2> sum(0.0, 0.0);

        for (size_t iBinValid = 0; iBinValid < m_nValidBins; ++iBinValid) {
            float t = 1 - std::pow(m_dynamic_probability,
                                   0.10 * m_validBinLens[iBinValid]);
            const float validBinConfidence =
                t * float(validBinInliers[iBinValid].size()) /
                m_validBinDataSizes[iBinValid];
            validBinConfidences[iBinValid] = validBinConfidence;
            vector<2> x = m_binLocations[m_mapValidBinToBin[iBinValid]];
            x *= validBinConfidence;
            sum += x;
            validBinConfidenceSum += validBinConfidence;
            validBinConfidenceSqSum += validBinConfidence * validBinConfidence;
        }
        float norm = 1.f / validBinConfidenceSum;
        vector<2> &mean = sum;
        mean *= norm;

        float Cxx = 0, Cxy = 0, Cyy = 0;
        for (size_t iBinValid = 0; iBinValid < m_nValidBins; ++iBinValid) {
            const float validBinConfidence = validBinConfidences[iBinValid];
            vector<2> x = m_binLocations[m_mapValidBinToBin[iBinValid]];
            vector<2> dx(x[0] - mean[0], x[1] - mean[1]);
            Cxx += (dx[0] * dx[0]) * validBinConfidence;
            Cxy += (dx[0] * dx[1]) * validBinConfidence;
            Cyy += (dx[1] * dx[1]) * validBinConfidence;
        }

        norm = validBinConfidenceSum /
               (validBinConfidenceSum * validBinConfidenceSum -
                validBinConfidenceSqSum);
        float imgRatio =
            norm * sqrt(Cxx * Cyy - Cxy * Cxy); // * M_PI / (2.0 * 2.0);
        float score = imgRatio * validBinConfidenceSum;
        return score;
    }

    void SetBins(const int nBinsX, const int nBinsY) {
        // Some bug occurred! the m_nBinsX is set to be the nBinsX at unknown
        // time！ if(m_nBinsX == nBinsX && m_nBinsY == nBinsY)
        //     return;

        m_nBinsX = nBinsX;
        m_nBinsY = nBinsY;
        m_nBins = m_nBinsX * m_nBinsY;
        m_BinHeight = 2 * m_norm_scale / m_nBinsY;
        m_BinWidth = 2 * m_norm_scale / m_nBinsX;
    }

    void CreateBucket() {
        m_binLocations.reserve(m_nBins);
        float y = m_BinHeight * 0.5;
        for (size_t i = 0; i < m_nBinsY; ++i, y += m_BinHeight) {
            float x = m_BinWidth * 0.5;
            for (size_t j = 0; j < m_nBinsX; ++j, x += m_BinWidth) {
                m_binLocations.emplace_back(
                    vector<2>(x - m_norm_scale, y - m_norm_scale));
            }
        }
    }

    template <typename DataTypes>
    void BucketData(const std::vector<DataTypes> &pts) {
        const size_t N = pts.size();
        m_mapDataToValidBin.resize(N);
        m_mapBinToValidBin = std::vector<size_t>(m_nBins, SIZE_MAX);
        for (size_t i = 0; i < N; ++i) {
            const vector<2> &p1 = pts[i];
            const size_t iBin =
                size_t((p1[0] + m_norm_scale) / m_BinWidth) +
                m_nBinsX * size_t((p1[1] + m_norm_scale) / m_BinHeight);
            const size_t iBinValid = m_mapBinToValidBin[iBin];
            if (iBinValid == SIZE_MAX) {
                m_mapBinToValidBin[iBin] = size_t(m_mapValidBinToBin.size());
                m_mapDataToValidBin[i] = size_t(m_mapValidBinToBin.size());
                ;
                m_mapValidBinToBin.push_back(iBin);
                m_validBinData.push_back(std::vector<size_t>(1, i));
                m_validBinDataSizes.push_back(1);
                m_validBinLens.push_back(m_lens[i]);
            } else {
                m_mapDataToValidBin[i] = iBinValid;
                m_validBinData[iBinValid].push_back(i);
                ++m_validBinDataSizes[iBinValid];
                m_validBinLens[iBinValid] += m_lens[i];
            }
        }

        m_nValidBins = m_validBinDataSizes.size();

        for (size_t i = 0; i < m_nValidBins; i++) {
            m_validBinLens[i] /= m_validBinDataSizes[i];
        }
    }

    void ConvertInliersListToValidBin(
        std::vector<char> &inliers_mask,
        std::vector<std::vector<size_t>> &validBinInliers) {
        validBinInliers = std::vector<std::vector<size_t>>(
            m_nValidBins, std::vector<size_t>());
        for (int iBinValid = 0; iBinValid < m_nValidBins; ++iBinValid) {
            validBinInliers[iBinValid].resize(0);
        }

        const size_t N = inliers_mask.size();
        for (size_t i = 0; i < N; ++i) {
            if (inliers_mask[i] == 1) {
                validBinInliers[m_mapDataToValidBin[i]].push_back(i);
            }
        }
    }

    void
    ConvertConfidencesBinToValidBin(const std::vector<float> &binConfidences,
                                    std::vector<float> &validBinConfidences) {

        validBinConfidences.resize(m_nValidBins);
        for (size_t iBin = 0; iBin < m_nValidBins; ++iBin) {
            validBinConfidences[iBin] =
                binConfidences[m_mapValidBinToBin[iBin]];
        }
    }

    void ConvertConfidencesValidBinToBin(
        const std::vector<float> &validBinConfidences,
        std::vector<float> &binConfidences) {

        binConfidences.resize(m_nBins);
        for (size_t iBin = 0; iBin < m_nBins; ++iBin) {
            if (m_mapBinToValidBin[iBin] == SIZE_MAX)
                binConfidences[iBin] = 0;
            else
                binConfidences[iBin] =
                    validBinConfidences[m_mapBinToValidBin[iBin]];
        }
    }

    void ThresholdAndNormalizeConfidences(std::vector<float> &confidences) {
        float confidenceSum = 0;
        const size_t N = confidences.size();
        for (size_t i = 0; i < N; ++i) {
            confidences[i] =
                std::max(m_parsacMinPriorBinConfidence, confidences[i]);
            confidenceSum += confidences[i];
        }
        const float norm = 1.0 / confidenceSum;
        for (size_t i = 0; i < N; ++i) {
            confidences[i] *= norm;
        }
    }

    void AccumulateConfidences(const std::vector<float> &confidences,
                               std::vector<float> &confidencesAccumulated) {
        const size_t N = confidences.size();
        confidencesAccumulated.resize(N + 1);
        confidencesAccumulated[0] = 0;
        for (size_t i = 0; i < N; ++i)
            confidencesAccumulated[i + 1] =
                confidencesAccumulated[i] + confidences[i];
        const float norm = 1.f / confidencesAccumulated[N];
        for (size_t i = 0; i < N; ++i)
            confidencesAccumulated[i] *= norm;
    }

  private:
    template <class Data, class Sample>
    void make_sample(Data &&data, Sample &&sample, size_t idata,
                     size_t isample) {
        size_t I =
            std::tuple_size<typename std::remove_reference<Data>::type>::value;
        std::get<0>(sample)[isample] = std::get<0>(data)[idata];
        std::get<1>(sample)[isample] = std::get<1>(data)[idata];
    }

    template <class Data, class Sample>
    void make_sample_by_prior(Data &&data, Sample &&sample, size_t idata,
                              size_t isample) {

        size_t idx =
            m_validBinData[idata][rand() % m_validBinData[idata].size()];
        std::get<0>(sample)[isample] = std::get<0>(data)[idx];
        std::get<1>(sample)[isample] = std::get<1>(data)[idx];
    }
};

} // namespace xrslam

#endif // XRSLAM_IMU_PARSAC_H
