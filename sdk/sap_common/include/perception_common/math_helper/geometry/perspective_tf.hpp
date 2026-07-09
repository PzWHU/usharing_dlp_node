/*
 * Copyright (C) 2017-2020 by SenseTime Group Limited. All rights reserved.
 * YU Chendi <yuchendi@sensetime.com>
 * He Yuan <heyuan@sensetime.com>
 */

#pragma once

#include <vector>
#include <memory>

#include "fastcv.hpp"  // NOLINT

#include "perception_common/log/error.hpp"
#include "perception_common/adapter/device_adapter.hpp"

namespace senseAD {
namespace perception {

template <typename T = float, EcoEnv_t type = FASTCV_DATA_TYPE_CPU>
class Transformer {
 public:
    Transformer() {}
    virtual ~Transformer() {}
    virtual apStatus_t Init(uint32_t height,
                            uint32_t width,
                            const std::shared_ptr<double> &matrix_data) = 0;
    virtual apStatus_t Transform(const HPC::fastcv::Point2_<T> &src,
                                 HPC::fastcv::Point2_<T> *dst) = 0;
    virtual apStatus_t Transform(
        const std::vector<HPC::fastcv::Point2_<T>> &src,
        std::vector<HPC::fastcv::Point2_<T>> *dst) = 0;
    virtual apStatus_t Transform(const HPC::fastcv::Mat<T, 2, type> &src,
                                 HPC::fastcv::Mat<T, 2, type> *dst) = 0;
};

template <typename T = float, EcoEnv_t type = FASTCV_DATA_TYPE_CPU>
class PerspectiveTransformer : public Transformer<T, type> {
 public:
    PerspectiveTransformer() = default;
    virtual ~PerspectiveTransformer() = default;
    virtual apStatus_t Init(uint32_t height,
                            uint32_t width,
                            const std::shared_ptr<double> &matrix_data) {
        transform_matrix_ =
            std::make_shared<HPC::fastcv::Mat<double, 1, type>>(height, width);
        transform_matrix_->fromHost(matrix_data.get());
        return AP_SUCCESS;
    }
    virtual apStatus_t Transform(const HPC::fastcv::Point2_<T> &src,
                                 HPC::fastcv::Point2_<T> *dst);
    virtual apStatus_t Transform(
        const std::vector<HPC::fastcv::Point2_<T>> &src,
        std::vector<HPC::fastcv::Point2_<T>> *dst);
    virtual apStatus_t Transform(const HPC::fastcv::Mat<T, 2, type> &src,
                                 HPC::fastcv::Mat<T, 2, type> *dst);

 private:
    std::shared_ptr<HPC::fastcv::Mat<double, 1, type>> transform_matrix_;

    apStatus_t ConvertVectorPointsToMat(
        const std::vector<HPC::fastcv::Point2_<T>> &src,
        HPC::fastcv::Mat<T, 2, type> *dst) const;

    apStatus_t ConvertMatToVectorPoints(
        const HPC::fastcv::Mat<T, 2, type> &src,
        std::vector<HPC::fastcv::Point2_<T>> *dst) const;
};

typedef std::shared_ptr<Transformer<float, FASTCV_DATA_TYPE_CPU>>
    TransformerPtr;

}  // namespace perception
}  // namespace senseAD
