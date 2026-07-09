/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@senseauto.com>
 */
#pragma once
#ifdef WITH_SERIALIZATION_INTERFACE
/*
 * @brief provide common boost::serialization and its extension for commonly
 * used datatype.
 */

#include <memory>
#include <vector>
#include <deque>
#include <string>

#include "Eigen/Core"

#include "boost/serialization/serialization.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/serialization/vector.hpp"
#include "boost/serialization/string.hpp"
#include "boost/serialization/deque.hpp"
#include "boost/serialization/map.hpp"
#include "boost/serialization/export.hpp"
#include "boost/serialization/unordered_map.hpp"
#include "boost/serialization/set.hpp"
#include "boost/serialization/shared_ptr.hpp"
#include "boost/serialization/unique_ptr.hpp"

/*
 * @brief eigen boost serialization extension
 */
namespace boost {
namespace serialization {

template <class Archive,
          typename _Scalar,
          int _Rows,
          int _Cols,
          int _Options,
          int _MaxRows,
          int _MaxCols>
inline void serialize(
    Archive& ar,  // NOLINT
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>&
        t,  // NOLINT
    const unsigned int file_version) {
    size_t rows = t.rows(), cols = t.cols();
    ar& rows;
    ar& cols;
    if (rows * cols != t.size()) {
        t.resize(rows, cols);
    }
    for (size_t i = 0; i < t.size(); i++) {
        ar& t.data()[i];
    }
}

}  // namespace serialization
}  // namespace boost
#endif
