/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 */

#pragma once

#include <unordered_map>
#include <string>
#include <map>
#include <vector>
#include <memory>

#include "Eigen/Core"

#include "perception_common/log/error.hpp"
#include "perception_common/log/common_log.hpp"

namespace senseAD {
namespace perception {

enum class ProcessModelType {
    UNKNOWN_PROCESSMODEL = -1,
    CV = 0,
    CA = 1,
    CJ = 2,
    CP = 3,
    CT = 4,
    CTRA = 5
};

static const std::unordered_map<std::string, ProcessModelType>
    kLabelToProcessModelType{
        {"UNKNOWN_PROCESSMODEL", ProcessModelType::UNKNOWN_PROCESSMODEL},
        {"CV", ProcessModelType::CV},
        {"CA", ProcessModelType::CA},
        {"CJ", ProcessModelType::CJ},
        {"CP", ProcessModelType::CP},
        {"CT", ProcessModelType::CT},
        {"CTRA", ProcessModelType::CTRA}};

static const std::map<ProcessModelType, std::string> kProcessModelTypeToString{
    {ProcessModelType::UNKNOWN_PROCESSMODEL, "UNKNOWN_PROCESSMODEL"},
    {ProcessModelType::CV, "CV"},
    {ProcessModelType::CA, "CA"},
    {ProcessModelType::CJ, "CJ"},
    {ProcessModelType::CP, "CP"},
    {ProcessModelType::CT, "CT"},
    {ProcessModelType::CTRA, "CTRA"}};

static const std::unordered_map<std::string, ProcessModelType>
    kProcessNameToProcessModelType{
        {"unknown_model", ProcessModelType::UNKNOWN_PROCESSMODEL},
        {"cv", ProcessModelType::CV},
        {"ca", ProcessModelType::CA},
        {"cj", ProcessModelType::CJ},
        {"co", ProcessModelType::CP},
        {"ct", ProcessModelType::CT},
        {"ctra", ProcessModelType::CTRA}};
inline std::string GetProcessModelTypeDescription(
    const ProcessModelType& status) {
    std::string status_description = "";
    auto iter = kProcessModelTypeToString.find(status);
    if (iter != kProcessModelTypeToString.end()) {
        status_description = iter->second;
    }
    return status_description;
}

inline apStatus_t CheckProcessModelNameExist(const std::string& sensor_name) {
    if (kProcessNameToProcessModelType.find(sensor_name) ==
        kProcessNameToProcessModelType.end()) {
        AP_LERROR(CheckProcessModelNameExist)
            << "Sensor name -> " << sensor_name
            << " not found in kProcessNameToProcessModelType, please check "
               "plugion/process_model/process_model_info.hpp";
        AP_CHECK_ERROR(AP_INTERNAL_ERROR);
    }
    return AP_SUCCESS;
}

inline bool IsCV(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckProcessModelNameExist(sensor_name)) {
        return false;
    }
    if (ProcessModelType::CV ==
        kProcessNameToProcessModelType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsCA(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckProcessModelNameExist(sensor_name)) {
        return false;
    }
    if (ProcessModelType::CA ==
        kProcessNameToProcessModelType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsCJ(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckProcessModelNameExist(sensor_name)) {
        return false;
    }
    if (ProcessModelType::CJ ==
        kProcessNameToProcessModelType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsCT(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckProcessModelNameExist(sensor_name)) {
        return false;
    }
    if (ProcessModelType::CT ==
        kProcessNameToProcessModelType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsCP(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckProcessModelNameExist(sensor_name)) {
        return false;
    }
    if (ProcessModelType::CP ==
        kProcessNameToProcessModelType.at(sensor_name)) {
        return true;
    }
    return false;
}

inline bool IsCTRA(const std::string& sensor_name) {
    if (AP_SUCCESS != CheckProcessModelNameExist(sensor_name)) {
        return false;
    }
    if (ProcessModelType::CTRA ==
        kProcessNameToProcessModelType.at(sensor_name)) {
        return true;
    }
    return false;
}

}  // namespace perception
}  // namespace senseAD
