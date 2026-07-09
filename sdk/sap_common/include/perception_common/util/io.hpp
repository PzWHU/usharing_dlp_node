/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Jiang Qinhong <jiangqinhong@sensetime.com>
 */
#pragma once

#include <fcntl.h>
#include <stdint.h>

#include <iostream>
#include <fstream>
#include <string>

#include "google/protobuf/text_format.h"
#include "google/protobuf/message.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

using google::protobuf::Message;
using google::protobuf::io::FileInputStream;
using google::protobuf::io::FileOutputStream;

inline apStatus_t ReadBinProtoFromFile(const char* file_path, Message* proto) {
    std::ifstream infile;
    std::string input;
    infile.open(file_path, std::ios::in);
    if (!infile.is_open()) {
        AP_LERROR(ReadBinProtoFromFile) << "invalid file " << file_path;
        AP_CHECK_ERROR(AP_INVALID_FILE_FORMAT);
        exit(1);
    }
    std::stringstream buffer;
    buffer << infile.rdbuf();
    std::string contents(buffer.str());
    int length = contents.size();
    size_t n = 0;
    while (n * 8 < length) {
        std::string p = contents.substr(n * 8, 8);
        n++;
        int x = 255 - std::stoi(p, 0, 2);
        input = input + static_cast<char>(x);
    }
    infile.close();
    bool success = google::protobuf::TextFormat::ParseFromString(input, proto);
    if (!success) {
        AP_LERROR(ReadBinProtoFromFile) << "invalid file format::" << file_path;
        AP_CHECK_ERROR(AP_INVALID_FILE_FORMAT);
    }
    return AP_SUCCESS;
}

inline apStatus_t ReadTxtProtoFromFile(const char* file_path, Message* proto) {
    AP_LINFO(ReadFileToString) << "start read file: " << file_path;
    int fd = open(file_path, O_RDONLY);
    if (-1 == fd) {
        AP_LERROR(ReadTxtProtoFromFile) << "file not found::" << file_path;
        AP_CHECK_ERROR(AP_FILE_NOT_FOUND);
    }
    FileInputStream* input = new FileInputStream(fd);
    bool success = google::protobuf::TextFormat::Parse(input, proto);
    delete input;
    close(fd);
    if (!success) {
        AP_LERROR(ReadTxtProtoFromFile) << "invalid file format::" << file_path;
        AP_CHECK_ERROR(AP_INVALID_FILE_FORMAT);
    }
    return AP_SUCCESS;
}

inline apStatus_t ReadProtoFromFile(const char* file_path, Message* proto) {
    if (nullptr == file_path || nullptr == proto) {
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    std::string str_file_path(file_path);
    std::string suffixStr =
        str_file_path.substr(str_file_path.find_last_of('.') + 1);
    if (suffixStr == "bin") {
        return ReadBinProtoFromFile(file_path, proto);
    } else {
        return ReadTxtProtoFromFile(file_path, proto);
    }
}

inline apStatus_t ReadProtoFromFile(const std::string& file_path,
                                    Message* proto) {
    if (nullptr == proto) {
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    return ReadProtoFromFile(file_path.c_str(), proto);
}

inline apStatus_t WriteProtoToFile(const Message& proto,
                                   const char* file_path) {
    if (nullptr == file_path) {
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    AP_LINFO(ReadFileToString) << "start read file: " << file_path;
    int fd = open(file_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    FileOutputStream* output = new FileOutputStream(fd);
    google::protobuf::TextFormat::Print(proto, output);
    delete output;
    close(fd);
    return AP_SUCCESS;
}

inline apStatus_t WriteProtoToFile(const Message& proto,
                                   const std::string& file_path) {
    return WriteProtoToFile(proto, file_path.c_str());
}

inline apStatus_t ReadFileToString(const std::string& file_path,
                                   std::string* str) {
    if (nullptr == str) {
        AP_CHECK_ERROR(AP_NULL_PTR);
    }
    AP_LINFO(ReadFileToString) << "start read file: " << file_path;
    std::filebuf in;
    if (!in.open(file_path, std::ios::in)) {
        AP_CHECK_ERROR(AP_FILE_NOT_FOUND);
    }

    std::istream iss(&in);
    std::istreambuf_iterator<char> eos;

    std::string config_json(std::istreambuf_iterator<char>(iss), eos);
    *str = config_json;

    in.close();

    AP_LINFO(ReadFileToString) << "Finish read file: " << file_path;
    return AP_SUCCESS;
}

inline std::string JoinPath(std::string root, std::string path) {
    if (root.empty()) {
        AP_LDEBUG(JoinPath) << "root path empty, may relative path used.";
        return path;
    }
    if (path.empty()) {
        AP_LDEBUG(JoinPath) << "file path empty, please check your file path.";
    }
    return (root.back() == '/' ? root + path : root + '/' + path);
}

}  // namespace perception
}  // namespace senseAD
