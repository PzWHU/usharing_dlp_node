/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 */
#pragma once

#include <stddef.h>
#include <sys/time.h>
#include <string.h>
#include <stdio.h>
#include <string>

time_t GetCurrentMillion();

void LogToFileSync(const char * format, ...);

class AutoTimerLog {
 public:
    explicit AutoTimerLog(
        const std::string & info,
        const std::string & file,
        unsigned int line);

    ~AutoTimerLog();

 private:
    timeval time_start_;
    timeval time_end_;
    std::string info_;
    std::string file_;
    unsigned int line_;
};


#define __CURRENT_FILENAME__ \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)



