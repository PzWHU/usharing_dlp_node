/*
 * Copyright (C) 2019-20 by SenseTime Group Limited. All rights reserved.
 * Wang Jiadong <wangjiadong@senseauto.com>
 */

/**
 * @file /include/ecl/ipc/shared_memory.hpp
 *
 * @brief Provides a class for sharing data between processes.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#pragma once

/*****************************************************************************
** Platform Detection
*****************************************************************************/

/*****************************************************************************
** Cross Platform Implementation
*****************************************************************************/

// #if defined(ECL_IS_POSIX)
//  #ifdef _POSIX_SHARED_MEMORY_OBJECTS
//    #if _POSIX_SHARED_MEMORY_OBJECTS > 0
#include "perception_common/util/shared_memory_pos.hpp"
//    #endif
//  #endif
// #endif

