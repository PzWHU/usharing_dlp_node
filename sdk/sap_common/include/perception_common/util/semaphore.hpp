/*
 * Copyright (C) 2019-20 by SenseTime Group Limited. All rights reserved.
 * Wang Jiadong <wangjiadong@senseauto.com>
 */

/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 */

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
//  #ifdef _POSIX_SEMAPHORES
//    #if _POSIX_SEMAPHORES > 0
#include "perception_common/util/semaphore_pos.hpp"
//    #endif
//  #endif
// #endif

