/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Zhang Shiquan <zhangshiquan@sensetime.com>
 * He Yuan <heyuan@sensetime.com>
 */

#pragma once

#include "fastcv.hpp"  // NOLINT

#include "perception_common/log/error.hpp"

namespace senseAD {
namespace perception {

// CUDA: grid stride looping
#ifdef USE_CUDA
#define CUDA_KERNEL_LOOP(i, n)                                   \
    for (int i = blockIdx.x * blockDim.x + threadIdx.x; i < (n); \
         i += blockDim.x * gridDim.x)

// CUDA: use 64 threads per block
const int CUDA_NUM_THREADS = 64;

// CUDA: number of blocks for threads.
inline int CUDA_GET_BLOCKS(const int N) {
    return (N + CUDA_NUM_THREADS - 1) / CUDA_NUM_THREADS;
}
#endif

// fastcv data type
#ifndef FASTCV_DATA_TYPE_GPU
#define FASTCV_DATA_TYPE_GPU EcoEnv_t::ECO_ENV_CUDA
#endif
#ifdef USE_ARM
#ifndef FASTCV_DATA_TYPE_CPU
#define FASTCV_DATA_TYPE_CPU EcoEnv_t::ECO_ENV_ARM
#endif
#elif defined(USE_X86)
#ifndef FASTCV_DATA_TYPE_CPU
#define FASTCV_DATA_TYPE_CPU EcoEnv_t::ECO_ENV_X86
#endif
#endif

#ifdef USE_CUDA
#ifndef FASTCV_DATA_TYPE_DEFAULT
#define FASTCV_DATA_TYPE_DEFAULT EcoEnv_t::ECO_ENV_CUDA
#endif
#elif defined(USE_ARM)
#ifndef FASTCV_DATA_TYPE_DEFAULT
#define FASTCV_DATA_TYPE_DEFAULT EcoEnv_t::ECO_ENV_ARM
#endif
#elif defined(USE_X86)
#ifndef FASTCV_DATA_TYPE_DEFAULT
#define FASTCV_DATA_TYPE_DEFAULT EcoEnv_t::ECO_ENV_X86
#endif
#endif

#ifdef USE_CUDA
// TODO(heyuan): optimize with cudaMemcpyAsync
#ifndef ATOM_MEM_COPY_UNIVERSAL
#define ATOM_MEM_COPY_UNIVERSAL(target_atom, source_atom, size)                \
    {                                                                          \
        AP_LDEBUG(ATOM_MEM_COPY_UNIVERSAL) << "USE_CUDA: cudaMemcpy ...";      \
        auto code =                                                            \
            cudaMemcpy(target_atom->MutableDevicePointer(),                    \
                       source_atom->DevicePointer(), size, cudaMemcpyDefault); \
        if (code != cudaSuccess) {                                             \
            AP_LFATAL(ATOM_MEM_COPY_UNIVERSAL) << "cudaMemcpy fail !!!";       \
        }                                                                      \
    }
#endif
#else
#ifndef ATOM_MEM_COPY_UNIVERSAL
#define ATOM_MEM_COPY_UNIVERSAL(target_atom, source_atom, size)               \
    {                                                                         \
        AP_LDEBUG(ATOM_MEM_COPY_UNIVERSAL) << "USE_X86: memcpy ...";          \
        memcpy(target_atom->MutableHostPointer(), source_atom->HostPointer(), \
               size);                                                         \
    }
#endif
#endif

}  // namespace perception
}  // namespace senseAD
